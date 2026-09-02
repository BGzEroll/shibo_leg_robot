#include "servo.h"

#include "esp32/rom/ets_sys.h"
#include "string.h"
#include "bus/uart_bus.h"

/* ---- STS3032 协议与运行状态 ---- */

static constexpr uint8_t INST_SYNC_READ = 0x82;
static constexpr uint8_t INST_SYNC_WRITE = 0x83;
static constexpr uint8_t INST_WRITE_DATA = 0x03;
static constexpr uint8_t SMS_STS_ACC = 0x29;
static constexpr uint8_t SERVO_COUNT = 2;

struct command_packet
{
    uint8_t id = 0;
    int16_t position = 0;
    int16_t speed = 0;
    uint8_t acceleration = 0;
};

static uart_bus servo_uart(2);
static command_packet packet[SERVO_COUNT];
static command_packet last_state[SERVO_COUNT];
static uint8_t tx_buf[128];
static uint8_t tx_len = 0;

hw::servo::state hw::servo::leg_status[2];
pwm_servo hw::servo::camera(4, 4, 50, 16, 540, 2600);
pwm_servo hw::servo::frontier(15, 15, 50, 16, 540, 2600);

/* ---- STS3032 协议收发 ---- */

/**
 * @brief 向舵机发送缓冲区追加数据
 *
 * @param data 数据缓冲区
 * @param length 数据长度
 *
 * @return 当前发送缓冲区长度
 */
static uint32_t append_tx(uint8_t *data, uint32_t length)
{
    while(length--)
    {
        if(tx_len >= sizeof(tx_buf)){break;}
        tx_buf[tx_len++] = *data++;
    }
    return tx_len;
}

/**
 * @brief 发送并清空舵机发送缓冲区
 */
static void flush_tx()
{
    ets_delay_us(15);
    if(!tx_len){return;}
    servo_uart.write_bytes(tx_buf, tx_len);
    tx_len = 0;
}

/**
 * @brief 拆分 int16_t 数据的低字节和高字节
 *
 * @param data_l 低字节输出地址
 * @param data_h 高字节输出地址
 * @param data 数据
 */
static void split_int16(uint8_t *data_l, uint8_t *data_h, int16_t data)
{
    *data_h = (uint8_t)(data >> 8);
    *data_l = (uint8_t)data;
}

/**
 * @brief 发送 STS 舵机同步读命令
 *
 * @param ids 舵机 ID 列表
 * @param servo_count 舵机数量
 * @param start_register 起始寄存器地址
 * @param length 数据长度
 */
static void sync_read(uint8_t *ids, uint8_t servo_count, uint8_t start_register, uint8_t length)
{
    uint8_t command[10];
    uint8_t index = 0;
    command[index++] = 0xFF;
    command[index++] = 0xFF;
    command[index++] = 0xFE;
    command[index++] = servo_count + 4;
    command[index++] = INST_SYNC_READ;
    command[index++] = start_register;
    command[index++] = length;
    for(uint8_t i = 0; i < servo_count; i++){command[index++] = ids[i];}

    uint8_t sum = 0;
    for(uint8_t i = 2; i < index; i++){sum += command[i];}
    command[index++] = (uint8_t)(~sum);
    servo_uart.write_bytes(command, index);
}

/**
 * @brief 发送 STS 舵机同步写命令
 *
 * @param servo_count 舵机数量
 * @param address 内存表起始地址
 * @param data 每个舵机的数据缓冲区
 * @param length 每个舵机的数据长度
 */
static void sync_write(uint8_t servo_count, uint8_t address, uint8_t *data, uint8_t length)
{
    uint8_t message_length = (length + 1) * servo_count + 4;
    uint8_t command[7];
    uint8_t index = 0;
    command[index++] = 0xFF;
    command[index++] = 0xFF;
    command[index++] = 0xFE;
    command[index++] = message_length;
    command[index++] = INST_SYNC_WRITE;
    command[index++] = address;
    command[index++] = length;
    append_tx(command, index);

    uint8_t sum = 0xFE + message_length + INST_SYNC_WRITE + address + length;
    for(uint8_t i = 0; i < servo_count; i++)
    {
        append_tx(&packet[i].id, 1);
        append_tx(&data[i * length], length);
        sum += packet[i].id;
        for(uint8_t j = 0; j < length; j++){sum += data[i * length + j];}
    }
    uint8_t checksum = (uint8_t)(~sum);
    append_tx(&checksum, 1);
    flush_tx();
}

/**
 * @brief 构建舵机位置数据并执行同步写
 *
 * @param servo_count 舵机数量
 */
static void sync_write_position(uint8_t servo_count)
{
    uint8_t data[servo_count * 7];
    for(uint8_t i = 0; i < servo_count; i++)
    {
        command_packet &current = packet[i];
        data[i * 7] = current.acceleration;
        split_int16(&data[i * 7 + 1], &data[i * 7 + 2], current.position);
        split_int16(&data[i * 7 + 3], &data[i * 7 + 4], 0);
        split_int16(&data[i * 7 + 5], &data[i * 7 + 6], current.speed);
    }
    sync_write(servo_count, SMS_STS_ACC, data, 7);
}

/**
 * @brief 解析 STS 舵机同步读返回帧
 *
 * @param ids 舵机 ID 列表
 * @param servo_count 舵机数量
 * @param output 状态输出
 *
 * @return 解析成功时返回 0
 */
static int32_t parse_sync_read(uint8_t *ids, uint8_t servo_count, hw::servo::state *output)
{
    uint8_t buffer[64];
    uint32_t length = 0;
    if(servo_uart.read_bytes(buffer, sizeof(buffer), length) != uart_result::OK){return -1;}

    uint32_t offset = 0;
    for(uint8_t i = 0; i < servo_count; i++)
    {
        if(offset + 7 > length){return -1;}
        if(buffer[offset] != 0xFF || buffer[offset + 1] != 0xFF ||
           buffer[offset + 2] != ids[i]){return -2;}

        uint8_t frame_length = buffer[offset + 3];
        uint8_t checksum = 0;
        for(uint8_t j = 2; j < frame_length + 3; j++){checksum += buffer[offset + j];}
        if((uint8_t)(~checksum) != buffer[offset + 3 + frame_length]){return -3;}

        output[i].id = ids[i];
        output[i].position = (int16_t)((buffer[offset + 6] << 8) | buffer[offset + 5]);
        int16_t raw_load = (int16_t)((buffer[offset + 10] << 8) | buffer[offset + 9]);
        int16_t duty = raw_load & 0x03FF;
        uint8_t direction = (uint8_t)((raw_load >> 10) & 0x01);
        output[i].load = direction ? (int16_t)-duty : duty;
        offset += frame_length + 4;
    }
    return 0;
}

/* ---- servo 公共 API ---- */

/**
 * @brief 读取左右腿舵机位置和负载
 */
void hw::servo::get_position_and_load()
{
    uint8_t ids[] = {LEG_LEFT, LEG_RIGHT};
    sync_read(ids, SERVO_COUNT, 0x38, 6);
    parse_sync_read(ids, SERVO_COUNT, leg_status);
}

/**
 * @brief 设置指定舵机扭矩开关模式
 *
 * @param id 舵机 ID
 * @param type 扭矩模式类型
 */
void hw::servo::set_torque(uint8_t id, uint8_t type)
{
    uint8_t command[8];
    uint8_t index = 0;
    command[index++] = 0xFF;
    command[index++] = 0xFF;
    command[index++] = id;
    command[index++] = 0x04;
    command[index++] = INST_WRITE_DATA;
    command[index++] = 0x28;
    command[index++] = type;
    uint8_t sum = 0;
    for(uint8_t i = 2; i < index; i++){sum += command[i];}
    command[index++] = (uint8_t)(~sum);
    append_tx(command, sizeof(command));
    flush_tx();
}

/**
 * @brief 设置指定舵机目标位置参数
 *
 * @param id 舵机 ID
 * @param position 目标位置计数
 * @param speed 舵机速度
 * @param acceleration 舵机加速度
 */
void hw::servo::set(uint8_t id, int16_t position, int16_t speed, uint8_t acceleration)
{
    command_packet *target = nullptr;
    if(id == LEG_LEFT){target = &packet[0];}
    else if(id == LEG_RIGHT){target = &packet[1];}
    if(!target){return;}

    target->id = id;
    target->position = position;
    target->speed = speed;
    target->acceleration = acceleration;
}

/**
 * @brief 在目标参数变化时同步移动舵机
 */
void hw::servo::move()
{
    for(uint8_t i = 0; i < SERVO_COUNT; i++)
    {
        if(memcmp(&last_state[i], &packet[i], sizeof(command_packet)))
        {
            sync_write_position(SERVO_COUNT);
            memcpy(last_state, packet, SERVO_COUNT * sizeof(command_packet));
            break;
        }
    }
}

/**
 * @brief 初始化 STS3032 舵机总线
 */
void hw::servo::init()
{
    servo_uart.init();
}
