#include "sts3032.h"

#include "esp32/rom/ets_sys.h"
#include <string.h>
#include "bus/uart_bus.h"

#define INST_SYNC_READ      0x82
#define INST_SYNC_WRITE     0x83
#define INST_WRITE_DATA     0x03
#define SMS_STS_ACC         0x29

static constexpr uint8_t SERVO_NUM = 2;

struct command_packet
{
    uint8_t id;
    int16_t position;
    int16_t speed;
    uint8_t acc;
};

static uart_bus servo_uart(2);
static command_packet packet[SERVO_NUM];      // [0] 为左舵机，[1] 为右舵机
static command_packet last_state[SERVO_NUM];      // 记录上一次的舵机目标参数
sts3032::status_t sts3032::status[2];
static uint8_t tx_buf[128], tx_len = 0;       // 发送缓冲区

/**
 * @brief 向舵机发送缓冲区追加数据
 *
 * @param data 数据缓冲区
 * @param len 数据长度
 *
 * @return 实际处理的数据长度
 */
static uint32_t append_tx(uint8_t *data, uint32_t len)
{
    while(len--)
    {
        if(tx_len < sizeof(tx_buf))
        {
            tx_buf[tx_len++] = *data;
            data++;
        }
    }

    return tx_len;
}

/**
 * @brief 发送并清空舵机发送缓冲区
 */
static void flush_tx()
{
    ets_delay_us(15);
    if(tx_len)
    {
        servo_uart.write_bytes(tx_buf, tx_len);
        tx_len = 0;
    }
}

/**
 * @brief 拆分 int16_t 数据的低字节和高字节
 *
 * @param data_l 低字节输出地址
 * @param data_h 高字节输出地址
 * @param data 数据缓冲区
 */
static void split_int16(uint8_t *data_l, uint8_t *data_h, int16_t data)
{
    *data_h = data >> 8;
    *data_l = data & 0xFF;
}

/**
 * @brief 发送 STS 舵机同步读命令
 *
 * @param ids 舵机 ID 列表
 * @param servo_num 舵机数量
 * @param start_reg 起始寄存器地址
 * @param len 数据长度
 */
static void sync_read(uint8_t *ids, uint8_t servo_num, uint8_t start_reg, uint8_t len)
{
    uint8_t cmd[10];
    uint8_t idx = 0;

    cmd[idx++] = 0xFF;
    cmd[idx++] = 0xFF;
    cmd[idx++] = 0xFE;       // 广播 id
    cmd[idx++] = servo_num + 4;      // 长度
    cmd[idx++] = INST_SYNC_READ;
    cmd[idx++] = start_reg;
    cmd[idx++] = len;       // 读取长度

    // 添加舵机 id
    for(uint8_t i = 0; i < servo_num; i++)
    {
        cmd[idx++] = ids[i];
    }

    // 校验和
    uint8_t sum = 0;
    for(uint8_t i = 2; i < idx; i++){sum += cmd[i];}
    cmd[idx++] = (~sum) & 0xFF;

    servo_uart.write_bytes(cmd, idx);
}

/**
 * @brief 发送 STS 舵机同步写命令
 *
 * @param id_num 舵机数量
 * @param mem_addr 内存表起始地址
 * @param data 每个舵机的数据缓冲区
 * @param len 每个舵机的数据长度
 */
static void sync_write(uint8_t id_num, uint8_t mem_addr, uint8_t *data, uint8_t len)
{
    uint8_t msg_len = (len + 1) * id_num + 4;

    uint8_t cmd[7], index = 0;

    cmd[index++] = 0xFF;
    cmd[index++] = 0xFF;
    cmd[index++] = 0xFE;     // 广播

    cmd[index++] = msg_len;
    cmd[index++] = INST_SYNC_WRITE;
    cmd[index++] = mem_addr;
    cmd[index++] = len;

    append_tx(cmd, index);

    uint8_t sum = 0xFE + msg_len + INST_SYNC_WRITE + mem_addr + len;
    for(uint8_t i = 0; i < id_num; i++)
    {
        append_tx(&packet[i].id, 1);
        append_tx(&data[i * len], len);

        sum += packet[i].id;
        for(uint8_t j = 0; j < len; j++)
        {
            sum += data[i * len + j];
        }
    }
    sum = ~sum;

    append_tx(&sum, 1);
    flush_tx();
}

/**
 * @brief 构建舵机位置数据并执行同步写
 *
 * @param id_num 舵机数量
 */
static void sync_write_position(uint8_t id_num)
{
    uint8_t cmd[id_num * 7];

    command_packet *p;
    for(uint8_t i = 0; i < id_num; i++)
    {
        p = &packet[i];

        cmd[i * 7 + 0] = p->acc;
        split_int16(&cmd[i * 7 + 1], &cmd[i * 7 + 2], p->position);
        split_int16(&cmd[i * 7 + 3], &cmd[i * 7 + 4], 0);
        split_int16(&cmd[i * 7 + 5], &cmd[i * 7 + 6], p->speed);
    }

    sync_write(id_num, SMS_STS_ACC, cmd, 7);
}

/**
 * @brief 解析 STS 舵机同步读返回帧
 *
 * @param ids 舵机 ID 列表
 * @param n 舵机数量
 * @param status 状态快照
 *
 * @return 解析结果状态码
 */
static int32_t parse_sync_read(uint8_t *ids, uint8_t n, sts3032::status_t *status)
{
    uint8_t buf[64];
    uint32_t len = servo_uart.read_bytes(buf, sizeof(buf));

    uint32_t offset = 0;
    for(uint8_t i = 0; i < n; i++)
    {
        if(offset + 7 > len) return -1; // 帧不完整

        if(buf[offset] != 0xFF || buf[offset+1] != 0xFF || buf[offset+2] != ids[i])
            return -2; // 帧头或ID不对

        uint8_t frame_len = buf[offset+3];  // LEN 字节
        uint8_t chk = 0;
        for(uint8_t j = 2; j < frame_len + 3; j++) chk += buf[offset + j];
        chk = ~chk;

        if(chk != buf[offset + 3 + frame_len]) return -3; // 校验失败

        // 解析位置
        status[i].id = ids[i];
        status[i].position = (int16_t)((buf[offset+6]<<8)|buf[offset+5]);

        // 解析负载（带方向，直接在这里计算正负）
        int16_t raw_load = (int16_t)((buf[offset+10]<<8)|buf[offset+9]);
        int16_t duty = raw_load & 0x03FF;        // BIT0~BIT9 占空比
        uint8_t dir = (raw_load >> 10) & 0x01;   // BIT10 方向
        status[i].load = dir ? -duty : duty;    // 方向位1反转，用负值表示

        offset += frame_len + 4; // 移动到下一帧
    }

    return 0;
}

/**
 * @brief 读取左右腿舵机的位置和负载
 */
void sts3032::get_position_and_load()
{
    uint8_t ids[] = {SERVO_LEFT, SERVO_RIGHT};

    sync_read(ids, SERVO_NUM, 0x38, 6);
    parse_sync_read(ids, 2, status);
}

/**
 * @brief 设置指定舵机的扭矩开关模式
 *
 * @param id 字段或设备 ID
 * @param type 扭矩模式类型
 */
void sts3032::set_torque_switch(uint8_t id, uint8_t type)
{
    uint8_t cmd[8];
    uint8_t idx = 0;

    cmd[idx++] = 0xFF;
    cmd[idx++] = 0xFF;
    cmd[idx++] = id;
    cmd[idx++] = 0x04;     // 长度
    cmd[idx++] = INST_WRITE_DATA;
    cmd[idx++] = 0x28;
    cmd[idx++] = type;

    // checksum
    uint8_t sum = 0;
    for(uint8_t i = 2; i < idx; i++)
    {
        sum += cmd[i];
    }
    cmd[idx++] = (~sum) & 0xFF;

    append_tx(cmd, sizeof(cmd));
    flush_tx();
}

/**
 * @brief 设置指定舵机的目标位置参数
 *
 * @param id 字段或设备 ID
 * @param position 舵机位置计数值
 * @param speed 舵机速度
 * @param acc acc
 */
void sts3032::set(uint8_t id, int16_t position, int16_t speed, uint8_t acc)
{
    command_packet *p = nullptr;

    if(id == SERVO_LEFT){p = &packet[0];}
    else if(id == SERVO_RIGHT){p = &packet[1];}
    if(!p){return;}

    p->id = id;
    p->position = position;
    p->speed = speed;
    p->acc = acc;
}

/**
 * @brief 在目标参数变化时同步移动舵机
 */
void sts3032::move()
{
    for(uint8_t i = 0; i < SERVO_NUM; i++)
    {
        if(memcmp(&last_state[i], &packet[i], sizeof(command_packet)))
        {
            sync_write_position(SERVO_NUM);
            memcpy(last_state, packet, SERVO_NUM * sizeof(command_packet));
            break;
        }
    }
}

/**
 * @brief 初始化 STS3032 舵机总线和扭矩状态
 */
void sts3032::init()
{
    servo_uart.init();

    // 中位校准前先关闭扭矩输出
    set_torque_switch(SERVO_LEFT, 0);
    delay(100);
    set_torque_switch(SERVO_RIGHT, 0);
    delay(1000);

    set_torque_switch(SERVO_LEFT, 128);
    set_torque_switch(SERVO_RIGHT, 128);
}
