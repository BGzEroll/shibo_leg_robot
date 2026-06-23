#include "sts3032.h"

#define INST_SYNC_READ      0x82
#define INST_SYNC_WRITE     0x83
#define INST_WRITE_DATA     0x03
#define SMS_STS_ACC         0x29

typedef struct servoCmdPacket
{
    uint8_t id;
    int16_t position;
    int16_t speed;
    uint8_t acc;
} servoCmdPacket_t;

static uart_bus_t *pUart = &uart2;
static servoCmdPacket_t packet[SERVO_NUM];      // [0] 为左舵机，[1] 为右舵机
static servoCmdPacket_t lastState[SERVO_NUM];       // 记录上一次的舵机目标参数
servoStatus_t sts3032::status[SERVO_NUM];
static uint8_t txBuf[128], txLen = 0;       // 发送缓冲区

/**
 * @brief 向舵机发送缓冲区追加数据
 *
 * @param data 数据缓冲区
 * @param len 数据长度
 *
 * @return 实际处理的数据长度
 */
static uint32_t write(uint8_t *data, uint32_t len)
{
    while(len--)
    {
        if(txLen < sizeof(txBuf))
        {
            txBuf[txLen++] = *data;
            data++;
        }
    }

    return txLen;
}

/**
 * @brief 发送并清空舵机发送缓冲区
 */
static void wFlush(void)
{
    ets_delay_us(15);
    if(txLen)
    {
        pUart->write_bytes(pUart, txBuf, txLen);
        txLen = 0;
    }
}

/**
 * @brief 拆分 int16 数据的低字节和高字节
 *
 * @param dataL 低字节输出地址
 * @param dataH 高字节输出地址
 * @param data 数据缓冲区
 */
static void swapByte(uint8_t *dataL, uint8_t *dataH, int16_t data)
{
    *dataH = data >> 8;
    *dataL = data & 0xFF;
}

/**
 * @brief 发送 STS 舵机同步读命令
 *
 * @param ids 舵机 ID 列表
 * @param servoNum 舵机数量
 * @param startReg 起始寄存器地址
 * @param len 数据长度
 */
static void syncRead(uint8_t *ids, uint8_t servoNum, uint8_t startReg, uint8_t len)
{
    uint8_t cmd[10];
    uint8_t idx = 0;

    cmd[idx++] = 0xFF;
    cmd[idx++] = 0xFF;
    cmd[idx++] = 0xFE;       // 广播 id
    cmd[idx++] = servoNum + 4;      // 长度
    cmd[idx++] = INST_SYNC_READ;
    cmd[idx++] = startReg;
    cmd[idx++] = len;       // 读取长度

    // 添加舵机 id
    for(uint8_t i = 0; i < servoNum; i++)
    {
        cmd[idx++] = ids[i];
    }

    // 校验和
    uint8_t sum = 0;
    for(uint8_t i = 2; i < idx; i++){sum += cmd[i];}
    cmd[idx++] = (~sum) & 0xFF;

    pUart->write_bytes(pUart, cmd, idx);
}

/**
 * @brief 发送 STS 舵机同步写命令
 *
 * @param idNum 舵机数量
 * @param memAddr 内存表起始地址
 * @param nData 每个舵机的数据缓冲区
 * @param nLen 每个舵机的数据长度
 */
static void syncWrite(uint8_t idNum, uint8_t memAddr, uint8_t *nData, uint8_t nLen)
{
    uint8_t mesLen = (nLen + 1) * idNum + 4;

    uint8_t cmd[7], index = 0;

    cmd[index++] = 0xFF;
    cmd[index++] = 0xFF;
    cmd[index++] = 0xFE;     // 广播

    cmd[index++] = mesLen;
    cmd[index++] = INST_SYNC_WRITE;
    cmd[index++] = memAddr;
    cmd[index++] = nLen;

    write(cmd, index);

    uint8_t sum = 0xFE + mesLen + INST_SYNC_WRITE + memAddr + nLen;
    for(uint8_t i = 0; i < idNum; i++)
    {
        write(&packet[i].id, 1);
        write(&nData[i * nLen], nLen);

        sum += packet[i].id;
        for(uint8_t j = 0; j < nLen; j++)
        {
            sum += nData[i * nLen + j];
        }
    }
    sum = ~sum;

    write(&sum, 1);
    wFlush();
}

/**
 * @brief 构建舵机位置数据并执行同步写
 *
 * @param idNum 舵机数量
 */
static void syncWritePosEx(uint8_t idNum)
{
    uint8_t cmd[idNum * 7];

    servoCmdPacket_t *p;
    for(uint8_t i = 0; i < idNum; i++)
    {
        p = &packet[i];

        cmd[i * 7 + 0] = p->acc;
        swapByte(&cmd[i * 7 + 1], &cmd[i * 7 + 2], p->position);
        swapByte(&cmd[i * 7 + 3], &cmd[i * 7 + 4], 0);
        swapByte(&cmd[i * 7 + 5], &cmd[i * 7 + 6], p->speed);
    }

    syncWrite(idNum, SMS_STS_ACC, cmd, 7);
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
static int32_t parseSyncRead(uint8_t *ids, uint8_t n, servoStatus_t *status)
{
    uint8_t buf[64];
    uint32_t len = pUart->read_bytes(pUart, buf, sizeof(buf));

    uint32_t offset = 0;
    for(uint8_t i = 0; i < n; i++)
    {
        if(offset + 7 > len) return -1; // 帧不完整

        if(buf[offset] != 0xFF || buf[offset+1] != 0xFF || buf[offset+2] != ids[i])
            return -2; // 帧头或ID不对

        uint8_t frameLen = buf[offset+3];  // LEN 字节
        uint8_t chk = 0;
        for(uint8_t j = 2; j < frameLen + 3; j++) chk += buf[offset + j];
        chk = ~chk;

        if(chk != buf[offset + 3 + frameLen]) return -3; // 校验失败

        // 解析位置
        status[i].id = ids[i];
        status[i].position = (int16_t)((buf[offset+6]<<8)|buf[offset+5]);

        // 解析负载（带方向，直接在这里计算正负）
        int16_t rawLoad = (int16_t)((buf[offset+10]<<8)|buf[offset+9]);
        int16_t duty = rawLoad & 0x03FF;        // BIT0~BIT9 占空比
        uint8_t dir = (rawLoad >> 10) & 0x01;   // BIT10 方向
        status[i].load = dir ? -duty : duty;    // 方向位1反转，用负值表示

        offset += frameLen + 4; // 移动到下一帧
    }

    return 0;
}

/**
 * @brief 读取左右腿舵机的位置和负载
 */
void sts3032::get_position_and_load()
{
    uint8_t ids[] = {SERVO_LEFT, SERVO_RIGHT};

    syncRead(ids, SERVO_NUM, 0x38, 6);
    parseSyncRead(ids, 2, status);
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

    write(cmd, sizeof(cmd));
    wFlush();
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
    servoCmdPacket_t *p;

    if(id == SERVO_LEFT){p = &packet[0];}
    else if(id == SERVO_RIGHT){p = &packet[1];}

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
        if(memcmp(&lastState[i], &packet[i], sizeof(servoCmdPacket_t)))
        {
            syncWritePosEx(SERVO_NUM);
            memcpy(lastState, packet, SERVO_NUM * sizeof(servoCmdPacket_t));
            break;
        }
    }
}

/**
 * @brief 初始化 STS3032 舵机总线和扭矩状态
 */
void sts3032::init()
{
    uart_bus_init(pUart);

    // 中位校准前先关闭扭矩输出
    set_torque_switch(SERVO_LEFT, 0);
    delay(100);
    set_torque_switch(SERVO_RIGHT, 0);
    delay(3000);

    set_torque_switch(SERVO_LEFT, 128);
    set_torque_switch(SERVO_RIGHT, 128);
}
