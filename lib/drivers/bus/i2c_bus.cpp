#include "i2c_bus.h"

/* ---- I2C 设备资源与内部实现 ---- */

static TwoWire wire_0 = TwoWire(0);
static TwoWire wire_1 = TwoWire(1);

static i2c_result map_end_transmission_result(uint8_t code);

class i2c_dev
{
    public:
        i2c_dev(uint8_t i2c_num, uint8_t scl_pin, uint8_t sda_pin, uint32_t freq)
            : i2c_num(i2c_num),
              scl_pin(scl_pin),
              sda_pin(sda_pin),
              freq(freq)
        {
            switch(i2c_num)
            {
                case 0: wire = &wire_0; break;
                case 1: wire = &wire_1; break;
                default: wire = nullptr; break;
            }
        }

        i2c_result init()
        {
            if(!wire){return i2c_result::INVALID_BUS;}
            if(is_init){return i2c_result::OK;}

            is_init = true;
            wire->begin(sda_pin, scl_pin, freq);
            return i2c_result::OK;
        }

        i2c_result read_bytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
        {
            if(!wire){return i2c_result::INVALID_BUS;}
            if(!is_init){return i2c_result::NOT_INITIALIZED;}
            if(!len){return i2c_result::OK;}
            if(!buf){return i2c_result::INVALID_ARG;}

            wire->beginTransmission(addr);
            if(wire->write(reg) != 1){return i2c_result::SHORT_WRITE;}

            i2c_result tx_result = map_end_transmission_result(wire->endTransmission(false));
            if(tx_result != i2c_result::OK){return tx_result;}

            size_t request_len = wire->requestFrom(addr, (size_t)len, true);
            if(request_len < len){return i2c_result::SHORT_READ;}

            size_t read_len = wire->readBytes(buf, (size_t)len);
            if(read_len < len){return i2c_result::SHORT_READ;}

            return i2c_result::OK;
        }

        i2c_result write_bytes(uint8_t addr, uint8_t reg, const uint8_t *buf, uint8_t len)
        {
            if(!wire){return i2c_result::INVALID_BUS;}
            if(!is_init){return i2c_result::NOT_INITIALIZED;}
            if(!len){return i2c_result::OK;}
            if(!buf){return i2c_result::INVALID_ARG;}

            wire->beginTransmission(addr);
            if(wire->write(reg) != 1){return i2c_result::SHORT_WRITE;}
            if(wire->write(buf, len) != len){return i2c_result::SHORT_WRITE;}

            return map_end_transmission_result(wire->endTransmission(true));
        }

        TwoWire *get_TwoWire_handle()
        {
            return wire;
        }

    private:
        uint8_t i2c_num;
        TwoWire *wire = nullptr;
        uint8_t scl_pin;
        uint8_t sda_pin;
        uint32_t freq;
        bool is_init = false;
};

// 静态 i2c 设备表（资源池）
static i2c_dev i2c_devs[] =
{
    i2c_dev(0, 18, 19, 400000),
    i2c_dev(1, 5, 23, 400000),
};
static constexpr uint8_t I2C_DEV_NUM = sizeof(i2c_devs) / sizeof(i2c_devs[0]);

/**
 * @brief 将 Arduino Wire 返回码转换为项目 I2C 结果
 *
 * @param code Wire.endTransmission() 返回码
 *
 * @return 对应的 I2C 操作结果
 */
static i2c_result map_end_transmission_result(uint8_t code)
{
    switch(code)
    {
        case 0: return i2c_result::OK;
        case 1: return i2c_result::TX_BUFFER_FULL;
        case 2: return i2c_result::ADDR_NACK;
        case 3: return i2c_result::DATA_NACK;
        case 5: return i2c_result::TIMEOUT;
        default: return i2c_result::OTHER_ERROR;
    }
}

/**
 * @brief 根据 bus_id 获取对应底层设备
 *
 * @note 超出范围时返回 nullptr，由上层返回 INVALID_BUS。
 */
static i2c_dev *get_dev(uint8_t bus_id)
{
    if(bus_id < I2C_DEV_NUM)
    {
        return &i2c_devs[bus_id];
    }
    return nullptr;
}

/* ---- i2c_bus 公共 API ---- */

/**
 * @brief i2c 总线构造函数
 *
 * @param bus_id i2c 总线编号
 */
i2c_bus::i2c_bus(uint8_t bus_id)
    : bus_id(bus_id)
{
}

/**
 * @brief 初始化 i2c 总线
 *
 * @return 初始化结果
 */
i2c_result i2c_bus::init()
{
    i2c_dev *dev = get_dev(bus_id);
    if(!dev){return i2c_result::INVALID_BUS;}

    return dev->init();
}

/**
 * @brief 连续读取寄存器数据
 *
 * @param addr 设备地址
 * @param reg  起始寄存器地址
 * @param buf  接收缓冲区
 * @param len  读取长度
 *
 * @return 读取结果
 */
i2c_result i2c_bus::read_bytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    i2c_dev *dev = get_dev(bus_id);
    if(!dev){return i2c_result::INVALID_BUS;}

    return dev->read_bytes(addr, reg, buf, len);
}

/**
 * @brief 连续写入寄存器数据
 *
 * @param addr 设备地址
 * @param reg  起始寄存器地址
 * @param buf  数据缓冲区
 * @param len  写入长度
 *
 * @return 写入结果
 */
i2c_result i2c_bus::write_bytes(uint8_t addr, uint8_t reg, const uint8_t *buf, uint8_t len)
{
    i2c_dev *dev = get_dev(bus_id);
    if(!dev){return i2c_result::INVALID_BUS;}

    return dev->write_bytes(addr, reg, buf, len);
}

/**
 * @brief 获取底层 TwoWire 句柄
 *
 * @return 底层 TwoWire 句柄，非法 bus_id 时返回 nullptr
 */
TwoWire *i2c_bus::get_TwoWire_handle() const
{
    i2c_dev *dev = get_dev(bus_id);
    if(!dev){return nullptr;}

    return dev->get_TwoWire_handle();
}
