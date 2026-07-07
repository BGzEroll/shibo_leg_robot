#include "uart_bus.h"

extern HardwareSerial Serial;
extern HardwareSerial Serial1;
extern HardwareSerial Serial2;

class uart_dev
{
    public:
        uart_dev(uint8_t uart_num, uint32_t baudrate)
            : uart_num(uart_num),
              baudrate(baudrate)
        {
            switch(uart_num)
            {
                case 0: serial = &Serial; break;
                case 1: serial = &Serial1; break;
                case 2: serial = &Serial2; break;
                default: serial = nullptr; break;
            }
        }

        uart_result init()
        {
            if(!serial){return uart_result::INVALID_BUS;}
            if(is_init){return uart_result::OK;}

            is_init = true;
            serial->begin(baudrate);
            return uart_result::OK;
        }

        uart_result read_bytes(uint8_t *buf, uint32_t max_len, uint32_t &out_len)
        {
            out_len = 0;
            if(!serial){return uart_result::INVALID_BUS;}
            if(!is_init){return uart_result::NOT_INITIALIZED;}
            if(!max_len){return uart_result::OK;}
            if(!buf){return uart_result::INVALID_ARG;}

            while(serial->available() && out_len < max_len)
            {
                buf[out_len++] = serial->read();
            }

            return uart_result::OK;
        }

        uart_result write_bytes(const uint8_t *buf, uint32_t len)
        {
            if(!serial){return uart_result::INVALID_BUS;}
            if(!is_init){return uart_result::NOT_INITIALIZED;}
            if(!len){return uart_result::OK;}
            if(!buf){return uart_result::INVALID_ARG;}

            size_t write_len = serial->write(buf, len);
            serial->flush();
            if(write_len < len){return uart_result::SHORT_WRITE;}

            return uart_result::OK;
        }

        HardwareSerial *get_HardwareSerial_handle()
        {
            return serial;
        }

    private:
        uint8_t uart_num;
        uint32_t baudrate;
        HardwareSerial *serial = nullptr;
        bool is_init = false;
};

// 静态 uart 设备表（资源池）
static uart_dev uart_devs[] =
{
    uart_dev(0, 230400),
    uart_dev(1, 115200),
    uart_dev(2, 1000000)
};
static constexpr uint8_t UART_DEV_NUM = sizeof(uart_devs) / sizeof(uart_devs[0]);

/**
 * @brief 根据 bus_id 获取对应底层设备
 *
 * @note 超出范围时返回 nullptr，由上层返回 INVALID_BUS。
 */
static uart_dev *get_dev(uint8_t bus_id)
{
    if(bus_id < UART_DEV_NUM)
    {
        return &uart_devs[bus_id];
    }
    return nullptr;
}

/**
 * @brief uart_bus 构造函数
 *
 * @param bus_id UART 总线编号
 */
uart_bus::uart_bus(uint8_t bus_id)
    : bus_id(bus_id)
{
}

/**
 * @brief 初始化 uart 总线
 *
 * @return 初始化结果
 */
uart_result uart_bus::init()
{
    uart_dev *dev = get_dev(bus_id);
    if(!dev){return uart_result::INVALID_BUS;}

    return dev->init();
}

/**
 * @brief 连续读取字节
 *
 * @param buf 存放读取数据的缓冲区
 * @param max_len 缓冲区最大长度
 * @param out_len 实际读取的字节数
 *
 * @return 读取结果
 */
uart_result uart_bus::read_bytes(uint8_t *buf, uint32_t max_len, uint32_t &out_len)
{
    uart_dev *dev = get_dev(bus_id);
    if(!dev)
    {
        out_len = 0;
        return uart_result::INVALID_BUS;
    }

    return dev->read_bytes(buf, max_len, out_len);
}

/**
 * @brief 连续写入字节
 *
 * @param buf 待写入的数据缓冲区
 * @param len 写入数据的长度
 *
 * @return 写入结果
 */
uart_result uart_bus::write_bytes(const uint8_t *buf, uint32_t len)
{
    uart_dev *dev = get_dev(bus_id);
    if(!dev){return uart_result::INVALID_BUS;}

    return dev->write_bytes(buf, len);
}

/**
 * @brief 获取底层 HardwareSerial 句柄
 *
 * @return HardwareSerial* 返回底层硬件串口的句柄，供其他库使用
 */
HardwareSerial *uart_bus::get_HardwareSerial_handle() const
{
    uart_dev *dev = get_dev(bus_id);
    if(!dev){return nullptr;}

    return dev->get_HardwareSerial_handle();
}
