#ifndef TEST_WIRE_H
#define TEST_WIRE_H

#include "Arduino.h"
#include <cassert>

// 固定返回 AS5600 大端角度寄存器，记录请求以检查总线调用次序。
class TwoWire
{
    public:
        uint16_t raw = 0;
        uint8_t index = 0;
        uint32_t requests = 0;
        void begin() {}
        void beginTransmission(uint8_t address) { assert(address == 0x36); }
        void write(uint8_t reg) { assert(reg == 0x0C); }
        int endTransmission(bool stop) { assert(!stop); return 0; }
        void requestFrom(uint8_t address, uint8_t count)
        {
            assert(address == 0x36 && count == 2);
            index = 0;
            requests++;
        }
        int read() { return index++ == 0 ? raw >> 8 : raw & 0xFF; }
};

inline TwoWire Wire;

#endif
