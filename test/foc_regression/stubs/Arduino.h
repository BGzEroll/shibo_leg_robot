#ifndef TEST_ARDUINO_H
#define TEST_ARDUINO_H

#include <algorithm>
#include <cmath>
#include <cstdint>

using std::abs;
using std::min;
using std::max;
using byte = uint8_t;

constexpr int OUTPUT = 1;
constexpr int INPUT = 0;
constexpr int INPUT_PULLUP = 2;
constexpr int HIGH = 1;
constexpr int LOW = 0;
extern uint32_t test_time_us;
extern uint32_t test_pwm_writes;
extern bool test_enabled;
inline unsigned long micros() { return test_time_us; }
inline void delay(unsigned long ms) { test_time_us += ms * 1000; }
inline void delayMicroseconds(unsigned int us) { test_time_us += us; }
inline int digitalRead(int) { return HIGH; }
inline void pinMode(int, int) {}
inline void digitalWrite(int, int value) { test_enabled = value != 0; }
class __FlashStringHelper;
class Print
{
    public:
        template<typename... args> void print(args...) {}
        template<typename... args> void println(args...) {}
};
inline Print Serial;
#define F(value) value

#endif
