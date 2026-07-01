#ifndef XBOX_H
#define XBOX_H

#include <Arduino.h>
#include "XboxSeriesXControllerESP32_asukiaaa.hpp"
#include <string.h>

#define BTN_A           0x0001
#define BTN_B           0x0002
#define BTN_X           0x0004
#define BTN_Y           0x0008
#define BTN_SHARE       0x0010
#define BTN_START       0x0020
#define BTN_SELECT      0x0040
#define BTN_XBOX        0x0080
#define BTN_LB          0x0100
#define BTN_RB          0x0200
#define BTN_LS          0x0400
#define BTN_RS          0x0800
#define BTN_UP          0x1000
#define BTN_LEFT        0x2000
#define BTN_RIGHT       0x4000
#define BTN_DOWN        0x8000

class xbox
{
    public:
        xbox(const char *mac);
        ~xbox();

    public:
        void init();
        void update();
        void set_key_vibration(uint8_t power, uint32_t duration);
        void set_trigger_vibration(uint8_t trigger, uint32_t duration);
        bool get_connection_state()
        {
            return was_connected;
        }

    public:
        uint16_t buttons;
        float axes[6];

    private:
        using xbox_core = XboxSeriesXControllerESP32_asukiaaa::Core;
        using xbox_reporter = XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase;

    private:
        xbox_core core;
        xbox_reporter reporter;

    private:
        void process_notification();
        void update_vibration();
        void parser_xbox_data();

    private:
        bool was_connected;

        enum class vibration_state
        {
            off,
            start,
            trigger
        };

        vibration_state vibration_state;
        uint32_t vibration_duration;
        uint32_t vibration_start_time;
};

#endif
