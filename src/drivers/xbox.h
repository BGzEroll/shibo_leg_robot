#ifndef XBOX_H
#define XBOX_H

#include <Arduino.h>
#include "XboxSeriesXControllerESP32_asukiaaa.hpp"

class xbox
{
    public:
        static constexpr uint16_t BUTTON_A = 0x0001;
        static constexpr uint16_t BUTTON_B = 0x0002;
        static constexpr uint16_t BUTTON_X = 0x0004;
        static constexpr uint16_t BUTTON_Y = 0x0008;
        static constexpr uint16_t BUTTON_SHARE = 0x0010;
        static constexpr uint16_t BUTTON_START = 0x0020;
        static constexpr uint16_t BUTTON_SELECT = 0x0040;
        static constexpr uint16_t BUTTON_XBOX = 0x0080;
        static constexpr uint16_t BUTTON_LB = 0x0100;
        static constexpr uint16_t BUTTON_RB = 0x0200;
        static constexpr uint16_t BUTTON_LS = 0x0400;
        static constexpr uint16_t BUTTON_RS = 0x0800;
        static constexpr uint16_t BUTTON_UP = 0x1000;
        static constexpr uint16_t BUTTON_LEFT = 0x2000;
        static constexpr uint16_t BUTTON_RIGHT = 0x4000;
        static constexpr uint16_t BUTTON_DOWN = 0x8000;

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
            OFF,
            START,
            TRIGGER
        };

        vibration_state vibration_state;
        uint32_t vibration_duration;
        uint32_t vibration_start_time;
};

#endif
