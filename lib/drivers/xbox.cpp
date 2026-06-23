#include "xbox.h"

/**
 * @brief 构造函数
 * 
 * @param mac Xbox 控制器的 MAC 地址
 */
xbox::xbox(const char *mac)
    : core(mac)
{
    buttons = 0;
    memset(axes, 0, sizeof(axes));

    was_connected = 0;
    vibration_state = vibration_state_t::off;
    vibration_duration = 0;
    vibration_start_time = 0;
}

/**
 * @brief 初始化 Xbox 控制器
 */
void xbox::init()
{
    core.begin();
}

/**
 * @brief 更新 Xbox 控制器数据
 */
void xbox::update()
{
    process_notification();
    parser_xbox_data();
    update_vibration();
}

/**
 * @brief 设置按键振动
 * 
 * @param power 振动功率
 * @param duration 振动时间，单位毫秒
 */
void xbox::set_key_vibration(uint8_t power, uint32_t duration)
{
    power = constrain(power, 0, 100);
    duration = constrain(duration, 50, 2000);

    vibration_duration = duration;

    if(vibration_state != vibration_state_t::off)
    {
        reporter.v.power.center = 0;
        reporter.v.power.shake = 0;
        core.writeHIDReport(reporter);
    }

    reporter.setAllOff();
    reporter.v.select.center = 1;
    reporter.v.select.left = 1;
    reporter.v.select.right = 1;
    reporter.v.select.shake = 1;
    reporter.v.power.center = power;
    reporter.v.power.shake = power;
    reporter.v.timeActive = 10;
    core.writeHIDReport(reporter);

    vibration_state = vibration_state_t::start;
    vibration_start_time = millis();
}

/**
 * @brief 设置触发振动
 * 
 * @param trigger 要设置的扳机，0 表示 RT，1 表示 LT
 * @param duration 振动时间，单位毫秒
 */
void xbox::set_trigger_vibration(uint8_t trigger, uint32_t duration)
{
    duration = constrain(duration, 50, 2000);

    uint16_t trigger_max = XboxControllerNotificationParser::maxTrig;
    uint8_t power;

    if(trigger)
    {
        power = (uint8_t)((float)core.xboxNotif.trigRT / trigger_max * 100.0f * 0.5f);
    }
    else
    {
        power = (uint8_t)((float)core.xboxNotif.trigLT / trigger_max * 100.0f * 0.5f);
    }

    vibration_duration = duration;

    if(!power)
    {
        reporter.v.power.center = 0;
        reporter.v.power.shake = 0;
        core.writeHIDReport(reporter);
        vibration_state = vibration_state_t::off;
        return;
    }

    reporter.setAllOff();
    reporter.v.select.center = 1;
    reporter.v.select.left = 1;
    reporter.v.select.right = 1;
    reporter.v.select.shake = 1;
    reporter.v.power.center = power;
    reporter.v.power.shake = power;
    reporter.v.timeActive = 10;
    core.writeHIDReport(reporter);

    vibration_state = vibration_state_t::trigger;
    vibration_start_time = millis();
}

/**
 * @brief 处理 Xbox 控制器通知
 */
void xbox::process_notification()
{
    core.onLoop();

    if(core.isConnected() && !core.isWaitingForFirstNotification())
    {
        if(!was_connected)
        {
            was_connected = 1;
            set_key_vibration(50, 1000);
        }
    }
    else
    {
        if(was_connected)
        {
            was_connected = 0;
            set_key_vibration(50, 200);
        }
    }
}

/**
 * @brief 更新 Xbox 控制器振动
 */
void xbox::update_vibration()
{
    if(vibration_state != vibration_state_t::off)
    {
        if(millis() - vibration_start_time >= vibration_duration)
        {
            reporter.v.power.center = 0;
            reporter.v.power.shake = 0;
            core.writeHIDReport(reporter);
            vibration_state = vibration_state_t::off;
        }
    }
}

/**
 * @brief 解析 Xbox 控制器数据
 */
void xbox::parser_xbox_data()
{
    // btn 先清零
    buttons = 0;
    // memset(axes, 0, sizeof(axes));

    if(!core.isConnected())
    {
        memset(axes, 0, sizeof(axes));
        return;
    }

    if(core.xboxNotif.btnA){buttons |= BTN_A;}
    if(core.xboxNotif.btnB){buttons |= BTN_B;}
    if(core.xboxNotif.btnX){buttons |= BTN_X;}
    if(core.xboxNotif.btnY){buttons |= BTN_Y;}

    if(core.xboxNotif.btnLB){buttons |= BTN_LB;}
    if(core.xboxNotif.btnRB){buttons |= BTN_RB;}
    if(core.xboxNotif.btnLS){buttons |= BTN_LS;}
    if(core.xboxNotif.btnRS){buttons |= BTN_RS;}

    if(core.xboxNotif.btnShare){buttons |= BTN_SHARE;}
    if(core.xboxNotif.btnStart){buttons |= BTN_START;}
    if(core.xboxNotif.btnSelect){buttons |= BTN_SELECT;}
    if(core.xboxNotif.btnXbox){buttons |= BTN_XBOX;}

    if(core.xboxNotif.btnDirUp){buttons |= BTN_UP;}
    if(core.xboxNotif.btnDirLeft){buttons |= BTN_LEFT;}
    if(core.xboxNotif.btnDirRight){buttons |= BTN_RIGHT;}
    if(core.xboxNotif.btnDirDown){buttons |= BTN_DOWN;}

    // 摇杆归一化到 [-1.0, 1.0]，扳机归一化到 [0.0, 1.0]
    axes[0] = (float)((int32_t)core.xboxNotif.joyLHori - 32768) / 32768.0f;
    axes[1] = -(float)((int32_t)core.xboxNotif.joyLVert - 32768) / 32768.0f;
    axes[2] = (float)((int32_t)core.xboxNotif.joyRHori - 32768) / 32768.0f;
    axes[3] = -(float)((int32_t)core.xboxNotif.joyRVert - 32768) / 32768.0f;
    axes[4] = (float)core.xboxNotif.trigLT / 1023.0f;
    axes[5] = (float)core.xboxNotif.trigRT / 1023.0f;
}
