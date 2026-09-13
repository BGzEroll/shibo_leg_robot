#include "input.h"

#include "action.h"
#include "esp_timer.h"
#include "hw/gamepad.h"
#include "io/host.h"
#include "io/web.h"
#include "string.h"

/* ---- 输入路由运行状态 ---- */

static constexpr uint32_t INPUT_TIMEOUT_US = 250000;
static constexpr uint8_t INPUT_BUTTON_COUNT = 16;
static constexpr uint8_t AXIS_YAW = 0;
static constexpr uint8_t AXIS_LINEAR = 3;

struct sampled_input
{
    control::input_source source = control::input_source::NONE;
    uint32_t timestamp_us = 0;
    uint16_t buttons = 0;
    uint16_t pressed_buttons = 0;
    float axes[6]{};
    bool fresh = false;
};

struct source_tracker
{
    uint32_t stream_id = 0;
    uint32_t sequence = 0;
    uint16_t press_count[INPUT_BUTTON_COUNT]{};
    bool initialized = false;
};

static control::input_source last_source = control::input_source::NONE;
static bool last_fresh = false;
static source_tracker trackers[4];

/* ---- 输入采样与归一化 ---- */

/**
 * @brief 对输入轴应用死区并重新归一化
 *
 * @param value 原始输入值
 * @param deadband 死区比例
 *
 * @return 归一化后的输入值
 */
static float apply_deadband(float value, float deadband)
{
    if(fabsf(value) <= deadband){return 0.0f;}
    float magnitude = (fabsf(value) - deadband) / (1.0f - deadband);
    return value > 0.0f ? magnitude : -magnitude;
}

/**
 * @brief 判断输入时间戳是否仍在有效期内
 *
 * @param timestamp_us 输入时间戳
 * @param now_us 当前时间戳
 *
 * @return 输入仍然新鲜时返回 true
 */
static bool input_fresh(uint32_t timestamp_us, uint32_t now_us)
{
    return timestamp_us != 0 &&
           (uint32_t)(now_us - timestamp_us) <= INPUT_TIMEOUT_US;
}

/**
 * @brief 获取输入源跟踪器
 *
 * @param source 输入源
 *
 * @return 对应输入源跟踪器
 */
static source_tracker &tracker_for_source(control::input_source source)
{
    return trackers[(uint8_t)source];
}

/**
 * @brief 更新指定输入源的按键累计计数跟踪状态
 *
 * @param data 输入快照
 * @param tracker 输入源消费跟踪状态
 * @param ordered 快照顺序是否有效输出
 *
 * @return 本次检测到的按下按钮位
 */
static uint16_t update_press_tracker(const control::remote_input &data,
    source_tracker &tracker, bool &ordered)
{
    bool stream_changed = !tracker.initialized || tracker.stream_id != data.stream_id;
    ordered = stream_changed || !data.valid ||
        (int32_t)(data.sequence - tracker.sequence) >= 0;
    if(!ordered){return 0;}

    if(stream_changed)
    {
        tracker = source_tracker{};
        tracker.stream_id = data.stream_id;
        tracker.initialized = true;
    }

    uint16_t pressed_buttons = 0;
    for(uint8_t i = 0; i < INPUT_BUTTON_COUNT; i++)
    {
        if(data.press_count[i] != tracker.press_count[i])
        {
            pressed_buttons |= (uint16_t)(1U << i);
            tracker.press_count[i] = data.press_count[i];
        }
    }
    tracker.sequence = data.sequence;
    return pressed_buttons;
}

/**
 * @brief 将一个统一遥控快照转换为路由采样结果
 *
 * @param source 输入源
 * @param data 输入快照
 * @param available 是否读到快照
 * @param now_us 当前时间戳
 *
 * @return 路由采样结果
 */
static sampled_input sample_source(control::input_source source,
    const control::remote_input &data, bool available, uint32_t now_us)
{
    sampled_input sample;
    sample.source = source;
    if(!available){return sample;}

    bool ordered = false;
    sample.pressed_buttons = update_press_tracker(
        data, tracker_for_source(source), ordered);
    if(!ordered){return sample;}

    sample.timestamp_us = data.timestamp_us;
    sample.fresh = data.valid && input_fresh(data.timestamp_us, now_us);
    if(!sample.fresh)
    {
        sample.pressed_buttons = 0;
        return sample;
    }

    sample.buttons = data.buttons;
    memcpy(sample.axes, data.axes, sizeof(sample.axes));
    return sample;
}

/**
 * @brief 读取所有输入源并选择当前优先级最高的快照
 *
 * @return 当前优先输入源的采样结果
 */
static sampled_input read_snapshot()
{
    uint32_t now_us = (uint32_t)esp_timer_get_time();
    control::remote_input xbox_data{};
    control::remote_input web_data{};
    control::remote_input host_data{};
    bool xbox_available = hw::gamepad::latest_input(xbox_data);
    bool web_available = io::web::latest_input(web_data);
    bool host_available = io::host::latest_input(host_data);

    sampled_input xbox_sample = sample_source(
        control::input_source::XBOX, xbox_data, xbox_available, now_us);
    sampled_input web_sample = sample_source(
        control::input_source::WEB, web_data, web_available, now_us);
    sampled_input host_sample = sample_source(
        control::input_source::HOST, host_data, host_available, now_us);

    if(hw::gamepad::connected()){return xbox_sample;}
    if(web_sample.fresh){return web_sample;}
    return host_sample;
}

/**
 * @brief 在输入源切换或恢复新鲜度时抑制历史按键边沿
 *
 * @param sample 当前选中的输入采样
 */
static void suppress_replayed_edges(sampled_input &sample)
{
    if(sample.source != last_source || !last_fresh || !sample.fresh)
    {
        sample.pressed_buttons = 0;
    }
    last_source = sample.source;
    last_fresh = sample.fresh;
}

/**
 * @brief 按当前动作模式生成唯一离散动作请求
 *
 * @param mode 当前动作模式
 * @param held_buttons 当前按住按钮
 * @param pressed_buttons 本周期新按下按钮
 * @param max_linear_vel 最大线速度
 * @param out 路由后的控制输入
 */
static void route_action_request(control::action::mode mode, uint16_t held_buttons,
    uint16_t pressed_buttons, float max_linear_vel, control::control_input &out)
{
    if(mode != control::action::mode::STOP &&
       (pressed_buttons & control::buttons::START))
    {
        out.action = control::action_request::STOP;
        return;
    }

    bool modifier = (held_buttons & control::buttons::SELECT) != 0;
    switch(mode)
    {
        case control::action::mode::BOOT:
            if(held_buttons & control::buttons::RB)
            {
                out.action = control::action_request::BOOT_CONFIRM;
            }
            break;

        case control::action::mode::STOP:
            if(held_buttons & control::buttons::RB)
            {
                out.action = control::action_request::BOOT;
            }
            break;

        case control::action::mode::BALANCE:
            out.reset_leg = (pressed_buttons & control::buttons::LS) &&
                fabsf(out.linear) < max_linear_vel * 0.05f;
            if(modifier)
            {
                if(pressed_buttons & control::buttons::X)
                {
                    out.action = control::action_request::KICK_PLACE;
                }
                else if(pressed_buttons & control::buttons::Y)
                {
                    out.action = control::action_request::KICK_RUN;
                }
                else if(pressed_buttons & control::buttons::B)
                {
                    out.action = control::action_request::RESET_BALANCE;
                }
                break;
            }
            if(pressed_buttons & control::buttons::B)
            {
                out.action = control::action_request::JUMP_RIGHT;
            }
            else if(pressed_buttons & control::buttons::X)
            {
                out.action = control::action_request::JUMP_LEFT;
            }
            else if(pressed_buttons & control::buttons::A)
            {
                out.action = control::action_request::JUMP_BACKWARD;
            }
            else if(pressed_buttons & control::buttons::Y)
            {
                out.action = control::action_request::JUMP_FORWARD;
            }
            else if(pressed_buttons & control::buttons::RS)
            {
                out.action = control::action_request::JUMP_IN_PLACE;
            }
            else if(pressed_buttons & control::buttons::LB)
            {
                out.action = control::action_request::SIT;
            }
            break;

        case control::action::mode::SIT:
            out.disable_leg_torque = (held_buttons & control::buttons::LS) != 0;
            if(held_buttons & control::buttons::RB)
            {
                out.action = control::action_request::EXIT;
            }
            break;

        case control::action::mode::MIDDLE_CALIBRATION:
            if(held_buttons & control::buttons::RB)
            {
                out.action = control::action_request::EXIT;
            }
            break;

        case control::action::mode::KICK_PLACE:
            if(modifier && (pressed_buttons & control::buttons::B))
            {
                out.action = control::action_request::KICK_EXIT;
            }
            else if(modifier && (pressed_buttons & control::buttons::Y))
            {
                out.action = control::action_request::KICK_RUN;
            }
            break;

        case control::action::mode::KICK_RUN:
            if(modifier && (pressed_buttons & control::buttons::B))
            {
                out.action = control::action_request::KICK_EXIT;
            }
            else if(modifier && (pressed_buttons & control::buttons::X))
            {
                out.action = control::action_request::KICK_PLACE;
            }
            break;

        case control::action::mode::JUMP:
        default:
            break;
    }
}

/* ---- input_router 公共 API ---- */

/**
 * @brief 更新统一控制输入和语义动作请求
 *
 * @param external_action 来自异步维护接口的动作请求
 * @param mode 当前动作模式
 * @param max_linear_vel 最大线速度
 * @param max_steer_vel 最大转向角速度
 * @param out 控制输入输出
 */
void control::input_router::update(control::action_request external_action,
    control::action::mode mode, float max_linear_vel, float max_steer_vel,
    control::control_input &out)
{
    out = control::control_input{};
    sampled_input snapshot = read_snapshot();
    suppress_replayed_edges(snapshot);

    out.source = snapshot.source;
    out.timestamp_us = snapshot.timestamp_us;
    out.fresh = snapshot.fresh;

    uint16_t raw_buttons = snapshot.buttons;
    if(raw_buttons & control::buttons::SELECT)
    {
        bool up = (raw_buttons & control::buttons::UP) != 0;
        bool down = (raw_buttons & control::buttons::DOWN) != 0;
        if(up && !down){out.camera_direction = 1;}
        if(down && !up){out.camera_direction = -1;}
    }

    uint16_t held_buttons = raw_buttons;
    if(held_buttons & control::buttons::SELECT)
    {
        held_buttons &= (uint16_t)~(
            control::buttons::UP | control::buttons::DOWN |
            control::buttons::LEFT | control::buttons::RIGHT);
    }

    float linear_axis = apply_deadband(snapshot.axes[AXIS_LINEAR], 0.05f);
    float yaw_axis = apply_deadband(snapshot.axes[AXIS_YAW], 0.05f);
    out.linear = linear_axis * max_linear_vel;
    if(linear_axis < 0.0f){out.linear *= 0.8f;}
    out.yaw = -yaw_axis * max_steer_vel;

    if(held_buttons == control::buttons::RIGHT){out.roll_direction = 1;}
    if(held_buttons == control::buttons::LEFT){out.roll_direction = -1;}
    if(held_buttons == control::buttons::UP){out.leg_height_direction = -1;}
    if(held_buttons == control::buttons::DOWN){out.leg_height_direction = 1;}

    route_action_request(mode, held_buttons, snapshot.pressed_buttons,
        max_linear_vel, out);
    if(external_action != control::action_request::NONE &&
       out.action != control::action_request::STOP)
    {
        out.action = external_action;
    }
}

/**
 * @brief 初始化输入路由跟踪状态
 */
void control::input_router::init()
{
    last_source = control::input_source::NONE;
    last_fresh = false;
    for(source_tracker &tracker : trackers){tracker = source_tracker{};}
}
