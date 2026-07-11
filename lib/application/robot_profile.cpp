#include "robot_profile.h"

#include "battery.h"
#include "actions.h"
#include "actions/action_jump.h"
#include "actions/action_kick.h"
#include "actions/action_sit.h"
#include "balance_core.h"
#include "balance_motion_adapter.h"
#include "controller.h"
#include "esp_http_server.h"
#include "host_comm.h"
#include "input_providers.h"
#include "input_router.h"
#include "led_dev.h"
#include "robot_actuator.h"
#include "rgb_dev.h"
#include "xbox_dev.h"

static const application::task_descriptor TASKS[] =
{
    {battery::task_entry, "battery_task", 2048, 2, 0},
    {led_dev::task_entry, "led_dev_task", 1024, 2, 0},
    {rgb_dev::task_entry, "rgb_dev_task", 2048, 2, 0},
    {xbox_dev::task_entry, "xbox_dev_task", 4096, 3, 0},
    {balance_core::core_task_entry, "balance_io_task", 4096, 5, 1},
    {balance_core::control_task_entry, "balance_ctl_task", 4096, 5, 0},
    {host_comm::task_entry, "host_comm_task", 4096, 3, 0},
    {esp_http_server::task_entry, "http_server_task", 4096, 2, 0}
};

static controller::input_provider *INPUT_PROVIDERS[] =
{
    &application::xbox_input(),
    &application::web_input(),
    &application::host_input()
};

static controller::actions::action *ACTIONS[] =
{
    &controller::actions::boot_action(),
    &controller::actions::balance_action(),
    &controller::actions::sit_action(),
    &controller::actions::middle_calibration_action(),
    &controller::actions::jump_action(),
    &controller::actions::kick_place_action(),
    &controller::actions::kick_run_action(),
    &controller::actions::stop_action()
};

/**
 * @brief 获取当前机器人配置启用的任务列表
 *
 * @param count 任务数量输出
 *
 * @return 任务描述列表
 */
const application::task_descriptor *application::task_list(uint8_t &count)
{
    count = sizeof(TASKS) / sizeof(TASKS[0]);
    return TASKS;
}

/**
 * @brief 按当前机器人配置装配并初始化模块
 */
void application::init()
{
    battery::init();
    led_dev::init();
    rgb_dev::init();
    xbox_dev::init();
    esp_http_server::init();

    controller::input_router::configure(
        INPUT_PROVIDERS,
        sizeof(INPUT_PROVIDERS) / sizeof(INPUT_PROVIDERS[0]));
    controller::actions::configure(
        ACTIONS,
        sizeof(ACTIONS) / sizeof(ACTIONS[0]));
    controller::configure(
        application::robot_actuator(),
        application::balance_motion());
    controller::init();
}
