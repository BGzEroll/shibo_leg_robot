#include "system_app.h"

#include "balance_core.h"
#include "actuator_adapter.h"
#include "battery.h"
#include "controller.h"
#include "esp_http_server.h"
#include "host_comm.h"
#include "led_dev.h"
#include "motor.h"
#include "mpu6050_dev.h"
#include "rgb_dev.h"
#include "sts3032.h"
#include "xbox_dev.h"
#include "wifi_dev.h"

/* ---- 系统静态端口 ---- */

static port::latest_value<battery::data> battery_status;
static port::latest_value<xbox_dev::data> xbox_control;
static port::latest_value<esp_http_server::remote_input_data> web_control;
static port::latest_value<host_comm::remote_data> host_control;
static port::latest_value<host_comm::vision_measurement> vision_measurement;
static port::latest_value<esp_http_server::calibration_request> calibration_request;
static port::latest_value<esp_http_server::calibration_status> calibration_status;
static port::latest_value<motor::encoder_data> motor_encoder;
static port::latest_value<motor::target_data> motor_target;
static port::latest_value<mpu6050_dev::data> imu_measurement;
static port::latest_value<balance_core::command> balance_command;
static port::latest_value<balance_core::motion_status> motion_status;
static port::latest_value<balance_core::debug_snapshot> debug_status;
static port::latest_value<actuator_port::leg_status> leg_status;

/**
 * @brief 初始化全部平级模块并连接静态端口
 */
void system_app::init()
{
    battery::init({battery_status.writer()});
    led_dev::init({battery_status.reader()});
    rgb_dev::init({battery_status.reader()});
    xbox_dev::init({xbox_control.writer()});
    wifi_dev::init();

    sts3032::init();
    actuator_adapter::init(leg_status.writer());
    mpu6050_dev::init({imu_measurement.writer()});
    motor::init(
        {motor_target.reader()},
        {motor_encoder.writer()});

    balance_core::init(
        {
            balance_command.reader(),
            motor_encoder.reader(),
            imu_measurement.reader(),
            leg_status.reader()
        },
        {
            motor_target.writer(),
            motion_status.writer(),
            debug_status.writer()
        });

    host_comm::init(
        {debug_status.reader()},
        {
            host_control.writer(),
            vision_measurement.writer()
        });

    esp_http_server::init(
        {calibration_status.reader()},
        {
            web_control.writer(),
            calibration_request.writer()
        });

    controller::init(
        {
            battery_status.reader(),
            motion_status.reader(),
            vision_measurement.reader(),
            leg_status.reader(),
            calibration_request.reader(),
            {
                xbox_control.reader(),
                web_control.reader(),
                host_control.reader()
            },
            actuator_adapter::services()
        },
        {
            balance_command.writer(),
            calibration_status.writer()
        });
}
