#include "app_robot.h"
#include "app_remote.h"
#include "app_config.h"
#include "control_gimbal.h"

static uint32_t last_update_ms;
static Gimbal_Command gimbal_command;

void App_Robot_Init(uint32_t now_ms)
{
    last_update_ms = now_ms;
    gimbal_command.enabled = 0U;
    gimbal_command.yaw_rate_rad_s = 0.0f;
}

void App_Robot_Poll(uint32_t now_ms)
{
    uint32_t elapsed_ms = (uint32_t)(now_ms - last_update_ms);
    if (elapsed_ms < APP_ROBOT_PERIOD_MS) return;
    last_update_ms = now_ms;
    App_Remote_Update(gimbal_command);
    Gimbal_Update(gimbal_command, now_ms, (float)elapsed_ms * 0.001f);
}
