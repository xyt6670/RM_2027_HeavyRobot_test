#include "app_remote.h"
#include "app_config.h"
#include "device_dr16.h"

static Class_DR16 *remote_device = nullptr;
static bool armed = false;
static const int32_t REMOTE_DEADZONE = 10;

static float MapChannel(uint16_t raw)
{
    int32_t centered = (int32_t)raw - (int32_t)DR16_CHANNEL_MID;
    int32_t magnitude = centered < 0 ? -centered : centered;
    if (magnitude <= REMOTE_DEADZONE) return 0.0f;
    if (magnitude > 660) magnitude = 660;
    return (centered < 0 ? -1.0f : 1.0f) *
           (float)(magnitude - REMOTE_DEADZONE) / (float)(660 - REMOTE_DEADZONE);
}

void App_Remote_Init(Class_DR16 &remote)
{
    remote_device = &remote;
    armed = false;
}

void App_Remote_Update(Gimbal_Command &command)
{
    command.enabled = 0U;
    command.yaw_rate_rad_s = 0.0f;
    if (remote_device == nullptr) return;

    Struct_DR16_State state;
    remote_device->Update();
    remote_device->GetSnapshot(state);
    if (!state.status.online)
    {
        armed = false; // 断线恢复后必须重新回到安全挡。
        return;
    }

    float stick = MapChannel(state.data.rc.ch0);
    /* DR16协议：1=上，2=下，3=中。下挡且摇杆回中，才允许切上挡。 */
    if (state.data.rc.s1 == 2U && stick == 0.0f) armed = true;
    if (armed && state.data.rc.s1 == 1U)
    {
        command.enabled = 1U;
        command.yaw_rate_rad_s = APP_YAW_DIRECTION * stick * APP_YAW_RATE_MAX_RAD_S;// 云台偏航角速度，单位：弧度/秒
    }
}
