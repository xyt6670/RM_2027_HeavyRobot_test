#include "app_remote.h"
#include "device_dr16.h"
#include <string.h>

volatile App_Remote_Debug g_remote_debug;
static Class_DR16 *remote_device = nullptr;
static const int32_t REMOTE_DEADZONE = 10;

static float MapChannel(int16_t centered)
{
    int32_t magnitude = centered < 0 ? -(int32_t)centered : (int32_t)centered;
    if (magnitude <= REMOTE_DEADZONE) return 0.0f;
    if (magnitude > 660) magnitude = 660;
    return (centered < 0 ? -1.0f : 1.0f) *
           (float)(magnitude - REMOTE_DEADZONE) / (float)(660 - REMOTE_DEADZONE);
}

void App_Remote_Init(Class_DR16 &remote)
{
    remote_device = &remote; // 同一个对象，不是复制一台遥控器。
    memset((void *)&g_remote_debug, 0, sizeof(g_remote_debug));
    App_Remote_Update();
}

void App_Remote_Update(void)
{
    if (remote_device == nullptr) return;
    Struct_DR16_State state;
    remote_device->Update();
    remote_device->GetSnapshot(state);//state是一个局部变量，GetSnapshot函数会把remote_device的状态复制到state中
    //快照存在的意义就是在调用者持有独立副本的情况下，避免在主循环中直接访问remote_device的状态，防止数据竞争和不一致性。
    g_remote_debug.ch0_raw = state.data.rc.ch0;
    g_remote_debug.ch1_raw = state.data.rc.ch1;
    g_remote_debug.ch2_raw = state.data.rc.ch2;
    g_remote_debug.ch3_raw = state.data.rc.ch3;
    g_remote_debug.ch0_centered = (int16_t)((int32_t)state.data.rc.ch0 - (int32_t)DR16_CHANNEL_MID);
    g_remote_debug.ch1_centered = (int16_t)((int32_t)state.data.rc.ch1 - (int32_t)DR16_CHANNEL_MID);
    g_remote_debug.s1 = state.data.rc.s1;
    g_remote_debug.s2 = state.data.rc.s2;
    g_remote_debug.online = state.status.online;
    g_remote_debug.has_valid_frame = state.status.has_valid_frame;
    g_remote_debug.age_ms = state.status.age_ms;
    g_remote_debug.last_valid_rx_ms = state.status.last_valid_rx_ms;
    g_remote_debug.valid_frames = state.status.valid_frame_count;
    g_remote_debug.invalid_lengths = state.status.invalid_length_count;
    g_remote_debug.invalid_frames = state.status.invalid_frame_count;
    g_remote_debug.timeout_count = state.status.timeout_count;
    /* 保留原始值便于诊断；离线映射必须为0，不能用历史摇杆值控制设备。 */
    g_remote_debug.ch0_normalized = state.status.online ? MapChannel(g_remote_debug.ch0_centered) : 0.0f;
    g_remote_debug.ch1_normalized = state.status.online ? MapChannel(g_remote_debug.ch1_centered) : 0.0f;
}
