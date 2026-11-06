#ifndef DEVICE_DR16_H
#define DEVICE_DR16_H
#include <stdint.h>

#define DR16_FRAME_LENGTH 18U
#define DR16_RX_BUFFER_LENGTH 36U
#define DR16_CHANNEL_MIN 364U
#define DR16_CHANNEL_MID 1024U
#define DR16_CHANNEL_MAX 1684U
#define DR16_OFFLINE_TIMEOUT_MS 100U
#define DR16_AGE_UNKNOWN_MS ((uint32_t)0xffffffffU)

enum Enum_DR16_KeyBit
{
    DR16_KEY_W = 0, DR16_KEY_S, DR16_KEY_A, DR16_KEY_D,
    DR16_KEY_SHIFT, DR16_KEY_CTRL, DR16_KEY_Q, DR16_KEY_E,
    DR16_KEY_R, DR16_KEY_F, DR16_KEY_G, DR16_KEY_Z,
    DR16_KEY_X, DR16_KEY_C, DR16_KEY_V, DR16_KEY_B
};

/* 原始值与在线诊断分开；不再公开可写的RC_CtrlData全局对象。 */
struct RC_Ctl_t
{
    struct { uint16_t ch0, ch1, ch2, ch3, ch4; uint8_t s1, s2; } rc;
    struct { int16_t x, y, z; uint8_t press_l, press_r; } mouse;
    struct { uint16_t v; } key;
};
struct DR16_Diagnostics
{
    uint32_t rx_event_count, valid_frame_count;
    uint32_t invalid_length_count, invalid_frame_count;
    uint32_t last_valid_rx_ms, age_ms, timeout_count;
    uint16_t last_rx_length;
    uint8_t has_valid_frame, online;
};
struct Struct_DR16_State
{
    RC_Ctl_t data;
    DR16_Diagnostics status;
};

class Class_DR16
{
public:
    /* 上电、接收启动前调用一次；不隐式操作串口。 */
    void Init();
    /* app_init的串口回调转交本次字节，必须在缓存被复用前完成。 */
    void UART_RxCpltCallback(const uint8_t *data, uint16_t length, uint32_t now_ms);
    /* 主循环周期调用；内部保护状态后取毫秒时间，检查有效帧超时。 */
    void Update();
    /* 内部短临界区复制；调用者持有独立副本。 */
    void GetSnapshot(Struct_DR16_State &out) const;

private:
    static bool Decode(const uint8_t *data, RC_Ctl_t &out);
    Struct_DR16_State state_;
};
#endif
