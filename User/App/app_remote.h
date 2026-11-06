#ifndef APP_REMOTE_H
#define APP_REMOTE_H
#include <stdint.h>

#ifdef __cplusplus
class Class_DR16;
void App_Remote_Init(Class_DR16 &remote);
extern "C" {
#endif

//这个结构体是为了让我在keil里面观察变量诞生的
typedef struct
{
    uint16_t ch0_raw, ch1_raw, ch2_raw, ch3_raw;
    int16_t ch0_centered, ch1_centered;
    float ch0_normalized, ch1_normalized;
    uint8_t s1, s2, online, has_valid_frame;
    uint32_t age_ms, valid_frames, invalid_lengths, invalid_frames, timeout_count;
    uint32_t last_valid_rx_ms;
} App_Remote_Debug;

extern volatile App_Remote_Debug g_remote_debug;
void App_Remote_Update(void);
#ifdef __cplusplus
}
#endif
#endif
