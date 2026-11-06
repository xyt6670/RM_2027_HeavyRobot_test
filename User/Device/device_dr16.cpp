#include "device_dr16.h"
#include "main.h"
#include <string.h>

void Class_DR16::Init()
{
    memset(&state_, 0, sizeof(state_));
    state_.data.rc.ch0 = state_.data.rc.ch1 = DR16_CHANNEL_MID;
    state_.data.rc.ch2 = state_.data.rc.ch3 = DR16_CHANNEL_MID;
    state_.data.rc.ch4 = DR16_CHANNEL_MID;
    state_.status.age_ms = DR16_AGE_UNKNOWN_MS;
}

bool Class_DR16::Decode(const uint8_t *data, RC_Ctl_t &out)
{
    if (data == nullptr) return false;
    uint16_t ch[4];
    ch[0] = (uint16_t)((data[0] | ((uint16_t)data[1] << 8)) & 0x07ffU);
    ch[1] = (uint16_t)(((data[1] >> 3) | ((uint16_t)data[2] << 5)) & 0x07ffU);
    ch[2] = (uint16_t)(((data[2] >> 6) | ((uint16_t)data[3] << 2) | ((uint16_t)data[4] << 10)) & 0x07ffU);
    ch[3] = (uint16_t)(((data[4] >> 1) | ((uint16_t)data[5] << 7)) & 0x07ffU);
    uint8_t s1 = (uint8_t)((data[5] >> 6) & 0x03U);
    uint8_t s2 = (uint8_t)((data[5] >> 4) & 0x03U);
    for (uint8_t i = 0; i < 4U; ++i)
        if (ch[i] < DR16_CHANNEL_MIN || ch[i] > DR16_CHANNEL_MAX) return false;
    if (s1 < 1U || s1 > 3U || s2 < 1U || s2 > 3U || data[12] > 1U || data[13] > 1U)
        return false;

    out.rc.ch0 = ch[0]; out.rc.ch1 = ch[1];
    out.rc.ch2 = ch[2]; out.rc.ch3 = ch[3];
    out.rc.ch4 = (uint16_t)((data[16] | ((uint16_t)data[17] << 8)) & 0x07ffU);
    out.rc.s1 = s1; out.rc.s2 = s2;
    out.mouse.x = (int16_t)(data[6] | ((uint16_t)data[7] << 8));
    out.mouse.y = (int16_t)(data[8] | ((uint16_t)data[9] << 8));
    out.mouse.z = (int16_t)(data[10] | ((uint16_t)data[11] << 8));
    out.mouse.press_l = data[12]; out.mouse.press_r = data[13];
    out.key.v = (uint16_t)(data[14] | ((uint16_t)data[15] << 8));
    return true;
}

//接收中断回调；data指向DMA缓冲区，length是本次接收的字节数，now_ms是当前毫秒时间
/*先在device层的class里完成中断回调解析函数，dma传输完成先进入RE回调，
再调用device层规定的回调函数,我们把回调函数全都声明在class里，上层可以改注册，
这样想换新的解析方法只需修改class里的回调函数或者添加新的class即可，
目的是把上层注册和下层解析解耦*/
void Class_DR16::UART_RxCpltCallback(const uint8_t *data, uint16_t length, uint32_t now_ms)
{
    ++state_.status.rx_event_count;
    state_.status.last_rx_length = length;
    if (data == nullptr || (length != DR16_FRAME_LENGTH && length != DR16_RX_BUFFER_LENGTH))
    {
        ++state_.status.invalid_length_count;
        return;
    }
    for (uint16_t offset = 0; offset < length; offset += DR16_FRAME_LENGTH)
    {
        RC_Ctl_t decoded = {};
        if (!Decode(data + offset, decoded))
        {
            ++state_.status.invalid_frame_count;
            continue;
        }
        state_.data = decoded;
        ++state_.status.valid_frame_count;
        state_.status.last_valid_rx_ms = now_ms;
        state_.status.age_ms = 0U;
        state_.status.has_valid_frame = 1U;
        state_.status.online = 1U;
    }
}

void Class_DR16::Update()
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    /* 时间也在保护状态后取，避免ISR刚更新的新时间造成无符号下溢。 */
    uint32_t now_ms = HAL_GetTick();
    uint8_t online = 0U;
    if (state_.status.has_valid_frame)
    {
        state_.status.age_ms = (uint32_t)(now_ms - state_.status.last_valid_rx_ms);
        online = (uint8_t)(state_.status.age_ms < DR16_OFFLINE_TIMEOUT_MS);
    }
    if (state_.status.online && !online) ++state_.status.timeout_count;
    state_.status.online = online;
    __set_PRIMASK(key);
}

void Class_DR16::GetSnapshot(Struct_DR16_State &out) const
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    out = state_;
    __set_PRIMASK(key);
}
