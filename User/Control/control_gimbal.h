#ifndef CONTROL_GIMBAL_H
#define CONTROL_GIMBAL_H
#include <stdint.h>
/* App只传意图，不把DR16协议结构带进云台。 */
struct Gimbal_Command
{
    float yaw_rate_rad_s;// 云台偏航角速度，单位：弧度/秒
    uint8_t enabled;
};
struct Gimbal_Config
{
    float kp, kd;
    float rate_max_rad_s, travel_rad;// 云台偏航角速度最大值，单位：弧度/秒；云台偏航角行程，单位：弧度
    uint32_t feedback_timeout_ms, enable_timeout_ms;
};
class Class_Motor_DM_Normal;
void Gimbal_Init(Class_Motor_DM_Normal &yaw, const Gimbal_Config &config);
void Gimbal_Update(const Gimbal_Command &command, uint32_t now_ms, float dt_s);
#endif
