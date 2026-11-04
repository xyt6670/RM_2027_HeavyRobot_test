#include "control_chassis.h"

void Chassis_Init(void)
{
    /* TODO: 初始化底盘控制状态；电机通信与协议由BSP/Device负责。 */
}

void Chassis_Update(float dt_s)
{
    /* TODO: 命令与反馈 -> 坐标变换 -> 麦轮解算 -> 四轮控制目标。 */
    (void)dt_s;
}
