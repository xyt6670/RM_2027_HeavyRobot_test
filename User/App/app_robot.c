#include "app_robot.h"
#include "app_remote.h"
#include "control_chassis.h"
#include "control_gimbal.h"
#include "control_shoot.h"
#include "main.h"

/* 初始练习周期沿用旧英雄10ms；后续按实测执行耗时调整。 */
#define APP_CONTROL_PERIOD_MS 10U

static uint32_t last_update_ms;

void App_Robot_Init(void)
{
    /* TODO: 先初始化设备状态和对象，再注册CAN/UART回调并启动接收。 */
    App_Remote_Init();
    Chassis_Init();
    Gimbal_Init();
    Shoot_Init();
    last_update_ms = HAL_GetTick();
}

void App_Robot_Poll(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed_ms = (uint32_t)(now_ms - last_update_ms);
    float dt_s;

    if (elapsed_ms < APP_CONTROL_PERIOD_MS)
    {
        return;
    }
    last_update_ms = now_ms;
    dt_s = (float)elapsed_ms * 0.001f;

    App_Remote_Update();
    /* TODO: 检查输入/反馈有效性，生成运行许可与各子系统命令。 */
    Chassis_Update(dt_s);
    Gimbal_Update(dt_s);
    Shoot_Update(dt_s);
    /* TODO: 统一组帧输出并统计发送失败；当前控制函数均为空骨架。 */
}
