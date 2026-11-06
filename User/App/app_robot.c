#include "app_robot.h"
#include "app_remote.h"

/* 本分支只观察遥控器；1ms刷新状态，不调用任何电机或Control。 */
static uint32_t last_update_ms;
void App_Robot_Init(uint32_t now_ms)
{
    last_update_ms = now_ms;
}
void App_Robot_Poll(uint32_t now_ms)
{
    if ((uint32_t)(now_ms - last_update_ms) < 1U) return;
    last_update_ms = now_ms;
    App_Remote_Update();
}
