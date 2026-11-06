#ifndef APP_ROBOT_H
#define APP_ROBOT_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void App_Robot_Init(uint32_t now_ms);
/* 裸机主循环反复调用，不使用HAL_Delay或RTOS。 */
void App_Robot_Poll(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* APP_ROBOT_H */
