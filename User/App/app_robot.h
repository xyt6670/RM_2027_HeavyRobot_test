#ifndef APP_ROBOT_H
#define APP_ROBOT_H

#ifdef __cplusplus
extern "C" {
#endif

void App_Robot_Init(void);
/* 裸机主循环反复调用，不使用HAL_Delay或RTOS。 */
void App_Robot_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_ROBOT_H */
