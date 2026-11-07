#ifndef APP_REMOTE_H
#define APP_REMOTE_H
#include "control_gimbal.h"
class Class_DR16;
void App_Remote_Init(Class_DR16 &remote);
/* 遥控输入转为普通命令；不访问电机，不启动通信。 */
void App_Remote_Update(Gimbal_Command &command);
#endif
