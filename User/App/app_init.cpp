#include "app_init.h"
#include "app_robot.h"
#include "app_remote.h"
#include "app_config.h"
#include "control_gimbal.h"
#include "device_dr16.h"
#include "device_motor_dm.h"
#include "bsp_drv_uart.h"
#include "bsp_drv_can.h"
#include "usart.h"
#include "can.h"

/* 对象只定义一次；接收回调和Control操作同一台设备。 */
static Class_DR16 remote;
static Class_Motor_DM_Normal yaw_motor;

static void Remote_RxCallback(uint8_t *data, uint16_t length)
{
    remote.UART_RxCpltCallback(data, length, HAL_GetTick());
}

static void Yaw_CAN_RxCallback(Struct_CAN_Rx_Buffer *frame)
{
    if (frame->Header.IDE == CAN_ID_STD && frame->Header.RTR == CAN_RTR_DATA &&
        frame->Header.StdId == APP_YAW_FEEDBACK_ID && frame->Header.DLC == 8U)
        yaw_motor.CAN_RxCpltCallback(frame->Data);
}

void App_Init(void)
{
    remote.Init();
    yaw_motor.Init(&APP_YAW_CAN_HANDLE, APP_YAW_FEEDBACK_ID, APP_YAW_CONTROL_ID,
                   Motor_DM_Control_Method_NORMAL_MIT,
                   APP_YAW_PMAX, APP_YAW_VMAX, APP_YAW_TMAX);
    App_Remote_Init(remote);
    const Gimbal_Config yaw_config = {
        APP_YAW_KP, APP_YAW_KD, APP_YAW_RATE_MAX_RAD_S, APP_YAW_TRAVEL_RAD,
        APP_YAW_FEEDBACK_TIMEOUT_MS, APP_YAW_ENABLE_TIMEOUT_MS
    };
    Gimbal_Init(yaw_motor, yaw_config);
    App_Robot_Init(HAL_GetTick());

    /* 先准备对象、依赖和回调，再开启可能立刻产生中断的接收。 */
    const uint16_t feedback_ids[] = { APP_YAW_FEEDBACK_ID };
    CAN_Init_With_Filters(&APP_YAW_CAN_HANDLE, Yaw_CAN_RxCallback, feedback_ids, 1U);
    UART_Init(&huart3, Remote_RxCallback, DR16_RX_BUFFER_LENGTH);
}
