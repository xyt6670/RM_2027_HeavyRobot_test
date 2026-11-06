#include "app_init.h"
#include "app_robot.h"
#include "app_remote.h"
#include "device_dr16.h"
#include "bsp_drv_uart.h"
#include "usart.h"

/* 对象只在此定义一次。App读取与UART回调操作的是同一个对象。 */
static Class_DR16 remote;

//保存的函数指针在UART_Init时传入，供BSP在DMA接收完成中断时调用
//上层注册由App_Init完成，避免中断访问尚未准备好的对象，并且上下层解耦
static void Remote_RxCallback(uint8_t *data, uint16_t length)
{
    remote.UART_RxCpltCallback(data, length, HAL_GetTick());
}

void App_Init(void)
{
    remote.Init();
    App_Remote_Init(remote);
    App_Robot_Init(HAL_GetTick());

    /* 最后启动接收，避免中断访问尚未准备好的对象。 */
    UART_Init(&huart3, Remote_RxCallback, DR16_RX_BUFFER_LENGTH);
    /* 本练习没有电机对象、CAN启动/发送或Control执行。 */
}
