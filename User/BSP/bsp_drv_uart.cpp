/**
 * UART BSP，源自USTC-RoboWalker (c) 2022。
 * 本分支补充C接口、HT屏蔽、空指针防护、状态统计与非阻塞重启。
 * 保留中科大按端口分发的接口，不引入设备注册表。
 */
#include "bsp_drv_uart.h"
#include <string.h>

Struct_UART_Manage_Object UART1_Manage_Object = {};
Struct_UART_Manage_Object UART2_Manage_Object = {};
Struct_UART_Manage_Object UART3_Manage_Object = {};
Struct_UART_Manage_Object UART4_Manage_Object = {};
Struct_UART_Manage_Object UART5_Manage_Object = {};
Struct_UART_Manage_Object UART6_Manage_Object = {};
Struct_UART_Manage_Object UART7_Manage_Object = {};
Struct_UART_Manage_Object UART8_Manage_Object = {};

static Struct_UART_Manage_Object *UART_Find(UART_HandleTypeDef *huart)
{
    if (huart == nullptr) return nullptr;
    if (huart->Instance == USART1) return &UART1_Manage_Object;
    if (huart->Instance == USART2) return &UART2_Manage_Object;
    if (huart->Instance == USART3) return &UART3_Manage_Object;
    if (huart->Instance == UART4) return &UART4_Manage_Object;
    if (huart->Instance == UART5) return &UART5_Manage_Object;
    if (huart->Instance == USART6) return &UART6_Manage_Object;
    return nullptr;
}

HAL_StatusTypeDef UART_Rx_EnsureRunning(UART_HandleTypeDef *huart)//确保UART接收DMA正在运行，若未运行则重启
{
    Struct_UART_Manage_Object *port = UART_Find(huart);
    if (port == nullptr || port->UART_Handler != huart || port->Callback_Function == nullptr ||
        port->Rx_Buffer_Length == 0U || port->Rx_Buffer_Length > UART_BUFFER_SIZE || huart->hdmarx == nullptr)
        return HAL_ERROR;
    if (huart->RxState == HAL_UART_STATE_BUSY_RX)
    {
        port->Last_Start_Status = HAL_OK;
        return HAL_OK;
    }
    if (huart->RxState != HAL_UART_STATE_READY) return HAL_BUSY;
    port->Last_Start_Status = HAL_UARTEx_ReceiveToIdle_DMA(huart, port->Rx_Buffer, port->Rx_Buffer_Length);
    if (port->Last_Start_Status == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        ++port->Restart_Count;
    }
    return port->Last_Start_Status;
}

void UART_Init(UART_HandleTypeDef *huart, UART_Call_Back callback, uint16_t length)
{
    Struct_UART_Manage_Object *port = UART_Find(huart);//找到对应的UART管理对象
    if (port == nullptr) return;
    memset(port, 0, sizeof(*port));
    port->UART_Handler = huart;
    port->Callback_Function = callback;
    port->Rx_Buffer_Length = length;
    port->Last_Start_Status = HAL_ERROR;
    (void)UART_Rx_EnsureRunning(huart);
}

void UART_Poll(void)
{
    Struct_UART_Manage_Object *ports[] = {
        &UART1_Manage_Object, &UART2_Manage_Object, &UART3_Manage_Object,
        &UART4_Manage_Object, &UART5_Manage_Object, &UART6_Manage_Object
    };
    for (uint8_t i = 0U; i < sizeof(ports) / sizeof(ports[0]); ++i)
        if (ports[i]->UART_Handler != nullptr)
            (void)UART_Rx_EnsureRunning(ports[i]->UART_Handler);
}

uint8_t UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length)
{
    if (huart == nullptr || data == nullptr || length == 0U) return HAL_ERROR;
    return (uint8_t)HAL_UART_Transmit_DMA(huart, data, length);
}
uint8_t TIM_UART_PeriodElapsedCallback(void)
{
    return UART_Send_Data(&huart1, UART1_Manage_Object.Tx_Buffer, 1U + 4U * sizeof(float));
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)//接收完成回调函数，size为本次接收的字节数
{
    Struct_UART_Manage_Object *port = UART_Find(huart);
    if (port == nullptr || port->UART_Handler != huart) return;
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT) return;
    ++port->Rx_Event_Count;
    port->Last_Rx_Size = size;
    if (port->Callback_Function != nullptr && size > 0U && size <= port->Rx_Buffer_Length)
        port->Callback_Function(port->Rx_Buffer, size);
    (void)UART_Rx_EnsureRunning(huart);
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Struct_UART_Manage_Object *port = UART_Find(huart);
    if (port == nullptr || port->UART_Handler != huart) return;
    ++port->Error_Count;
    port->Last_Error_Code = huart->ErrorCode;
    /* HAL结束/中止错误接收，主循环等RxState就绪后重启，不在ISR忙等。 */
}
