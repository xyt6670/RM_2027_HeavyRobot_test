#ifndef REMOTE_TEST_HAL_H
#define REMOTE_TEST_HAL_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef enum { HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3 } HAL_StatusTypeDef;
typedef struct { uint8_t half_enabled; } DMA_HandleTypeDef;
typedef struct {
    void *Instance;
    DMA_HandleTypeDef *hdmarx;
    uint32_t RxState;
    uint32_t ErrorCode;
} UART_HandleTypeDef;
typedef uint32_t HAL_UART_RxEventTypeTypeDef;
#define USART1 ((void *)(uintptr_t)1)
#define USART2 ((void *)(uintptr_t)2)
#define USART3 ((void *)(uintptr_t)3)
#define UART4 ((void *)(uintptr_t)4)
#define UART5 ((void *)(uintptr_t)5)
#define USART6 ((void *)(uintptr_t)6)
#define HAL_UART_STATE_READY 0x20U
#define HAL_UART_STATE_BUSY_RX 0x22U
#define HAL_UART_RXEVENT_TC 0U
#define HAL_UART_RXEVENT_HT 1U
#define HAL_UART_RXEVENT_IDLE 2U
#define DMA_IT_HT 1U
#define __HAL_DMA_DISABLE_IT(dma, flag) ((dma)->half_enabled = 0U)
extern uint32_t fake_primask;
static inline uint32_t __get_PRIMASK(void) { return fake_primask; }
static inline void __disable_irq(void) { fake_primask = 1U; }
static inline void __set_PRIMASK(uint32_t value) { fake_primask = value; }
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *, uint16_t);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
#ifdef __cplusplus
}
#endif
#endif
