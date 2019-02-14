
#include "debug_uart.h"

#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>

#include "mini-printf/mini-printf.h"

#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_bus.h"

#define DEBUG_UART_BUFFER_SIZE 128

typedef struct DebugUart_context_s {
    uint8_t tx_buffer[DEBUG_UART_BUFFER_SIZE];
} DebugUart_context_t;

DebugUart_context_t *debug_uart_context;

static void DebugUart_blocking_tx(const uint8_t *data, uint16_t length)
{
    while (length)
    {
        while (!LL_USART_IsActiveFlag_TXE(USART3))
        {
        }

        LL_USART_TransmitData8(USART3, *data++);
        length--;
    }

    while (!LL_USART_IsActiveFlag_TC(USART3))
    {
    }
}

void DebugUart_init(void)
{
    if (debug_uart_context != NULL)
    {
        return;
    }

    debug_uart_context = malloc(sizeof(DebugUart_context_t));
    if (debug_uart_context == NULL)
    {
        return;
    }

    /* Using PD8 (TX) and PD9 (RX) for USART3. */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

    LL_GPIO_InitTypeDef gpio;
    LL_USART_InitTypeDef usart;

    /* Configuring USART3, 115200 8-N-1, TX only. */
    LL_USART_StructInit(&usart);
    usart.BaudRate = 115200;
    usart.TransferDirection = LL_USART_DIRECTION_TX;
    LL_USART_Init(USART3, &usart);

    /* Configuring USART3 TX on PD8 */
    gpio.Pin        = LL_GPIO_PIN_8;
    gpio.Mode       = LL_GPIO_MODE_ALTERNATE;
    gpio.Speed      = LL_GPIO_SPEED_FREQ_LOW;
    gpio.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    gpio.Pull       = LL_GPIO_PULL_UP;
    gpio.Alternate  = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOD, &gpio);

    /* Configuring USART3 RX on PD9 */
    gpio.Pin        = LL_GPIO_PIN_9;
    gpio.Mode       = LL_GPIO_MODE_ALTERNATE;
    gpio.Speed      = LL_GPIO_SPEED_FREQ_LOW;
    gpio.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    gpio.Pull       = LL_GPIO_PULL_UP;
    gpio.Alternate  = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOD, &gpio);

    LL_USART_Enable(USART3);
}

void DebugUart_printf(const char *fmt, ...)
{
    size_t len;
    va_list args;

    if (debug_uart_context == NULL)
    {
        return;
    }

    va_start(args, fmt);
    len = vsnprintf((char *)debug_uart_context->tx_buffer, DEBUG_UART_BUFFER_SIZE, fmt, args);
    va_end(args);

    // TODO: DMA-driven buffers
    DebugUart_blocking_tx(debug_uart_context->tx_buffer, len);
}
