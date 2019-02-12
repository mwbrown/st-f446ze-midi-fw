/**
  ******************************************************************************
  * @file    Templates_LL/Src/main.c
  * @author  MCD Application Team
  * @brief   Main program body through the LL API
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include <string.h>

/** @addtogroup STM32F4xx_LL_Examples
  * @{
  */

/** @addtogroup Templates_LL
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void     SystemClock_Config(void);

/* Private functions ---------------------------------------------------------*/

static void BoardInit_LEDs(void)
{
  LL_GPIO_InitTypeDef gpio;

  /* LEDs are on PB0, PB7, PB14. */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  gpio.Pin        = LED1_PIN | LED2_PIN | LED3_PIN;
  gpio.Mode       = LL_GPIO_MODE_OUTPUT;
  gpio.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
  gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio.Pull       = LL_GPIO_PULL_NO;
  gpio.Alternate  = LL_GPIO_AF_0;

  LL_GPIO_Init(GPIOB, &gpio);
}

static void BoardInit_USART(void)
{
  /* Using PD8 (TX) and PD9 (RX) for USART3. */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  LL_GPIO_InitTypeDef gpio;
  LL_USART_InitTypeDef usart;
  LL_USART_ClockInitTypeDef usart_ck;

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

/** Blocking TX for USART3. */
static void USART3_Tx(const uint8_t *data, uint16_t length)
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

static void BoardInit(void)
{
  BoardInit_LEDs();
  BoardInit_USART();
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* Configure the system clock to 180 MHz */
  SystemClock_Config();

  /* Initialize GPIOs and other peripherals. */
  BoardInit();

  /* Infinite loop */
  while (1)
  {
    for(int led_index = 0; led_index < 3; led_index++)
    {
      const uint32_t led_lookup[] = {LED1_PIN, LED2_PIN, LED3_PIN};
      const uint32_t all_leds_mask = LED1_PIN | LED2_PIN | LED3_PIN;
      const char *usart_data[] = {
        "LED1\r\n",
        "LED2\r\n",
        "LED3\r\n"
      };

      uint32_t mask;

      /* Dummy delay until we get proper timekeeping/RTOS. */
      volatile int x = 1800000;
      while(x--) { }

      /* Build a mask that turns on the current LED and turns
         off the other two (i.e. to write to the BSRR). */

      mask = ((all_leds_mask & ~led_lookup[led_index]) << 16) | (led_lookup[led_index]);
      GPIOB->BSRR = mask;

      USART3_Tx((const uint8_t *)usart_data[led_index], strlen(usart_data[led_index]));
    }
  }
}

/* ==============   BOARD SPECIFIC CONFIGURATION CODE BEGIN    ============== */
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  /* Enable HSE oscillator */
  LL_RCC_HSE_Enable();
  while(LL_RCC_HSE_IsReady() != 1)
  {
  };

  /* Set FLASH latency */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);

  /* Enable PWR clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* Activation OverDrive Mode */
  LL_PWR_EnableOverDriveMode();
  while(LL_PWR_IsActiveFlag_OD() != 1)
  {
  };

  /* Activation OverDrive Switching */
  LL_PWR_EnableOverDriveSwitching();
  while(LL_PWR_IsActiveFlag_ODSW() != 1)
  {
  };

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 360, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  };

  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  };

  /* Set APB1 & APB2 prescaler */
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);

  /* Set systick to 1ms */
  SysTick_Config(180000000 / 1000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  SystemCoreClock = 180000000;
}

/* ==============   BOARD SPECIFIC CONFIGURATION CODE END      ============== */

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
