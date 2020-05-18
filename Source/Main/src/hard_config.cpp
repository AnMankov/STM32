#include "hard_config.h"

TPot_HW Pot_HW =
{
  {           //CS
    GPIOB,
    LL_GPIO_PIN_12,
    LL_AHB2_GRP1_PERIPH_GPIOB,
    LL_AHB2_GRP1_EnableClock,
    0
  },
  {           //SHDN
    GPIOB,
    LL_GPIO_PIN_1,
    LL_AHB2_GRP1_PERIPH_GPIOB,
    LL_AHB2_GRP1_EnableClock,
    0
  },
  {           //RS
    GPIOB,
    LL_GPIO_PIN_2,
    LL_AHB2_GRP1_PERIPH_GPIOB,
    LL_AHB2_GRP1_EnableClock,
    0
  },
  {           //CLK
    GPIOB,
    LL_GPIO_PIN_13,
    LL_AHB2_GRP1_PERIPH_GPIOB,
    LL_AHB2_GRP1_EnableClock,
    0
  },
  {           //SDI
    GPIOB,
    LL_GPIO_PIN_15,
    LL_AHB2_GRP1_PERIPH_GPIOB,
    LL_AHB2_GRP1_EnableClock,
    0
  }
};

TUsart_HW UsartExt_HW =
{
  USART1,
  USART1_IRQn,
  {
    LL_APB2_GRP1_EnableClock,
    LL_APB2_GRP1_PERIPH_USART1,
    LL_RCC_USART1_CLKSOURCE_SYSCLK,
    LL_RCC_USART1_CLKSOURCE
  },
  {           //Tx
    GPIOA,
    LL_GPIO_PIN_9,
    LL_AHB2_GRP1_PERIPH_GPIOA,
    LL_AHB2_GRP1_EnableClock,
    0
  },
//  {           //Tx
//    GPIOB,
//    LL_GPIO_PIN_6,
//    LL_AHB2_GRP1_PERIPH_GPIOB,
//    LL_AHB2_GRP1_EnableClock,
//    0
//  },
  {           //Rx
    GPIOA,
    LL_GPIO_PIN_10,
    LL_AHB2_GRP1_PERIPH_GPIOB,
    LL_AHB2_GRP1_EnableClock,
    0
  },
  {           //DE
    GPIOA,
    LL_GPIO_PIN_12,
    LL_AHB2_GRP1_PERIPH_GPIOA,
    LL_AHB2_GRP1_EnableClock,
    0
  }
};

TTmr_HW TmrFreg_HW =
{
  TIM15,
  TIM1_BRK_TIM15_IRQn,
  LL_TIM_CHANNEL_CH1,
  65535U,
  {
    LL_APB2_GRP1_EnableClock,
    LL_APB2_GRP1_PERIPH_TIM15,
    0,
    0
  },
  {
    GPIOB,
    LL_GPIO_PIN_14,
    LL_AHB2_GRP1_PERIPH_GPIOB,
    LL_AHB2_GRP1_EnableClock,
    LL_GPIO_AF_14
  }
};

TTmr_HW TmrCmp_HW =
{
  TIM2,
  TIM1_BRK_TIM15_IRQn,
  LL_TIM_CHANNEL_CH2,
  4294967295U,
  {
    LL_APB1_GRP1_EnableClock,
    LL_APB1_GRP1_PERIPH_TIM2,
    0,
    0
  },
  {
    GPIOA,
    LL_GPIO_PIN_1,
    LL_AHB2_GRP1_PERIPH_GPIOA,
    LL_AHB2_GRP1_EnableClock,
    LL_GPIO_AF_1
  }
};

TTmr_HW TmrDiff_HW =
{
  TIM1,
  TIM1_BRK_TIM15_IRQn,
  LL_TIM_CHANNEL_CH3,
  65535U,
  {
    LL_APB2_GRP1_EnableClock,
    LL_APB2_GRP1_PERIPH_TIM1,
    0,
    0
  },
  {
    GPIOA,
    LL_GPIO_PIN_10,
    LL_AHB2_GRP1_PERIPH_GPIOA,
    LL_AHB2_GRP1_EnableClock,
    LL_GPIO_AF_1
  }
};

TAdc_HW Adc_HW =
{
  ADC1,
  ADC1_COMMON,
  ADC1_IRQn,
  LL_ADC_CHANNEL_5,
  {
    LL_AHB2_GRP1_EnableClock,
    LL_AHB2_GRP1_PERIPH_ADC,
    0,
    0
  },
  LL_RCC_SetADCClockSource,
  LL_RCC_ADC_CLKSOURCE_SYSCLK,  
  {
    GPIOA,
    LL_GPIO_PIN_0,
    LL_AHB2_GRP1_PERIPH_GPIOA,
    LL_AHB2_GRP1_EnableClock,
    0U
  },
  {
    GPIOA,
    LL_GPIO_PIN_10,
    LL_AHB2_GRP1_PERIPH_GPIOA,
    LL_AHB2_GRP1_EnableClock,
    0U
  },
	{
	  LL_SYSCFG_EXTI_PORTA,
	  LL_SYSCFG_EXTI_LINE10,
    LL_EXTI_LINE_10,
	  LL_EXTI_MODE_IT,
	  LL_EXTI_TRIGGER_FALLING,
	  EXTI15_10_IRQn,
    LL_APB2_GRP1_EnableClock,
    LL_APB2_GRP1_PERIPH_SYSCFG
	}
};

//TUsart_HW UsartInt_HW =
//{
//  USART2,
//  USART2_IRQn,
//  {
//    LL_APB1_GRP1_EnableClock,
//    LL_APB1_GRP1_PERIPH_USART2,
//    LL_RCC_USART2_CLKSOURCE_SYSCLK,
//    LL_RCC_USART2_CLKSOURCE
//  },
//  {           //Tx
//    GPIOA,
//    LL_GPIO_PIN_2,
//    LL_AHB2_GRP1_PERIPH_GPIOA,
//    LL_AHB2_GRP1_EnableClock
//  },
//  {           //Rx
//    GPIOA,
//    LL_GPIO_PIN_3,
//    LL_AHB2_GRP1_PERIPH_GPIOA,
//    LL_AHB2_GRP1_EnableClock
//  },
//  {           //DE
//    GPIOA,
//    LL_GPIO_PIN_1,
//    LL_AHB2_GRP1_PERIPH_GPIOA,
//    LL_AHB2_GRP1_EnableClock
//  }
//};

TSpi_HW Spi_HW =
{
  SPI1,
  SPI1_IRQn,
  {
    LL_APB2_GRP1_EnableClock,
    LL_APB2_GRP1_PERIPH_SPI1,
    0,
    0
  },
  {           //SO
    GPIOA,
    LL_GPIO_PIN_11,
    LL_AHB2_GRP1_PERIPH_GPIOA,
    LL_AHB2_GRP1_EnableClock,
    0
  },
  {           //SI
    GPIOA,
    LL_GPIO_PIN_7,
    LL_AHB2_GRP1_PERIPH_GPIOA,
    LL_AHB2_GRP1_EnableClock,
    0
  },
  {           //SCK
    GPIOA,
    LL_GPIO_PIN_5,
    LL_AHB2_GRP1_PERIPH_GPIOA,
    LL_AHB2_GRP1_EnableClock,
    0
  },
  {           //CS
    GPIOA,
    LL_GPIO_PIN_0,
    LL_AHB2_GRP1_PERIPH_GPIOA,
    LL_AHB2_GRP1_EnableClock,
    0
  }
};

TPin DevDetect_HW =
{
  GPIOB,
  LL_GPIO_PIN_4,
  LL_AHB2_GRP1_PERIPH_GPIOB,
	LL_AHB2_GRP1_EnableClock,
  0
};

TPin UCG1_HW =
{
  GPIOA,
  LL_GPIO_PIN_11,
  LL_AHB2_GRP1_PERIPH_GPIOA,
	LL_AHB2_GRP1_EnableClock,
  0
};

TDevAddr_HWr DevAddr_HW =
{
  { { GPIOB, LL_GPIO_PIN_0, LL_AHB2_GRP1_PERIPH_GPIOB, LL_AHB2_GRP1_EnableClock }, 1U, _UP },
  { { GPIOB, LL_GPIO_PIN_1, LL_AHB2_GRP1_PERIPH_GPIOB, LL_AHB2_GRP1_EnableClock }, 2U, _UP },
  { { GPIOB, LL_GPIO_PIN_2, LL_AHB2_GRP1_PERIPH_GPIOB, LL_AHB2_GRP1_EnableClock }, 4U, _UP },
  { { GPIOB, LL_GPIO_PIN_3, LL_AHB2_GRP1_PERIPH_GPIOB, LL_AHB2_GRP1_EnableClock }, 8U, _UP },
};

TPin DO_HW =
{
  GPIOB,
  LL_GPIO_PIN_5,
  LL_AHB2_GRP1_PERIPH_GPIOB,
	LL_AHB2_GRP1_EnableClock,
  0
};

TPin Led_HW =
{
  GPIOB,
  LL_GPIO_PIN_13,
  LL_AHB2_GRP1_PERIPH_GPIOB,
	LL_AHB2_GRP1_EnableClock,
  0
};

TPin Button_HW =
{
  GPIOA,
  LL_GPIO_PIN_5,
  LL_AHB2_GRP1_PERIPH_GPIOA,
	LL_AHB2_GRP1_EnableClock,
  0
};
