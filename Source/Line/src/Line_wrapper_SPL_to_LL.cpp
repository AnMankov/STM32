#include "Line_wrapper_SPL_to_LL.h"

constexpr uint32_t ExtiLine[] =
{
  LL_EXTI_LINE_0,
  LL_EXTI_LINE_1,
  LL_EXTI_LINE_2,
  LL_EXTI_LINE_3,
  LL_EXTI_LINE_4,
  LL_EXTI_LINE_5,
  LL_EXTI_LINE_6,
  LL_EXTI_LINE_7,
  LL_EXTI_LINE_8,
  LL_EXTI_LINE_9,
  LL_EXTI_LINE_10,
  LL_EXTI_LINE_11,
  LL_EXTI_LINE_12,
  LL_EXTI_LINE_13,
  LL_EXTI_LINE_14,
  LL_EXTI_LINE_15,
  LL_EXTI_LINE_16,
  LL_EXTI_LINE_18,
  LL_EXTI_LINE_19,
  LL_EXTI_LINE_20  
};
  
void EXTI_ClearFlag(uint32_t EXTI_Line)//This parameter can be any combination of EXTI_Linex where x can be (0..20).
{   
  LL_EXTI_ClearFlag_0_31(ExtiLine[EXTI_Line]);
}

FlagStatus EXTI_GetFlagStatus(uint32_t EXTI_Line) //This parameter can be any combination of EXTI_Linex where x can be (0..20).
{
  return (FlagStatus)LL_EXTI_ReadFlag_0_31(ExtiLine[EXTI_Line]);
}

void EXTI_Init(EXTI_InitTypeDef* EXTI_InitStruct)
{
  LL_EXTI_InitTypeDef LL_EXTI_InitStruct;
  
  LL_EXTI_InitStruct.Line_0_31   = ExtiLine[EXTI_InitStruct->EXTI_Line]; 
  LL_EXTI_InitStruct.LineCommand = EXTI_InitStruct->EXTI_LineCmd;
  
  switch (EXTI_InitStruct->EXTI_Mode)
  {
    case EXTI_Mode_Interrupt:
	      LL_EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
	      break;
    case EXTI_Mode_Event:
	      LL_EXTI_InitStruct.Mode = LL_EXTI_MODE_EVENT;
	      break;
    default:
	      break;
  }
  
  switch (EXTI_InitStruct->EXTI_Trigger)
  {
    case EXTI_Trigger_Rising:
	      LL_EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
	      break;
    case EXTI_Trigger_Falling:
	      LL_EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
	      break;
    case EXTI_Trigger_Rising_Falling:
	      LL_EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
	      break;
    default:
	      break;
  }
     
  LL_EXTI_Init(&LL_EXTI_InitStruct);
}

void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct)
{
  LL_GPIO_InitTypeDef LL_GPIO_InitStruct;
  
  LL_GPIO_InitStruct.Pin  = GPIO_InitStruct->GPIO_Pin;
  LL_GPIO_InitStruct.Mode = GPIO_InitStruct->GPIO_Mode;  
  
  switch (GPIO_InitStruct->GPIO_Speed)
  {
    case GPIO_Speed_Level_1:
	      LL_GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	      break;
    case GPIO_Speed_Level_2:
	      LL_GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
	      break;
    case GPIO_Speed_Level_3:
	      LL_GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	      break;
    default:
	      break;        
  }  
  
  LL_GPIO_InitStruct.OutputType = GPIO_InitStruct->GPIO_OType;
  LL_GPIO_InitStruct.Pull       = GPIO_InitStruct->GPIO_PuPd;
  LL_GPIO_InitStruct.Alternate  = LL_GPIO_AF_0; //в аналогичной структуре SPL нет установки AF
                                                //AF устанавливается отдельными функциями
  LL_GPIO_Init(GPIOx, &LL_GPIO_InitStruct);
}

void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  LL_GPIO_ResetOutputPin(GPIOx, (uint32_t)GPIO_Pin);
}

void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal)
{
  switch (BitVal)
  {
    case Bit_RESET:
	      LL_GPIO_ResetOutputPin(GPIOx, GPIO_Pin);
	      break;
    case Bit_SET:
	      LL_GPIO_SetOutputPin(GPIOx, GPIO_Pin);
	      break;
    default:
	      break;
  }
}

void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState)
{
  switch (NewState)
  {
    case DISABLE:
	       if (LL_APB1_GRP1_IsEnabledClock(RCC_APB1Periph))
			   {
			     LL_APB1_GRP1_DisableClock(RCC_APB1Periph);
			   }
	       break;
    case ENABLE:
	       if (!LL_APB1_GRP1_IsEnabledClock(RCC_APB1Periph))
			   {
			     LL_APB1_GRP1_EnableClock(RCC_APB1Periph);
			   }
	       break;
    default:
	       break;
  }
}

void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks)
{
  LL_RCC_ClocksTypeDef SystemClocksFreq;
  LL_RCC_GetSystemClocksFreq(&SystemClocksFreq);
  
  RCC_Clocks->SYSCLK_Frequency    = SystemClocksFreq.SYSCLK_Frequency;
  RCC_Clocks->HCLK_Frequency      = SystemClocksFreq.HCLK_Frequency;
  RCC_Clocks->PCLK1_Frequency     = SystemClocksFreq.PCLK1_Frequency;
  RCC_Clocks->PCLK2_Frequency     = SystemClocksFreq.PCLK2_Frequency;
//  RCC_Clocks->ADC12CLK_Frequency  = LL_RCC_GetADCClockFreq(LL_RCC_ADC12_CLKSOURCE);
//  RCC_Clocks->ADC34CLK_Frequency  = LL_RCC_GetADCClockFreq(LL_RCC_ADC34_CLKSOURCE);
  RCC_Clocks->I2C1CLK_Frequency   = LL_RCC_GetI2CClockFreq(LL_RCC_I2C1_CLKSOURCE);
  RCC_Clocks->I2C2CLK_Frequency   = LL_RCC_GetI2CClockFreq(LL_RCC_I2C2_CLKSOURCE);
#if defined(RCC_CFGR3_I2C3SW)
  RCC_Clocks->I2C3CLK_Frequency   = LL_RCC_GetI2CClockFreq(LL_RCC_I2C3_CLKSOURCE);
#endif
//  RCC_Clocks->TIM1CLK_Frequency   = LL_RCC_GetTIMClockFreq(LL_RCC_TIM1_CLKSOURCE);
#if defined(HRTIM1)
  RCC_Clocks->HRTIM1CLK_Frequency = LL_RCC_GetHRTIMClockFreq(LL_RCC_HRTIM1_CLKSOURCE);
#endif
//  RCC_Clocks->TIM8CLK_Frequency   = LL_RCC_GetTIMClockFreq(LL_RCC_TIM8_CLKSOURCE);
#if defined(LL_RCC_TIM2_CLKSOURCE)
  RCC_Clocks->TIM2CLK_Frequency   = LL_RCC_GetTIMClockFreq(LL_RCC_TIM2_CLKSOURCE);
#endif
  RCC_Clocks->USART1CLK_Frequency = LL_RCC_GetUSARTClockFreq(LL_RCC_USART1_CLKSOURCE);
  RCC_Clocks->USART2CLK_Frequency = LL_RCC_GetUSARTClockFreq(LL_RCC_USART2_CLKSOURCE);;
  RCC_Clocks->USART3CLK_Frequency = LL_RCC_GetUSARTClockFreq(LL_RCC_USART3_CLKSOURCE);;
//  RCC_Clocks->UART4CLK_Frequency  = LL_RCC_GetUARTClockFreq(LL_RCC_UART4_CLKSOURCE);
//  RCC_Clocks->UART5CLK_Frequency  = LL_RCC_GetUARTClockFreq(LL_RCC_UART5_CLKSOURCE);
#if defined(LL_RCC_TIM15_CLKSOURCE)
  RCC_Clocks->TIM15CLK_Frequency  = LL_RCC_GetTIMClockFreq(LL_RCC_TIM15_CLKSOURCE);
#endif
#if defined(LL_RCC_TIM16_CLKSOURCE)
  RCC_Clocks->TIM16CLK_Frequency  = LL_RCC_GetTIMClockFreq(LL_RCC_TIM16_CLKSOURCE);
#endif
#if defined(LL_RCC_TIM17_CLKSOURCE)
  RCC_Clocks->TIM17CLK_Frequency  = LL_RCC_GetTIMClockFreq(LL_RCC_TIM17_CLKSOURCE);
#endif
#if defined(LL_RCC_TIM20_CLKSOURCE)
  RCC_Clocks->TIM20CLK_Frequency  = LL_RCC_GetTIMClockFreq(LL_RCC_TIM20_CLKSOURCE);
#endif
}

void SYSCFG_EXTILineConfig(uint8_t EXTI_PortSourceGPIOx, uint8_t EXTI_PinSourcex)
{
  constexpr uint32_t ExtiLine[] = 
  {
    LL_SYSCFG_EXTI_LINE0, 
    LL_SYSCFG_EXTI_LINE1, 
    LL_SYSCFG_EXTI_LINE2, 
    LL_SYSCFG_EXTI_LINE3, 
    LL_SYSCFG_EXTI_LINE4, 
    LL_SYSCFG_EXTI_LINE5, 
    LL_SYSCFG_EXTI_LINE6, 
    LL_SYSCFG_EXTI_LINE7, 
    LL_SYSCFG_EXTI_LINE8, 
    LL_SYSCFG_EXTI_LINE9, 
    LL_SYSCFG_EXTI_LINE10,
    LL_SYSCFG_EXTI_LINE11,
    LL_SYSCFG_EXTI_LINE12,
    LL_SYSCFG_EXTI_LINE13,
    LL_SYSCFG_EXTI_LINE14,
    LL_SYSCFG_EXTI_LINE15
  };  
  LL_SYSCFG_SetEXTISource(EXTI_PortSourceGPIOx, ExtiLine[EXTI_PinSourcex]);
}

void TIM_ClearFlag(TIM_TypeDef* TIMx, uint16_t TIM_FLAG)
{
  typedef void (*TClrFunc)(TIM_TypeDef *TIMx);
  
  struct TFlagHandler
  {
    uint32_t TimFlag;
	  TClrFunc ClrFunc;
  };
  
  constexpr TFlagHandler FlagHandler[] = //реализовано только то, что используется в sensline.c
  {
    {TIM_FLAG_CC1 , LL_TIM_ClearFlag_CC1},
    {TIM_FLAG_CC2 , LL_TIM_ClearFlag_CC2},
    {TIM_FLAG_CC3 , LL_TIM_ClearFlag_CC3},
  };
  
  for (auto item : FlagHandler)
  {
    if (item.TimFlag == TIM_FLAG)
	 {
	   item.ClrFunc(TIMx);
		return;
	 }
  }
}

void TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState NewState)
{
  switch (NewState)
  {
    case DISABLE:
	       if ( LL_TIM_IsEnabledCounter(TIMx) ) //если счетчик включен
         {
			     LL_TIM_DisableCounter(TIMx);
			   }			
	       break;
    case ENABLE:  
	       if ( !LL_TIM_IsEnabledCounter(TIMx) ) //если счетчик отключен
         {
			     LL_TIM_EnableCounter(TIMx);
			   }
	       break;
    default:
	       break;
  }
}

void TIM_DeInit(TIM_TypeDef* TIMx)
{
  LL_TIM_DeInit(TIMx);
}

uint32_t TIM_GetCounter(TIM_TypeDef* TIMx)
{
  return LL_TIM_GetCounter(TIMx);
}

ITStatus TIM_GetITStatus(TIM_TypeDef* TIMx, uint16_t TIM_IT)
{
  return ( LL_TIM_IsActiveFlag_CC2(TIMx) )  //в sensline.c используется только CC2
         &&
         ( LL_TIM_IsEnabledIT_CC2(TIMx) )
         ? ITStatus::SET  
         : ITStatus::RESET;
}

void TIM_ITConfig(TIM_TypeDef* TIMx, uint16_t TIM_IT, FunctionalState NewState)
{
  typedef void (*THandler)(TIM_TypeDef *);

  struct TItConfig
  {
    uint16_t SplIt;        //константа в библиотеке spl
    FunctionalState State;
    THandler Handler;
  };  
  
  TItConfig ItConfig[] =
  {
    {
      TIM_IT_CC1,
      DISABLE,
      LL_TIM_DisableIT_CC1,
    },
    {
      TIM_IT_CC1,
      ENABLE,
      LL_TIM_EnableIT_CC1
    },
    {
      TIM_IT_CC2,
      DISABLE,
      LL_TIM_DisableIT_CC2,
    },
    {
      TIM_IT_CC2,
      ENABLE,
      LL_TIM_EnableIT_CC2,
    },
    {
      TIM_IT_CC3,
      DISABLE,
      LL_TIM_DisableIT_CC3,
    },
    {
      TIM_IT_CC3,
      ENABLE,
      LL_TIM_EnableIT_CC3,
    },
  };
  
  for (auto item : ItConfig)
  {
    if (
         ( item.SplIt == TIM_IT )
         &&
         ( item.State == NewState )
       )
    {
      item.Handler(TIMx);
    }
  }
}

void TIM_InternalClockConfig(TIM_TypeDef* TIMx)
{
  LL_TIM_SetClockSource(TIMx, LL_TIM_CLOCKSOURCE_INTERNAL);
}

void TIM_OC1Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct)
{
  LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_FROZEN);  
  
  constexpr uint32_t LL_TIM_ACTIVEOUTPUT = 0x00;
  LL_TIM_IC_SetActiveInput(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEOUTPUT);
  
  LL_TIM_OC_SetPolarity(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH1);
  
  LL_TIM_OC_SetCompareCH1(TIMx, TIM_OCInitStruct->TIM_Pulse);
}

void TIM_OC2Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct)
{
  LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_FROZEN);  
  
  constexpr uint32_t LL_TIM_ACTIVEOUTPUT = 0x00;
  LL_TIM_IC_SetActiveInput(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEOUTPUT);
  
  LL_TIM_OC_SetPolarity(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH2);
  
  LL_TIM_OC_SetCompareCH1(TIMx, TIM_OCInitStruct->TIM_Pulse);
}

void TIM_OC3Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct)
{
  LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_FROZEN);  
  
  constexpr uint32_t LL_TIM_ACTIVEOUTPUT = 0x00;
  LL_TIM_IC_SetActiveInput(TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_ACTIVEOUTPUT);
  
  LL_TIM_OC_SetPolarity(TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH3);
  
  LL_TIM_OC_SetCompareCH1(TIMx, TIM_OCInitStruct->TIM_Pulse);
}

void TIM_TimeBaseInit(TIM_TypeDef* TIMx, TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct)
{
  LL_TIM_SetCounterMode(TIMx, LL_TIM_COUNTERMODE_UP);
  LL_TIM_SetAutoReload(TIMx, TIM_TimeBaseInitStruct->TIM_Period);
  LL_TIM_SetPrescaler(TIMx, TIM_TimeBaseInitStruct->TIM_Prescaler);
}

void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct)
{
  /* Set the default configuration */
  TIM_TimeBaseInitStruct->TIM_Period = 0xFFFFFFFF;
  TIM_TimeBaseInitStruct->TIM_Prescaler = 0x0000;
  TIM_TimeBaseInitStruct->TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct->TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStruct->TIM_RepetitionCounter = 0x0000;
}
