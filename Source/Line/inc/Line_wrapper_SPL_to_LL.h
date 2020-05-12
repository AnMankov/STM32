#ifndef __LINE_WRAPPER_SPL_TO_LL_H
#define __LINE_WRAPPER_SPL_TO_LL_H

//#include "main.h"

/*
 * Модуль функций-оберток над Low-layer APIs вызовами,
 * необходимый для обратной совместимости с устаревшей (на 2018г.) SPL
 * Реализовано только то, что необходимо для работы sensline.c
*/

#ifdef __cplusplus
extern "C"
{
#endif

//----- Low-layer API --------------------------------------------------------------------------
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_tim.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l431xx.h"
//-----------------------------------------------------------------------------------------------

//----- OLD API ---------------------------------------------------------------------------------
#include "spl_old_types.h"
//-----------------------------------------------------------------------------------------------


  void EXTI_ClearFlag(uint32_t EXTI_Line);
  FlagStatus EXTI_GetFlagStatus(uint32_t EXTI_Line);
  void EXTI_Init(EXTI_InitTypeDef* EXTI_InitStruct);
  
  void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
  void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
  void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
  
  void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
  void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);
  
  void SYSCFG_EXTILineConfig(uint8_t EXTI_PortSourceGPIOx, uint8_t EXTI_PinSourcex);
  
  void TIM_ClearFlag(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);
  void TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState NewState);
  void TIM_DeInit(TIM_TypeDef* TIMx);
  uint32_t TIM_GetCounter(TIM_TypeDef* TIMx);
  ITStatus TIM_GetITStatus(TIM_TypeDef* TIMx, uint16_t TIM_IT);
  void TIM_ITConfig(TIM_TypeDef* TIMx, uint16_t TIM_IT, FunctionalState NewState);
  void TIM_InternalClockConfig(TIM_TypeDef* TIMx);
  void TIM_OC1Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
  void TIM_OC2Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
  void TIM_OC3Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
  void TIM_TimeBaseInit(TIM_TypeDef* TIMx, TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);
  void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);


#ifdef __cplusplus
}
#endif


#endif
