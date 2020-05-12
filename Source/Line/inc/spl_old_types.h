#ifndef __SPL_OLD_TYPES_H
#define __SPL_OLD_TYPES_H

#include "stdint.h"

//----- EXTI -----
typedef enum
{
  EXTI_Mode_Interrupt = 0x00,
  EXTI_Mode_Event     = 0x04
} EXTIMode_TypeDef;

typedef enum
{
  EXTI_Trigger_Rising = 0x08,
  EXTI_Trigger_Falling = 0x0C,
  EXTI_Trigger_Rising_Falling = 0x10
} EXTITrigger_TypeDef;

typedef struct
{
  uint32_t EXTI_Line;               /*!< Specifies the EXTI lines to be enabled or disabled.
                                         This parameter can be any combination of @ref EXTI_Lines */
   
  EXTIMode_TypeDef EXTI_Mode;       /*!< Specifies the mode for the EXTI lines.
                                         This parameter can be a value of @ref EXTIMode_TypeDef */

  EXTITrigger_TypeDef EXTI_Trigger; /*!< Specifies the trigger signal active edge for the EXTI lines.
                                         This parameter can be a value of @ref EXTITrigger_TypeDef */

  FunctionalState EXTI_LineCmd;     /*!< Specifies the new state of the selected EXTI lines.
                                         This parameter can be set either to ENABLE or DISABLE */
} EXTI_InitTypeDef;
//----- EXTI -----//

//----- GPIO -----
typedef enum
{ 
  GPIO_Mode_IN   = 0x00, /*!< GPIO Input Mode */
  GPIO_Mode_OUT  = 0x01, /*!< GPIO Output Mode */
  GPIO_Mode_AF   = 0x02, /*!< GPIO Alternate function Mode */
  GPIO_Mode_AN   = 0x03  /*!< GPIO Analog In/Out Mode      */
} GPIOMode_TypeDef;

typedef enum
{ 
  GPIO_Speed_Level_1  = 0x01, /*!< Fast Speed     */
  GPIO_Speed_Level_2  = 0x02, /*!< Meduim Speed   */
  GPIO_Speed_Level_3  = 0x03  /*!< High Speed     */
} GPIOSpeed_TypeDef;

typedef enum
{ 
  GPIO_OType_PP = 0x00,
  GPIO_OType_OD = 0x01
} GPIOOType_TypeDef;

typedef enum
{
  GPIO_PuPd_NOPULL = 0x00,
  GPIO_PuPd_UP     = 0x01,
  GPIO_PuPd_DOWN   = 0x02
} GPIOPuPd_TypeDef;

typedef struct
{
  uint32_t GPIO_Pin;              /*!< Specifies the GPIO pins to be configured.
                                       This parameter can be any value of @ref GPIO_pins_define */
                                       
  GPIOMode_TypeDef GPIO_Mode;     /*!< Specifies the operating mode for the selected pins.
                                       This parameter can be a value of @ref GPIOMode_TypeDef   */

  GPIOSpeed_TypeDef GPIO_Speed;   /*!< Specifies the speed for the selected pins.
                                       This parameter can be a value of @ref GPIOSpeed_TypeDef  */

  GPIOOType_TypeDef GPIO_OType;   /*!< Specifies the operating output type for the selected pins.
                                       This parameter can be a value of @ref GPIOOType_TypeDef  */

  GPIOPuPd_TypeDef GPIO_PuPd;     /*!< Specifies the operating Pull-up/Pull down for the selected pins.
                                       This parameter can be a value of @ref GPIOPuPd_TypeDef   */
}GPIO_InitTypeDef;

typedef enum
{ 
  Bit_RESET = 0,
  Bit_SET
} BitAction;

#define EXTI_Line6                         LL_EXTI_LINE_6
#define GPIO_Pin_6                         LL_GPIO_PIN_6
#define GPIO_Pin_5                         LL_GPIO_PIN_5
#define EXTI_PortSourceGPIOA               LL_SYSCFG_EXTI_PORTA
#define EXTI_PinSource6                    ((uint8_t)0x06)
//----- GPIO -----//

//----- RCC -----
typedef struct
{
  uint32_t SYSCLK_Frequency;
  uint32_t HCLK_Frequency;
  uint32_t PCLK1_Frequency;
  uint32_t PCLK2_Frequency;
  uint32_t ADC12CLK_Frequency;
  uint32_t ADC34CLK_Frequency;
  uint32_t I2C1CLK_Frequency;
  uint32_t I2C2CLK_Frequency;
  uint32_t I2C3CLK_Frequency;
  uint32_t TIM1CLK_Frequency;
  uint32_t HRTIM1CLK_Frequency;
  uint32_t TIM8CLK_Frequency;
  uint32_t TIM2CLK_Frequency;
  uint32_t TIM3CLK_Frequency;
  uint32_t USART1CLK_Frequency;
  uint32_t USART2CLK_Frequency;
  uint32_t USART3CLK_Frequency;
  uint32_t UART4CLK_Frequency;
  uint32_t UART5CLK_Frequency;
  uint32_t TIM15CLK_Frequency;
  uint32_t TIM16CLK_Frequency;
  uint32_t TIM17CLK_Frequency;
  uint32_t TIM20CLK_Frequency;
} RCC_ClocksTypeDef;

#define RCC_APB1Periph_TIM3                RCC_APB1ENR1_TIM2EN
//----- RCC -----//

//----- TIM -----
typedef struct
{
  uint32_t TIM_OCMode;        /*!< Specifies the TIM mode.
                                   This parameter can be a value of @ref TIM_Output_Compare_and_PWM_modes */

  uint16_t TIM_OutputState;   /*!< Specifies the TIM Output Compare state.
                                   This parameter can be a value of @ref TIM_Output_Compare_State */

  uint16_t TIM_OutputNState;  /*!< Specifies the TIM complementary Output Compare state.
                                   This parameter can be a value of @ref TIM_Output_Compare_N_State
                                   @note This parameter is valid only for TIM1 and TIM8. */

  uint32_t TIM_Pulse;         /*!< Specifies the pulse value to be loaded into the Capture Compare Register. 
                                   This parameter can be a number between 0x0000 and 0xFFFF */

  uint16_t TIM_OCPolarity;    /*!< Specifies the output polarity.
                                   This parameter can be a value of @ref TIM_Output_Compare_Polarity */

  uint16_t TIM_OCNPolarity;   /*!< Specifies the complementary output polarity.
                                   This parameter can be a value of @ref TIM_Output_Compare_N_Polarity
                                   @note This parameter is valid only for TIM1 and TIM8. */

  uint16_t TIM_OCIdleState;   /*!< Specifies the TIM Output Compare pin state during Idle state.
                                   This parameter can be a value of @ref TIM_Output_Compare_Idle_State
                                   @note This parameter is valid only for TIM1 and TIM8. */

  uint16_t TIM_OCNIdleState;  /*!< Specifies the TIM Output Compare pin state during Idle state.
                                   This parameter can be a value of @ref TIM_Output_Compare_N_Idle_State
                                   @note This parameter is valid only for TIM1 and TIM8. */
} TIM_OCInitTypeDef;

typedef struct
{
  uint16_t TIM_Prescaler;         /*!< Specifies the prescaler value used to divide the TIM clock.
                                       This parameter can be a number between 0x0000 and 0xFFFF */

  uint16_t TIM_CounterMode;       /*!< Specifies the counter mode.
                                       This parameter can be a value of @ref TIM_Counter_Mode */

  uint32_t TIM_Period;            /*!< Specifies the period value to be loaded into the active
                                       Auto-Reload Register at the next update event.
                                       This parameter must be a number between 0x0000 and 0xFFFF.  */ 

  uint16_t TIM_ClockDivision;     /*!< Specifies the clock division.
                                      This parameter can be a value of @ref TIM_Clock_Division_CKD */

  uint16_t TIM_RepetitionCounter;  /*!< Specifies the repetition counter value. Each time the RCR downcounter
                                       reaches zero, an update event is generated and counting restarts
                                       from the RCR value (N).
                                       This means in PWM mode that (N+1) corresponds to:
                                          - the number of PWM periods in edge-aligned mode
                                          - the number of half PWM period in center-aligned mode
                                       This parameter must be a number between 0x00 and 0xFF. 
                                       @note This parameter is valid only for TIM1 and TIM8. */
} TIM_TimeBaseInitTypeDef;

#define TIM_FLAG_Update                    ((uint32_t)0x00001)
#define TIM_FLAG_CC1                       ((uint32_t)0x00002)
#define TIM_FLAG_CC2                       ((uint32_t)0x00004)
#define TIM_FLAG_CC3                       ((uint32_t)0x00008)
#define TIM_FLAG_CC4                       ((uint32_t)0x00010)
#define TIM_FLAG_COM                       ((uint32_t)0x00020)
#define TIM_FLAG_Trigger                   ((uint32_t)0x00040)
#define TIM_FLAG_Break                     ((uint32_t)0x00080)
#define TIM_FLAG_Break2                    ((uint32_t)0x00100)
#define TIM_FLAG_CC1OF                     ((uint32_t)0x00200)
#define TIM_FLAG_CC2OF                     ((uint32_t)0x00400)
#define TIM_FLAG_CC3OF                     ((uint32_t)0x00800)
#define TIM_FLAG_CC4OF                     ((uint32_t)0x01000)
#define TIM_FLAG_CC5                       ((uint32_t)0x10000)
#define TIM_FLAG_CC6                       ((uint32_t)0x20000)

#define TIM_PSCReloadMode_Immediate        ((uint16_t)0x0001)
#define TIM_CKD_DIV1                       ((uint16_t)0x0000)
#define TIM_CounterMode_Up                 ((uint16_t)0x0000)

#define TIM3                               TIM2
#define TIM_IT_CC1                         LL_TIM_DIER_CC1IE
#define TIM_IT_CC2                         LL_TIM_DIER_CC2IE
#define TIM_IT_CC3                         LL_TIM_DIER_CC3IE
#define TIM_OCMode_Timing                  ((uint32_t)0x00000)
#define TIM_OutputState_Disable            ((uint16_t)0x0000)
#define TIM_OCPolarity_High                ((uint16_t)0x0000)

#define TIM3_IRQn                          TIM2_IRQn
//----- TIM -----//


#endif //__SPL_OLD_TYPES_H
