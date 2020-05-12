/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

//#pragma anon_unions

  /* Includes ------------------------------------------------------------------*/
//#include <bitset>

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

//#include "data_types.h"  //Основные типы данных в устройстве

//#include "USART_driver.h"
//#include "CAN_driver.h"
#include "motion_tl.h"
#include "relay.h"

#include "MPU-9250_acc_gyro_mag_driver_hl.h"
#include "Flash_driver.h"

//#include "FreeRTOSConfig.h"
//#include "FreeRTOS.h"
//#include "croutine.h"
//#include "task.h"
//#include "queue.h"
//#include "semphr.h"
//#include "portmacro.h"
//#include "timers.h"

#include "sys_error_codes.h"
#include "sensline.h"
#include "sens_types.h"

#define DBG_VERSION
#ifdef DBG_VERSION
  #define DBG_PRINT_1_STR(X)        printf(#X" = %s\n", (X).data())
  #define DBG_PRINT_2_STR(X, Y)     printf(#X" = %s\n"#Y" = %s\n", (X).data(), (Y).data())
  #define DBG_PRINT_3_STR(X, Y, Z)  printf(#X" = %s\n"#Y" = %s\n"#Z" = %s\n", (X).data(), (Y).data(), (Z).data())
#else
  #define DBG_PRINT_1_STR(X)        
  #define DBG_PRINT_2_STR(X, Y)     
  #define DBG_PRINT_3_STR(X, Y, Z)  
#endif

/* Private define ------------------------------------------------------------*/
//#define USE_FULL_LL_DRIVER

#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* ITM Trace Enable Register Definitions */
#define ITM_TER_PRINTF_Pos                   0U                                            /*!< ITM TER: PRINTF Position */
#define ITM_TER_PRINTF_Msk                   (1UL << ITM_TER_PRINTF_Pos)                   /*!< ITM TER: PRINTF Mask */

#define LD4                          LL_GPIO_PIN_8
#define LD9                          LL_GPIO_PIN_12
#define LED_BLUE_ONE                 LD4  
#define LED_BLUE_TWO                 LD9
                                     
#define LD8                          LL_GPIO_PIN_14
#define LED_ORANGE_TWO               LD8


/* Время в мс */
#define SECOND         1000U
#define DECISECOND     100U

#define MAIN_TIMER     TIM7

/* Адреса регистров модуля Data Watchpoint and Trace Unit*/
#define    DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define    DWT_CONTROL   *(volatile unsigned long *)0xE0001000
#define    SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC

/* Определение количества элементов в массиве */
#define N_ELEMENTS(X)    (sizeof(X) / sizeof(*(X)))

namespace TClkSrc
{
  enum T : uint8_t
  {
    _MIN   = 0,
    _HSE   = 0,
    _HSI16 = 1,
    _MSI   = 2,
    _PLL   = 3,
    
    _MAX,
  };
}

namespace TVcoreRange
{
  enum T : uint8_t
  {
    _MIN    = 0,
    _RANGE1 = 0,
    _RANGE2 = 1,
    
    _MAX,
  }; 
}

namespace TLatency
{
  enum T : uint8_t
  {
    _MIN  = 0,
    _0_WS = 0,
    _1_WS = 1,
    _2_WS = 2,
    _3_WS = 3,
    _4_WS = 4,
    
    _MAX,
  }; 
}

constexpr uint16_t MAX_COVER_OPENED_ANGLE   = 271U; //максимальный угол, на который физически открывается люк, в градусах
constexpr uint16_t MAX_COVER_UNSEALED_ANGLE = 271U; //максимальный угол, на который физически открывается замок, в градусах

//extern const relay::TRelay RelOne;
extern const relay::TRelay RelTwo;
extern const relay::TRelay RelFour;
extern const relay::TRelay RelThree;

extern uint32_t RtosHeapSize;

//extern USART::TUSART Usart;
extern MPU_9250::TAccGyroMagDriver_HL AccMagGyro;

//******************************************************************************
//  Секция объявлений функций (declaration)
//******************************************************************************

#endif /* __MAIN_H */
