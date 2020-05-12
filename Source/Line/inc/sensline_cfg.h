/////////////////////////////////////////////////////////////////////////////
// Реализация транспортного уровня линии СЕНС - заголовочный файл конфигурации аппаратуры
// Версия 0.0
// 
#ifndef SENSLINE_CFG_H
#define SENSLINE_CFG_H

#include "stm32l431xx.h"      // Определяется контроллером
#include "Line_wrapper_SPL_to_LL.h"



// При использовании внутреннего компаратора заняты ресурсы:
// Компаратор COMP1, ножки PA0,PA1, PA6 - для подключения компаратора к прерываниям
// При этом ножка PA6 является входом сигнала с линии
// Определяет использование внутреннего компаратора
// Возможные значения:
#define NO_USE_COMP   // Не использовать компаратор
//#define USE_COMP1_051 // компаратор контроллера STM32F051
//#define USE_COMP1_303 // компаратор контроллера STM32F303

// Подключение портов к шине производится В ОСНОВНОЙ ПРОГРАММЕ

// Определения входной ножки
#define INPORT GPIOA
#define INPIN  GPIO_Pin_6
#define EXTI_LINE EXTI_Line6
#define NVIC_IRQn EXTI9_5_IRQn
#define EXTI_SOURCE EXTI_PortSourceGPIOA  // Источник прерываний
#define EXTI_PINSOURCE EXTI_PinSource6    // 
/// Инициализация без компаратора
// Для 303 - GPIO_AF_8
// Для 051 - GPIO_AF_7
#define INPINAF GPIO_AF_8 // Наименование функции-обработчика внешних прерываний
// Для STM32F303 и ножки PA6 - EXTI9_5_IRQHandler(void)
// Для STM32F103 и ножки PA6 - EXTI4_15_IRQHandler(void)
#define INTERRUPT_HANDLER() EXTI9_5_IRQHandler(void)


// Определяет ножку - выход на линию
#define OUTPORT GPIOA
#define OUTPIN  GPIO_Pin_5

// Инструкции вход в критическую секцию и выхода из критической секции (без RTOS - просто запрет и разрешение прерываний)
#define IN_CRITICAL() __disable_irq()
#define OUT_CRITICAL() __enable_irq()


#endif
