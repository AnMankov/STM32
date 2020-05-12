#ifndef MODBUS_H
#define MODBUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx.h"
/*
	Реализация протокола ModBus-Slave RTU в соответствии с 
	"ModBus application protocol specification v1.1b" (2006)
*/

// Инициализация ModBus
void MB_Init(uint8_t);
// Сброс счетчиков
void MB_ResetCnt(void);

uint8_t get_mb_addr(); //возврат текущего установленного значения ModBus адреса

// Функция разбора пакета ModBus (u.rsbuf) и формирования ответа (u.trbuf)
// Возвращает количество байт в ответе (u.trcnt)
uint16_t MB_Parse(void);

// Функция переставляет байты местами
uint16_t ByteSwap(uint16_t val);

#ifdef __cplusplus
}
#endif

#endif
