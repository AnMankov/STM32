#ifndef MAP_REG_H
#define MAP_REG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sens_types.h"

#include "stm32l431xx.h"

// Читает значение регистра ввода по адресу adr. Целые значения возвращаются в формате big-endian, части таблиц и float передаются как хранятся в ОЗУ
uint16_t GetInputRegisters(uint16_t adr);
// Читает значение регистра хранения по адресу adr. Целые значения возвращаются в формате big-endian, части таблиц и float передаются как хранятся в ОЗУ
uint16_t GetHoldingRegisters(uint16_t adr);
// Записывает значение beVal регистра хранения по адресу adr. beVal передается в формате, как в ModBus
// Возвращает: 0-успех; 1 - запись неудачная;
uint16_t SetHoldingRegisters(uint16_t adr, uint16_t beVal);

extern TPntFlag PntFlag;

#ifdef __cplusplus
}
#endif

#endif
