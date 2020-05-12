// Файл определения структуры параметров устройства СЕНС
#ifndef SENSDEV_H
#define SENSDEV_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l431xx.h"
#include "SensProc.h"
#include "sens_types.h"

//#include "CommonDev.h"

////////////// Общие параметры
extern uint8_t ProgNum[3];
extern uint16_t errcode;    // Код ошибки
extern float Addr;          // Адрес устройства
extern const float cnNumTPnt[3];    // Количество калибровочных точек
extern float NumTPnt;
// Считывает настройки из внешней памяти в ОЗУ (в HardDSt) с проверкой CRC
// Возвращает 0 если все считалось нормально
uint8_t ReadConst(void);
uint8_t ReadTCTab(void);

void TCPProc(void);  // Вызывать постоянно в основном цикле
void FlashCopyProc(void); // Вызывать каждый GetNC

void HardSaveConst(uint8_t *Flag); // Сохраняет настройки

uint8_t IsEmuMode(void);  // Возвращает 1 если включен режим эмуляции

void SetProgNum(uint32_t pgn); // Устанавливает номер программы для возврата по линии СЕНС

uint8_t CheckEizm(uint8_t val);

extern uint32_t emumode; // Время включения режима эмуляции или 0 если выключен
void EmulMode(uint8_t state);
void ReinitMemory(void);

//extern MySettings HardDSt;
//extern MySettings DSt;

#ifdef __cplusplus
}
#endif

#endif
