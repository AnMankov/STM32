#ifndef SENSOR_H
#define SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

//#include "main.h"
#include <stdint.h>

#define ED_IZM_TBL_SIZE			8
#define MAX_IDX_ED_IZM_TBL	(ED_IZM_TBL_SIZE - 1)

#define MAXSWIM 5

#define MAXOPDT 800

// состояния автомата в SensStep (работа с АЦП)
#define ADCOFF    0					// автомат измерения отключен
#define START     1					// вход для перекалибровки и переходу к измерению
#define START_A   2					// калибровка смещения
#define START_B   3					// калибровка масштаба
#define START_C   4					// запуск измерения
#define GETDATA   5					// непрерывное получение данных

// вход для измерения температуры	
#define TEMPIZM_A 40				// запись в регистр IO
#define TEMPIZM_B 41				// запись в регистр CONFIG
#define TEMPIZM_C 42 				// самокалибровка смещения нуля
#define TEMPIZM_D 43				// калибровка масштаба
#define TEMPIZM_E 44 				// запуск одного измерения температуры
#define TEMPIZM_F 45 				// сохранение результата
#define PRESTART  46 				// запись в регистр IO при измерении давления

// состояния автомата в CalcProc (обработка измерений)
#define WAIT_NEWREZ   0			// ожидание новых данных
#define CALC_CURPRESS 1			// расчет текущего давления
#define CALC_DAMPING  2			// демпфирование результата
#define CALC_OUTPARAM 3			// формирование выходных параметров
#define CALC_CRITICAL 4			// формирование критических уровней

#define NUMKOEFF 5

// коды ошибок
#define ERR_CODE_01  0x0001		// ошибка измерения давления
#define ERR_CODE_02  0x0002		// ошибка измерения температуры
#define ERR_CODE_04  0x0004		// ошибка единиц измерения
#define ERR_CODE_08  0x0008		// ошибка установки дипапзона давления
//#define ERR_CODE_80  0x80		// ошибка при самотестировании EEPROM

extern uint8_t gds;			// состояние автомата в SensStep

///// Настройки измерения
// Алгоритмы измерения:
// 0 - старый, с опорным магнитом
// 1 - базовый, с двойным отражением
// 3 - улучшенный, с уменьшенным верхним неизмеряемым уровнем
extern volatile uint8_t itype;           // Алгоритм расчета и измерения
extern volatile uint8_t invert;          // Флаг перевернутого датчика
extern volatile uint32_t swimnum;         // Число поплавков в сигнале
///// Результат измерения
// Времена импульсов поплавков, начиная сверху
extern volatile uint32_t times[MAXSWIM];  // Времена импульсов, нс
extern volatile uint32_t curbasetime;     // Измеренное базовое время, нс
extern volatile uint32_t dtime;           // Время начала измерения. 
#define IERROK        0   // измерение успешно выполнено
#define IERRSWIMNUM   2   // неверное значение SWIMNUM (0 или более максимума)
#define IERRLOWIMPNUM 4   // измерение успешно выполнено
#define IERROPTIMEERR 8   // Разница расчетных опорных времен для разных импульсов
#define IERRORTYPE    16  // Ошибочный вариант алгоритма
#define IERRLSTOPTIME 32  // Разница с предыдущим опорным временем слишком велика
#define IWAITSTART    65536  // Ожидание перезапуска
extern volatile uint32_t izmflag;         // Флаг состояния измерения
//
extern volatile float icels[8];  // Измеренные температуры
//
extern volatile uint8_t CelsDatOk; // Флаг наличия данных о температуре

void ConfigIZM(void); // Конфигурация аппаратуры - выполнять до __enable_irq
void InitTermo(void); // Конфигурация термодатчиков - выполнять после __enable_irq

// Вызывать всегда. 
// Когда результат будет готов - установится значение dtime
// Также значение izmflag установится в код от 0 до 32 в зависимости от результата
void SensProc(void);  
// Вызывать для проверки, можно ли использовать fasti2c в основной программе
// 0 - использовать нельзя
// 1 - использовать можно
// Связано с изменением резисторов масштаба в процессе измерения
char CanFastI2C(void); 


uint8_t SetBaseTime(void); // Установка базового времени

float GetHeadCels(void); // Получить температуру головы датчика в градусах

uint8_t GetReadOnly(void);   // Возвращает 1 если запись запрещена


extern float temperatureC;

//////////////////////////////////////// Переключает делители между HCLK и PCLK1
// fast=1 - частота ядра максимальна, делитель включен на PCLK1
// fast=0 - частота ядра минимальна, делитель включен на HCLK
void SetClockState(uint8_t fast);
uint8_t CurSpeedHi(void);         // Возвращает 1 если частота ядра максимальна


// Таблицы
#define TEMPADDR 104 // таблица термокомпенсации
#define GISTADDR 440 // таблица гистерезисов
#define LTADDR   35  // таблица критических уровней

#define erddouble(x)  (*((float*)x))
#define erdint(x)			(*((uint16_t *)x))

// Работа с таблицей термокомпенсации :
// чтение начального давления P0 (4 байта)
#define P0_TempComp (erddouble(TEMPADDR))  
// чтение шага давления Pdlt (4 байта)
#define Pdlt_TempComp (erddouble(TEMPADDR+4))
// чтение температуры кривой (в кодах АЦП) TC(i) (2 байта) в i строке
#define TC(i) ((uint16_t/*long*/)(erdint((TEMPADDR+8)+(i)*26)))
// чтение температуры кривой (по эталонному термометру) TCst(i) (2 байта в i строке
#define TCst(i) ((uint16_t)(erdint((TEMPADDR+10)+(i)*26)))
// чтение n-ого значения (n=0..11) кода АЦП AC(i) в i строке
//#define AC(i,n) ((/*unsigned int*/ long)(erdint((TEMPADDR+12)+(n)*2+(i)*26)))
//unsigned int AC(char i, char n);


uint8_t StartSensor(void);
void SensStep(void);
int CalcProc(void);

uint8_t GetEizm(float val);

int cal_01(float *val);
int cal_02(float *val);

void EIzmProc(void);

//char SensAnswerPkg(uint8_t * buf);       // Функция отвечает на пакет СЕНС

#ifdef __cplusplus
}
#endif

#endif
