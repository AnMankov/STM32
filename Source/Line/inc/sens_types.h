#ifndef __SENS_TYPES_H
#define __SENS_TYPES_H

/* Типы и константы, связанные с Логическим уровнем Линии 
*/

#include <stdint.h>

////// Флаги
#define OPT_UART 0x80	// используется UART
#define SAVEDATA 0x20   // Сохранить настройки в память 
#define SAVETC   0x10	// Сохранить таблицу термокомпенсации

//// тип таблица давления на соответствующей температуре 
//typedef __packed struct
//{
//	uint16_t TC;			// температура кривой, в кодах АЦП
//	int16_t TCst;			// температура кривой эталонная, в 1/10 градусах Цельсия
//	uint16_t AC[11]; 	// давление в кодах АЦП
//} TPTTab;

//// тип таблица термокомпенсации
//typedef __packed struct
//{
//	float P0;      // начальное давление
//	float Pdlt;    // шаг давления	
//	TPTTab pt[10];	// сами таблицы давления
//} TTCTab;

typedef enum
{
  _N_OPENED = 0, //разомкнуто
  _N_CLOSED = 1, //замкнуто
  
  _N_MAX    = _N_CLOSED + 1,
} TContact;

typedef enum
{
  __BASE         = 0U,
  __OTHER_MASTER = 1U, 
} TInterconn;

typedef __packed struct
{
  uint8_t Thr;            //порог срабатывания по углу
  uint8_t Hyst;           //гистерезис
  int8_t  Bias;           //смещение
  uint8_t AxisRotate;     //поворот оси базы (на плате базы) / датчика (на плате датчика)
  uint8_t SensAxisRotate; //поворот оси датчика (на плате базы) / не используется (на плате датчика)
  int8_t  RollBiasAngle;  //смещения крена
  int8_t  PitchBiasAngle; //смещения тангажа
} TDevSets;

typedef __packed union TBits
{
  __packed struct
  {
    TContact   Normal    : 1; //0 - НР; 1 - НЗ
    TInterconn Interconn : 1; //тип взаимодействия Датчика (с Базой или с другим мастером)
    uint8_t    Reserved1 : 6;   
  } Item;
  uint8_t Byte;
} TBits;

typedef __packed struct
{
  TBits   Bits;
  uint8_t Reserved2;
  uint8_t Reserved3;
} T_ADPBITS;

typedef __packed struct
{
  uint8_t MBAddr; //адрес Modbus
  uint8_t USpeed; //скорость 
  uint8_t UPar;   //четность и количество стоп битов
} TIf;

typedef __packed struct 
{
/* Обязательные
*/
  uint8_t Addr;        //адрес устройства по протоколу СЕНС
  float   PswAdmin;    //пароль администратора
  float   PswSuper;    //пароль суперадминистратора
  
/* Настройки устройства
*/
  TDevSets DevSets;    //в данной версии к базе может быть подключен 1 датчик
  T_ADPBITS AdpBits;
  
/* Настройки интерфейсов
*/
  TIf If;
} MySettings;

// Результаты
typedef struct 
{
/* транслируемые
*/
  float HC1_Angle;       //угол, на который открыта крышка
  float HC2_Angle;       //угол, на который открыт замок
  float SysStatus;       //
  float PTemper;         //температура кристалла датчика на платформе

/* скрытые и служебные
*/
  float srab;            //номер параметра, вызвавшего срабатывание	EPRM
  uint32_t errcode;      //код  ошибки ERRCODE
  
} CurData;

typedef struct
{
  uint8_t Len;
  uint8_t Addr;
  uint8_t Cmd;	 
} TPre;                  //Общие поля для пакета запроса и пакета ответа

typedef struct
{
  TPre    Pre;
  uint8_t CalCmd; //номер калибровочной команды
} TDataReq;

typedef struct
{
  TPre Pre;
  uint8_t Res; //результат выполнения запроса
  uint8_t CalCmd; //номер калибровочной команды
} TDataAns;

enum TCalResult
{
  _DENY       = 0x00, //отказ в выполнении
  _SLOW       = 0x55, //медленное выполнение 
  _SUCCESS    = 0x5A, //успешное выполнение
  _PARAMS_REQ = 0x20  //запрос параметров для выполнения
};

enum TPntFlag
{
  _FAILURE     =   0, //отказ в выполнении
  _PERFORMING  =  85, //идет выполнение
  _DONE        =  90, //выполнено
  _NO_CMD_EXEC =  99, //команды калибровки не выполнялись с включения датчика
  _IMPOSSIBLE  = 255, //невозможная ситтуация
  _REQ         =   7, //запрос дополнительных параметров
  _INTERM      =   1, //промежуточное состояние
  _UNKNOWN     =   8  //!неизвестно откуда взялось это значение и для чего оно
};

enum TCalCmd
{
  _PLATFORM_CAL = 0x01,
  _HC_1_CAL     = 0x02,
  _HC_2_CAL     = 0x03  
};

extern MySettings HardDSt;
extern MySettings DSt;
extern const MySettings DefDSt;

extern CurData cdt;

extern uint8_t MainFlags;

void SetMainFlag(uint8_t Flag); // Устанавливает "основные флаги"

#endif
