#ifndef __DATA_TYPES_H
#define __DATA_TYPES_H

#pragma anon_unions
//******************************************************************************
//  Секция include: здесь подключаются заголовочные файлы используемых модулей
//******************************************************************************

#include <cstdint>
#include "lib.h"

#include "sens_types.h"
#include "Relay.h"

using std::uint32_t;
using std::int32_t;
using std::uint8_t;

//******************************************************************************
//  Секция определения типов
//******************************************************************************

enum TIsMagCalib : uint8_t
{
  _MAG_CALIBRATED   = 0x00,
  _MAG_UNCALIBRATED = 0xFF,
};

// Расширенный пакет состояний
enum TLineSync : uint8_t
{
  _BLOCK_GENERATE  = 0,
  _PERMIT_GENERATE = 0x80
};

enum TLineMasterChange
{
  _MASTERS_END_POLL = 0,
  _MASTERS_CONTINUOUS_POLL
};

enum TLineDirection : uint8_t
{
  _REQUEST = 0,
  _RESPONSE,
};

enum TLineMode : uint8_t
{
  _LINE_NORMAL = 0,
  _LINE_EMULATION
};

__packed struct TGyroData
{
  float X;                    //пересчитанное значение угловой скорости по оси X
  float Y;                    //пересчитанное значение угловой скорости по оси Y
  float Z;                    //пересчитанное значение угловой скорости по оси Z
};

__packed struct TMagData
{
  float X;                   //пересчитанное значение плотости магнитного потока по оси X			
  float Y;                   //пересчитанное значение плотости магнитного потока по оси Y			
  float Z;                   //пересчитанное значение плотости магнитного потока по оси Z
};

__packed struct THyst
{
  int16_t Low;
  int16_t High;
};

__packed struct TThr
{
  THyst LowThr;
  THyst HighThr;
};
  
constexpr TThr _PLATFORM_ANGLE = //пороги определения валидности угла для платформы
{
  { 50,  60}, //LowThr.Low, LowThr.High
  {-50, -60}, //HighThr.Low, HighThr.High
//  { 210, 220}, //LowThr.Low, LowThr.High
//  {-50, -60}, //HighThr.Low, HighThr.High
};

constexpr TThr _HC_ANGLE = //пороги определения валидности угла для крышки люка
{
  {-65, -75}, //LowThr.Low, LowThr.High
  {240, 250}, //HighThr.Low, HighThr.High
};

constexpr THyst _OPEN_HYST = //пороги для определения валидности угла открытия
{
  10, 15 //Low, High
};

enum TLineSendMoment : uint8_t
{
  _IMMEDIATELY    = 0U,
  _SYNC_WITH_LINE = 255U
};

enum TRangeDevice : uint8_t
{
  _UNDER = 0U, //ниже диапазона
  _RANGE,      //в диапазоне изменения состояния (диапазон = величине гистерезиса)
  _OVER,       //выше дипазона
};


enum class TMagCalSt : uint8_t
{
  _NO       = 0U, //калибровка не завершена или не запускалась
  _COMPLETE = 1U  //калибровка запускалась и была завершена
};

enum class TAccCalSt : uint8_t
{
  _NO       = 0U, //калибровка не завершена или не запускалась
  _COMPLETE = 1U  //калибровка запускалась и была завершена
};

enum class TCanDataReq : uint8_t
{
  _NO  = 0U,
  _YES = 1U,
};

__packed struct TCanExt
{
  __packed union
  {
    __packed struct
	  {
	    uint8_t HcCalib  : 1;
	    uint8_t DataReq  : 1;
		  uint8_t Reserved : 6;
	  };
	  uint8_t Reg;
  };
  float Angle;
  uint32_t Free;
};

__packed struct TDevState
{
  uint8_t Dev    :2; //флаг состояния устройства
  uint8_t MagCal :1; //флаг калибровки магнитометра
};

enum class TCanCmd : uint8_t
{
  START_MEASURE   = 0x77,
  START_ACC_CALIB = 0x5A,
};

enum class TMThrFlag : uint8_t
{
  _NO  = 0,
  _YES = 1, //порог превышен
};

__packed union TCanData
{
  __packed struct //данные, передаваемые платформой
  {
    TCanCmd Cmd;
    uint8_t  Reserved1;
    uint16_t Reserved2;
    uint32_t Reserved3;
  } TxPlatf;
  __packed struct  //данные, передаваемые крышкой
  {
    int16_t    Angle;
    TMThrFlag ThrFlag    : 1; //флаг превышения порога срабатывания для акселерометра
    TAccCalSt  AccCalSt   : 1; //флаг калибровки акселерометра
    uint8_t    Reserved1  : 6;
    uint8_t    Reserved2;
    uint32_t   Reserved3;
  } TxHC;
};

enum TDevice : uint8_t 
{
  _HC_1    = 0, //крышка люка 1
  _HC_2    = 1, //крышка люка 2
  _HC_3    = 2, //крышка люка 3
  _HC_4    = 3, //крышка люка 4
  _HC_5    = 4, //крышка люка 5
  _HC_6    = 5, //крышка люка 6
  _HC_7    = 6, //крышка люка 7
  _HC_8    = 7, //крышка люка 8
           
  _PLATF   = 8, //платформа
  _MAX_DEV = _PLATF,
};

enum TSwCal : uint8_t
{
  _STOP_CALIB  = 0U,
  _START_CALIB = 1U
};  

__packed struct TResults
{
  int32_t W;
  int32_t X;
  int32_t Y;
  int32_t Z;
  int32_t Heading;
  int32_t Pitch;
  int32_t Roll;
  int32_t Yaw;
};

struct TPinHw
{
  GPIO_TypeDef *GPIOx;  //порт
  unsigned int ClkMask; //маска для разрешения тактирования
  unsigned int PinMask; //вывод
  char *Sign;           //подпись
};

enum class TProc : uint8_t
{
  _AUTO = 0,      //автоопределение
  _MODBUS,
  _SENS,
  _OMNICOMM,
  _DKS_TO_PC,
};

enum class TRelayNum : uint8_t
{
  RELAY_ONE   = 0,
  RELAY_TWO   = 1,
  RELAY_THREE = 2,
  RELAY_FOUR  = 3,
  
  MAX_RELAY
};

typedef void (*T_SWITCH)(void);

__packed struct TQuaternion
{
  float q0;    //Qw
  float q1;    //Qx
  float q2;    //Qy
  float q3;    //Qz
};                                        //данные кватерниона 

__packed struct TAngles
{
  float Yaw;   //рыскание
  float Pitch; //тангаж
  float Roll;  //крен
};                                        //данные углов Эйлера

__packed struct TGravity
{
  float X;
  float Y;
  float Z;
};                                        //составляющие вектора гравитации

__packed struct TLinearAccel
{
  float X;
  float Y;
  float Z;
};                                        //данные линейного ускорения

__packed struct TDampMems
{
  float GyroX_Damp;
  float GyroY_Damp;
  float GyroZ_Damp;
  float AccelX_Damp; 
  float AccelY_Damp; 
  float AccelZ_Damp; 	
};

__packed struct TAngleThreshold
{
  uint16_t LowAngleThreshold;           //нижний  порог срабатывания по углу в градусах
  uint16_t HighAngleThreshold;          //верхний порог срабатывания по углу в градусах
};

__packed struct TMagneticThreshold
{
  int32_t LowMagneticThreshold;         //нижний  порог срабатывания по величине плотности магнитного потока
  int32_t HighMagneticThreshold;        //верхний порог срабатывания по величине плотности магнитного потока
};

__packed struct TAccGyroAccum
{
  int32_t MeasureWx;                    //аккумулятор данных с гироскопа по оси X 					
  int32_t MeasureWy;                    //аккумулятор данных с гироскопа по оси Y 						
  int32_t MeasureWz;                    //аккумулятор данных с гироскопа по оси Z 
     
  int32_t MeasureAx;                    //аккумулятор данных с акселерометра по оси X 		
  int32_t MeasureAy;                    //аккумулятор данных с акселерометра по оси Y 			
  int32_t MeasureAz;                    //аккумулятор данных с акселерометра по оси Z 
};
__packed struct TRawDataGyro
{
  int16_t MeasureWx;                    //данные с гироскопа по оси X 					
  int16_t MeasureWy;                    //данные с гироскопа по оси Y 						
  int16_t MeasureWz;                    //данные с гироскопа по оси Z
};

__packed struct TRawDataAccGyro
{
  int16_t MeasureWx;                    //данные с гироскопа по оси X 					
  int16_t MeasureWy;                    //данные с гироскопа по оси Y 						
  int16_t MeasureWz;                    //данные с гироскопа по оси Z 
  
  int16_t MeasureAx;                    //данные с акселерометра по оси X 		
  int16_t MeasureAy;                    //данные с акселерометра по оси Y 			
  int16_t MeasureAz;                    //данные с акселерометра по оси Z 
};

//__packed struct TAccGyroData
//{
//  TGyroData GyroData;
//  TAccelData AccelData;
//};

//__packed struct TAccGyroMagData
//{
//  TGyroData GyroData;
//  TAccelData AccelData;
//  TMagData MagData;
//};

__packed struct TMotionFXData
{
  TAngles      Angles;                   //данные углов Эйлера без демпфирования
  TQuaternion  Quaternion;               //данные кватерниона без демпфирования
  TGravity     Gravity;                  //составляющие вектора гравитации
  TLinearAccel LinearAccel;              //данные линейного ускорения
  float        Heading;                  //направление магнитного севера
};

enum TDevConnectVal : uint8_t
{
  _CONNECTED     = 0,
  _NOT_CONNECTED = 1,
};

enum TCalib : uint8_t
{
  _CALIBRATED     = 0,
  _NOT_CALIBRATED = 1,
};

enum TQuadrant
{
  _Q_ONE = 0,
  _Q_TWO,
  _Q_THREE,
  _Q_FOUR,
};

enum TPlatfState : uint8_t
{
  _VALID   = 0,
  _INVALID = 1
};

__packed union TDevConnect
{
  __packed struct
  {
    uint8_t HC_1 :1; //_CONNECTED = 0, _NOT_CONNECTED = 1
    uint8_t HC_2 :1; //_CONNECTED = 0, _NOT_CONNECTED = 1
    uint8_t HC_3 :1; //_CONNECTED = 0, _NOT_CONNECTED = 1
    uint8_t HC_4 :1; //_CONNECTED = 0, _NOT_CONNECTED = 1
    uint8_t HC_5 :1; //_CONNECTED = 0, _NOT_CONNECTED = 1
    uint8_t HC_6 :1; //_CONNECTED = 0, _NOT_CONNECTED = 1
    uint8_t HC_7 :1; //_CONNECTED = 0, _NOT_CONNECTED = 1
    uint8_t HC_8 :1; //_CONNECTED = 0, _NOT_CONNECTED = 1
  };
  uint8_t Reg;
};

// Данные, отправляемые на ПК
//__packed struct TPcSendData
//{
//  TAccelData     AccelData;                          //данные акселерометра в g
//  TGyroData      GyroData;                           //данные гироскопа в dps
//  TMagData       MagData;                            //данные магнитометра в uT
//  float          Temperature;                        //температура с датчика
//  uint8_t        MagId;                              //идентификатор магнитометра
//  TMotionFXData  MotionFXData;                       //данные, полученные из библиотеки MotionFX
//  int16_t        OpenedAngle[TDevice::_MAX_DEV - 1];
//  TCanData       HatchCoverData;                     //данные, полученные от крышки люка
//  TCanData       LockData;                           //данные, полученные от замка
//  MySettings     AllSets;                            //настройки устройства 
//  TDevState      State[TDevice::_MAX_DEV];           //флаги состояния всех устройств
//  TDevConnect    DevConnect;                         //данные о связи датчиков устройств с платформой
//};

enum TRelayState : uint8_t
{
  _N_CLOSED   = 0, //нормально замкнуто
  _N_UNCLOSED,     //нормально разомкнуто
  
  _MAX_REL_STATE,
};

__packed struct TRelSet
{
  uint8_t RelOneFlag               :1; //флаг работы 1-го реле (false - нормально замкнуто, true - нормально разомкнуто)
  uint8_t RelTwoFlag               :1; //флаг работы 1-го реле (false - нормально замкнуто, true - нормально разомкнуто)
  uint8_t RelThreeFlag             :1; //флаг работы 1-го реле (false - нормально замкнуто, true - нормально разомкнуто)
  uint8_t RelFourFlag              :1; //флаг работы 1-го реле (false - нормально замкнуто, true - нормально разомкнуто)
};

// Данные, приходящие с ПК и хранимые на последней странице Flash
__packed struct TPcReceiveData                  
{
  TAngleThreshold AngleThrValues;               //верхний и нижний пороги срабатывания по углу
  TRelSet RelSet;
};

//__packed struct TMainData
//{
//  TRawDataAccGyro RawDataAccGyro;
//  TMagData        MagOffset;        //the hard iron offset array [µT]/50
//  TPcSendData     PcSendData;
//  TPcReceiveData  PcReceiveData;
//};

enum class TIsDamp : uint8_t
{
  _NO_DAMP,
  _YES_DAMP,
};
 
enum class TDampNum : uint8_t
{
  _PREV_DAMP,
  _CURRENT_DAMP,
};

enum class TMState : uint8_t
{
  _CLOSE        = 0,
  _OPEN         = 1,
  _ERR_POSITION = 2,
};

enum class TMConnect : uint8_t
{
  _NO  = 0,
  _YES = 1,
};

enum class TMSampleFlag : uint8_t
{
  _NO  = 0,
  _YES = 1,
};

__packed struct TModule
{
  int16_t      AccelAngle;      //угол, полученный от самой крышки, пересчитанный из данных от акселерометра
  TMState      State       : 2;
  TMConnect    Connect     : 1;
  TMThrFlag    ThrFlag     : 1; //флаг превышения порога срабатывания для акселерометра
  TMSampleFlag SampleFlag  : 1; //флаг получения от крышки текущей выборки
  TDevSets     Sets;            //настройки, применяемые к конкретному модулю
  float        OpenAngle;       //результирующий угол открытия крышки
  const relay::TRelay *pRelay;  //указатель на реле привязывается к крышке
};

enum class TPState : uint8_t
{
  _OK           = 0,
  _ERR_POSITION = 1,
};

//__packed struct TPlatf
//{
//  int16_t AccelAngle;
//  TAccelData AccelData;
//  TPState State         : 1;
//  TMThrFlag ThrFlag     : 1; //флаг превышения порога срабатывания для акселерометра
//};

//__packed struct TSys
//{
//  TPlatf  Platf; 
//  TModule HC[TDevice::_MAX_DEV];
//  uint8_t ConnectCtr[TDevice::_MAX_DEV];
//  float FreqZ;
////  TFlashSets Sets;
//};

__packed struct TSysStates
{
  uint8_t        Platform            :1; //флаг состояния плтаформы (TPlatformState)
  uint8_t        HatchCover          :2; //флаг состояния крышки (THatchCoverState)
  uint8_t        Lock                :2; //флаг состояния замка (TLockState)
  uint8_t        RxSetFlag           :1; //флаг успешного приема настроек (false - настройки не принимались или приняты с ошибкой; true - настройки приняты)
  uint8_t        MagCalibFlag        :1; //флаг калибровки магнитометра (0 - калиброван, 1 - не калиброван)
};

union TLineCmdCode
{
  struct
  {
    uint8_t CmdCode       : 5;
    uint8_t ChangeMasters : 1;
	  uint8_t EmulMode      : 1;
	  uint8_t Direction     : 1;
  };
  uint8_t Reg;  
};

struct TLineDataPkt
{
  uint8_t      Len;      //длина пакета LEN без учета данного байта
  uint8_t      Addr;     //адрес, согласно протокола СЕНС
  TLineCmdCode Cmd;      //команда, согласно протокола СЕНС, включая все служебные биты
  uint8_t      Data[61]; //данные пакета, DN=LEN-2
};

__packed union TStateByte
{
  __packed struct
  {
    TMConnect HC1Conn  : 1;
	  TMConnect HC2Conn  : 1;
	  TPState   PState   : 1;
	  TMState   HC1State : 2;
	  TMState   HC2State : 2;
	  uint8_t   Reserved : 1;
  };
  uint8_t Reg;
};

struct UnStdSostPkg //структура с кривыми именами оставлена для совместимости со структурой из других приборов СЕНС
{
  uint8_t      syncnum;  //служебные
  uint8_t      pause;    //служебные
  uint8_t      num;      //число передаваемых байт, не менее 3 (adr, comm, dat)
  uint8_t      adr;      //
  TLineCmdCode comm;     //0xA0
  TStateByte   dat;      //биты состояния
  uint8_t      data[61]; //остальные данные пакета (любые)
};

  // Обновление данных для отправки по линии
  //
  //----- поддержка типа S_PROGNUM --------------------------------------
  __packed union TSProgNum
  {
    __packed struct
    {
      __packed TStateByte	StByte; //младший байт кода ошибки повторяет байт состояния
      uint8_t ReservedByte2;
      uint8_t Unavailable1; //использовать нельзя (МСК)
      uint8_t Unavailable2; //использовать нельзя
    };
    __packed uint32_t Total;
  };
  //---------------------------------------------------------------------

//******************************************************************************
//  Секция определения глобальных переменных
//******************************************************************************

//******************************************************************************
//  Секция прототипов глобальных функций (declaration)
//******************************************************************************

#endif

//******************************************************************************
//  ENF OF FILE
//******************************************************************************
