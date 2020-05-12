#ifndef __SETTINGS_H
#define __SETTINGS_H

#pragma anon_unions

#include <stdint.h>

#include "sens_types.h"

namespace CONSTS
{
  constexpr int16_t MIN_DEV_ANGLE        = -80;
  constexpr int16_t MAX_DEV_ANGLE        = 260;
                                         
  constexpr int16_t MAX_BASE_ANGLE       =  70; //максимальный подъем платформы, на которой находится база
  constexpr int16_t MIN_BASE_ANGLE       = -70; //максимальный спуск платформы, на которой находится база
  constexpr int16_t HYST_BASE_ANGLE      =  10;
                                         
  constexpr int16_t THR_ERR_OPEN_ANGLE   = -15;
  constexpr int16_t HYST_ERR_OPEN_ANGLE  =   5;
  
  constexpr int16_t THR_SENS_OPEN_ANGLE  =  70;
  constexpr int16_t HYST_SENS_OPEN_ANGLE =  10;
  
  constexpr float   DAMP_COEFF           =   2.0f;
}


class TSettings
{
public:
  
  enum TLedMode : uint8_t
  {
    __CONST_OFF = 0,
    __CONST_ON  = 1,
    __BLINK_1_S = 2,
  };

  enum TPress : uint8_t
  {
    __RELEASE                 = 0, //не нажата
    __START_HOLD              = 1, //начало удержания в нажатом состоянии(время на антидребезг прошло)
    __HOLD_EQ_OR_MORE_1_SEC   = 2, //удержание более 1 секунды
    __HOLD_EQ_OR_MORE_3_5_SEC = 3, //удержание более 3,5 секунд
    __RELEASE_LESS_1_SEC      = 4, //отпущена при времени нажатия <= 1 сек (короткое нажатие)
    __RELEASE_LESS_3_5_SEC    = 5, //отпущена при времени нажатия > 1 сек, но <= 3,5 сек (среднее нажатие)
    __RELEASE_MORE_3_5_SEC    = 6, //отпущена при времени нажатия > 3,5 сек (длинное нажатие)
    
    __MAX_PRESS               = __RELEASE_MORE_3_5_SEC + 1U,
  };
	
  enum TAccess : uint8_t
  {
    __USER       = 0U,
    __ADMIN      = 1U,
    __SUPER      = 2U,
    
    __ACCESS_MAX = __SUPER + 1U,
  };

  enum TAxisRotate : uint8_t
  {
    __0_DEG      = 0,
    __90_DEG     = 1,
    __180_DEG    = 2,
    __270_DEG    = 3,
    
    __MAX_ROTATE = __270_DEG + 1,
  };

  enum TDevType : uint8_t
  {
    _BASE      = 0, //базовый блок (всегда в паре с крышкой)
    _HC        = 1, //датчик на крышке (может быть один без базового блока)
    _UNDEFINED = 2, //неустойчивое состояние на выводе микроконтроллера (Пр. непропай)

		_MAX       = _HC + 1,
  };

  enum TConnect : bool
  {
    _NOT = 0,
    _YES = 1,
  };

  enum TPosErr : bool
  {
    _POS_ERR = 0,
    _POS_OK  = 1,
  };
  
  enum THC : bool
  {
    _CLOSED = 0,
    _OPENED = 1,
    
    _HC_MAX = _OPENED + 1,
  };
  
  enum TCalibProcess : uint8_t
  {
    __DENY            = 10U, //отказ в выполнении
    __PERFORMING      = 85U, //идет выполнение
    __PERFORMED       = 90U, //выполнено
    __NEVER_PERFORMED = 99U, //калибровка не выполнялась с момента включения
  };
	
	enum TCalib : bool
	{
	  __UNCALIBRATED = 0U,
		__CALIBRATED   = 1U,
	};
	
	enum TMainState : bool
	{
		__ERR = 0U,
	  __OK  = 1U,
	};
	
	__packed union TState
	{
    __packed struct
    {
      THC        HC            : 1; //Состояние крышки ( 0-закрыта; 1-открыта )                            (на датчике - состояние самой крышки; \
                                                                                                            на базе - результирующее состояние)
      TPosErr    HC_PosErr     : 1; //Положение крышки ( 0-ошибка положения; 1-нет ошибки положения )
      TPosErr    Base_PosErr   : 1; //Положение базы ( 0-ошибка положения; 1-нет ошибки положения )        ( только для базы )
      TConnect   Connect       : 1; //Связь с датчиком на крышке ( 0-нет; 1-есть )                         ( только для базы )
      TCalib     AccCalib      : 1; //Калибровка акселерометра на плате, где исполняется программа ( 0-не калиброван; 1-калиброван )
      TCalib     HC_AccCalib   : 1; //Калибровка акселерометра на крышке ( 0-не калиброван; 1-калиброван ) ( для базы )
      TMainState HC_Mems       : 1; //Состояние МЭМС на крышке ( 0-какая-то ошибка; 1-работает )
      TMainState Base_Mems     : 1; //Состояние МЭМС на базе ( 0-какая-то ошибка; 1-работает )
    };
    uint8_t Total;
	};

  enum TValidSign : bool
  {
    __INVALID = 0U,
    __VALID   = 1U,
  };
  
  enum TMeas : bool
  {
    __STOP_MEAS  = 0U,
    __START_MEAS = 1U,
  };
  
  __packed union TCommunicate
  {
    uint8_t Val;
    __packed struct
    {
      TValidSign SampleValidSign : 1; //валидность текущего окна выборок ( 0 - invalid; 1 - valid )
      TMeas StartMeasCmd         : 1; //команда на запуск нового окна измерений от базы к датчику
      uint8_t Reserved           : 6;
    };
  };
  
  __packed struct TSens
  {
    uint8_t    CodeSw;          //данные с кодового переключателя на крышке ( только для базы )
    uint16_t   ProgNbr;         //версия ПО с крышки ( только для базы )
    int16_t    Angle;           //угол платы датчика ( только для базы )
    TValidSign SampleValidSign; //валидность текущего окна выборок ( 0 - invalid; 1 - valid )
//    uint8_t    AxisRotate;      //поворот оси
    uint8_t    BufAxisRotate;   //буфер поворота оси
    int8_t     RollBiasAngle;   //заводская настройка смещения крена (только для базы)
    int8_t     PitchBiasAngle;  //заводская настройка смещения тангажа (только для базы)
  };

  __packed struct TDevData
  {
    uint8_t       CodeSw;           //данные с кодового переключателя платы, на которой выполняется программа
    TSens         Sens;             //данные базы, полученные с платы датчика
	  float         OpenAngle;        //Результирующий угол, (на датчике не используется)
		TState        State;            //Состояние устройства              
		int16_t       MyAngle;          //собственный угол платы
    int8_t        RawRoll;          //сырое значение крена
    int8_t        RawPitch;         //сырое значение тангажа
    float         AccelZ;           //значение по оси Z
    TCalibProcess CalibProcess;     //результат выполнения команды калибровки
    uint32_t      CalibPositionCtr; //счетчик калибровочных положений
    float         CalibParameter;   //параметр калибровки
    uint8_t       TransactSign;     //признак обмена данными с другой платой
		TAccess       Access;           //права доступа к данным устройства
    const void   *AuthorizeItemPtr; //флаг ожидания пароля
    TCommunicate  Communicate;      //команды от базы к датчику
    float         PdPressure;       //давление от СЕНС ПД (тест)
  };
  
  enum TIsFlashProg : uint8_t
  {
    _PROGRAMMED   = 0x00,
    _UNPROGRAMMED = 0xFF,
  };

  enum TIsAccCalib : uint8_t
  {
    _ACC_CALIBRATED   = 0x00,
    _ACC_UNCALIBRATED = 0xFF,
  };
  
  enum TBootModeFlag : uint16_t
  {
    __WORK_MODE = 0xFFFF,
    __BOOT_MODE = 0xA55A,
  };

  __packed struct TAccelData
  {
    float X;                    //пересчитанное значение ускорения по оси X
    float Y;                    //пересчитанное значение ускорения по оси Y
    float Z;                    //пересчитанное значение ускорения по оси Z
  };                            
                                
  __packed struct TAccCalData            
  {
    TAccelData  Offset;         //Смещения по осям в g
    TAccelData  Gain;           //Scale Factor по осям
  };

  __packed struct TFlashData //настройки, хранящиеся во Flash-памяти микроконтроллера
  {
    TBootModeFlag BootModeFlag; //флаг нахождения в загрузчике
    TIsFlashProg  IsFlashProg;  //признак программирования Flash   ( 0x00 - программировалась; 0xFF - не программировалась )
    TIsAccCalib   IsAccCalib;   //признак калибровки акселерометра ( 0x00 - калибровался;      0xFF - не калибровался )
    MySettings    AllSets;      //все настройки устройства
    TAccCalData   AccCalData;   //калибровочные данные акселерометра
  };

  struct TRange
  {
    int16_t Min;
    int16_t Max;
  };

public:
  TSettings();
  ~TSettings();

  void chk_dev( MySettings & );
//  void chk_if(
//              uint8_t BaudRateNbr,
//              uint8_t ParityAndStopsNbr
//             );
             
  bool chk_thr( int16_t ) const;              //проверка порога срабатывания по углу открытия
  bool chk_hyst( int16_t ) const;             //проверка гистерезиса порога срабатывания по углу открытия
  bool chk_bias( int16_t ) const;             //проверка смещения по углу крышки
  bool chk_axis_rotate( int16_t ) const;      //проверка поворота оси наклона
  bool chk_roll_bias_angle( int16_t ) const;  //проверка смещения крена
  bool chk_pitch_bias_angle( int16_t ) const; //проверка смещения тангажа
  bool chk_mb_addr( int16_t ) const;          //проверка Modbus-адреса
  bool chk_u_baud_rate( int16_t ) const;      //проверка скорости обмена данными через интерфейс USART
  bool chk_u_par( int16_t ) const;            //проверка четности и стоп-битов при обмене данными через интерфейс USART
  bool chk_calib( int16_t ) const;            //проверка команды калибровки
  bool chk_access( int16_t ) const;           //проверка команды смены режима доступа
protected:
  const TRange  __THR;              //допустимый диапазон для порога срабатывания по углу открытия
  const TRange  __HYST;             //допустимый диапазон для гистерезиса порога срабатывания по углу открытия
  const TRange  __BIAS;             //допустимый диапазон для смещения по углу крышки
  const TRange  __AXIS_ROTATE;      //допустимый диапазон для поворота оси
  const TRange  __ROLL_BIAS_ANGLE;  //допустимый диапазон для угла смещения крена
  const TRange  __PITCH_BIAS_ANGLE; //допустимый диапазон для угла смещения тангажа
  const int16_t __LOW_DIF;          //минимальное значение для возврата в состояние "закрыта"
  const TRange  __MB_ADDR;          //допустимый диапазон для Modbus-адресов
  const TRange  __U_SPEED;          //допустимый диапазон для скорости обмена данными через интерфейс USART
  const TRange  __U_PAR;            //допустимый диапазон для четности и стоп-битов при обмене данными через интерфейс USART
  const TRange  __CALIB;            //допустимый диапазон для команд калибровки
  const TRange  __ACCESS;           //допустимый диапазон для команд уровня доступа
private:

  template< typename T >
  bool chk_param( const TRange &Range, T Param ) const //true => параметр находится в указанном диапазоне
  {
    return (
            Param >= Range.Min
            &&
            Param <= Range.Max
           );
  }

};

#endif //__SETTINGS_H
