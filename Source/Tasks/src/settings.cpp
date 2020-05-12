
#include <math.h>

#include "settings.h"
#include "main.h"
#include "rtos_headers.h"
#include "Flash_driver.h"
#include "SensProc.h"

TSettings Settings;

TSettings::TSettings()
:
__THR(               {  15,                                      50 } ),
__HYST(              {   5,                                      40 } ),
__BIAS(              { -15,                                      15 } ),
__AXIS_ROTATE(       { TAxisRotate::__0_DEG, TAxisRotate::__270_DEG } ),
__ROLL_BIAS_ANGLE(   {  -7,                                       7 } ),
__PITCH_BIAS_ANGLE(  {  -7,                                       7 } ),
__LOW_DIF( __THR.Min - __HYST.Min ),
__MB_ADDR(           {   1,                                     247 } ),
__U_SPEED(           {   0,                                       9 } ),
__U_PAR(             {   0,                                       3 } ),
//__CALIB(             {   0,                                       5 } ) //для калибровки по первичным данным акселерометра в g
__CALIB(             {   0,                                       1 } ), //для калибровки по углу
__ACCESS(            { 230,                                     232 } )  //для уровней доступа
{

}

TSettings::~TSettings()
{

}

bool TSettings::chk_thr( int16_t Param ) const
{
  return chk_param( __THR, Param );
}

bool TSettings::chk_hyst( int16_t Param ) const
{
  return chk_param( __HYST, Param );
}

bool TSettings::chk_bias( int16_t Param ) const  
{
  return chk_param( __BIAS, Param );
}

bool TSettings::chk_axis_rotate( int16_t Param ) const
{
  return chk_param( __AXIS_ROTATE, Param );
}

bool TSettings::chk_roll_bias_angle( int16_t Param ) const
{
  return chk_param( __ROLL_BIAS_ANGLE, Param );
}

bool TSettings::chk_pitch_bias_angle( int16_t Param ) const
{
  return chk_param( __PITCH_BIAS_ANGLE, Param );
}

bool TSettings::chk_mb_addr( int16_t Param ) const   
{
  return chk_param( __MB_ADDR, Param );
}

bool TSettings::chk_u_baud_rate( int16_t Param ) const  
{
  return chk_param( __U_SPEED, Param );
}

bool TSettings::chk_u_par( int16_t Param ) const  
{
  return chk_param( __U_PAR, Param );
}

void TSettings::chk_dev( MySettings &New )
{
//  New.DevSets.Thr = ( chk_param( __THR, New.DevSets.Thr ) )
////                    ? Model.get_thr()                             //восстановить предыдущее значение настройки
////                    : roundf( New.DevSets.Thr );                  //округлить новое значение

////  New.DevSets.Hyst = ( chk_param( __HYST, New.DevSets.Hyst ) )
////                     ? Model.get_hyst()                           //восстановить предыдущее значение настройки
////                     : roundf( New.DevSets.Hyst );                //округлить новое значение

  //дополнительная проверка для гистерезиса
  if ( New.DevSets.Thr - New.DevSets.Hyst < __LOW_DIF )        //если значение для возвата в состояние "закрыта" меньше минимального, ...
  {                                                               
    New.DevSets.Hyst = New.DevSets.Thr - __LOW_DIF;            //...то установить максимально допустимое значение гистерезиса для данного порога
  }

//  New.DevSets.Bias = ( chk_param( __BIAS, New.DevSets.Bias ) )
//                     ? Model.get_bias()                           //восстановить предыдущее значение настройки
//                     : roundf( New.DevSets.Bias );                //округлить новое значение
}

bool TSettings::chk_calib( int16_t Param ) const
{
  return ( chk_param( __CALIB, Param ) );
}

bool TSettings::chk_access( int16_t Param ) const
{
  return ( chk_param( __ACCESS, Param ) );
}

//void TSettings::chk_if( 
//                       uint8_t BaudRateNbr, 
//                       uint8_t ParityAndStopsNbr
//                      )
//{
//  constexpr uint8_t BAUD_RATE_DEF_VAL        = 5U;
//  constexpr uint8_t PARITY_AND_STOPS_DEF_VAL = 3U;
//    
//  bool ReinitFlag = false; //флаг необходимости повторной инициализации
//  
//  TBaudRate BaudRate[] =
//  {
//    TUsart::TBaudRate::_1200,   //0
//    TUsart::TBaudRate::_2400,   //1
//    TUsart::TBaudRate::_4800,   //2
//    TUsart::TBaudRate::_9600,   //3
//    TUsart::TBaudRate::_14400,  //4
//    TUsart::TBaudRate::_19200,  //5 - настройка по умолчанию
//    TUsart::TBaudRate::_38400,  //6
//    TUsart::TBaudRate::_56000,  //7
//    TUsart::TBaudRate::_57600,  //8
//    TUsart::TBaudRate::_115200, //9
//  };
//  
//  struct TParityAndStops
//  {
//    TUsart::TParity Parity;
//    TUsart::TStops  Stops;
//  };

//  TParityAndStops ParityAndStops[] =
//  {
//    { TUsart::TParity::_NONE, TUsart::TStops::_STOPBITS_1 }, //0
//    { TUsart::TParity::_NONE, TUsart::TStops::_STOPBITS_2 }, //1
//    { TUsart::TParity::_ODD,  TUsart::TStops::_STOPBITS_1 }, //2
//    { TUsart::TParity::_EVEN, TUsart::TStops::_STOPBITS_1 }, //3 - настройка по умолчанию
//  };

//  if ( chk_baud_rate_nbr( BaudRateNbr ) != true ) //если во Flash записано что-то не то,
//  {
//    Sets.BaudRate = BaudRate[BAUD_RATE_DEF_VAL];  //установить настройки по умолчанию
//  }
//  else
//  {
//    if ( Sets.BaudRate != BaudRate[BaudRateNbr] )
//    {
//      Sets.BaudRate = BaudRate[BaudRateNbr];
//      
//      ReinitFlag    = true;
//    }
//    else
//    {
//    
//    }
//  }

//  if ( chk_parity_and_stops_nbr( ParityAndStopsNbr ) != true ) //если во Flash записано что-то не то,
//  {
//    Sets.Parity = ParityAndStops[ PARITY_AND_STOPS_DEF_VAL ].Parity; //установить настройки по умолчанию
//    Sets.Stops  = ParityAndStops[ PARITY_AND_STOPS_DEF_VAL ].Stops;  //установить настройки по умолчанию
//  }
//  else
//  {
//    if (
//        Sets.Parity != ParityAndStops[ParityAndStopsNbr].Parity
//        ||
//        Sets.Stops != ParityAndStops[ParityAndStopsNbr].Stops
//       )
//    {
//      Sets.Parity = ParityAndStops[ParityAndStopsNbr].Parity;
//      Sets.Stops  = ParityAndStops[ParityAndStopsNbr].Stops;
//      
//      ReinitFlag  = true;
//    }
//    else
//    {
//    
//    }
//  }
//  
//  if ( ReinitFlag )
//  {
//    //отключить USART
//    //вызвать функцию hw_init();
//  }
//}
