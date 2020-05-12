
#include "model.h"
#include "Flash_driver.h"

TModel Model( MainMut );

const MySettings DefDSt = // Настройки текущие
{
/* Обязательные
*/
     1,    //Addr;
  1234,    //PswAdmin;
    31.41, //PswSuper;

/* Настройки устройства
*/
// порог, гистерезис, смещение  
  {25.0f, 10.0f,      0.0f}, //DevSets
  {
//   .Bits.Byte = 0x00, 
   _N_OPENED, //Нормально разомкнуто
   __BASE,    //Взаимодействие Датчик<->База
   
   0xFF,
   0x00
  },                      //ADPBITS

/* Настройки интерфейсов
*/
  {
    1, //MBAddr;            //адрес Modbus
    5, //USpeed;            //скорость 5 - 19200
    3, //UPar;              //четность и количество стоп битов: 3 - 8E1
  }
};

TModel::TModel( SemaphoreHandle_t &MainMut )
:
Mut( MainMut )
{
  TFlashData FlashData;
  
  uint16_t FLASH_DATA_SIZE = sizeof (TFlashData);
  
  Flash.get_sets(
                 reinterpret_cast<uint8_t *>(&FlashData),
                 FLASH_DATA_SIZE
                );
  //проверка признака программирования Flash
  if ( FlashData.IsFlashProg == TIsFlashProg::_PROGRAMMED ) //если Flash ранее программировалась
  {
    Main.FlashData = FlashData; //записать данные, хранящиеся во Flash в главный буфер программы
  }
  else
  {
    FlashData.IsFlashProg  = TIsFlashProg::_PROGRAMMED;
    FlashData.IsAccCalib   = TIsAccCalib::_ACC_UNCALIBRATED;
    FlashData.BootModeFlag = TBootModeFlag::__WORK_MODE;
    FlashData.AllSets      = DefDSt;                         //настройки по умолчанию
    
    TModel::TAccCalData Tmp =
    {
      { 0.0f, 0.0f, 0.0f },
      { 1.0f, 1.0f, 1.0f }
    };
    
    FlashData.AccCalData.Offset = Tmp.Offset;
    FlashData.AccCalData.Gain   = Tmp.Gain;
    
    Flash.write_sets(
                     reinterpret_cast<uint8_t *>(&FlashData),
                     FLASH_DATA_SIZE
                    );
                    
//    Main.FlashData = FlashData;
    
    Flash.get_sets(
                   reinterpret_cast<uint8_t *>(&Main.FlashData),
                   sizeof (TFlashData)
                  );
  }
  
  Main.DevData.CalibProcess     = TCalibProcess::__NEVER_PERFORMED;
  Main.DevData.AuthorizeItemPtr = nullptr;
}

TModel::~TModel()
{

}
  
//----- интерфейс к состоянию кнопки ------------------------
TModel::TPress TModel::get_btn_mode()
{
  TPress Dest;
  access_prot( Dest, Main.Press );
  
  return Dest;
}

void TModel::set_btn_mode( TPress Src )
{
  access_prot(  Main.Press, Src ); //записать в главный буфер в ОЗУ
}
//-----------------------------------------------------------

//----- интерфейс к режиму работы светодиода ----------------
TModel::TLedMode TModel::get_led_mode()
{
  TLedMode Dest;
  access_prot( Dest, Main.LedMode );
  
  return Dest;
}

void TModel::set_led_mode( TLedMode Src )
{
  access_prot(  Main.LedMode, Src ); //записать в главный буфер в ОЗУ
}
//-----------------------------------------------------------

//----- интерфейс к типу устройства -------------------------
TModel::TDevType TModel::get_dev_type()
{
  TDevType Dest;
  access_prot( Dest, Main.DevType );
  
  return Dest;
}
//-----------------------------------------------------------

//----- интерфейс к данным кодового переключателя на базе ---
uint8_t TModel::get_code_sw()
{
  uint8_t Dest;
  
  access_prot( Dest, Main.DevData.CodeSw );
  
  return Dest;
}

void TModel::set_code_sw( uint8_t Src )
{
  if (
      Src >= CodeRange.Min
      &&
      Src <= CodeRange.Max
     )
  {
    access_prot( Main.DevData.CodeSw, Src );
  }
  else
  {
    //Main.CodeSw остается без изменения
  }
}
//-----------------------------------------------------------

//----- интерфейс к данным кодового переключателя на крышке -
uint8_t TModel::get_hc_code_sw()
{
  uint8_t Dest;
  
  access_prot( Dest, Main.DevData.Sens.CodeSw );
  
  return Dest;
}

void TModel::set_hc_code_sw( uint8_t Src )
{
  if (
      Src >= CodeRange.Min
      &&
      Src <= CodeRange.Max
     )
  {
    access_prot( Main.DevData.Sens.CodeSw, Src );
  }
  else
  {
    //Main.CodeSw остается без изменения
  }
}
//-----------------------------------------------------------

//----- интерфейс к версия ПО с крышки ----------------------
uint16_t TModel::get_hc_prog_nbr()
{
  uint16_t Dest;  
  access_prot( Dest, Main.DevData.Sens.ProgNbr );
  
  return Dest;
}

void TModel::set_hc_prog_nbr( uint16_t Src )
{
  access_prot( Main.DevData.Sens.ProgNbr, Src );
}
//-----------------------------------------------------------
  
//----- интерфейс к результирующему углу --------------------
float TModel::get_open_angle()
{
  float Dest;  
  access_prot( Dest, Main.DevData.OpenAngle );
  
  return Dest;
}

void TModel::set_open_angle( float Src )
{
  access_prot( Main.DevData.OpenAngle, Src );
}
//-----------------------------------------------------------

//----- интерфейс к результирующему состоянию крышки --------
TModel::THC TModel::get_hc_state()
{
  TState Dest;
  access_prot( Dest, Main.DevData.State );
  
  return Dest.HC;
}

void TModel::set_hc_state( THC HC )
{
  TState Dest;
  
  access_prot( Dest, Main.DevData.State ); //чтение
  Dest.HC = HC;                            //модификация
  access_prot( Main.DevData.State, Dest ); //запись
}
//-----------------------------------------------------------

//----- интерфейс к ошибке положения крышки -----------------
TModel::TPosErr TModel::get_hc_pos_err()
{
  TState Dest;
  access_prot( Dest, Main.DevData.State );
  
  return Dest.HC_PosErr;
}

void TModel::set_hc_pos_err( TPosErr PosErr )
{
  TState Dest;
  
  access_prot( Dest, Main.DevData.State ); //чтение
  Dest.HC_PosErr = PosErr;                 //модификация
  access_prot( Main.DevData.State, Dest ); //запись
}
//-----------------------------------------------------------

//----- интерфейс к ошибке положения базы -------------------
TModel::TPosErr TModel::get_base_pos_err()
{
  TState Dest;
  access_prot( Dest, Main.DevData.State );
  
  return Dest.Base_PosErr;
}

void TModel::set_base_pos_err( TPosErr PosErr )
{
  TState Dest;
  
  access_prot( Dest, Main.DevData.State ); //чтение
  Dest.Base_PosErr = PosErr;               //модификация
  access_prot( Main.DevData.State, Dest ); //запись
}
//-----------------------------------------------------------

//----- интерфейс к связи с датчиком на крышке --------------
TModel::TConnect TModel::get_connect()
{
  TState Dest;
  access_prot( Dest, Main.DevData.State );
  
  return Dest.Connect;
}

void TModel::set_connect( TConnect Connect )
{
  TState Dest;
  
  access_prot( Dest, Main.DevData.State ); //чтение
  Dest.Connect = Connect;                  //модификация
  access_prot( Main.DevData.State, Dest ); //запись
}
//-----------------------------------------------------------
                                                          
//----- интерфейс к калибровке акселерометра на крышке ------
TModel::TCalib TModel::get_hc_acc_calib()
{
  TState Dest;
  access_prot( Dest, Main.DevData.State );
  
  return Dest.HC_AccCalib;
}

void TModel::set_hc_acc_calib( TCalib Calib )
{
  TState Dest;
  
  access_prot( Dest, Main.DevData.State ); //чтение
  Dest.HC_AccCalib = Calib;                //модификация
  access_prot( Main.DevData.State, Dest ); //запись
}
//-----------------------------------------------------------
																												
//----- интерфейс к калибровке акселерометра на базе --------
TModel::TCalib TModel::get_acc_calib()
{  
  bool Tmp    = !static_cast<bool>( get_accel_calib_sign() ); //получить признак калибровки, который хранится во флэш микроконтроллера
  TCalib Dest = static_cast<TCalib>(Tmp);
  set_acc_calib( Dest );
  
  return Dest;
}

void TModel::set_acc_calib( TCalib Calib )
{
  TState Dest;
  
  access_prot( Dest, Main.DevData.State ); //чтение
  Dest.AccCalib = Calib;                   //модификация
  access_prot( Main.DevData.State, Dest ); //запись
}
//-----------------------------------------------------------
                                                          
//----- интерфейс к состоянию МЭМС на крышке ----------------
TModel::TMainState TModel::get_hc_mems()
{
  TState Dest;
  access_prot( Dest, Main.DevData.State );
  
  return Dest.HC_Mems;
}

void TModel::set_hc_mems( TMainState NewState )
{
  TState Dest;
  
  access_prot( Dest, Main.DevData.State ); //чтение
  Dest.HC_Mems = NewState;                 //модификация
  access_prot( Main.DevData.State, Dest ); //запись
}
//-----------------------------------------------------------
                                                        
//----- интерфейс к состоянию МЭМС на базе ------------------
TModel::TMainState TModel::get_base_mems()
{
  TState Dest;
  access_prot( Dest, Main.DevData.State );
  
  return Dest.Base_Mems;
}

void TModel::set_base_mems( TMainState NewState )
{
  TState Dest;
  
  access_prot( Dest, Main.DevData.State ); //чтение
  Dest.Base_Mems = NewState;               //модификация
  access_prot( Main.DevData.State, Dest ); //запись
}
//-----------------------------------------------------------
                                                          
//----- интерфейс к состоянию устройства --------------------
uint8_t TModel::get_dev_state()
{
  TState Dest;
  access_prot( Dest, Main.DevData.State );
  
  return Dest.Total;
}              
//-----------------------------------------------------------

//----- интерфейс к собственному углу платы -----------------
int16_t TModel::get_my_angle()
{
  int16_t Dest;
  
  access_prot( Dest, Main.DevData.MyAngle );
  
  return Dest;
}

void TModel::set_my_angle( int16_t Src )
{
  access_prot(  Main.DevData.MyAngle, Src );
}
//-----------------------------------------------------------

//----- интерфейс к углу платы датчика ----------------------
int16_t TModel::get_sens_angle()
{
  int16_t Dest;
  
  access_prot( Dest, Main.DevData.Sens.Angle );
  
  return Dest;
}

void TModel::set_sens_angle( int16_t Src )
{
  access_prot(  Main.DevData.Sens.Angle, Src );
}
//-----------------------------------------------------------
                                                        
//----- интерфейс к валидности текущего окна выборок датчика
TModel::TValidSign TModel::get_sens_sample_valid_sign()
{
  TValidSign Dest;
  
  access_prot( Dest, Main.DevData.Sens.SampleValidSign );
  
  return Dest;
}

void TModel::set_sens_sample_valid_sign( TValidSign Src )
{
  access_prot(  Main.DevData.Sens.SampleValidSign, Src );
}
//-----------------------------------------------------------
                                                          
//----- интерфейс к повороту оси платы датчика --------------
uint8_t TModel::get_sens_axis_rotate()
{
  uint8_t Dest;
  
  access_prot( Dest, Main.FlashData.AllSets.DevSets.SensAxisRotate );
  
  return Dest;
}

bool TModel::set_sens_axis_rotate( uint8_t Src )
{
  if ( chk_axis_rotate( Src ) )
  {
    if ( get_sens_axis_rotate() != Src )
    {
      access_prot( Main.FlashData.AllSets.DevSets.SensAxisRotate, Src ); //записать в главный буфер в ОЗУ
      set_flash_data();                                                  //записать во Flash
    }
    else
    {
      //нет необходимости в перезаписи если новое значение не отличается от старого
    }
    
    return true;
  }
  else
  {
    return false; //параметр не записан ни в ОЗУ ни во Flash
  }
}
//-----------------------------------------------------------
                                                          
//----- интерфейс к буферному значению поворота оси платы датчика
uint8_t TModel::get_sens_buf_axis_rotate()
{
  uint8_t Dest;
  
  access_prot( Dest, Main.DevData.Sens.BufAxisRotate );
  
  return Dest;
}

bool TModel::set_sens_buf_axis_rotate( uint8_t Src )
{  
  if ( chk_axis_rotate( Src ) )
  {
    access_prot(  Main.DevData.Sens.BufAxisRotate, Src );
    
    return true;
  }
  else
  {
    return false;
  }
}
//-----------------------------------------------------------
                                                        
//----- интерфейс к настройке смещения крена платы датчика --
int8_t TModel::get_sens_roll_bias_angle()
{
  int8_t Dest;
  
  access_prot( Dest, Main.DevData.Sens.RollBiasAngle );
  
  return Dest;
}

void TModel::set_sens_roll_bias_angle( int8_t Src )
{
  access_prot(  Main.DevData.Sens.RollBiasAngle, Src );
}
//-----------------------------------------------------------
                                                        
//----- интерфейс к настройке смещения тангажа платы датчика
int8_t TModel::get_sens_pitch_bias_angle()
{
  int8_t Dest;
  
  access_prot( Dest, Main.DevData.Sens.PitchBiasAngle );
  
  return Dest;
}

void TModel::set_sens_pitch_bias_angle( int8_t Src )
{
  access_prot(  Main.DevData.Sens.PitchBiasAngle, Src );
}
//-----------------------------------------------------------
                                                          
//----- интерфейс к сырому значению тангажа -----------------
int8_t TModel::get_raw_pitch()
{
  int8_t Dest;
  
  access_prot( Dest, Main.DevData.RawPitch );
  
  return Dest;
}
                       
void TModel::set_raw_pitch( int8_t Src )
{
  access_prot(  Main.DevData.RawPitch, Src );
}                
//-----------------------------------------------------------
                                                        
//----- интерфейс к сырому значению крена -------------------
int8_t TModel::get_raw_roll()
{
  int8_t Dest;
  
  access_prot( Dest, Main.DevData.RawRoll );
  
  return Dest;
}
                       
void TModel::set_raw_roll( int8_t Src )
{
  access_prot(  Main.DevData.RawRoll, Src );
}                  
//-----------------------------------------------------------
                                                          
//----- интерфейс к значению по оси Z -----------------------
float TModel::get_accel_z()
{
  float Dest;
  
  access_prot( Dest, Main.DevData.AccelZ );
  
  return Dest;
}
                                
void TModel::set_accel_z( float Src )
{
  access_prot(  Main.DevData.AccelZ, Src );
}                                   
//-----------------------------------------------------------
                                                          
//----- интерфейс к результату выполнения команды калибровки
TModel::TCalibProcess TModel::get_calib_process()
{
  TCalibProcess Dest;  
  access_prot( Dest, Main.DevData.CalibProcess );
  
  return Dest;
}

void TModel::set_calib_process( TCalibProcess Src )
{
  access_prot( Main.DevData.CalibProcess, Src );
}

bool TModel::set_calib_cmd( uint16_t Src )
{
  //Необходимо: \
    1. не допустить несуществующую команду калибровки \
    2. не допустить прерывание процесса выполнения   
  typedef bool ( TSettings::*TChk )( int16_t ) const;
  typedef void ( TModel::*THandler )( int16_t, bool * );

  struct TCalibHandler
  {
    TChk     chk_fnct;
    THandler handler_fnct;
  };

  TCalibHandler CalibHandler[] =
  {
    { &TModel::chk_calib,  &TModel::handler_calib, },
    { &TModel::chk_access, &TModel::handler_access, },
  };
  
  bool CalibProcess = false;
  
  for ( auto item : CalibHandler )
  {
    if ( ( this->*item.chk_fnct )( Src ) )
    {
      CalibProcess = true;
      ( this->*item.handler_fnct )( Src, &CalibProcess );
      
      break;
    }
  }
  
  if ( !CalibProcess )
  {
    set_calib_process( TCalibProcess::__DENY ); //запрощенная команда не найдена
  }
  
  return true;
}

void TModel::handler_calib( int16_t Src, bool *CalibProcess )
{
  if ( get_calib_process() == TCalibProcess::__PERFORMING ) //если какая-то калибровочная команда выполняется в данный момент
  {
    //семафор на смену калибровочного положения выдавать НЕЛЬЗЯ!
  }
  else
  {
//    if ( get_access() >= AngleCalibSem[ Src ].Access ) //проверка доступа к калибровочной команде
//    {
//      set_calib_process( TCalibProcess::__PERFORMING );
//      *CalibProcess = true;
//      xSemaphoreGive( *AngleCalibSem[ Src ].Sem );      
//    }
//    else
//    {
//      *CalibProcess = false;
//    }    
  }
}

void TModel::handler_access( int16_t Src, bool *CalibProcess )
{
//  set_calib_process( TCalibProcess::__PERFORMING ); //необходим дозапрос параметра => перевод калибровочного процесса в __PERFORMING
  
  for ( uint8_t Ctr = 0; Ctr < ( sizeof Authorize / sizeof Authorize[0U] ); ++Ctr )
  {
    if ( Authorize[ Ctr ].Access == Src % 10U )
    {
      if ( Authorize[ Ctr ].get_psw != nullptr )
      {
        set_authorize_item_ptr( &Authorize[ Ctr ] );
        set_calib_process( TCalibProcess::__PERFORMING );
      }
      else
      {
        set_access( Authorize[ Ctr ].Access ); //пароль не нужен
        set_authorize_item_ptr( nullptr );
        set_calib_process( TCalibProcess::__PERFORMED );
      }
           
      break;
    }
    else
    {
    
    }
  }
  
  //далее ожидание действий пользователя по установке
}
//-----------------------------------------------------------
  
//----- интерфейс к счетчику калибровочных положений --------
uint32_t TModel::get_calib_position_ctr()
{
  uint32_t Dest;  
  access_prot( Dest, Main.DevData.CalibPositionCtr );
  
  return Dest;
}

void TModel::set_calib_position_ctr( TMemsOrient Src )
{
  //поразрядное преобразование
  if ( Src == TMemsOrient::_RESET )
  {
    access_prot( Main.DevData.CalibPositionCtr, 111111U );
  }
  else
  {
    uint32_t Res = 1U; 
    uint8_t Ctr = static_cast<uint8_t>(Src);   
    
    for ( ; Ctr > 0U ; --Ctr)
    {
      Res *= 10U;
    }
    Res = get_calib_position_ctr() - Res;
    
    access_prot( Main.DevData.CalibPositionCtr, Res );
  }  
}

void TModel::clr_calib_position_ctr( TMemsOrient Src )
{
  uint8_t Ctr = static_cast<uint8_t>(Src); 
  uint32_t Res = get_calib_position_ctr();
  uint32_t Tmp = Res;
  
  uint32_t Mask = 1U;
  for ( ; Ctr > 0U ; --Ctr)
  {
    Tmp  /= 10U;
    Mask *= 10U;
  }
  
  if ( Tmp % 2U == true ) //если нечетное
  {
    //сбрасывать не надо
  }
  else
  {
    Res += Mask;
    access_prot( Main.DevData.CalibPositionCtr, Res );
  } 
}
//-----------------------------------------------------------
  
//----- интерфейс к параметру калибровки --------------------
float TModel::get_calib_parameter()
{
  float Dest;  
  access_prot( Dest, Main.DevData.CalibParameter );
  
  return Dest;
}

void TModel::set_calib_parameter( float Src )
{
  access_prot( Main.DevData.CalibParameter, Src );
}
//-----------------------------------------------------------
  
//----- интерфейс к признаку обмена данными -----------------
void TModel::get_transact_sign( uint8_t &Dest )
{
  access_prot( Dest, Main.DevData.TransactSign );
}

void TModel::set_transact_sign( const uint8_t &Src )
{
  access_prot(  Main.DevData.TransactSign, Src );
}
//-----------------------------------------------------------
       
//----- интерфейс к правам доступа к данным устройства ------
TModel::TAccess TModel::get_access()
{
  TAccess Dest;
  access_prot( Dest, Main.DevData.Access );
  
  return Dest;
}

void TModel::set_access( TAccess Src )
{
  access_prot(  Main.DevData.Access, Src );
}
//-----------------------------------------------------------
                                                          
//----- интерфейс к типу авторизации, запрашиваемому пользователем
const TModel::TAuthorize *TModel::get_authorize_item_ptr()
{
  return static_cast<const TAuthorize *>( Main.DevData.AuthorizeItemPtr );
}
                    
void TModel::set_authorize_item_ptr( const TAuthorize *Src )
{
  Main.DevData.AuthorizeItemPtr = static_cast<const void *>( Src );
}      
//-----------------------------------------------------------
                                                          
//----- интерфейс к командному байту связи устройств --------
TModel::TCommunicate TModel::get_communicate_byte()
{
  TCommunicate Dest;
  access_prot( Dest, Main.DevData.Communicate );
  
  return Dest;
}

void TModel::set_communicate_byte( TCommunicate Src )
{
  access_prot(  Main.DevData.Communicate, Src );
}
//-----------------------------------------------------------
                                                        
//----- интерфейс к признаку валидности текущей выборки -----
TModel::TValidSign TModel::get_sample_valid_sign()
{
  TCommunicate Dest = get_communicate_byte();
  
  return Dest.SampleValidSign;
}

void TModel::set_sample_valid_sign( TValidSign Src )
{
  TCommunicate Byte = get_communicate_byte();
  Byte.SampleValidSign = Src;
  
  access_prot(  Main.DevData.Communicate, Byte );
}
//-----------------------------------------------------------
                                                        
//----- интерфейс к команде запуска измерений ---------------
TModel::TMeas TModel::get_start_meas_cmd()
{
  TCommunicate Dest = get_communicate_byte();
  
  return Dest.StartMeasCmd;
}

void TModel::set_start_meas_cmd( TMeas Src )
{
  TCommunicate Byte = get_communicate_byte();
  Byte.StartMeasCmd = Src;
  
  access_prot(  Main.DevData.Communicate, Byte );
}
//-----------------------------------------------------------

//------- НАСТРОЙКИ УСТРОЙСТВА -------------------------------------
//----- интерфейс к флагу нахождения в загрузчике -----------
TModel::TBootModeFlag TModel::get_boot_mode_flag()
{
  TBootModeFlag Dest;
  access_prot( Dest, Main.FlashData.BootModeFlag );
  
  return Dest;
}

void TModel::set_boot_mode_flag( TBootModeFlag Src )
{
  access_prot(  Main.FlashData.BootModeFlag, Src ); //записать в главный буфер в ОЗУ
  set_flash_data();                                 //записать во Flash
}
//-----------------------------------------------------------

//----- интерфейс к признаку программирования Flash ---------
TModel::TIsFlashProg TModel::get_flash_prog_sign()
{
  TIsFlashProg Dest;
  access_prot( Dest, Main.FlashData.IsFlashProg );
  
  return Dest;
}

void TModel::set_flash_prog_sign( const TIsFlashProg &Src )
{
  access_prot(  Main.FlashData.IsFlashProg, Src ); //записать в главный буфер в ОЗУ
  set_flash_data();                                //записать во Flash
}
//-----------------------------------------------------------

//----- интерфейс к признаку калибровки акселерометра -------
TModel::TIsAccCalib TModel::get_accel_calib_sign()
{
  TIsAccCalib Dest;
  access_prot( Dest, Main.FlashData.IsAccCalib );
  
  return Dest;
}

void TModel::set_accel_calib_sign( const TIsAccCalib &Src )
{
  access_prot(  Main.FlashData.IsAccCalib, Src ); //записать в главный буфер в ОЗУ
  set_flash_data();                               //записать во Flash
}
//-----------------------------------------------------------

//----- интерфейс к адресу устройства по протоколу СЕНС -----
uint8_t TModel::get_sens_addr()
{
  uint8_t Dest;
  access_prot( Dest, Main.FlashData.AllSets.Addr );
  
  return Dest;
}

void TModel::set_sens_addr( const uint8_t &Src )
{
  access_prot(  Main.FlashData.AllSets.Addr, Src ); //записать в главный буфер в ОЗУ
  set_flash_data();                                 //записать во Flash
}
//-----------------------------------------------------------

//----- интерфейс к паролю администратора -------------------
float TModel::get_psw_admin()
{
  float Dest;
  access_prot( Dest, Main.FlashData.AllSets.PswAdmin );
  
  return Dest;
}

void TModel::set_psw_admin( float Src )
{
  if ( Src != get_psw_admin() )
	{
		access_prot(  Main.FlashData.AllSets.PswAdmin, Src ); //записать в главный буфер в ОЗУ
		set_flash_data();                                     //записать во Flash
	}
}
//-----------------------------------------------------------

//----- интерфейс к паролю суперадминистратора --------------
float TModel::get_psw_super()
{
  float Dest;
  access_prot( Dest, Main.FlashData.AllSets.PswSuper );
  
  return Dest;
}

void TModel::set_psw_super( float Src )
{
  if ( Src != get_psw_super() )
	{
		access_prot(  Main.FlashData.AllSets.PswSuper, Src ); //записать в главный буфер в ОЗУ
		set_flash_data();                                     //записать во Flash
	}
}
//-----------------------------------------------------------

//----- интерфейс к настройке порога срабатывания по углу ---
uint8_t TModel::get_thr()
{
  uint8_t Dest;
  access_prot( Dest, Main.FlashData.AllSets.DevSets.Thr );

  return Dest;
}

bool TModel::set_thr( uint8_t Src )
{
  if ( chk_thr( Src ) )
  {
    if ( get_thr() != Src )
    {
      access_prot( Main.FlashData.AllSets.DevSets.Thr, Src ); //записать в главный буфер в ОЗУ
      set_flash_data();                                       //записать во Flash
    }
    else
    {
      //нет необходимости в перезаписи если новое значение не отличается от старого
    }
       
    return true;
  }
  else
  {
    return false; //параметр не записан ни в ОЗУ ни во Flash
  }
}
//-----------------------------------------------------------

//----- интерфейс к настройке гистерезиса срабатывания ------
uint8_t TModel::get_hyst()
{
  uint8_t Dest;
  access_prot( Dest, Main.FlashData.AllSets.DevSets.Hyst );

  return Dest;
}

bool TModel::set_hyst( uint8_t Src )
{
  if ( chk_hyst( Src ) )
  {
    if ( get_hyst() != Src )
    {
      access_prot( Main.FlashData.AllSets.DevSets.Hyst, Src ); //записать в главный буфер в ОЗУ
      set_flash_data();                                        //записать во Flash
    }
    else
    {
      //нет необходимости в перезаписи если новое значение не отличается от старого
    }
        
    return true;
  }
  else
  {
    return false; //параметр не записан ни в ОЗУ ни во Flash
  }
}
//-----------------------------------------------------------

//----- интерфейс к настройке смещения ----------------------
int8_t TModel::get_bias()
{
  int8_t Dest;
  access_prot( Dest, Main.FlashData.AllSets.DevSets.Bias );

  return Dest;
}

bool TModel::set_bias( int8_t Src )
{
  if ( chk_bias( Src ) )
  {
    if ( get_bias() != Src )
    {
      access_prot( Main.FlashData.AllSets.DevSets.Bias, Src ); //записать в главный буфер в ОЗУ
      set_flash_data();                                        //записать во Flash
    }
    else
    {
      //нет необходимости в перезаписи если новое значение не отличается от старого
    }
    
    return true;
  }
  else
  {
    return false; //параметр не записан ни в ОЗУ ни во Flash
  }
}
//-----------------------------------------------------------

//----- интерфейс к настройке поворота оси ------------------
uint8_t TModel::get_axis_rotate()
{
  uint8_t Dest;
  access_prot( Dest, Main.FlashData.AllSets.DevSets.AxisRotate );

  return Dest;
}

bool TModel::set_axis_rotate( uint8_t Src )
{
  if ( chk_axis_rotate( Src ) )
  {
    if ( get_axis_rotate() != Src )
    {
      access_prot( Main.FlashData.AllSets.DevSets.AxisRotate, Src ); //записать в главный буфер в ОЗУ
      set_flash_data();                                              //записать во Flash
    }
    else
    {
      //нет необходимости в перезаписи если новое значение не отличается от старого
    }
    
    return true;
  }
  else
  {
    return false; //параметр не записан ни в ОЗУ ни во Flash
  }
}
//-----------------------------------------------------------

//----- интерфейс к настройке смещения крена ----------------
int8_t TModel::get_roll_bias_angle()
{
  int8_t Dest;
  access_prot( Dest, Main.FlashData.AllSets.DevSets.RollBiasAngle );

  return Dest;
}

bool TModel::set_roll_bias_angle( int8_t Src )
{
  if ( chk_roll_bias_angle( Src ) )
  {
    if ( get_roll_bias_angle() != Src )
    {
      access_prot( Main.FlashData.AllSets.DevSets.RollBiasAngle, Src ); //записать в главный буфер в ОЗУ
      set_flash_data();                                                 //записать во Flash
    }
    else
    {
      //нет необходимости в перезаписи если новое значение не отличается от старого
    }
    
    return true;
  }
  else
  {
    return false; //параметр не записан ни в ОЗУ ни во Flash
  }
}
//-----------------------------------------------------------

//----- интерфейс к настройке смещения тангажа --------------
int8_t TModel::get_pitch_bias_angle()
{
  int8_t Dest;
      
  access_prot( Dest, Main.FlashData.AllSets.DevSets.PitchBiasAngle );

  return Dest;
}

bool TModel::set_pitch_bias_angle( int8_t Src )
{
  if ( chk_pitch_bias_angle( Src ) )
  {
    if ( get_pitch_bias_angle() != Src )
    {
      access_prot( Main.FlashData.AllSets.DevSets.PitchBiasAngle, Src ); //записать в главный буфер в ОЗУ
      set_flash_data();                                                  //записать во Flash
    }
    else
    {
      //нет необходимости в перезаписи если новое значение не отличается от старого
    }
    
    return true;
  }
  else
  {
    return false; //параметр не записан ни в ОЗУ ни во Flash
  }
}
//-----------------------------------------------------------

//----- интерфейс к настройке дискретного выхода ------------
TContact TModel::get_d_o_sets()
{  
  TBits Dest;
  access_prot( Dest, Main.FlashData.AllSets.AdpBits.Bits );
  
  return Dest.Item.Normal;  
}

void TModel::set_d_o_sets( TContact Src )
{
  //чтение -> модификация -> запись
  TBits New;
  access_prot( New, Main.FlashData.AllSets.AdpBits.Bits ); //в New в данный момент старые настройки
  
  if ( New.Item.Normal != Src )
  {
    New.Item.Normal = Src;

    access_prot(  Main.FlashData.AllSets.AdpBits.Bits, New ); //записать в главный буфер в ОЗУ
    set_flash_data();                                         //записать во Flash
  }
  else
  {
    //защита от перезаписи того же значения
  }
}
//-----------------------------------------------------------

//----- интерфейс к настройке типа взаимодействия датчика ---
TInterconn TModel::get_interconn()
{  
  TBits Dest;
  access_prot( Dest, Main.FlashData.AllSets.AdpBits.Bits );
  
  return Dest.Item.Interconn;  
}

void TModel::set_interconn( TInterconn Src )
{
  //чтение -> модификация -> запись
  TBits New;
  access_prot( New, Main.FlashData.AllSets.AdpBits.Bits ); //в New в данный момент старые настройки
  
  if ( New.Item.Interconn != Src )
  {
    New.Item.Interconn = Src;

    access_prot(  Main.FlashData.AllSets.AdpBits.Bits, New ); //записать в главный буфер в ОЗУ
    set_flash_data();                                         //записать во Flash
  }
  else
  {
    //защита от перезаписи того же значения
  }
}
//-----------------------------------------------------------

//----- интерфейс к адресу в сети Modbus --------------------
uint8_t TModel::get_mb_addr()
{
  uint8_t Dest;
  access_prot( Dest, Main.FlashData.AllSets.If.MBAddr );

  return Dest;
}

bool TModel::set_mb_addr( int16_t Src )
{
  if ( chk_mb_addr( Src ) )
  {
    if ( get_mb_addr() != Src )
    {
      access_prot( Main.FlashData.AllSets.If.MBAddr, Src ); //записать в главный буфер в ОЗУ
      set_flash_data();                                     //записать во Flash
    }
    else
    {
      //нет необходимости в перезаписи если новое значение не отличается от старого
    }
    
    return true;
  }
  else
  {
    return false; //параметр не записан ни в ОЗУ ни во Flash
  }
}
//-----------------------------------------------------------

//----- интерфейс к скорости интерфейса Usart ---------------
uint8_t TModel::get_u_baud_rate()
{
  uint8_t Dest;
  access_prot( Dest, Main.FlashData.AllSets.If.USpeed );

  return Dest;
}

uint32_t TModel::get_u_baud_rate_bit_to_sec()
{
  uint8_t Dest;
  access_prot( Dest, Main.FlashData.AllSets.If.USpeed );
  
  uint32_t BitToSec[] = 
  {
      1200U, //0
      2400U, //1
      4800U, //2
      9600U, //3
     14400U, //4
     19200U, //5
     38400U, //6
     56000U, //7
     57600U, //8
    115200U, //9
  };
  
  return BitToSec[ Dest ];
};

bool TModel::set_u_baud_rate( uint8_t Src )
{
  if ( chk_u_baud_rate( Src ) )
  {
    if ( get_u_baud_rate() != Src )
    {
      access_prot( Main.FlashData.AllSets.If.USpeed, Src ); //записать в главный буфер в ОЗУ
      set_flash_data();                                     //записать во Flash
    }
    else
    {
      //нет необходимости в перезаписи если новое значение не отличается от старого
    }
    
    return true;
  }
  else
  {
    return false; //параметр не записан ни в ОЗУ ни во Flash
  }
}
//-----------------------------------------------------------

//----- интерфейс к четности и стоп-битам в USART -----------
uint8_t TModel::get_u_par()
{
  uint8_t Dest;
  access_prot( Dest, Main.FlashData.AllSets.If.UPar );

  return Dest;
}

bool TModel::set_u_par( uint8_t Src )
{
  if ( chk_u_par( Src ) )
  {
    if ( get_u_par() != Src )
    {
      access_prot( Main.FlashData.AllSets.If.UPar, Src ); //записать в главный буфер в ОЗУ
      set_flash_data();                                   //записать во Flash
    }
    else
    {
      //нет необходимости в перезаписи если новое значение не отличается от старого
    }
    
    return true;
  }
  else
  {
    return false; //параметр не записан ни в ОЗУ ни во Flash
  }
}
//-----------------------------------------------------------

//----- интерфейс к настройкам ------------------------------
void TModel::get_sets( MySettings &Dest )
{
  access_prot( Dest, Main.FlashData.AllSets );
}

void TModel::set_sets( const MySettings &Src )
{
  access_prot(  Main.FlashData.AllSets, Src ); //записать в главный буфер в ОЗУ
  set_flash_data();                            //записать во Flash
}
//-----------------------------------------------------------

//----- интерфейс к настройкам интерфейса -------------------
void TModel::get_if_sets( TIf &Dest )
{
  access_prot( Dest, Main.FlashData.AllSets.If );
}

void TModel::set_if_sets( const TIf &Src )
{
  access_prot(  Main.FlashData.AllSets.If, Src ); //записать в главный буфер в ОЗУ
  set_flash_data();                               //записать во Flash
}
//-----------------------------------------------------------

//----- интерфейс к калибровочным данным акселерометра ------
void TModel::get_accel_calib_data( TAccCalData &Dest )
{
  access_prot( Dest, Main.FlashData.AccCalData );
}

void TModel::set_accel_calib_data( const TAccCalData &Src )
{
  access_prot(  Main.FlashData.AccCalData, Src ); //записать в главный буфер в ОЗУ
  set_flash_data();                               //записать во Flash
}
//-----------------------------------------------------------

//----- интерфейс к копии данных, хранящихся во Flash -------
void TModel::get_flash_data( TFlashData &Dest )
{
  access_prot( Dest, Main.FlashData );
}

void TModel::set_flash_data()
{
  TFlashData Dest;
  access_prot( Dest, Main.FlashData );
    
  Flash.write_sets(
                   reinterpret_cast<uint8_t *>(&Dest),
                   sizeof Dest
                  );
}
//-----------------------------------------------------------

//----- интерфейс к давлению СЕНС ПД (тест) -----------------
float TModel::get_pd_pressure()
{
  float Dest;
  access_prot( Dest, Main.DevData.PdPressure );
  
  return Dest;
}

void TModel::set_pd_pressure( float Src )
{
  if ( Src != get_psw_admin() )
	{
		access_prot(  Main.DevData.PdPressure, Src ); //записать в главный буфер в ОЗУ
		set_flash_data();                             //записать во Flash
	}
}
//-----------------------------------------------------------
//------------------------------------------------------------------

/*
*
*/
