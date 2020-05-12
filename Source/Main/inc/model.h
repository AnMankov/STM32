#ifndef __MODEL_H
#define __MODEL_H

#include "rtos_headers.h"
#include "settings.h"

class TModel final : public TSettings
{
friend class TDevDeterm;

public:
  __packed struct TMain
  {
    __packed union
    {
      __packed struct
      {
        TDevType DevType  : 2; //тип устройства (база/датчик)
        uint8_t  Reserved : 6;
      };
      uint8_t Byte;
    };
		
    TPress     Press;
    TLedMode   LedMode;

    TDevData DevData;     //основные данные устройства
    TFlashData FlashData; //копия того, что хранится во Flash
  };
  
  enum TMemsOrient : uint8_t
  {
    _X_UP   = 0U,
    _X_DOWN = 1U,
    _Y_UP   = 2U,
    _Y_DOWN = 3U,
    _Z_UP   = 4U,
    _Z_DOWN = 5U,
    
    _MIN    = _X_UP,
    _MAX    = _Z_DOWN + 1U,
    
    _RESET  = 55U,
  };
  
  enum TAngleCalibType : uint8_t
  {
    __USER_CAL    = 0U,
    __FACTORY_CAL = 1U,
    
    __MAX_CAL     = __FACTORY_CAL + 1U,
  };
  
  struct TSem
  {
    SemaphoreHandle_t *Sem;
    TMemsOrient        MemsOrient;
  };
  
  struct TAngleSem
  {
    SemaphoreHandle_t *Sem;
    TAccess            Access;
  };
  
  typedef float ( TModel::*TGetPsw )();
  
  struct TAuthorize
  {
    TAccess Access;
    TGetPsw get_psw;
  };

public:
  TModel( SemaphoreHandle_t &MainMut );
  ~TModel();

  //----- интерфейс к типу устройства -------------------------
  TDevType get_dev_type();
  //-----------------------------------------------------------
  
//------- ДАННЫЕ УСТРОЙСТВА ----------------------------------------

  //----- интерфейс к состоянию кнопки ------------------------
  TPress get_btn_mode();
  void set_btn_mode( TPress );
  //-----------------------------------------------------------

  //----- интерфейс к режиму работы светодиода ----------------
  TLedMode get_led_mode();
  void set_led_mode( TLedMode );
  //-----------------------------------------------------------
	
  //----- интерфейс к данным кодового переключателя платы, ---\
    на которой выполняется программа 
  uint8_t get_code_sw();
  void set_code_sw( uint8_t );
  //-----------------------------------------------------------
  
  //----- интерфейс к данным кодового переключателя на крышке -
  uint8_t get_hc_code_sw();
  void set_hc_code_sw( uint8_t );
  //-----------------------------------------------------------
  
  //----- интерфейс к версия ПО с крышки ----------------------
  uint16_t get_hc_prog_nbr();
  void set_hc_prog_nbr( uint16_t );
  //-----------------------------------------------------------
	
  //----- интерфейс к результирующему углу --------------------
  float get_open_angle();                       
  void set_open_angle( float );                 
  //-----------------------------------------------------------
                                                          
  //----- интерфейс к результирующему состоянию крышки --------
  THC get_hc_state();                                     
  void set_hc_state( THC );
  //-----------------------------------------------------------
                                                          
  //----- интерфейс к ошибке положения крышки -----------------
  TPosErr get_hc_pos_err();                               
  void set_hc_pos_err( TPosErr );                         
  //-----------------------------------------------------------
                                                          
  //----- интерфейс к ошибке положения базы -------------------
  TPosErr get_base_pos_err();
  void set_base_pos_err( TPosErr );
  //-----------------------------------------------------------
                                                          
  //----- интерфейс к связи с датчиком на крышке --------------
  TConnect get_connect();                                 
  void set_connect( TConnect );                           
  //-----------------------------------------------------------
                                                          
  //----- интерфейс к калибровке акселерометра на крышке ------
  TCalib get_hc_acc_calib();                                 
  void set_hc_acc_calib( TCalib );                           
  //-----------------------------------------------------------
                                                          
  //----- интерфейс к калибровке акселерометра на базе --------
  TCalib get_acc_calib();                                 
  void set_acc_calib( TCalib );                           
  //-----------------------------------------------------------
                                                          
  //----- интерфейс к состоянию МЭМС на крышке ----------------
  TMainState get_hc_mems();
  void set_hc_mems( TMainState );
  //-----------------------------------------------------------
                                                          
  //----- интерфейс к состоянию МЭМС на базе ------------------
  TMainState get_base_mems();
  void set_base_mems( TMainState );
  //-----------------------------------------------------------
                                                          
  //----- интерфейс к состоянию устройства --------------------
  uint8_t get_dev_state();                        
  void set_dev_state( const uint8_t & );                  
  //-----------------------------------------------------------
                                                          
  //----- интерфейс к собственному углу платы -----------------
  int16_t get_my_angle();                         
  void set_my_angle( int16_t );                    
  //-----------------------------------------------------------
                                                          
  //----- интерфейс к углу платы датчика ----------------------
  int16_t get_sens_angle();                       
  void set_sens_angle( int16_t );                  
  //-----------------------------------------------------------
                                                          
  //----- интерфейс к валидности текущего окна выборок датчика
  TValidSign get_sens_sample_valid_sign();                       
  void set_sens_sample_valid_sign( TValidSign );                  
  //-----------------------------------------------------------
                                                          
  //----- интерфейс к повороту оси платы датчика -------------- (то, что база получает от датчика)
  uint8_t get_sens_axis_rotate();                       
  bool set_sens_axis_rotate( uint8_t );                  
  //-----------------------------------------------------------
                                                          
  //----- интерфейс к буферному значению поворота оси платы датчика (то, что поступает с ПК на базу)
  uint8_t get_sens_buf_axis_rotate();                       
  bool set_sens_buf_axis_rotate( uint8_t );                  
  //-----------------------------------------------------------
                                                          
  //----- интерфейс к настройке смещения крена платы датчика --
  int8_t get_sens_roll_bias_angle();                       
  void set_sens_roll_bias_angle( int8_t );                  
  //-----------------------------------------------------------
                                                          
  //----- интерфейс к настройке смещения тангажа платы датчика
  int8_t get_sens_pitch_bias_angle();                       
  void set_sens_pitch_bias_angle( int8_t );                  
  //-----------------------------------------------------------
                                                          
  //----- интерфейс к сырому значению тангажа -----------------
  int8_t get_raw_pitch();                       
  void set_raw_pitch( int8_t );                  
  //-----------------------------------------------------------
                                                          
  //----- интерфейс к сырому значению крена -------------------
  int8_t get_raw_roll();                       
  void set_raw_roll( int8_t );                  
  //-----------------------------------------------------------
                                                          
  //----- интерфейс к значению по оси Z -----------------------
  float get_accel_z();                       
  void set_accel_z( float );                  
  //-----------------------------------------------------------
                                                          
  //----- интерфейс к результату выполнения команды калибровки
  TCalibProcess get_calib_process();
  void set_calib_process( TCalibProcess );
  bool set_calib_cmd( uint16_t ); 
  void handler_calib( int16_t, bool * );
  void handler_access( int16_t, bool * );
  //-----------------------------------------------------------
  
  //----- интерфейс к счетчику калибровочных положений --------
  uint32_t get_calib_position_ctr();
  void set_calib_position_ctr( TMemsOrient );
  void clr_calib_position_ctr( TMemsOrient );
  //-----------------------------------------------------------
  
  //----- интерфейс к параметру калибровки --------------------
  float get_calib_parameter();
  void set_calib_parameter( float );
  //-----------------------------------------------------------
                                                          
  //----- интерфейс к признаку обмена данными -----------------
  void get_transact_sign( uint8_t & );                    
  void set_transact_sign( const uint8_t & );              
  //-----------------------------------------------------------
                                                          
  //----- интерфейс к правам доступа к данным устройства ------
  TAccess get_access();
  void set_access(TAccess);
  //-----------------------------------------------------------
                                                          
  //----- интерфейс к типу авторизации, запрашиваемому пользователем
  const TAuthorize *get_authorize_item_ptr();
  void set_authorize_item_ptr( const TAuthorize * );
  //-----------------------------------------------------------
                                                          
  //----- интерфейс к командному байту связи устройств --------
  TCommunicate get_communicate_byte();
  void set_communicate_byte( TCommunicate );
  //-----------------------------------------------------------
                                                          
  //----- интерфейс к признаку валидности текущей выборки -----
  TValidSign get_sample_valid_sign();
  void set_sample_valid_sign( TValidSign );
  //-----------------------------------------------------------
                                                          
  //----- интерфейс к команде запуска измерений ---------------
  TMeas get_start_meas_cmd();
  void set_start_meas_cmd( TMeas );
  //-----------------------------------------------------------
//------------------------------------------------------------------


//------- НАСТРОЙКИ УСТРОЙСТВА -------------------------------------
  //----- интерфейс к флагу нахождения в загрузчике -----------
  TBootModeFlag get_boot_mode_flag();
  void set_boot_mode_flag( TBootModeFlag );
  //-----------------------------------------------------------
  
  //----- интерфейс к признаку программирования Flash ---------
  TIsFlashProg get_flash_prog_sign();
  void set_flash_prog_sign( const TIsFlashProg & );
  //-----------------------------------------------------------

  //----- интерфейс к признаку калибровки акселерометра -------
  TIsAccCalib get_accel_calib_sign();
  void set_accel_calib_sign( const TIsAccCalib & );
  //-----------------------------------------------------------

  //----- интерфейс к адресу устройства по протоколу СЕНС -----
  uint8_t get_sens_addr();
  void set_sens_addr( const uint8_t & );
  //-----------------------------------------------------------

  //----- интерфейс к паролю администратора -------------------
  float get_psw_admin();
  void set_psw_admin( float );
  //-----------------------------------------------------------

  //----- интерфейс к паролю суперадминистратора --------------
  float get_psw_super();
  void set_psw_super( float );
  //-----------------------------------------------------------

  //----- интерфейс к настройке порога срабатывания по углу ---
  uint8_t get_thr();
  bool set_thr( uint8_t );
  //-----------------------------------------------------------

  //----- интерфейс к настройке гистерезиса срабатывания ------
  uint8_t get_hyst();
  bool set_hyst( uint8_t );
  //-----------------------------------------------------------

  //----- интерфейс к настройке смещения ----------------------
  int8_t get_bias();
  bool set_bias( int8_t );
  //-----------------------------------------------------------

  //----- интерфейс к настройке поворота оси ------------------
  uint8_t get_axis_rotate();
  bool set_axis_rotate( uint8_t );
  //-----------------------------------------------------------

  //----- интерфейс к настройке смещения крена ----------------
  int8_t get_roll_bias_angle();
  bool set_roll_bias_angle( int8_t );
  //-----------------------------------------------------------

  //----- интерфейс к настройке смещения тангажа --------------
  int8_t get_pitch_bias_angle();
  bool set_pitch_bias_angle( int8_t );
  //-----------------------------------------------------------

  //----- интерфейс к настройке дискретного выхода ------------
  TContact get_d_o_sets();
  void set_d_o_sets( TContact );
  //-----------------------------------------------------------

  //----- интерфейс к настройке типа взаимодействия датчика ---
  TInterconn get_interconn();
  void set_interconn( TInterconn );
  //-----------------------------------------------------------

  //----- интерфейс к адресу в сети Modbus --------------------
  uint8_t get_mb_addr();
  bool set_mb_addr( int16_t );
  //-----------------------------------------------------------

  //----- интерфейс к скорости интерфейса Usart ---------------
  uint8_t get_u_baud_rate();
  bool set_u_baud_rate( uint8_t );
  uint32_t get_u_baud_rate_bit_to_sec(); //для преобразования
  //-----------------------------------------------------------

  //----- интерфейс к четности и стоп-битам в USART -----------
  uint8_t get_u_par();
  bool set_u_par( uint8_t );
  //-----------------------------------------------------------

  //----- интерфейс к настройкам ------------------------------
  void get_sets( MySettings & );
  void set_sets( const MySettings & );
  //-----------------------------------------------------------

  //----- интерфейс к настройкам интерфейса -------------------
  void get_if_sets( TIf & );
  void set_if_sets( const TIf & );
  //-----------------------------------------------------------

  //----- интерфейс к калибровочным данным акселерометра ------
  void get_accel_calib_data( TAccCalData & );
  void set_accel_calib_data( const TAccCalData & );
  //-----------------------------------------------------------

  //----- интерфейс к копии данных, хранящихся во Flash -------
  void get_flash_data( TFlashData & );
  void set_flash_data();
  //-----------------------------------------------------------

  //----- интерфейс к давлению СЕНС ПД (тест) -----------------
  float get_pd_pressure();
  void set_pd_pressure( float );
  //-----------------------------------------------------------
//------------------------------------------------------------------
  
  constexpr static TRange CodeRange       = { 0U, 15U };
/*
*
*/
//  const TSem CalibSem[TMemsOrient::_MAX] =
//  {
//    { &AccCalib_X_UP_Sem,   _X_UP   },
//    { &AccCalib_X_DOWN_Sem, _X_DOWN },
//    { &AccCalib_Y_UP_Sem,   _Y_UP   },
//    { &AccCalib_Y_DOWN_Sem, _Y_DOWN },
//    { &AccCalib_Z_UP_Sem,   _Z_UP   },
//    { &AccCalib_Z_DOWN_Sem, _Z_DOWN },
//  };
  
//  const TAngleSem AngleCalibSem[TAngleCalibType::__MAX_CAL] =
//  {
//    { &User_Zeroing_Out_Sem,    __ADMIN },
//    { &Factory_Zeroing_Out_Sem, __SUPER },
//  };
  
  const TAuthorize Authorize[ __ACCESS_MAX ] =
  {
    { __USER,  nullptr                }, //для пользователя пароль не нужен
    { __ADMIN, &TModel::get_psw_admin },
    { __SUPER, &TModel::get_psw_super },
  };

protected:
private:
  TMain Main;             //данные защищены мьютексом для доступа из разных потоков
  SemaphoreHandle_t &Mut; //мьютекс, защищающий данные при доступе из разных потоков

  template< typename TDest, typename TSrc >
  void access_prot(TDest &Dest, const TSrc &Src )
  {
    xSemaphoreTake(Mut, portMAX_DELAY); //захватить мьютекс для атомарной работы с основными данными устройства
      Dest = Src;
    xSemaphoreGive(Mut);                //освободить мьютекс для атомарной работы с основными данными устройства
  }
};

extern TModel Model;

#endif //__MODEL_H
