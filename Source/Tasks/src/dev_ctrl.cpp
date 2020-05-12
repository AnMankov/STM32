#include <numeric>

#include "dev_ctrl.h"
#include "model.h"
#include "rtos_headers.h"
#include "discrete_out.h"
#include "angles_cnt.h"

TDevCtrl DevCtrl;

TDevCtrl::TDevCtrl()
:
SensDevCtr( { 0U, __SENS_DEV_OK_QTY } )
{

}

TDevCtrl::~TDevCtrl()
{

}

static void base_ctrl();
static void hc_ctrl();
static bool sens_dev_stage( const SemaphoreHandle_t *Sem, uint32_t Waiting );
static void factory_calib( TModel::TDevType DevType );
static void user_calib( int16_t );
static void process_factory_calib( TModel::TDevType DevType );
static int16_t cnt_average( int16_t *Beg, int16_t *End, uint8_t MAX_SAMPLE_CTR );
static void open_angle_ctrl( uint8_t *pGoodCtr );

//----- Задача RTOS --------------------------------------------------
void dev_ctrl( void *Params ) //управление устройством
{

  typedef void (*TFnct)();
  
  TFnct Fnct[] =
  {
    base_ctrl,
    hc_ctrl
  };
  
  TModel::TDevType DevType = Model.get_dev_type();  
  
  for ( ;; )
  {  
    factory_calib( DevType );   //для Базы и для Датчика
    Fnct[ DevType ]();
  }
  
}
//--------------------------------------------------------------------

//----- Вспомогательные функции задачи RTOS -------------------------------------------------------------------------
static void factory_calib( TModel::TDevType DevType )
{
  //при первом включении необходимо защелкнуть задачу на калибровку по углу
  TickType_t xTicksToWait = 0U;
  
  xTicksToWait = ( Model.get_accel_calib_sign() == TModel::TIsAccCalib::_ACC_UNCALIBRATED )
               ? portMAX_DELAY //задача защелкивается на ожидании семафора заводской калибровки
               : 0U;
               
  if ( xSemaphoreTake( Factory_Zeroing_Out_Sem, xTicksToWait ) == pdPASS )
  {
    process_factory_calib( DevType );
  }
}

static void user_calib( int16_t OpenAngle )
{
  static bool CalibProcess = false;
  static uint8_t CalibCtr  = 0U; 
  constexpr uint8_t MAX_SAMPLE_CTR = 20U; //из расчета появления новой валидной выборки каждые 300мс
  static int16_t OpenAngles[ MAX_SAMPLE_CTR ]  = {};
  
  
  if ( 
      CalibProcess == false
      &&
      xSemaphoreTake( User_Zeroing_Out_Sem, 0U ) == pdFAIL 
     )
  {
    Angles.DampCoeff = CONSTS::DAMP_COEFF;
  }
  else
  {
    Angles.DampCoeff = 1U;
    Model.set_bias( 0U );    
    CalibProcess = true;
    
    OpenAngles[ CalibCtr ] = OpenAngle;
    if ( ++CalibCtr < MAX_SAMPLE_CTR )
    {
      //продолжение заполнения массива
    }
    else
    {
      OpenAngles[ 0U ] = OpenAngles [ 1U ];
      CalibCtr = 0U;      
      CalibProcess = false;
      
      int16_t *Beg = &OpenAngles[ 0U ];
      int16_t *End = &OpenAngles[ MAX_SAMPLE_CTR ];
      
      TModel::TCalibProcess CalibProcess = TModel::TCalibProcess::__DENY;
      
      CalibProcess = ( Model.set_bias( cnt_average( Beg, End, MAX_SAMPLE_CTR ) ) == false )
                   ? TModel::TCalibProcess::__DENY
                   : TModel::TCalibProcess::__PERFORMED;
      
      Model.set_calib_process( CalibProcess );   
    }
  }
}

static void process_factory_calib( TModel::TDevType DevType )
{
  //допустимая коррекция углов при заводской калибровке ±5°
  constexpr uint8_t MAX_SAMPLE_CTR = 10U; //из расчета появления новой выборки каждые 300мс
  static uint32_t SampleCtr = 0U;
  SampleCtr = 0U;
  
  static int16_t RollAngles[ MAX_SAMPLE_CTR ]   = {};
  static int16_t PitchAngles[ MAX_SAMPLE_CTR ]  = {};
  
  int16_t *BegR = nullptr;
  int16_t *EndR = nullptr;  
  int16_t *BegP = nullptr;
  int16_t *EndP = nullptr;
  
  do
  {
    SampleCtr = 0U;
    
    do
    {
      xSemaphoreGive( DevMemsStartSem ); //выдать семафор запуска измерений своего МЭМС
      if ( 
          xSemaphoreTake( MemsDevDoneSem, pdMS_TO_TICKS( TDevCtrl::__WAITING_MEMS_DEV_DONE_MS ) )                                          
          == 
          pdFAIL
         )
      {
        Model.set_base_mems( TModel::TMainState::__ERR );
        Model.set_calib_process( TModel::TCalibProcess::__DENY );
        
        return;
      }
      else
      {
        Model.set_base_mems( TModel::TMainState::__OK );
              
        Angles.cnt_my_angle( DevType, TModel::TAxisRotate::__0_DEG, false );  //для высчитывания roll
        int16_t __0_DEG = Model.get_my_angle();
        
        Angles.cnt_my_angle( DevType, TModel::TAxisRotate::__90_DEG, false ); //для высчитывания pitch
        int16_t __90_DEG = Model.get_my_angle();
        
        RollAngles[ SampleCtr ]    = __0_DEG;
        PitchAngles[ SampleCtr++ ] = __90_DEG;       
      }    
    } while ( SampleCtr < MAX_SAMPLE_CTR );
    
    BegR = &RollAngles[ 0U ];
    EndR = &RollAngles[ MAX_SAMPLE_CTR ];

    BegP = &PitchAngles[ 0U ];
    EndP = &PitchAngles[ MAX_SAMPLE_CTR ];
  } while (
           Model.set_roll_bias_angle( cnt_average( BegR, EndR, MAX_SAMPLE_CTR ) ) == false
           ||
           Model.set_pitch_bias_angle( cnt_average( BegP, EndP, MAX_SAMPLE_CTR ) ) == false
          );
  Model.set_accel_calib_sign( TModel::TIsAccCalib::_ACC_CALIBRATED );
  Model.set_calib_process( TModel::TCalibProcess::__PERFORMED );
}

static int16_t cnt_average( int16_t *Beg, int16_t *End, uint8_t MAX_SAMPLE_CTR )
{
  int16_t Res =  
  std::accumulate( Beg, End, 0U, []( int16_t Res, int16_t Item ){
    return Res += Item;
  } );
    
  Res = ( Res >= 0 )
      ? ( Res + MAX_SAMPLE_CTR / 2 ) / MAX_SAMPLE_CTR
      : ( Res - MAX_SAMPLE_CTR / 2 ) / MAX_SAMPLE_CTR;  
  
  return Res;
}

static void base_ctrl()
{
  if (
      xSemaphoreTake( MemsDevRdySem, pdMS_TO_TICKS( TDevCtrl::__WAITING_MEMS_DEV_RDY_MS ) ) //ожидание семафора готовности измерять от МЭМС
      == 
      pdFAIL
     )
  {
    Model.set_base_mems( TModel::TMainState::__ERR );
    return;
  }
  else
  {
//    Model.set_base_mems( TModel::TMainState::__OK );
  }
  
  Model.set_start_meas_cmd( TModel::TMeas::__START_MEAS );
  xSemaphoreGive( DevSensStartSem ); //выдать семафор для отправки команды датчику на запуск измерений 
  if ( sens_dev_stage( &SensDevOkSem, TDevCtrl::__WAITING_SENS_DEV_OK_MS ) == false )
  {
    return;
  }
  
//  Do.closed();
  xSemaphoreGive( DevMemsStartSem ); //выдать семафор запуска измерений своего МЭМС
  
  if ( 
      xSemaphoreTake( MemsDevDoneSem, pdMS_TO_TICKS( TDevCtrl::__WAITING_MEMS_DEV_DONE_MS ) )                                          
      == 
      pdFAIL
     )
  {
    Model.set_base_mems( TModel::TMainState::__ERR );    
    return;
  }
  else
  {
    Model.set_base_mems( TModel::TMainState::__OK );
  }
  
  xSemaphoreGive( DevSensResSem ); //выдать семафор на запрос результатов измерений МЭМС на плате датчика
  if ( sens_dev_stage( &SensDevResSem, TDevCtrl::__WAITING_SENS_DEV_RES_MS ) == false )
  {
    return;
  }
//  Do.open();
	
  static uint8_t GoodCtr = 0U;
  
  if (                                                                   //проверка валидности текущего окна выборок для базы и крышки
      Model.get_sample_valid_sign() == TModel::TValidSign::__INVALID
      ||
      Model.get_sens_sample_valid_sign() == TModel::TValidSign::__INVALID
     )
  {
    //сохраняются предыдущие значения
    GoodCtr = 0U;
  }
  else 
  {
    open_angle_ctrl( &GoodCtr );
  }

  //выдать семафор задаче логгера для записи очередной точки
	xSemaphoreGive( WrFlash );
  
}

static void open_angle_ctrl( uint8_t *pGoodCtr )
{
  TModel::TAxisRotate AxisRotate = static_cast<TModel::TAxisRotate>(Model.get_axis_rotate());  
  Angles.cnt_my_angle( TModel::TDevType::_BASE, AxisRotate, true ); //преобразовать угол базы в формат ( 0..260, -1..-80 ) (угол датчика уже приходит преобразованным)
  
//  if ( ++*pGoodCtr > 3U )
//  {
//    *pGoodCtr = 3U;
    
    Angles.cnt_open_angle();                        //вычислить результирующий угол

    int16_t OpenAngle = roundf( Model.get_open_angle() );     //обновленный угол

    user_calib( OpenAngle );                        //пользовательская калибровка \
                                                      проводится только для Базы - обнуление результирующего угла с сохранением \
                                                      смещения в настройки (их можно поменять вручную)
    if (
        OpenAngle < CONSTS::THR_ERR_OPEN_ANGLE
        ||
        (
         OpenAngle <= CONSTS::THR_ERR_OPEN_ANGLE + CONSTS::HYST_ERR_OPEN_ANGLE
         &&
         Model.get_hc_pos_err() == TModel::TPosErr::_POS_ERR
        )
       )
    {
      Model.set_hc_pos_err( TModel::TPosErr::_POS_ERR );
      //состояние крышки остается прежним, значение угла обновлено
    }
    else
    {
      Model.set_hc_pos_err( TModel::TPosErr::_POS_OK );
      
      //определить новое состояние
      uint8_t THR  = Model.get_thr();
      uint8_t HYST = Model.get_hyst();
      
      TModel::THC NewState;

      if (
          OpenAngle > THR
          ||
          (
           OpenAngle >= THR - HYST
           &&
           Model.get_hc_state() == TModel::THC::_OPENED
          )
         )
      {
        NewState = TModel::THC::_OPENED;
      }
      else
      {
        NewState = TModel::THC::_CLOSED;
      }
      
//      static uint16_t ErrOpenAngleCtr = 0U;
//      if ( 
//          OpenAngle > 7
//          ||
//          OpenAngle < -7
//         )
//      {
//        ErrOpenAngleCtr++;
//      }
      
      Model.set_hc_state( NewState );
      
      typedef void (TDiscreteOut::*TFnct)();
      
      TFnct Out[ TContact::_N_MAX ][ TModel::THC::_HC_MAX ] =
      {
        {
         &TDiscreteOut::open,   //[_N_OPENED][_CLOSED],
         &TDiscreteOut::closed, //[_N_OPENED][_OPENED],
        },
        {
         &TDiscreteOut::closed, //[_N_CLOSED][_CLOSED],
         &TDiscreteOut::open,   //[_N_CLOSED][_OPENED],
        },
      };
      
      ( Do.*Out[ Model.get_d_o_sets() ][ NewState ] )();      
    }
//  }
}

static bool sens_dev_stage( const SemaphoreHandle_t *Sem, uint32_t Waiting )
{
  if ( 
      xSemaphoreTake( *Sem, pdMS_TO_TICKS( Waiting ) )                                          
      == 
      pdFAIL
     )
  {
    if ( ++DevCtrl.SensDevCtr.Val >= DevCtrl.SensDevCtr.Max )
    {
      DevCtrl.SensDevCtr.Val = 0U;
      Model.set_connect( TModel::TConnect::_NOT );
    }    
    return false;
  }
  else
  {
    DevCtrl.SensDevCtr.Val = 0U;
    Model.set_connect( TModel::TConnect::_YES );
    
    return true;
  }
}

static void hc_ctrl()
{
//  do {} while ( Model.get_hc_acc_calib() == TModel::TCalib::__UNCALIBRATED  ); //зависание в ожидании калибровки датчика
  if ( 
      Model.get_start_meas_cmd() == TModel::TMeas::__START_MEAS //если получена команда на запуск измерений...
      ||                                                        //...или...
      Model.get_interconn() == TInterconn::__OTHER_MASTER       //...крышка взаимодействует не с базой
     )
  {
    Model.set_start_meas_cmd( TModel::TMeas::__STOP_MEAS ); //сброс флага до прихода следующего запроса
    
//    Do.closed();
    xSemaphoreGive( DevMemsStartSem ); //выдать семафор запуска измерений своего МЭМС
  
    
    if ( 
        xSemaphoreTake( MemsDevDoneSem, pdMS_TO_TICKS( TDevCtrl::__WAITING_MEMS_DEV_DONE_MS ) )                                          
        ==
        pdFAIL
       )
    {
      Model.set_hc_mems( TModel::TMainState::__ERR );               //ошибка МЭМС
      Model.set_sample_valid_sign( TModel::TValidSign::__INVALID ); //текущее окно выборок невалидно (даже если МЭМС установил окно валидным, \
                                                                      но по каким-то причинам не успел отдать семафор)
      return;
    }
    else
    {
      //валидность текущего окна выборок определяется в МЭМС
      if ( Model.get_sample_valid_sign() == TModel::TValidSign::__INVALID )
      {
        //сохраняются предыдущие значения
      }
      else
      {
        TModel::TAxisRotate AxisRotate = static_cast<TModel::TAxisRotate>(Model.get_axis_rotate());
        Model.set_hc_mems( TModel::TMainState::__OK);
        Angles.cnt_my_angle( TModel::TDevType::_HC, AxisRotate, true ); //преобразовать угол датчика в формат ( 0..260, -1..-80 )
        
        int16_t OpenAngle = Model.get_my_angle();
        
        if (
            OpenAngle < CONSTS::THR_ERR_OPEN_ANGLE
            ||
            (
             OpenAngle <= CONSTS::THR_ERR_OPEN_ANGLE + CONSTS::HYST_ERR_OPEN_ANGLE
             &&
             Model.get_hc_pos_err() == TModel::TPosErr::_POS_ERR
            )
           )
        {
          Model.set_hc_pos_err( TModel::TPosErr::_POS_ERR );
          //состояние крышки остается прежним, значение угла обновлено
        }
        else
        {
          Model.set_hc_pos_err( TModel::TPosErr::_POS_OK );
          //определить новое состояние
          uint8_t THR  = CONSTS::THR_SENS_OPEN_ANGLE;
          uint8_t HYST = CONSTS::HYST_SENS_OPEN_ANGLE;
          
          TModel::THC NewState;

          if (
              OpenAngle > THR
              ||
              (
               OpenAngle >= THR - HYST
               &&
               Model.get_hc_state() == TModel::THC::_OPENED
              )
             )
          {
            NewState = TModel::THC::_OPENED;
          }
          else
          {
            NewState = TModel::THC::_CLOSED;
          }
      
          Model.set_hc_state( NewState );
          
          typedef void (TDiscreteOut::*TFnct)();
          
          if ( Model.get_interconn() == TInterconn::__BASE ) //если взаимодействие с базой
          {
            Do.open();
          }
          else
          {
            TFnct Out[ TContact::_N_MAX ][ TModel::THC::_HC_MAX ] =
            {
              {
               &TDiscreteOut::open,   //[_N_OPENED][_CLOSED],
               &TDiscreteOut::closed, //[_N_OPENED][_OPENED],
              },
              {
               &TDiscreteOut::closed, //[_N_CLOSED][_CLOSED],
               &TDiscreteOut::open,   //[_N_CLOSED][_OPENED],
              },
            };
            
            ( Do.*Out[ Model.get_d_o_sets() ][ NewState ] )();
          }
        }
      }
      
      
//      Do.open();
    }    
  }
  else
  {
    vTaskDelay( pdMS_TO_TICKS( 3U ) );
  }
  
  
       
//  xTimerStart( CodeSwTmr, 0U );
//  
//  do
//  {
//    if ( xSemaphoreTake( MemsWaitStopSem, 0U ) == pdPASS ) //если время ожидания ответа от датчика истекло
//    {
//      return; //повторная отправка запроса
//    }
//  }      
//  while ( Model.get_start_meas_cmd() != TModel::TMeas::__START_MEAS );
//  mems_process();
}
//-------------------------------------------------------------------------------------------------------------------
