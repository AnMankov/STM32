
#include <algorithm>
#include <map>

#include "mems.h"
#include "model.h"
#include "motion_tl.h"
#include "discrete_out.h"

TMems Mems = { AccMagGyro };

uint8_t PosCtr;

TMemsAggregate AccelBuf; //массив данных акселерометра для обработки

constexpr double M_PI = 3.14159265358979323846;
constexpr TMems::TAxesHyst AxesHyst[TModel::TMemsOrient::_MAX] =
{
  {                  //X_UP
    {  0.9f,  1.1f },
    { -0.1f,  0.1f },
    { -0.1f,  0.1f },
  },
  {                  //X_DOWN
    { -1.1f, -0.9f },
    { -0.1f,  0.1f },
    { -0.1f,  0.1f },
  },
  {                  //Y_UP
    { -0.1f,  0.1f },
    {  0.9f,  1.1f },
    { -0.1f,  0.1f },
  },
  {                  //Y_DOWN
    { -0.1f,  0.1f },
    { -1.1f, -0.9f },
    { -0.1f,  0.1f },
  },
  {                  //Z_UP
    { -0.1f,  0.1f },
    { -0.1f,  0.1f },
    {  0.9f,  1.1f },
  },
  {                  //Z_DOWN
    { -0.1f,  0.1f },
    { -0.1f,  0.1f },
    { -1.1f, -0.9f },
  },
};

//constexpr TMems::TSem CalibSem[TMems::TMemsOrient::_MAX] =
//{
//  { &AccCalib_X_UP_Sem,   TMems::TMemsOrient::_X_UP   },
//  { &AccCalib_X_DOWN_Sem, TMems::TMemsOrient::_X_DOWN },
//  { &AccCalib_Y_UP_Sem,   TMems::TMemsOrient::_Y_UP   },
//  { &AccCalib_Y_DOWN_Sem, TMems::TMemsOrient::_Y_DOWN },
//  { &AccCalib_Z_UP_Sem,   TMems::TMemsOrient::_Z_UP   },
//  { &AccCalib_Z_DOWN_Sem, TMems::TMemsOrient::_Z_DOWN },
//};

TMems::TMems( MPU_9250::TAccGyroMagDriver_HL &_Hw )
:
Hw( _Hw )
{

}

TMems::~TMems()
{

}

bool TMems::chk_sample(const TModel::TAccelData &Data, TModel::TMemsOrient _ORIENT)
{
  return (
          ( Data.X >= AxesHyst[_ORIENT].X.Low )
          &&         
          ( Data.X <= AxesHyst[_ORIENT].X.High )
          &&         
          ( Data.Y >= AxesHyst[_ORIENT].Y.Low )
          &&         
          ( Data.Y <= AxesHyst[_ORIENT].Y.High )
          &&         
          ( Data.Z >= AxesHyst[_ORIENT].Z.Low )
          &&         
          ( Data.Z <= AxesHyst[_ORIENT].Z.High )
         );
}

//----- процедура калибровки акселерометра ---------------------------
void TMems::acc_calib()
{
//  constexpr uint16_t DLY_MS         =   5U;
  constexpr uint16_t ACCEL_BUF_SIZE = 100U;
  uint32_t           SampleCtr      =   0U;
  TModel::TAccelData AccelBuf[ACCEL_BUF_SIZE]; //массив калибровочных данных акселерометра для обработки
  TModel::TAccelData Data;
  MTL_acc_cal_t      acc_cal;
  
  bool Repeat = false;     //флаг повторной калибровки

  auto begC = Model.CalibSem;
  auto endC = &Model.CalibSem[TModel::TMemsOrient::_MAX];

  auto cEl = begC;
  cEl = std::find_if( begC, endC, []( TModel::TSem item ){
    return ( xSemaphoreTake( *item.Sem, 0U ) ); //pdPASS эквивалентно true
  } );
 
  size_t RtosHeapSize = xPortGetFreeHeapSize();
 
  if (
      endC == cEl
     )
  {
    //калибровочных семафоров выдано не было
  }
  else //если какой-либо из калибровочных семафоров был выдан, то защелкиваем задачу на режим калибровки
  {
    //для получения достовеного результата калибровки\
      необходимо за один заход откалибровать\
      датчик во всех направлениях
    do
    {
      if ( Repeat == true ) //если установлен флаг повторной калибровки
      {
        wait_sem( endC, begC, &cEl );
      }
      
//      Model.set_calib_position_ctr( TModel::TMemsOrient::_MAX  );
      Model.set_calib_position_ctr( TModel::TMemsOrient::_RESET );
    
      SemaphoreHandle_t *Tmp[ TModel::TMemsOrient::_MAX ] = 
      {
        nullptr, nullptr, nullptr, nullptr, nullptr, nullptr
      };

      auto begT = Tmp;
      auto endT = &Tmp[TModel::TMemsOrient::_MAX];

      for (;;)
      {
        while ( SampleCtr < ACCEL_BUF_SIZE )
        {
          if ( !chk_mems_state() ) //ожидание выборки
					{
					  return;
					}

          Hw.collect_acc_data();
          Data = *Hw.get_acc_data();

          //сохранение выборок по 3-м осям
          AccelBuf[SampleCtr] = Data;

          if ( !chk_sample( AccelBuf[SampleCtr++], cEl->MemsOrient ) ) //если выборка невалидная
          {
            --SampleCtr; //следующая итерация перезапишет невалидную выборку
          }
        }
        SampleCtr = 0U;
        //набрано ACCEL_BUF_SIZE выборок

        float _AccelBuf[ACCEL_BUF_SIZE][3];
        auto beg = reinterpret_cast<float *>(AccelBuf);
        auto end = reinterpret_cast<float *>(&AccelBuf[ACCEL_BUF_SIZE]);
        std::copy( beg, end, reinterpret_cast<float *>(_AccelBuf) ); //копия в промежуточный массив

        MotionTL_CalibratePosition( _AccelBuf, ACCEL_BUF_SIZE, static_cast<MTL_cal_position_t>(cEl->MemsOrient) );
        std::fill( beg, end, 0U ); //обнуление массива с выборками

        Model.set_calib_process( TModel::TCalibProcess::__PERFORMED ); //ожидание следующей команды калибровки
               
//        Model.set_calib_position_ctr( Model.get_calib_position_ctr() - 1U );

        Tmp[cEl->MemsOrient] = cEl->Sem; //если был выдан тот же семафор что уже был выдан ранее,\
                                           то произойдет повторная калибровка в этом же направлении
                                           
//        uint8_t UncalibPosCtr = 6U;
//        for ( auto item : Tmp )
//        {
//          if ( item != nullptr )
//          {
//            --UncalibPosCtr;
//          }
//        }
        PosCtr = static_cast<uint8_t>(cEl->MemsOrient);
        
        Model.set_calib_position_ctr( cEl->MemsOrient );

        if ( 
            endT != std::find_if( begT, endT, []( SemaphoreHandle_t *item ){
                      return ( item == nullptr ); //pdPASS эквивалентно true
                    } )
           ) //если еще есть положения, в которых не было произведено калибровки
        {
          wait_sem( endC, begC, &cEl );
          Model.clr_calib_position_ctr( cEl->MemsOrient ); //сброс счетчика при повторной калибровке, если это необходимо          
        }
        else
        {          
          if ( MotionTL_GetCalValues( &acc_cal ) != MTL_cal_result_t::CAL_PASS )
          {
            Repeat = true; //необходима повторная калибровка
            Model.set_calib_process( TModel::TCalibProcess::__DENY ); //ожидание следующей команды калибровки
          }
          else
          {
            Repeat = false; //калибровка окончена успешно
            Model.set_calib_process( TModel::TCalibProcess::__PERFORMED );
          }
          
          break;
        }
      }
    } 
		while ( Repeat );

    Model.set_accel_calib_sign( TModel::TIsAccCalib::_ACC_CALIBRATED );
    Model.set_accel_calib_data( *(TModel::TAccCalData *)&acc_cal );
  }
}
//--------------------------------------------------------------------

bool TMems::chk_mems_state()
{
  TModel::TDevType DevType = Model.get_dev_type();
	TModel::TMainState Err = TModel::TMainState::__ERR;
	
	if ( xSemaphoreTake( RawDataMems_RdySem, pdMS_TO_TICKS( 100U ) ) == pdFAIL ) //waiting external interrupt
  {
		if ( DevType == TModel::TDevType::_BASE )
    {
  	  Model.set_base_mems( Err );
  	}
    else
    {
  	  Model.set_hc_mems( Err );
  	}
		
  	return false;
  }
	
	return true;
}

void TMems::wait_sem( const TModel::TSem *endC, const TModel::TSem *begC, const TModel::TSem **cEl )
{
  constexpr uint16_t DLY_MS = 5U;
  
  while (
          endC == ( *cEl = std::find_if( begC, endC, []( TModel::TSem item ){
            return ( xSemaphoreTake( *item.Sem, 0U ) ); //pdPASS эквивалентно true
          } ) )
        ) //пока не выдан никакой новый калибровочный семафор, задача подвисает в ожидании 
  {
    vTaskDelay( pdMS_TO_TICKS( DLY_MS ) );
  }
}

//----- сбор и накопление данных -------------------------------------
void TMems::acquire_data( TMemsAggregate &Aggregate )
{
  uint16_t SampleCtr = 0U;
  TModel::TAccelData Data;
  
  while ( SampleCtr < ACCEL_BUF_SIZE )
  {
    xSemaphoreTake(RawDataMems_RdySem, portMAX_DELAY); //waiting external interrupt
      Hw.collect_acc_data();
      AccelBuf[SampleCtr++] = *Hw.get_acc_data(); //сохранение выборок по 3-м осям
  }
}
//--------------------------------------------------------------------

//----- вычисление среднего значения из накопленных данных -----------
void TMems::cnt_average( const TMemsAggregate &Aggregate, TModel::TAccelData &Average )
{
  uint16_t Cnt = 0U;
  
  for ( auto item : Aggregate )
  {
    Average.X += item.X;
    Average.Y += item.Y;
    Average.Z += item.Z;
    
    ++Cnt;
  }
  
  if ( Cnt != 0U )
  {
    Average.X /= Cnt;
    Average.Y /= Cnt;
    Average.Z /= Cnt;
  }
}
//--------------------------------------------------------------------

//----- проверка валидности накопленных данных с выборками -----------
bool TMems::is_valid_data( const TMemsAggregate &Aggregate ) //возвращает true если все выборки валидные
{  
  constexpr float dG = 0.3f; 
  
  for ( auto item : Aggregate )
  {
    static auto Prev = item;
    
    if (
        item.X > VALID_THR
        ||
        item.X < -VALID_THR
        ||
        item.Y > VALID_THR
        ||
        item.Y < -VALID_THR
        ||
        item.Z > VALID_THR
        ||
        item.Z < -VALID_THR
        ||
        Prev.X - item.X > dG
        ||
        Prev.X - item.X < -dG
        ||
        Prev.Y - item.Y > dG
        ||
        Prev.Y - item.Y < -dG
        ||
        Prev.Z - item.Z > dG
        ||
        Prev.Z - item.Z < -dG
       )
    {
      Prev = item;
      return false;
    }
    
    Prev = item;
  }
  
  return true;
}
//--------------------------------------------------------------------

//----- демпфирование ------------------------------------------------
void TMems::cnt_damp( __packed float &damp_axis, float cur_axis, float coeff )
{
  damp_axis = damp_axis + ( cur_axis - damp_axis ) / coeff;
}
//--------------------------------------------------------------------

static void mems_process();

//----- Задача RTOS --------------------------------------------------
void mems( void *Params ) //работа с МЭМС-датчиком
{
//  constexpr uint8_t DLY = 10U; //[мс]
  constexpr uint8_t WAIT_CALIB_DLY = 10U; 

  do
  {
    AccMagGyro.init_driver();
    AccMagGyro.init_aux();
    AccMagGyro.init_chip();
  } while( AccMagGyro.check_id_acc_gyro() == false );

  LL_AHB1_GRP1_EnableClock( LL_AHB1_GRP1_PERIPH_CRC ); //UM2277, rev.6, page 3: the CRC module in STM32 microcontroller \
                                                         (in RCC peripheral clock enable register) has to be enabled \
                                                         before using the library
  MotionTL_Initialize();

  constexpr char *acc_orientation = "enu";
  MotionTL_SetOrientation_Acc( acc_orientation );

  constexpr uint8_t LIB_VERSION_BUF_SIZE  = 35;
  char LibAcVersion[LIB_VERSION_BUF_SIZE] = { 0 };
  MotionTL_GetLibVersion( LibAcVersion );

  for ( ;; )
  {
    mems_process();
  }
}
//\---- Задача RTOS -------------------------------------------------------------------------------------------------

static void mems_process()
{
  static TModel::TAccelData AccAverage = { 0.0f, 0.0f, 0.0f };
  constexpr uint32_t RE_READY_DLY = 5U;
  
//  Model.set_start_meas_cmd( TModel::TMeas::__STOP_MEAS ); //сброс команды до начала следующего окна
  
	xSemaphoreGive( MemsDevRdySem ); //МЭМС готов измерять
  if ( xSemaphoreTake( DevMemsStartSem, RE_READY_DLY ) == pdFAIL )
  {
    static uint8_t Ctr;
    ++Ctr;
    
    return; //повторная попытка начать измерения
  }
//  Do.closed();
  Mems.acquire_data( AccelBuf );
//  Do.open();

  if ( Mems.is_valid_data( AccelBuf ) == true ) //если все выборки текущего окна валидные
  {
    
    Model.set_sample_valid_sign( TModel::TValidSign::__VALID );
    
    AccAverage = { 0.0f, 0.0f, 0.0f };
    Mems.cnt_average( AccelBuf, AccAverage ); //вычисление средних значений по осям
    
    static float AccelAxisSum = 0.0f;
    AccelAxisSum = AccAverage.X + AccAverage.Y + AccAverage.Z;

    struct TCnt
    {
      float Numerator;
      float Denominator;
    };
    
    TCnt RawPitch = { AccAverage.X, sqrt( pow( AccAverage.Y, 2 ) + pow( AccAverage.Z, 2 ) ) };
    TCnt RawRoll  = { AccAverage.Y, sqrt( pow( AccAverage.X, 2 ) + pow( AccAverage.Z, 2 ) ) };
        
    Model.set_raw_pitch( roundf( atan2f( RawPitch.Numerator, RawPitch.Denominator ) * 180.0f / M_PI ) );
    Model.set_raw_roll( roundf( atan2f( RawRoll.Numerator, RawRoll.Denominator ) * 180.0f / M_PI ) );
    
    Model.set_accel_z( AccAverage.Z );

//    float NumeratorP   = AccAverage.X;
//    float Denominator = sqrt( pow( AccAverage.Y, 2 ) + pow( AccAverage.Z, 2 ) );
//    float DenominatorRoll = sqrt( pow( AccAverage.X, 2 ) + pow( AccAverage.Z, 2 ) );
//    static float PitchOld;
//    static float Pitch;
//    static float RollOld;
//    static __packed float PrevPitch = 0.0f;

//    PitchOld = atan2f( Numerator, Denominator ) * 180.0f / M_PI;
//    RollOld = atan2f( AccAverage.Y, DenominatorRoll ) * 180.0f / M_PI;
//    
//    
//    Pitch = Mems.convert_angle( PitchOld, AccAverage.Z, Model.get_dev_type() );
//    Mems.cnt_damp( PrevPitch, Pitch, 1.0f ); //1.0f => без демпфирования
//    
//    Model.set_my_angle( roundf( PrevPitch ) );
    
    auto beg = reinterpret_cast<float *>(AccelBuf);
    auto end = reinterpret_cast<float *>(&AccelBuf[ACCEL_BUF_SIZE]);
    std::fill( beg, end, 0U ); //обнуление массива с выборками
    
    
    
//	  xSemaphoreGive( MemsDevDoneSem ); //измерения готовы
    
  }
  else
  {
    Model.set_sample_valid_sign( TModel::TValidSign::__INVALID );
  }
	
  xSemaphoreGive( MemsDevDoneSem ); //измерения готовы
}
//\------------------------------------------------------------------------------------------------------------------

//----- Таймер RTOS -------------------------------------------------------------------------------------------------
//void mems_tmr( TimerHandle_t xTimer ) //обработка таймера кодового переключателя
//{
//  xSemaphoreGive(MemsWaitStopSem);
//}
//\---- Таймер RTOS -------------------------------------------------------------------------------------------------
