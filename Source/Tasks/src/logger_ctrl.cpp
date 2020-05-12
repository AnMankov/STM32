#include <algorithm>

#include "rtos_headers.h"
#include "logger_ctrl.h"
//#include "AT45DB321D_driver_hl.h"
#include "model.h"
#include "led.h"
#include "button.h"
#include "Flash_driver.h"

static TLoggerState LoggerState = TLoggerState::__OFF;

uint8_t Buf[ TFlash::PAGE_SIZE ];

static void get_data( TRec &, TModel & );
static void buf_process( const TRec *, uint8_t *Buf, uint16_t BufSize );


//AT45DB321D_DRIVER_HL AT45( 
//                           Spi_HW, 
//                           AT45DB321D_DRIVER_HL::TRate::__MAX 
//                          );

void logger_ctrl( void *Params )
{
  constexpr uint32_t TASK_DLY_MS = 1000U;
  
  if ( Model.get_dev_type() != TModel::TDevType::_BASE )
  {
    for ( ;; )
    {
      vTaskDelay( pdMS_TO_TICKS( TASK_DLY_MS ) );
    }
  }
  Model.set_led_mode( TModel::TLedMode::__CONST_ON );
	

//  AT45.init_driver();
//  AT45.check_chip();

//  TRec Rec;
//  get_data( Rec, Model ); //получать данные необходимо синхронно с их появлением на базе
//                          //когда данные записывать разберется драйвер флэш
//  AT45.write( static_cast<const uint8_t *>(&Rec), __REC_SIZE );
  

  for ( ;; )
  {
	  if ( Model.get_btn_mode() == TModel::TPress::__RELEASE_LESS_1_SEC )
    {
      Model.set_btn_mode( TModel::TPress::__RELEASE );
      Model.set_led_mode( TModel::TLedMode::__BLINK_1_S );
      
      for ( ;; )
      {
        if ( xSemaphoreTake( WrFlash, 0U ) == pdPASS )
        {
          static TRec Rec;
          get_data( Rec, Model ); //получать данные необходимо синхронно с их появлением на базе
                                  //когда данные записывать разберется драйвер флэш
          buf_process( &Rec, Buf, TFlash::PAGE_SIZE );
        }
        
        if ( Model.get_btn_mode() == TModel::TPress::__RELEASE_LESS_1_SEC )
        {
          Model.set_btn_mode( TModel::TPress::__RELEASE );
          Model.set_led_mode( TModel::TLedMode::__CONST_ON );
          break;
        }
        
        vTaskDelay( pdMS_TO_TICKS( 10U ) );
        
      }
    }
    
    vTaskDelay( pdMS_TO_TICKS( 10U ) );
    												
		  
//      vTaskDelay( pdMS_TO_TICKS( TASK_DLY_MS ) );
  }  
}

static void get_data( TRec &Rec, TModel &Model )
{
  Rec.Base.CodeSw          = Model.get_code_sw();
  Rec.Base.Angle           = Model.get_my_angle();
  Rec.Base.SampleValidSign = Model.get_sample_valid_sign();
  
  Rec.Sens.CodeSw          = Model.get_hc_code_sw();
  Rec.Sens.Angle           = Model.get_sens_angle();
  Rec.Sens.SampleValidSign = Model.get_sens_sample_valid_sign();
  
  Rec.OpenAngle            = Model.get_open_angle();
  Rec.State                = Model.get_dev_state();
  
}

static void buf_process( const TRec *Src, uint8_t *Buf, uint16_t BufSize )
{
  static uint16_t Ix = 0U;
	uint16_t NewIx = 0U;
	
	NewIx = Ix + __REC_SIZE;
	
	if ( NewIx > BufSize ) //если новая запись не помещается в буфер
	{
	  NewIx = 0U;
		//записать текущий буфер во флэш
    Model.set_led_mode( TModel::TLedMode::__CONST_OFF );
    
    if ( Flash.DataPageCtr == TFlash::DATA_PAGE_QTY ) //проверка счетчика страниц под запись данных во Flash
    {
      Model.set_btn_mode( TModel::TPress::__RELEASE_LESS_1_SEC );
      return;
    }  
		Flash.write_data( Buf );
    Model.set_led_mode( TModel::TLedMode::__BLINK_1_S );
    
    //очистить буфер
    auto beg = reinterpret_cast<uint8_t *>( Buf );
    auto end = reinterpret_cast<uint8_t *>( &Buf[ BufSize ] );   
    std::fill( beg, end, 0U ); //обнуление массива с выборками
		
    //записать в пустой буфер новую запись
		auto Beg = reinterpret_cast<const uint8_t *>( Src );
    auto End = &(reinterpret_cast<const uint8_t *>( Src ))[__REC_SIZE];
    std::copy( Beg, End, reinterpret_cast<uint8_t *>( Buf ) );   
	}
	else
	{
    //записать в буфер новую запись
		auto Beg = reinterpret_cast<const uint8_t *>( Src );
    auto End = &(reinterpret_cast<const uint8_t *>( Src ))[__REC_SIZE];
    std::copy( Beg, End, &(reinterpret_cast<uint8_t *>( Buf ))[ Ix ] );
	}
	
	Ix = NewIx;
}
