#include <limits.h>
#include <math.h>

#include <string.h>
#include <numeric>
#include "rtos_tasks_master.h"
#include "SensProc.h"
#include "SensDev.h"
#include "modbus.h"
#include "cmdcodes_1.h"
//#include "angles_handler.h"
#include "angles_cnt.h"

enum class TCntState : uint8_t //состояние вычислительного процесса
{
  _NEW_SAMPLE = 0,
  _CTR_CHK    = 1,
};

enum class TSampleLocate : uint8_t //признак текущего положения выборки в периоде
{
  _START = 0, //начало периода
  _POS   = 1, //положительная полуволна
  _NEG   = 2, //отрицательная полуволна
};

enum class TFirstHWaveSign : uint8_t //знак первой полуволны
{
  _POS = 0, //положительная полуволна
  _NEG = 1, //отрицательная полуволна
};

struct TSigIndicate
{
  TSampleLocate SampleLocate;
  TFirstHWaveSign FirstHWave;
};

struct TAxesSigIndicate
{
  TSigIndicate X;
  TSigIndicate Y;
  TSigIndicate Z;
};

struct TAccSampleCtr
{
  uint16_t GreaterThan; //счетчик значений, больших чем среднее
  uint16_t LessThan;    //счетчик значений, меньших чем среднее  
  uint8_t  CrossOver;   //счетчик переходов через среднее значение
};

struct TAxesAccSampleCtr
{
  TAccSampleCtr X;
  TAccSampleCtr Y;
  TAccSampleCtr Z;
};

struct TGoodAcc
{
  uint16_t Ctr; //счетчик валидных выборок
  float Sum;    //сумма валидных выборок
};

struct TAxesGoodAcc
{
  TGoodAcc X;
  TGoodAcc Y;
  TGoodAcc Z;
};

//----- Переменные и константы -----------------------------------------------------------------------------------------------------
uint8_t MainBuf[MAIN_BUF_SIZE];         //буфер основных данных работы устройства

static TSys *pSys = (TSys *)MainBuf;

constexpr uint8_t  MIN_LEN_SATE_BUF    =    2U; //минимальное количество пакетов состояний в буфере \
                                                  для передачи одного пакета состояния
constexpr uint16_t ACCEL_BUF_SIZE = 200U;
TAccelData AccelBuf[ACCEL_BUF_SIZE];       //массив данных акселерометра для обработки
                                                  
UnStdSostPkg StateBuf[MIN_LEN_SATE_BUF]; //буфер пакетов состояний

constexpr uint8_t LIN_MAX_RX_BUF_SIZE = 255U;
uint8_t mbuf[LIN_MAX_RX_BUF_SIZE];       //буфер приема пакетов Линии

TGyroData GyroBiasData = {0.0f, 0.0f, 0.0f};

uint8_t FrameTxBuf[TX_BUF_SIZE];                //кадр данных для отправки на ПК

static uint8_t ErrCtr = 0;
bool TxEndFlag = true;

static TConnect Connect = []()->TDevConnect
                          {
                            TDevConnect DevConnect;
                            DevConnect.Reg = 0xFF; //по умолчанию все устройства _NOT_CONNECTED
                            
                            return DevConnect;
                          }
                          ();

const __packed float *pAngle = &((TMainData *)MainBuf)->PcSendData.MotionFXData.Angles.Roll;
volatile TCanData *rx0CanData;
volatile TCanData *rx1CanData;
volatile uint8_t State = 0U;

static uint8_t x;
static uint8_t y;
static uint8_t z;

//TAnglesHandler AnglesHandler(100U); //глобальный => в задачах должен быть защищен мьютексом
//----------------------------------------------------------------------------------------------------------------------------------

//----- Объявления служебных функций (Declaration) ---------------------------------------------------------------------------------
static uint8_t SENS_Parse();
static void UART_Process();
static bool chk_dev_sets(MySettings &DevSets, const MySettings &CurSets); //проверка допустимости настроек \
                                                                            функция подтверждает только валидные настройки \
														                                                невалидные - возвращаются в предыдущее состояние
static void chk_cal_req(); //действия в зависимости от интерфейса через который поступил запрос на калибровку магнитометра
static bool chk_trig(uint8_t Cur, uint8_t Prev, bool TrigLevel); //проверка срабатывания бита в байте состояния

static void cnt_damp(__packed float &damp_axis, float cur_axis, float coeff);
static TCntState axis_handle(float Sample, float Average, TAccSampleCtr &, TSigIndicate &);
static void motion_tl_init();
static bool chk_sample(const TAccelData &, const TMemsOrient); //проверка допустимости текущей выборки для калибровки
																				
extern TPntFlag PntFlag;

//----------------------------------------------------------------------------------------------------------------------------------
static TCntState axis_handle(float Sample, float Average, TAccSampleCtr &Ctr, TSigIndicate &Sig)
{
  if (
      ( Sample >= Average )
      &&
      ( Sig.SampleLocate == TSampleLocate::_START )
     )
  {
    Sig.SampleLocate = TSampleLocate::_POS;
    Sig.FirstHWave   = TFirstHWaveSign::_POS;
    ++Ctr.GreaterThan;
    return TCntState::_NEW_SAMPLE; //необходимо получить новую выборку    
  }
  else
  {
    if ( Sample >= Average )
    {
      //признак положения в периоде - не старт
      if ( Sig.SampleLocate == TSampleLocate::_NEG )
      {
        //переход из отрицательной полуволны в положительную
        Sig.SampleLocate = TSampleLocate::_POS;
        ++Ctr.CrossOver;
        return TCntState::_CTR_CHK; //далее необходимо проверять счетчик переходов
      }
      else
      {
        Sig.SampleLocate = TSampleLocate::_POS;
        ++Ctr.GreaterThan;
        return TCntState::_NEW_SAMPLE; //необходимо получить новую выборку
      }
    }
    else
    {
      if ( Sig.SampleLocate == TSampleLocate::_START )
      {
        Sig.SampleLocate = TSampleLocate::_NEG;
        Sig.FirstHWave   = TFirstHWaveSign::_NEG;
        ++Ctr.LessThan;
        return TCntState::_NEW_SAMPLE; //необходимо получить новую выборку
      }
      else
      {
        if ( Sig.SampleLocate == TSampleLocate::_POS )
        {
          //переход из положительной полуволны в отрицательную
          Sig.SampleLocate = TSampleLocate::_NEG;
          ++Ctr.CrossOver;
          return TCntState::_CTR_CHK; //далее необходимо проверять счетчик переходов          
        }
        else
        {
          Sig.SampleLocate = TSampleLocate::_NEG;
          ++Ctr.LessThan;
          return TCntState::_NEW_SAMPLE; //необходимо получить новую выборку
        }
      }
    }
  }
}


static void cnt_damp(__packed float &damp_axis, float cur_axis, float coeff)
{
  damp_axis = damp_axis + (cur_axis - damp_axis) / coeff;
}

//----- Задачи FreeRTOS ------------------------------------------------------------------------------------------------------------
void vUSART(void *pvParameters) //задача определена только для платформы
{
//  uint8_t CurProtMod;     //текущее значение модификатора протокола
  bool ReinitUsartReq = false; //флаг защиты текущей передаваемой посылки от внезапного отключения интерфейса (при переинициализации) \
                                 запрос на переинициализацию USART может поступить по протоколу "СЕНС" через "ЛИН" и через RS-485
  bool ReinitProcReq = false;  //флаг защиты при изменении настроек протокола (Пр. адрес Modbus; модификатор протокола)

//  uint8_t CurProc;
  uint8_t MBAddr;         //адрес Modbus
  
  taskENTER_CRITICAL(); //к HardDSt без критической секции можно обращаться только в задаче vLineData
    MBAddr = HardDSt.MBAddr;
  taskEXIT_CRITICAL();

  constexpr uint32_t DELAY_TIME_MS = 1000U;

  if (Device != TDevice::_PLATF) //
  {
    for (;;)
	  {
       vTaskDelay(DELAY_TIME_MS / TICK_RATE_MS); //если устройство не платформа, то задача подвисает
	  }
  }

  Usart.pin_clk_config();
  Usart.usart_hw_init();

  MB_Init(MBAddr);
  MB_ResetCnt();

  for (;;)
  {
	  /* Обработка запроса
	  */
    auto sens_mb = [
                     &ReinitUsartReq,
                     &ReinitProcReq,
                     &MBAddr
                   ]()
    {   
      MySettings Sets;
      xSemaphoreTake(SENSMutex, portMAX_DELAY); //захватить мьютекс для атомарной работы с HardDSt        
        taskENTER_CRITICAL();                   //к HardDSt обращаться только в критической секции
          Sets = HardDSt;
        taskEXIT_CRITICAL();		     	
      xSemaphoreGive(SENSMutex);                //освободить мьютекс для атомарной работы с HardDSt
      
//	    uint8_t CurMBAddr = Sets.MBAddr;
      
	    USART::TSet USet;
      
	    constexpr USART::TBaudRate BR[] = 
	    {
        USART::TBaudRate::_1200, 
        USART::TBaudRate::_2400, 
        USART::TBaudRate::_4800, 
        USART::TBaudRate::_9600, 
        USART::TBaudRate::_14400, 
        USART::TBaudRate::_19200, 
        USART::TBaudRate::_38400, 
        USART::TBaudRate::_56000, 
        USART::TBaudRate::_57600, 
        USART::TBaudRate::_115200
      };
	    
	    struct TParStop
	    {
	      USART::TParity Parity;
	      USART::TStops  Stops;
	    };
	    
	    constexpr TParStop ParStop[] = 
	    {
	      {USART::TParity::_NONE, USART::TStops::_STOPBITS_1}, 
	      {USART::TParity::_NONE, USART::TStops::_STOPBITS_2}, 
	      {USART::TParity::_ODD,  USART::TStops::_STOPBITS_1}, 
	      {USART::TParity::_EVEN, USART::TStops::_STOPBITS_1}
	    };
	    
	    USet.BaudRate = BR[Sets.USpeed];
	    USet.Parity   = ParStop[Sets.UPar].Parity;
	    USet.Stops    = ParStop[Sets.UPar].Stops;

	    taskENTER_CRITICAL();
	      if ( !Usart.set_cmp(USet) && Usart.u.ustate == USART::TUsartState::U_IDLE ) //если настройки не совпадают с уже установленными \
                                                                                     переинициализацию необходимо осуществлять когда USART \
                                                                                     находится в режиме "тишина" (не передает и не принимает)
	      {
	        Usart.set_chng(USet); //обновить данные класса для дальнейшего аппаратного применения настроек \
	                               изменение настроек не должно произойти ранее инициализации интерфейса USART
          Usart.hw_reinit();   //данные объекта Usart с аппаратными настройками уже изменены, \
                                 необходимо переинициализировать интерфейс в соответствии новыми настройками
          MB_Init(Sets.MBAddr);
          MB_ResetCnt();
	      }

        if (
             ( get_mb_addr() != Sets.MBAddr || (uint8_t)Usart.Proc != Sets.ProtMod )
             &&
             ( Usart.u.ustate == USART::TUsartState::U_IDLE )
           )
        
        {
          Usart.Proc = (TProc)Sets.ProtMod;
          
          MB_Init(Sets.MBAddr);
          MB_ResetCnt();
        }
      taskEXIT_CRITICAL();
      
      UART_Process();
    };
  
    constexpr uint32_t DELAY_TIME_MS = 1U;
    vTaskDelay(DELAY_TIME_MS / TICK_RATE_MS); //задержка для ускорения переключения контекста на более высокоприоритетные задачи

    auto dks = []()
    {
      xSemaphoreTake(DataAnglesReadySemphr, portMAX_DELAY); //     
      xSemaphoreTake(MainBufMutex, portMAX_DELAY); //захватить мьютекс для атомарной работы с MainBuf 
        if ( TxEndFlag )
        {
          TxEndFlag = false; //предыдущий пакет отправлен, начало передачи следующего пакета
        }
        else
        {
          ++ErrCtr; //передача не закончилась, в буфере на отправку кривые данные
        }
        
//        *(TPcSendData *)FrameTxBuf = ((TMainData *)MainBuf)->PcSendData;
        *(TSys *)FrameTxBuf = *(TSys *)MainBuf;
        
//        if (
//            ((TPcSendData *)FrameTxBuf)->MotionFXData.Angles.Yaw >= 50 ||
//            ((TPcSendData *)FrameTxBuf)->MotionFXData.Angles.Roll >= 50 ||
//            ((TPcSendData *)FrameTxBuf)->MotionFXData.Angles.Pitch >= 50
//           )
//        {
//          ++ErrCtr; //передача не закончилась, в буфере на отправку кривые данные
//        }              
      xSemaphoreGive(MainBufMutex); //освободить мьютекс для атомарной работы с MainBuf
                    
//      Usart.tx_burst(FrameTxBuf, sizeof(TPcSendData), TProc::_DKS_TO_PC); //по USART передается копия TxBuf
       
      Usart.tx_burst(FrameTxBuf, sizeof(TSys), TProc::_DKS_TO_PC); //по USART передается копия TxBuf
       
           
      xSemaphoreTake(UsartTxSemphr, portMAX_DELAY); //ждать семафор окончания отправки кадра с данными из прерывания \
                                                      пока не полуен семафор окончания отправки кадра, к приему не переходить                                                     
    };
    
    if ( Usart.Proc == TProc::_DKS_TO_PC )
    {
      dks(); //если usart интерфейс настроен для работы по протоколу _DKS_TO_PC
    }
    else
    {
      sens_mb(); //если usart интерфейс настроен для работы по протоколам Modbus или Sens
    }
  }
}

void vMemsHandler(void *pvParameters)
{
  constexpr uint32_t DELAY_TIME_MS = 150;

  constexpr int32_t CAN_WAIT_TIME = 20U;                           // [ms]
  
  enum TSwLibrary : uint8_t
  {
    DISABLE_LIBRARY = 0,
	  ENABLE_LIBRARY  = 1
  };
  
  RtosHeapSize = xPortGetFreeHeapSize();
  vTaskDelay(DELAY_TIME_MS / TICK_RATE_MS);                      //задержка отправки пакета для тестирования
    
//  do
//  {
    AccMagGyro.init_driver();
    AccMagGyro.init_aux();
    AccMagGyro.init_chip();
//  } while(AccMagGyro.AKM_ID != AccMagGyro.WIA);
  
  if ( Flash.get_settings().IsAccCalib == TIsAccCalib::_ACC_UNCALIBRATED )
  {
    motion_tl_init();
  }
  
  static TAccelData AccAverage     = {0.0f, 0.0f, 0.0f};
  static TAccelData PrevAccAverage = {0.0f, 0.0f, -1.0f};
  static TAccelData PrevAccel      = {0.0f, 0.0f, -1.0f};
  constexpr double M_PI = 3.14159265358979323846;
  static uint16_t SampleCtr = 0U;
  static TAccelData ViewData;
  static float Freq;
  static TAccelData Data;
       
  static TAxesSigIndicate Sig =
  {
    {                        //X
      TSampleLocate::_START,
      TFirstHWaveSign::_POS,
    },
    {                        //Y
      TSampleLocate::_START,
      TFirstHWaveSign::_POS,
    },
    {                        //Z
      TSampleLocate::_START,
      TFirstHWaveSign::_POS,
    },
  };

  static TAxesAccSampleCtr Ctr =
  {
    {0U, 0U, 0U}, //X
    {0U, 0U, 0U}, //Y
    {0U, 0U, 0U}  //Z
  };

  static TAxesGoodAcc GoodAcc =
  {
    {0U, 0.0f}, //X
    {0U, 0.0f}, //Y
    {0U, 0.0f}  //Z
  };

  if (::Device == TDevice::_PLATF)
  {
    xTimerStart(CheckConnectTimer, 0U); //для платформы - запуск таймера проверки связи
  }

  for (;;)
  {
	  if (::Device == TDevice::_PLATF)
    {
      xSemaphoreGive(AccRdySemphr);                   //семафор готовности работы с акселерометром задаче vCANTx
    }

    xSemaphoreTake(StartMeasSemphr, portMAX_DELAY);   //для платформы семафор выдает vCANTx - команда на запуск измерений крышкам отрправлена \
                                                        для крышек семафор выдает vCANRx0   - команда на запуск измерений от платформы принята
//    RelFour.on();
    while (SampleCtr < ACCEL_BUF_SIZE)
    {
      xSemaphoreTake(RawDataMemsRdySemphr, portMAX_DELAY); //waiting external interrupt
        
        
	      AccMagGyro.collect_acc_data();

        ViewData = *AccMagGyro.get_acc_data();
        Data     = *AccMagGyro.get_acc_data();

        
        //акселерометр уже откалиброван => к выборке можно применять смещение и масштабный коэффициент
        static TAccCalData Cal;
        Cal = Flash.get_settings().AccCalData;

        Data.X = ( Data.X - Cal.Offset.X ) * Cal.Gain.X;
        Data.Y = ( Data.Y - Cal.Offset.Y ) * Cal.Gain.Y;
        Data.Z = ( Data.Z - Cal.Offset.Z ) * Cal.Gain.Z;

        //сохранение выборок по 3-м осям
        AccelBuf[SampleCtr++] = Data;

        //накопление суммы для вычисления средних значений по осям
        AccAverage.X += Data.X;
        AccAverage.Y += Data.Y;
        AccAverage.Z += Data.Z;
    }
    
//    RelFour.off();
    //вычисление средних значений по осям за ≈275мс (1000 выборок)
    AccAverage.X /= ACCEL_BUF_SIZE;
    AccAverage.Y /= ACCEL_BUF_SIZE;
    AccAverage.Z /= ACCEL_BUF_SIZE;

	  if (::Device == TDevice::_PLATF) //для платформы
    {
      //!код не универсальный! Только для двух крышек
      //дождаться семафоров необходимо для синхронизации устройств
      if ( xSemaphoreTake(HC1DataRxdSemphr, CAN_WAIT_TIME / TICK_RATE_MS) == pdTRUE) //ожидание данных от крышки_1 - максимум 20мс
      {
        //сообщение от крышки_1 принято
      }
      else //за 20мс сообщение от крышки_1 не принято
      {
        xSemaphoreTake(MainBufMutex, portMAX_DELAY);
          ((TSys *)MainBuf)->HC[TDevice::_HC_1].SampleFlag = TMSampleFlag::_NO;
        xSemaphoreGive(MainBufMutex);
      }
      if ( xSemaphoreTake(HC2DataRxdSemphr, CAN_WAIT_TIME / TICK_RATE_MS) == pdTRUE) //ожидание данных от крышки_2 - максимум 20мс
      {
        //сообщение от крышки_2 принято
      }
      else //за 20мс сообщение от крышки_2 не принято
      {
        xSemaphoreTake(MainBufMutex, portMAX_DELAY);
          ((TSys *)MainBuf)->HC[TDevice::_HC_2].SampleFlag = TMSampleFlag::_NO;
        xSemaphoreGive(MainBufMutex);
      }
    }
    else //для крышек
    {

    }

    TMThrFlag ThrFlag = TMThrFlag::_NO;

    for ( uint16_t BufCtr = 0; BufCtr < ACCEL_BUF_SIZE; ++BufCtr )
    {
      
      if (
          AccelBuf[BufCtr].X > 1.5f 
          ||
          AccelBuf[BufCtr].Y > 1.5f 
          ||
          AccelBuf[BufCtr].Z > 1.5f
          ||
          AccelBuf[BufCtr].X < -1.5f 
          ||
          AccelBuf[BufCtr].Y < -1.5f 
          ||
          AccelBuf[BufCtr].Z < -1.5f
         )
      {
        ThrFlag = TMThrFlag::_YES; //порог превышен => полученное окно с выборками невалидно
//        break;
      }

      if ( axis_handle(AccelBuf[BufCtr].X, AccAverage.X, Ctr.X, Sig.X) == TCntState::_CTR_CHK )
      {
        if ( Ctr.X.CrossOver == 2U )
        {
          //есть целый период сигнала
          Ctr.X.CrossOver    = 0U;
          if ( Ctr.X.GreaterThan + Ctr.X.LessThan < 727) 
          {
            //срез частот в 100Гц (36 выборок за 10мс) и выше
            //срез частот в 20Гц (181 выборок за 50мс) и выше
          }
          else
          {
            GoodAcc.X.Ctr += Ctr.X.GreaterThan + Ctr.X.LessThan;
            //надо из буфера взять сумму GoodAcc.X.Ctr выборок
            uint16_t TmpBufCtr = BufCtr;
            uint16_t Thr = Ctr.X.GreaterThan + Ctr.X.LessThan;
            for (uint16_t Ctr = 0U; Ctr < Thr ; ++Ctr)
            {
              GoodAcc.X.Sum += AccelBuf[TmpBufCtr--].X;
            }
          }
          if ( Ctr.X.GreaterThan > 1 && Ctr.X.LessThan > 1)
          {
//            static int x;
            ++x;
          }
          
          if ( Sig.X.SampleLocate == TSampleLocate::_NEG )
          {
            Ctr.X.LessThan    = 1U;
            Ctr.X.GreaterThan = 0U;
          }
          else
          {
            Ctr.X.LessThan    = 0U;
            Ctr.X.GreaterThan = 1U;
          }
          
          Sig.X.SampleLocate = TSampleLocate::_START;
        }
        else
        {
          if ( Sig.X.SampleLocate == TSampleLocate::_NEG)
          {
            ++Ctr.X.LessThan;
          }
          else
          {
            ++Ctr.X.GreaterThan;
          }
        }
      }
      else
      {
        //переход к новой выборке
      }
      
      if ( axis_handle(AccelBuf[BufCtr].Y, AccAverage.Y, Ctr.Y, Sig.Y) == TCntState::_CTR_CHK )
      {
        if ( Ctr.Y.CrossOver == 2U )
        {
          //есть целый период сигнала
          Ctr.Y.CrossOver    = 0U;
          if ( Ctr.Y.GreaterThan + Ctr.Y.LessThan < 727) 
          {
            //срез частот в 100Гц (36 выборок за 10мс) и выше
          }
          else
          {
            GoodAcc.Y.Ctr += Ctr.Y.GreaterThan + Ctr.Y.LessThan;
            //надо из буфера взять сумму GoodAcc.Y.Ctr выборок
            uint16_t TmpBufCtr = BufCtr;
            uint16_t Thr = Ctr.Y.GreaterThan + Ctr.Y.LessThan;
            for (uint16_t Ctr = 0U; Ctr < Thr ; ++Ctr)
            {
              GoodAcc.Y.Sum += AccelBuf[TmpBufCtr--].Y;
            }
          }
          if ( Ctr.Y.GreaterThan > 1 && Ctr.Y.LessThan > 1)
          {
//            static int y;
            ++y;
          }
          
          if ( Sig.Y.SampleLocate == TSampleLocate::_NEG )
          {
            Ctr.Y.LessThan    = 1U;
            Ctr.Y.GreaterThan = 0U;
          }
          else
          {
            Ctr.Y.LessThan    = 0U;
            Ctr.Y.GreaterThan = 1U;
          }
          
          Sig.Y.SampleLocate = TSampleLocate::_START;
        }
        else
        {
          if ( Sig.Y.SampleLocate == TSampleLocate::_NEG)
          {
            ++Ctr.Y.LessThan;
          }
          else
          {
            ++Ctr.Y.GreaterThan;
          }
        }
      }
      else
      {
        //переход к новой выборке
      }
       
      if ( axis_handle(AccelBuf[BufCtr].Z, AccAverage.Z, Ctr.Z, Sig.Z) == TCntState::_CTR_CHK )
      {
        if ( Ctr.Z.CrossOver == 2U )
        {
          //есть целый период сигнала
          Ctr.Z.CrossOver    = 0U;
          Freq = roundf(1000.0f / ( (Ctr.Z.GreaterThan + Ctr.Z.LessThan) * 0.275f ));
          if ( Ctr.Z.GreaterThan + Ctr.Z.LessThan < 727) 
          {
            //срез частот в 100Гц (36 выборок за 10мс) и выше
          }
          else
          {
            GoodAcc.Z.Ctr += Ctr.Z.GreaterThan + Ctr.Z.LessThan;
            //надо из буфера взять сумму GoodAcc.Z.Ctr выборок
            uint16_t TmpBufCtr = BufCtr;
            uint16_t Thr = Ctr.Z.GreaterThan + Ctr.Z.LessThan;
            for (uint16_t Ctr = 0U; Ctr < Thr ; ++Ctr)
            {
              GoodAcc.Z.Sum += AccelBuf[TmpBufCtr--].Z;
            }
          }
          
          if ( Ctr.X.GreaterThan > 1 && Ctr.X.LessThan > 1)
          {
//            static int z;
            ++z;
          }
          
          if ( Sig.Z.SampleLocate == TSampleLocate::_NEG )
          {
            Ctr.Z.LessThan    = 1U;
            Ctr.Z.GreaterThan = 0U;
          }
          else
          {
            Ctr.Z.LessThan    = 0U;
            Ctr.Z.GreaterThan = 1U;
          }
          
          Sig.Z.SampleLocate = TSampleLocate::_START;
        }
        else
        {
          if ( Sig.Z.SampleLocate == TSampleLocate::_NEG)
          {
            ++Ctr.Z.LessThan;
          }
          else
          {
            ++Ctr.Z.GreaterThan;
          }
        }
      }
      else
      {
        //переход к новой выборке
      }
    }
    
    xSemaphoreTake(MainBufMutex, portMAX_DELAY);
      ((TSys *)MainBuf)->Platf.ThrFlag = ThrFlag;
    xSemaphoreGive(MainBufMutex);
    
    if ( ThrFlag == TMThrFlag::_YES ) //если порог хотя-бы 1 раз был превышен
    {
      //текущее окно с выборками невалидно
      AccAverage.X = PrevAccAverage.X;
      AccAverage.Y = PrevAccAverage.Y;
      AccAverage.Z = PrevAccAverage.Z;
    }
    else //если порог ни одной из осей во всем окне не превышен
    {
      if ( GoodAcc.X.Ctr != 0 )
      {
        AccAverage.X     = GoodAcc.X.Sum / GoodAcc.X.Ctr;
        PrevAccAverage.X = AccAverage.X;
      }
      else
      {
        PrevAccAverage.X = AccAverage.X;
      }
      if ( GoodAcc.Y.Ctr != 0 )
      {
        AccAverage.Y     = GoodAcc.Y.Sum / GoodAcc.Y.Ctr;
        PrevAccAverage.Y = AccAverage.Y;
      }
      else
      {
        PrevAccAverage.Y = AccAverage.Y;
      }
      if ( GoodAcc.Z.Ctr != 0 )
      {
        AccAverage.Z     = GoodAcc.Z.Sum / GoodAcc.Z.Ctr;
        PrevAccAverage.Z = AccAverage.Z;
      }
      else
      {
        PrevAccAverage.Z = AccAverage.Z;
      }
    }

    float Numerator   = AccAverage.X;
    float Denominator = sqrt( pow(AccAverage.Y, 2) + pow(AccAverage.Z, 2) );
    static float Pitch;
    static __packed float PrevPitch;

    Pitch = atan2f( Numerator, Denominator ) * 180.0f / M_PI;

    Pitch = Cnt::Angles::convert_angle(Pitch, AccAverage.Z, ::Device);
    cnt_damp(PrevPitch, Pitch, 1.0f); //1.0f => без демпфирования

    xSemaphoreTake(MainBufMutex, portMAX_DELAY);
      ((TSys *)MainBuf)->Platf.AccelAngle = roundf(PrevPitch);
      ((TSys *)MainBuf)->Platf.AccelData  = AccAverage;
      ((TSys *)MainBuf)->FreqZ            = Freq;
    xSemaphoreGive(MainBufMutex);

    AccAverage.X = 0;
    AccAverage.Y = 0;
    AccAverage.Z = 0;

    memset(AccelBuf, 0U, sizeof AccelBuf); //обнуление массива

    //новая выборка должна быть обработана
    //накопление выборки по 3-м осям
    SampleCtr = 0;
    AccelBuf[SampleCtr++] = Data;

    //и сразу вычисления средних значений по осям
    AccAverage.X += Data.X;
    AccAverage.Y += Data.Y;
    AccAverage.Z += Data.Z;

    Sig.X.SampleLocate = TSampleLocate::_START;
    Sig.Y.SampleLocate = TSampleLocate::_START;
    Sig.Z.SampleLocate = TSampleLocate::_START;

    GoodAcc.X.Ctr = 0U;
    GoodAcc.Y.Ctr = 0U;
    GoodAcc.Z.Ctr = 0U;
    GoodAcc.X.Sum = 0.0f;
    GoodAcc.Y.Sum = 0.0f;
    GoodAcc.Z.Sum = 0.0f;

    Ctr.X.CrossOver = 0U;
    Ctr.Y.CrossOver = 0U;
    Ctr.Z.CrossOver = 0U;

    Ctr.X.GreaterThan = 0U;
    Ctr.Y.GreaterThan = 0U;
    Ctr.Z.GreaterThan = 0U;

    Ctr.X.LessThan = 0U;
    Ctr.Y.LessThan = 0U;
    Ctr.Z.LessThan = 0U;

    if (
        ( ::Device == TDevice::_PLATF ) //для платформы
       )
    {
      //если хотя бы в одной из крышек превышен порог срабатывания по акселю, \
        то все текущее окно всех устройств невалидно
      TModule HC[TDevice::_MAX_DEV];
      xSemaphoreTake(MainBufMutex, portMAX_DELAY);
        for (uint8_t Ctr = 0U; Ctr < TDevice::_MAX_DEV; ++Ctr)
        {
          HC[Ctr] = (((TSys *)MainBuf)->HC)[Ctr];
        }
      xSemaphoreGive(MainBufMutex);

      bool GreaterThanFlag = false;
      for (auto item : HC)
      {
        if ( item.ThrFlag == TMThrFlag::_NO ) //если нет превышения порога
        {

        }
        else
        {
          GreaterThanFlag = true;
          break;
        }
      }

      if (
          !GreaterThanFlag //если нигде не было превышения порога
          &&               //и есть текущая выборка хотя бы от одной крышки
          (
            HC[TDevice::_HC_1].SampleFlag == TMSampleFlag::_YES
            ||
            HC[TDevice::_HC_2].SampleFlag == TMSampleFlag::_YES
          )
          &&
          ( ThrFlag == TMThrFlag::_NO )
         )
      {
        xSemaphoreGive(MemsDataRdySemphr);
      }
      else
      {
        xSemaphoreGive(DataAnglesReadySemphr); //вывод предыдущего значения
      }
    }
    else                                  //для крышек
    {
      xSemaphoreGive(CanDataRdySemphr);
    }

    PrevAccel.X = Data.X;
    PrevAccel.Y = Data.Y;
    PrevAccel.Z = Data.Z;

  }
}

void vCANTx(void *pvParameters)
{
  //аппаратная инициализация интерфейса перенесена в main()  
  uint8_t Data[] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    
  for (;;)
  {
		switch (::Device)
    {
      case TDevice::_PLATF: //для платформы
           xSemaphoreTake(AccRdySemphr, portMAX_DELAY); //ожидание семафора готовности работы с акселерометром из задачи vMemsHandler
             ((TCanData *)Data)->TxPlatf.Cmd = TCanCmd::START_MEASURE;
             Can.transmit_msg(Data); //передать сообщение с данными
             //CAN сообщение передано
             xSemaphoreGive(StartMeasSemphr); //семафор запуска измерений акселерометра для задачи vMemsHandler \
                                                по приему пакета крышкой на ней также должны быть запущены измерения
           break;
      default:              //для крышек
           xSemaphoreTake(CanDataRdySemphr, portMAX_DELAY); //данные устройства с датчиком для отправки по CAN на платформу готовы \
                                                                семафор выдает задача vMemsHandler
             xSemaphoreTake(MainBufMutex, portMAX_DELAY);                                                      //захватить мьютекс для атомарной работы с MainBuf
               ((TCanData *)Data)->TxHC.Angle   = ((TSys *)MainBuf)->Platf.AccelAngle;
               ((TCanData *)Data)->TxHC.ThrFlag = ((TSys *)MainBuf)->Platf.ThrFlag;
               //добавить флаг калибровки
	           xSemaphoreGive(MainBufMutex);
             
             Can.transmit_msg(Data); //передать сообщение с данными
           
           break;
    }  
  }
}

void vCANRx0(void *pvParameters)
{
  uint8_t CanMsg[] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
  uint32_t Id = 0U;

//  constexpr uint32_t DELAY_TIME_MS = 1000U;

  //Тип устройства к этому моменту уже определен - master/slave1/slave2

  RtosHeapSize = xPortGetFreeHeapSize();
  
  for (;;) //обработка принятого сообщения только для платформы
  {
    Can.receive_msg(CanMsg, bxCAN::TRxFifoNum::_NULL, &Id);
    rx0CanData = (TCanData *)CanMsg;

    switch (::Device)
    {
      case TDevice::_PLATF: //для платформы
           if ( Id >= TDevice::_HC_1 && Id < TDevice::_MAX_DEV ) //если принято сообщение с идентификатором крышки
           {
             xSemaphoreTake(MainBufMutex, portMAX_DELAY);
               ((TSys *)MainBuf)->HC[Id].AccelAngle = ((TCanData *)&CanMsg)->TxHC.Angle
                                                    + ((TSys *)MainBuf)->HC[Id].Sets.Bias;
               ((TSys *)MainBuf)->HC[Id].ThrFlag    = ((TCanData *)&CanMsg)->TxHC.ThrFlag;
               ((TSys *)MainBuf)->HC[Id].SampleFlag = TMSampleFlag::_YES;
               ++((TSys *)MainBuf)->ConnectCtr[Id];
             xSemaphoreGive(MainBufMutex); //освободить мьютекс для атомарной работы с MainBuf
           }
           
           xSemaphoreGive(HC1DataRxdSemphr); //данные от крышки_1 приняты
           
           break;
      default:              //для крышек
           if ( Id == TDevice::_PLATF ) //крышки принимают сообщение только от платформы
           {
             if ( ((TCanData *)CanMsg)->TxPlatf.Cmd == TCanCmd::START_MEASURE )
             {
               xSemaphoreGive(StartMeasSemphr); //выдача семафора на запуск измерений
             }
             //другие команды реализовать при необходимости
           }

           break;
    }   
  }
}

void vCANRx1(void *pvParameters) //задача определена только для платформы
{
  uint8_t CanMsg[] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
  uint32_t Id = 0U;

  constexpr uint32_t DELAY_TIME_MS = 1000U;

  //Тип устройства к этому моменту уже определен - master/slave1/slave2

  RtosHeapSize = xPortGetFreeHeapSize();
  
  if (::Device != TDevice::_PLATF)
  {
    for (;;)
	  {
      vTaskDelay(DELAY_TIME_MS / TICK_RATE_MS); //для крышек задача подвисает
	  }
  }
  
  for (;;) //обработка принятого сообщения только для платформы
  {
    Can.receive_msg(CanMsg, bxCAN::TRxFifoNum::_ONE, &Id);
    rx1CanData = (TCanData *)CanMsg;
    if ( Id >= TDevice::_HC_1 && Id < TDevice::_MAX_DEV ) //если принято сообщение с идентификатором крышки
    {
      xSemaphoreTake(MainBufMutex, portMAX_DELAY);
        ((TSys *)MainBuf)->HC[Id].AccelAngle = ((TCanData *)&CanMsg)->TxHC.Angle
                                               + ((TSys *)MainBuf)->HC[Id].Sets.Bias;
        ((TSys *)MainBuf)->HC[Id].ThrFlag    = ((TCanData *)&CanMsg)->TxHC.ThrFlag;
        ((TSys *)MainBuf)->HC[Id].SampleFlag = TMSampleFlag::_YES;
        ++((TSys *)MainBuf)->ConnectCtr[Id];
      xSemaphoreGive(MainBufMutex); //освободить мьютекс для атомарной работы с MainBuf
    }
           
    xSemaphoreGive(HC2DataRxdSemphr); //данные от крышки_2 приняты
  }
}

void vResultCnt(void *pvParameters)
{  
  //- определение состояния всех устройств системы \
    - расчет результирующих углов

  constexpr uint32_t DELAY_TIME_MS = 1000U;
  
  ((TSys *)MainBuf)->HC[TDevice::_HC_1].pRelay = &RelOne;
  ((TSys *)MainBuf)->HC[TDevice::_HC_2].pRelay = &RelTwo;
  ((TSys *)MainBuf)->HC[TDevice::_HC_3].pRelay = nullptr;
  ((TSys *)MainBuf)->HC[TDevice::_HC_4].pRelay = nullptr;
  ((TSys *)MainBuf)->HC[TDevice::_HC_5].pRelay = nullptr;
  ((TSys *)MainBuf)->HC[TDevice::_HC_6].pRelay = nullptr;
  ((TSys *)MainBuf)->HC[TDevice::_HC_7].pRelay = nullptr;
  ((TSys *)MainBuf)->HC[TDevice::_HC_8].pRelay = nullptr;
  
  if (::Device != TDevice::_PLATF)
  {
    for (;;)
	  {
      vTaskDelay(DELAY_TIME_MS / TICK_RATE_MS); //для крышек задача подвисает
	  }
  }
  
  static int16_t PAngle = 0;
  static TPState PState = TPState::_OK;
  static __packed TModule HC[TDevice::_MAX_DEV];
  
  for (;;)
  {
    xSemaphoreTake(MemsDataRdySemphr, portMAX_DELAY);
      xSemaphoreTake(MainBufMutex, portMAX_DELAY);
        PAngle = ((TSys *)MainBuf)->Platf.AccelAngle;
        PState = ((TSys *)MainBuf)->Platf.State;
        
        for (uint8_t Ctr = 0U; Ctr < TDevice::_MAX_DEV; ++Ctr)
        {
          HC[Ctr] = (((TSys *)MainBuf)->HC)[Ctr];
        }
        
      xSemaphoreGive(MainBufMutex); //освободить мьютекс для атомарной работы с MainBuf
      
      
      //проверка положения платформы
      if (
           (
             PAngle > _PLATFORM_ANGLE.LowThr.High
           )
           ||
           (
             PAngle > _PLATFORM_ANGLE.LowThr.Low
             &&
             PState == TPState::_ERR_POSITION
           )
           ||
           (
             PAngle < _PLATFORM_ANGLE.HighThr.High
           )
           ||
           (
             PAngle < _PLATFORM_ANGLE.HighThr.Low
             &&
             PState == TPState::_ERR_POSITION
           )
         )
      {
        //ошибка положения платформы \
          состояние всех крышек устанавливается в ошибочное \
          углы крышек сбрасываются
        PState = TPState::_ERR_POSITION;
        
        for (auto &item : HC)
        {
          item.State = TMState::_ERR_POSITION;
          item.AccelAngle = 0;
          item.OpenAngle  = 0;
          if ( item.pRelay != nullptr )
          {
            item.pRelay->off();
          }
        }
      }
      else
      {
        //положение платформы - валидно \
          если связь с крышкой есть, то \
          пересчет углов открытия всех крышек \
          и определение их состояния
        PState = TPState::_OK;
        
        for (auto &item : HC)
        {
          if ( item.Connect == TMConnect::_YES ) //если связь с крышкой есть
          {
            //проверка положения крышки
            if (
                 (
                   item.AccelAngle > _HC_ANGLE.HighThr.High
                 )
                 ||
                 (
                   item.AccelAngle > _HC_ANGLE.HighThr.Low
                   &&
                   item.State == TMState::_ERR_POSITION
                 )
                 ||
                 (
                   item.AccelAngle < _HC_ANGLE.LowThr.High
                 )
                 ||
                 (
                   item.AccelAngle < _HC_ANGLE.LowThr.Low
                   &&
                   item.State == TMState::_ERR_POSITION
                 )
               )
            {
              //ошибка положения крышки
              item.State = TMState::_ERR_POSITION;
//              item.AccelAngle = 0;
              item.OpenAngle  = 0;
              if ( item.pRelay != nullptr ) //если существует реле, привязанное к крышке
              {
                item.pRelay->off();         //отключить
              }            
            }
            else
            {
              //пересчет угла открытия
              cnt_damp( item.OpenAngle, item.AccelAngle - PAngle, 10.0f);
              
              //опеределение состояния крышки
              if (
                   (
                     item.OpenAngle > item.Sets.Thr                  //_OPEN_HYST.High
                   )
                   ||
                   (
                     item.OpenAngle > item.Sets.Thr - item.Sets.Hyst //_OPEN_HYST.Low
                     &&
                     item.State == TMState::_OPEN
                   )
                 )
              {
                item.State = TMState::_OPEN;
                if ( item.pRelay != nullptr ) //если существует реле, привязанное к крышке
                {
                  item.pRelay->on();         //включить
                }
              }
              else
              {
                item.State = TMState::_CLOSE;
                if ( item.pRelay != nullptr ) //если существует реле, привязанное к крышке
                {
                  item.pRelay->off();         //отключить
                }
              }
            }                     
          }
          else //если связи с крышкой нет
          {
            //ошибка положения крышки
            item.State = TMState::_ERR_POSITION;
            item.AccelAngle = 0;
            item.OpenAngle  = 0;
            if ( item.pRelay != nullptr ) //если существует реле, привязанное к крышке
            {
              item.pRelay->off();         //отключить
            }
          }
        }
        //крышки обработаны - в массиве HC актуальные данные
      }

      xSemaphoreTake(MainBufMutex, portMAX_DELAY);
        ((TSys *)MainBuf)->Platf.State = PState;

        for (uint8_t Ctr = 0U; Ctr < TDevice::_MAX_DEV; ++Ctr)
        {
          (((TSys *)MainBuf)->HC)[Ctr] = HC[Ctr];
        }

        TMConnect HC1Connect = ((TSys *)MainBuf)->HC[TDevice::_HC_1].Connect;
        TMState   HC1State   = ((TSys *)MainBuf)->HC[TDevice::_HC_1].State;
        TMConnect HC2Connect = ((TSys *)MainBuf)->HC[TDevice::_HC_2].Connect;
        TMState   HC2State   = ((TSys *)MainBuf)->HC[TDevice::_HC_2].State;
      xSemaphoreGive(MainBufMutex); //освободить мьютекс для атомарной работы с MainBuf

      xSemaphoreTake(SENSMutex, portMAX_DELAY); //захватить мьютекс для атомарной работы с cdt
        taskENTER_CRITICAL();                   //к cdt без критической секции можно обращать только в задаче vLineData
          cdt.HC1_Angle = roundf( ((TSys *)MainBuf)->HC[TDevice::_HC_1].OpenAngle );
          cdt.HC2_Angle = roundf( ((TSys *)MainBuf)->HC[TDevice::_HC_2].OpenAngle );

          uint8_t Tmp = 0U;
          ((TStateByte *)&Tmp)->HC1Conn  = HC1Connect;
          ((TStateByte *)&Tmp)->HC1State = HC1State;
          ((TStateByte *)&Tmp)->HC2Conn  = HC2Connect;
          ((TStateByte *)&Tmp)->HC2State = HC2State;
          ((TStateByte *)&Tmp)->PState   = PState;
          
          cdt.errcode = Tmp;

          static float Nbr;
          Nbr = 0;
//          uint32_t Ctr = 1;
//          
//          while (Tmp)
//          {
//            Nbr += (Tmp % 2U) ? 1 * Ctr 
//                              : 0 * Ctr
//                              ;           

//            Tmp /= 2U;
//            Ctr *= 10;
//          }
            Nbr +=     1U * (uint8_t)HC1Connect;
            Nbr +=    10U * (uint8_t)HC2Connect;
            Nbr +=   100U * (uint8_t)PState;
            Nbr +=  1000U * (uint8_t)HC1State;
            Nbr += 10000U * (uint8_t)HC2State;

          cdt.SysStatus = Nbr;
//          cdt.SysStatus = 11111.0f;

        taskEXIT_CRITICAL();
      xSemaphoreGive(SENSMutex);  //освободить мьютекс для атомарной работы с cdt

    xSemaphoreGive(DataAnglesReadySemphr);
  }
}

/*************************************************************
 *
 *  Function:       vLineByteState
 *
 *------------------------------------------------------------
 *
 *  description:    формирование байта состояния.
 *                  Выполняется для master'a
 *
 *  parameters:     pvParameters - параметры, передаваемые задаче
 *
 *  on return:      void
 *
 *************************************************************
 * Список изменений
 * 
 * (18.07.2018):
 *   1. Сохранение предыдущего значения байта состояния перед обновлением для дальнейшей проверки и 
 *      установки номера параметра в cdt.srab, если произошло срабатывание (!не изменение, а срабатывание)
 *  
 *     
 *     
 *     
 * ...
 ***************************************************/
void vLineByteState(void *pvParameters)
{
  constexpr uint32_t DELAY_TIME_MS  = 1000U; //минимальный цикл линии (без данных) = 551мс => \
                                              можно затормозить задачу на 100мс
//  constexpr uint32_t DELAY_TIME_MS  = 100U; //минимальный цикл линии (без данных) = 551мс => \
//                                              можно затормозить задачу на 100мс
                                              
//  if (Device != TDevice::_PLATF) //Тип устройства к этому моменту уже определен - master/slave1/slave2
//  {
//    for (;;)
//	  {
//      vTaskDelay(DELAY_TIME_MS / TICK_RATE_MS); //если устройство не мастер, то задача подвисает
//	  }
//  }
  
  for (;;)
  {
    vTaskDelay(DELAY_TIME_MS / TICK_RATE_MS); //если устройство не мастер, то задача подвисает
  }
/*
//  TDevConnect DevConnect; //данные о связи slave устройств с master'ом
//  TSysStates  SysStates;  //основные системные флаги
  TPState PState;
  TMState HC1State;
  TMState HC2State;


  constexpr uint8_t DKS_ADDR        = 7U;   //адрес устройства (default)
  constexpr uint8_t STD_PACK_LEN    = 3U;   //длина стандартного пакета состояний
//  constexpr uint8_t CMD_GETSTATE    = 0x00; //команда - получение байта состояния
  constexpr uint8_t ADDR_LAST_PACK  = 0U;   //адрес последнего пакета
  constexpr uint8_t LAST_PACK_LEN   = 0U;   //длина последнего пакета
  constexpr uint8_t STATE_BYTE_INIT = 0U;

  StateBuf[0].adr                = DKS_ADDR;
  StateBuf[0].dat.Reg            = STATE_BYTE_INIT;
  StateBuf[0].num                = STD_PACK_LEN;
  StateBuf[0].comm.CmdCode       = CMD_GETSTATE;
  StateBuf[0].comm.Direction     = _RESPONSE;
  StateBuf[0].comm.EmulMode      = _LINE_NORMAL;
  StateBuf[0].comm.ChangeMasters = _MASTERS_CONTINUOUS_POLL;
  StateBuf[1].adr                = ADDR_LAST_PACK;
  StateBuf[1].num                = LAST_PACK_LEN;

  if (Device != TDevice::_PLATF) //Тип устройства к этому моменту уже определен - master/slave1/slave2
  {
    for (;;)
	   {
      vTaskDelay(DELAY_TIME_MS / TICK_RATE_MS); //если устройство не мастер, то задача подвисает
	   }
  }
  
//  HardDSt = DefDSt; //при первом включении устанавливаем для HardDSt настройки по умолчанию  
//  LoadConst();      //HardDSt в DSt. Смысл - обновление настроек и параметров для дальнейшей отправки по линии
  
  ConfigSENS(_BLOCK_GENERATE);
  xSemaphoreGive(LineEnabledSemphr); //выдать семафор задаче работы с пакетами данных по линии \
                                       автопереключение контекста, ручное переключение не нужно

  SetBSBuf(StateBuf); //подключение буфера
  UnblockST();        //разерешение передачи состояний

  for (;;)
  {	 
    vTaskDelay(DELAY_TIME_MS / TICK_RATE_MS); //задержка опроса завершения очередного цикла линии
	 
	  xSemaphoreTake(MainBufMutex, portMAX_DELAY);                        //захватить мьютекс для атомарной работы с MainBuf
      PState   = ((TSys *)MainBuf)->Platf.State;
      HC1State = ((TSys *)MainBuf)->HC[TDevice::_HC_1].State;
      HC2State = ((TSys *)MainBuf)->HC[TDevice::_HC_2].State;
	  xSemaphoreGive(MainBufMutex);

	  //
	  // Обновление байта состояния
	  //
	  TStateByte Prev;        //предыдущее значение байта состояния
	  Prev = StateBuf[0].dat;
	  
	  __disable_irq();
	    if (!CheckBSTr()) //если пакет состояний сейчас не передается, \
		                    то можно его изменять  
		{
		  StateBuf[0].adr                   = HardDSt.Addr;
//		  StateBuf[0].dat.HatchCoverConnect = DevConnect.HatchCover;
//		  StateBuf[0].dat.LockConnect       = DevConnect.Lock;
//		  StateBuf[0].dat.HatchCoverConnect = 0U;
//		  StateBuf[0].dat.LockConnect       = 0U;
		  StateBuf[0].dat.PlatformState     = (uint8_t)PState;
		  StateBuf[0].dat.HatchCoverState   = (uint8_t)HC1State;
		  StateBuf[0].dat.LockState         = (uint8_t)HC2State;
		  StateBuf[0].dat.Reserved          = 0U;
		}
      __enable_irq();

  if (StateBuf[0].dat.Reg != Prev.Reg) //если есть изменения в текущем байте состояния по сравнению с предыдущим 
  {
    TStateByte *Cur = &StateBuf[0].dat;
      
	  	struct TStBit
	  	{
	  	  uint8_t CurVal;
	  	  uint8_t PrevVal;
	  	  bool    Trig;
	  	  uint8_t Param;
	  	};
      
	  	TStBit StBit[] =
	  	{
	  	  {Cur->HatchCoverConnect   , Prev.HatchCoverConnect   , false, PNT7},
	  	  {Cur->LockConnect         , Prev.LockConnect         , false, PNT8},
	  	  {Cur->PlatformState       , Prev.PlatformState       , true , PNT7},
	  	  {Cur->HatchCoverState & 1U, Prev.HatchCoverState & 1U, true , PNT7}, //открыт/закрыт       (1/0)
	  	  {Cur->HatchCoverState & 2U, Prev.HatchCoverState & 2U, true , PNT7}, //ошибка/нет ошибки   (1/0)
	  	  {Cur->LockState       & 1U, Prev.LockState       & 1U, true , PNT8}, //вскрыт/опломбирован (1/0)
	  	  {Cur->LockState       & 2U, Prev.LockState       & 2U, true , PNT8}  //ошибка/нет ошибки   (1/0)
	  	};
	  	
	  	for (auto item : StBit)
	  	{
	  	  if ( chk_trig(item.CurVal, item.PrevVal, item.Trig) ) //если бит сработал
	  	  {
	  	    cdt.srab = item.Param; //заполнение EPRM номером параметра, вызвавшим срабатывание
	  	  }
	  	}		
  }

//	 
//	 Рабочая версия манипуляций с байтом состояния
//	 Отказался от нее из-за кода ПМП-201
//	 
//	 if (GetNC())   //если завершился очередной цикл линии
//	 {
//	   RtosHeapSize = xPortGetFreeHeapSize();
//		BlockST();   //блокировка передачи для изменения данных в пакете
//
//		xSemaphoreTake(MainBufMutex, portMAX_DELAY);                        //захватить мьютекс для атомарной работы с MainBuf
//	     DeviceConnect = ((TMainData *)MainBuf)->PcSendData.DeviceConnect;
//		  SysStates     = ((TMainData *)MainBuf)->PcSendData.SysStates;	 
//	   xSemaphoreGive(MainBufMutex);
//
//		StateBuf[0].dat.HatchCoverConnect = DeviceConnect.HatchCover;
//		StateBuf[0].dat.LockConnect       = DeviceConnect.Lock;
//		StateBuf[0].dat.PlatformState     = SysStates.Platform;
//		StateBuf[0].dat.HatchCoverState   = SysStates.HatchCover;
//		StateBuf[0].dat.LockState         = SysStates.Lock;
//		StateBuf[0].dat.Reserved          = 0U;
//
//      UnblockST(); //разерешение передачи состояний		
//	 }
//	 else
//	 {
//      vTaskDelay(DELAY_TIME_MS / TICK_RATE_MS); //задержка опроса завершения очередного цикла линии
//	 }
//
  }
  */
}

/*************************************************************
 *
 *  Function:       vLineData
 *
 *------------------------------------------------------------
 *
 *  description:    прием и формирование пакетов со значениями параметров для отправки по ЛИН
 *
 *  parameters:     pvParameters - параметры, передаваемые задаче
 *
 *  on return:      void
 *
 *************************************************************
 * Список изменений
 * 
 * (20.07.2018):
 *   1. В структуру TSProgNum добавлены состояния для всех трех устройств
 *  
 *     
 *     
 *     
 * ...
 ***************************************************/
void vLineData(void *pvParameters)
{ 
  constexpr uint32_t DELAY_TIME_MS = 1000U;
//  constexpr uint32_t DELAY_TIME_MS = 5U; //минимальный синхроимпульс + время простоя = 17ms \
//                                           необходимо хотя бы 1 раз за синхроимпульс проверить наличие пакета => \
//														               можно затормозить задачу максимум на 10ms (с запасом на ожидание отработки других задач)
                                           
//  if (Device != TDevice::_PLATF) //Тип устройства к этому моменту уже определен - master/slave1/slave2
//  {
//    for (;;)
//	  {
//      vTaskDelay(DELAY_TIME_MS / TICK_RATE_MS); //если устройство не мастер, то задача подвисает
//	  }
//  }
  
  for (;;)
	{
    vTaskDelay(DELAY_TIME_MS / TICK_RATE_MS); //если устройство не мастер, то задача подвисает
	}

}

void vWriteFlash(void *pvParameters) //задача работает только в master'e
{
  for (;;)
  {
    xSemaphoreTake(WriteFlashSetsSemphr, portMAX_DELAY); //для slave устройств семафор не будет получен, т.к. ни ЛИН ни USART запущены не будут => \
                                                           настройки не будут получены => задача на slave зависнет \
                                                           принятые настройки уже в HardDSt
		  MySettings HardSets;
		  MySettings *const MainSets = &((TSys *)MainBuf)->Sets.AllSets;

      xSemaphoreTake(SENSMutex, portMAX_DELAY); //захватить мьютекс для атомарной работы с HardDSt   
        taskENTER_CRITICAL();                   //к HardDSt без критической секции можно обращаться только в задаче vLineData
          HardSets = HardDSt;
        taskEXIT_CRITICAL();
		  xSemaphoreGive(SENSMutex);                //освободить мьютекс для атомарной работы с HardDSt

		  if (chk_dev_sets(HardSets, *MainSets)) //проверка допустимости сделанных настроек \
		                                           недопустимые настройки устанавливаются в текущее значение (до ввода настроек) \
                                               HardSets - обновленные настройки \
                                               MainSets - старые настройки
      {
      
		  }
      xSemaphoreTake(SENSMutex, portMAX_DELAY); //захватить мьютекс для атомарной работы с HardDSt 
        taskENTER_CRITICAL();                   //к HardDSt и DSt без критической секции можно обращаться только в задаче vLineData
          HardDSt = HardSets;                   //какая-то настройка является недопустимой => она вернется к предыдущему значению для HardDSt
		      LoadConst();                          //и для DSt
        taskEXIT_CRITICAL();
		  xSemaphoreGive(SENSMutex);                //освободить мьютекс для атомарной работы с HardDSt
//      else
//      {
//        //все настройки валидны и восстанавливать их в HardDSt и DSt не нужно
//      }

      Flash.write_sets(HardSets);     
//      RelFour.on();

      xSemaphoreTake(MainBufMutex, portMAX_DELAY);      //захватить мьютекс для атомарной работы с MainBuf
	      ((TSys *)MainBuf)->Sets = Flash.get_settings(); //работа через прослойку HardDSt оставлена для \
                                                          визуальной совместимости ПО с другими устройствами СЕНС

        ((TSys *)MainBuf)->HC[TDevice::_HC_1].Sets = ((TSys *)MainBuf)->Sets.AllSets.DevSets[TDevice::_HC_1];
        ((TSys *)MainBuf)->HC[TDevice::_HC_2].Sets = ((TSys *)MainBuf)->Sets.AllSets.DevSets[TDevice::_HC_2];
      xSemaphoreGive(MainBufMutex);                     //освободить мьютекс для атомарной работы с MainBuf
  }
}

void vCheckConnectTimerCallback(TimerHandle_t xTimer)   //обработчик вызывается только для платформы
{
  // по срабатыванию таймера текущий кадр с флагами должен быть уже доступен в объекте типа TConnect
  // необходимо разобрать объект типа TConnect для инкремента счетчиков связи
  //
  
  constexpr uint8_t _CONNECT_VAL = 3U;
  
  __packed uint8_t (&ConnectCtr)[TDevice::_MAX_DEV] = ((TSys *)MainBuf)->ConnectCtr;
  
  xSemaphoreTake(MainBufMutex, portMAX_DELAY); //захватить мьютекс для атомарной работы с MainBuf
    uint8_t Ctr = 0U;
    for (auto &item : ConnectCtr)
    {
      if ( item >=_CONNECT_VAL )
      {
        ((TSys *)MainBuf)->HC[Ctr].Connect = TMConnect::_YES;
      }
      else
      {
        ((TSys *)MainBuf)->HC[Ctr].Connect = TMConnect::_NO;     
      }
      ++Ctr;
      item = 0U; //сброс проверенного счетчика
    }
  xSemaphoreGive(MainBufMutex);                //освободить мьютекс для атомарной работы с MainBuf
  
  //таймер периодический => продолжает считать
}
//----------------------------------------------------------------------------------------------------------------------------------

//----- Служебные функции задач FreeRTOS (Definition) ------------------------------------------------------------------------------
static uint8_t SENS_Parse()
{
  uint8_t pkbuf[65]; //буфер, в котором формируется ответ
  uint8_t crc;
  uint8_t len;
  uint8_t n;
  uint8_t adr;

  if ((Usart.u.rscnt < 5) || (Usart.u.rsbuf[0] != 0xB5))
  {
    //в буфере приема меньше 5 байт или преамбула не соответствует протоколу СЕНС
    return 10;
  }

  len = Usart.u.rsbuf[2];

  if (len > Usart.u.rscnt)
  {
    //количество байт в буфере приема не соответствует ожидаемому
    return 10;
  }

  //формирование служебной части пакета
  crc      = 0;
  pkbuf[0] = len + 2;              //длина
  adr      = Usart.u.rsbuf[1]; 
  pkbuf[1] = adr;                  //адрес
  pkbuf[2] = Usart.u.rsbuf[3];     //команда
  crc      = len + adr + pkbuf[2];
  
  //формирование данных в пакете
  for (n = 0; n < len; n++)
  {
    pkbuf[n+3] = Usart.u.rsbuf[n+4];
    crc       += Usart.u.rsbuf[n+4];
  }

  if (crc == Usart.u.rsbuf[n+4]) 
  {
    //пакет верный
    if (!(pkbuf[2] & 0x80))
    {
      uint8_t HAddr;
		
		taskENTER_CRITICAL();
		  HAddr = HardDSt.Addr;
		taskEXIT_CRITICAL();
		
		if ((adr == HAddr) || (adr == 0))
      {
        pkbuf[1] = HAddr; //ненужное действие
        
        uint8_t SensAnswerPkgRet;
		  
		  xSemaphoreTake(SENSMutex, portMAX_DELAY);  //захватить мьютекс для атомарной работы с нереентерабельной функцией SensAnswerPkg
		    SensAnswerPkgRet = SensAnswerPkg(pkbuf);
		  xSemaphoreGive(SENSMutex);                 //освободить мьютекс для атомарной работы с нереентерабельной функцией SensAnswerPkg
		  
		  if (!SensAnswerPkgRet)
        {
          //формируем результат
          Usart.u.trcnt    = pkbuf[0] + 3;
          Usart.u.trbuf[0] = 0xB5;
          Usart.u.trbuf[1] = pkbuf[1];
          len              = pkbuf[0] - 2;
          Usart.u.trbuf[2] = len;
          crc              = pkbuf[1] + Usart.u.trbuf[2];
          
          for (n = 0; n <= len; n++) 
          {
            crc += pkbuf[2+n];
            Usart.u.trbuf[3+n] = pkbuf[2+n];
          }

          Usart.u.trbuf[3+n] = crc;
          
          return 0;
        }
      }
    }
    return 1;
  }
  return 10;
}

static void UART_Process()
{
  if (Usart.u.ustate == USART::TUsartState::U_WT) //если состоние - ожидание обработки принятого пакета
  {
    // Приняли пакет
    static uint8_t curpt = 0;
    uint8_t upt          = (uint8_t)Usart.Proc;
    uint8_t ept;
    
    if (upt)         //если в HardDSt не автоопределение
    {
      curpt = upt;   //curpt = 0..3
    }
    
    ept = curpt & 3; //ept = 0..3

    do
    {
      switch (curpt) 
      {
        case (2): //СЕНС
             switch (SENS_Parse()) //реализация СЕНС 
             {
               case (0):
                    Usart.transmit();
                    ept = 100;
                    break;
               case (1):
                    ept = 10;
                    break;
               default:
                    ++curpt;
             }
             break;
        case (1): //ModBus
             switch (MB_Parse()) //реализация ModBus
             {   
               case (0):
                    Usart.transmit();
                    ept = 100;
                    break;
               case (1):
                    ept = 10;
                    break;
               default:
                    ++curpt;
             }      
             break;
        case (0): //автомат
        case (3): //OmniComm
             ++curpt;
             break;
      }

      if (curpt > 3)
      {
        curpt = 0;
      }

    } while ((ept < 10) && (curpt != ept) && (upt == 0));
    
    if (ept != 100)
    {
      Usart.reset_receive();
    } 
  }
}


/*************************************************************
 *
 *  Function:       chk_dev_sets
 *
 *------------------------------------------------------------
 *
 *  description:    проверка валидности настроек устройства 
 *
 *  parameters:     AllSets - проверяемые настройки
 *                  CurSets    - текущие настройки
 *
 *  on return:      true - если хотя бы одна из настроек не соответствует диапазону допустимых значений
 *
 *************************************************************
 * Список изменений
 * 
 * (12.07.2018):
 *   1. ...
 *     
 *     
 *     
 *     
 * ...
 ***************************************************/

static bool chk_dev_sets(MySettings &NewSets, const MySettings &CurSets)
{

  __packed struct TRange
  {
    int16_t Low;
    int16_t High;
  };

  constexpr TRange _H_C_THR =  //допустимый диапазон для порога срабатывания по углу открытия
  {15, 50};

  constexpr TRange _H_C_HYST = //допустимый диапазон для гистерезиса порога срабатывания по углу открытия
  {5, 40};

  constexpr TRange _H_C_BIAS = //допустимый диапазон для смещения по углу крышки
  {-5, 5};
  
  constexpr int16_t _H_C_LOW_DIF = _H_C_THR.Low - _H_C_HYST.Low; //минимальное значение для возврата в состояние "закрыта"
  
  bool ChngFlag = false; //флаг изменения хотя бы одной нстройки на текущее значение
  
  if ( NewSets.DevSets[TDevice::_HC_1].Thr < _H_C_THR.Low || NewSets.DevSets[TDevice::_HC_1].Thr > _H_C_THR.High ) //если настройка невалидна
  {
    NewSets.DevSets[TDevice::_HC_1].Thr = CurSets.DevSets[TDevice::_HC_1].Thr; //восстановить предыдущее значение настройки
	  ChngFlag = true;
  }
  else
  {
    NewSets.DevSets[TDevice::_HC_1].Thr = roundf(NewSets.DevSets[TDevice::_HC_1].Thr);
  }
  
  if ( NewSets.DevSets[TDevice::_HC_1].Hyst < _H_C_HYST.Low || NewSets.DevSets[TDevice::_HC_1].Hyst > _H_C_HYST.High ) //если настройка невалидна
  {
    NewSets.DevSets[TDevice::_HC_1].Hyst = CurSets.DevSets[TDevice::_HC_1].Hyst; //восстановить предыдущее значение настройки
	  ChngFlag = true;
  }
  else
  {
    NewSets.DevSets[TDevice::_HC_1].Hyst = roundf(NewSets.DevSets[TDevice::_HC_1].Hyst);
  }
  
  //дополнительная проверка для гистерезиса
  if ( (NewSets.DevSets[TDevice::_HC_1].Thr - NewSets.DevSets[TDevice::_HC_1].Hyst) < _H_C_LOW_DIF ) //если значение для возвата в состояние "закрыта" меньше минимального
  {
    NewSets.DevSets[TDevice::_HC_1].Hyst = NewSets.DevSets[TDevice::_HC_1].Thr - _H_C_LOW_DIF; //то установить максимально допустимое значение гистерезиса для данного порога
	  ChngFlag = true;
  }

  if ( NewSets.DevSets[TDevice::_HC_2].Thr < _H_C_THR.Low || NewSets.DevSets[TDevice::_HC_2].Thr > _H_C_THR.High ) //если настройка невалидна
  {
    NewSets.DevSets[TDevice::_HC_2].Thr = CurSets.DevSets[TDevice::_HC_2].Thr; //восстановить предыдущее значение настройки
	  ChngFlag = true;
  }
  else
  {
    NewSets.DevSets[TDevice::_HC_2].Thr = roundf(NewSets.DevSets[TDevice::_HC_2].Thr);
  }

  if ( NewSets.DevSets[TDevice::_HC_2].Hyst < _H_C_HYST.Low || NewSets.DevSets[TDevice::_HC_2].Hyst > _H_C_HYST.High ) //если настройка невалидна
  {
    NewSets.DevSets[TDevice::_HC_2].Hyst = CurSets.DevSets[TDevice::_HC_2].Hyst; //восстановить предыдущее значение настройки
	  ChngFlag = true;
  }
  else
  {
    NewSets.DevSets[TDevice::_HC_2].Hyst = roundf(NewSets.DevSets[TDevice::_HC_2].Hyst);
  }

  //дополнительная проверка для гистерезиса
  if ((NewSets.DevSets[TDevice::_HC_2].Thr - NewSets.DevSets[TDevice::_HC_2].Hyst) < _H_C_LOW_DIF) //если значение для возвата в состояние "закрыта" меньше минимального
  {
    NewSets.DevSets[TDevice::_HC_2].Hyst = NewSets.DevSets[TDevice::_HC_2].Thr - _H_C_LOW_DIF; //то установить максимально допустимое значение гистерезиса для данного порога
	  ChngFlag = true;
  }
    
  if ( NewSets.DevSets[TDevice::_HC_1].Bias < _H_C_BIAS.Low || NewSets.DevSets[TDevice::_HC_1].Bias > _H_C_BIAS.High ) //если настройка невалидна
  {
    NewSets.DevSets[TDevice::_HC_1].Bias = CurSets.DevSets[TDevice::_HC_1].Bias; //восстановить предыдущее значение настройки
	  ChngFlag = true;
  }
  else
  {
    NewSets.DevSets[TDevice::_HC_1].Bias = roundf(NewSets.DevSets[TDevice::_HC_1].Bias);
  }
  
  if ( NewSets.DevSets[TDevice::_HC_2].Bias < _H_C_BIAS.Low || NewSets.DevSets[TDevice::_HC_2].Bias > _H_C_BIAS.High ) //если настройка невалидна
  {
    NewSets.DevSets[TDevice::_HC_2].Bias = CurSets.DevSets[TDevice::_HC_2].Bias; //восстановить предыдущее значение настройки
	  ChngFlag = true;
  }
  else
  {
    NewSets.DevSets[TDevice::_HC_2].Bias = roundf(NewSets.DevSets[TDevice::_HC_2].Bias);
  }
  
  return ChngFlag;
}

static void chk_cal_req()
{		
  TPntFlag Flag;
//  bool     CalReq;
  
  taskENTER_CRITICAL();
    Flag   = PntFlag;
//	 CalReq = MagCalReq;
  taskEXIT_CRITICAL();			
    
//  if (CalReq == true) //если был запрос на калибровку
//  {				      
    if (Flag != _PERFORMING) //если запрос калибровки был не по Modbus
    {
      xSemaphoreGive(MagCalFinishSemphr); //калибровка по запросу окончена \
    	                                      для Slave устройств PntFlag в _NO_CMD_EXEC => \
    													  => отработает эта ветка и семафор \
    													  будет отдан для задачи отправки результата через CAN
    }
    else //если запрос калибровки был по Modbus
    {
      taskENTER_CRITICAL();
        PntFlag = _DONE;
      taskEXIT_CRITICAL();
    }
    
    taskENTER_CRITICAL();                 						 
      MagCalReq = false;
    taskEXIT_CRITICAL();
//  }	
}

/*************************************************************
 *
 *  Function:       chk_trig
 *
 *------------------------------------------------------------
 *
 *  description:    проверка срабатывания бита в байте состояния 
 *
 *  parameters:     Cur       - текущее значение бита
 *                  Prev      - предыдущее значение бита
 *                  TrigLevel - уровень срабатывания (высокий - true/низкий - false)
 *
 *  on return:      //false - нет срабатывания
 *                  //true  - есть срабатывание
 *
 *************************************************************
 * Список изменений
 * 
 * (12.07.2018):
 *   1. ...
 *     
 *     
 *     
 *     
 * ...
 ***************************************************/  		  
static bool chk_trig(uint8_t Cur, uint8_t Prev, bool TrigLevel)
{
  return (Cur != Prev && Cur == TrigLevel);
}

//----------------------------------------------------------------------------------------------------------------------------------
static bool chk_sample(const TAccelData &Data, const TMemsOrient _ORIENT)
{
  if (
      ( Data.X < AxesHyst[_ORIENT].X.Low )
      ||         
      ( Data.X > AxesHyst[_ORIENT].X.High )
      ||         
      ( Data.Y < AxesHyst[_ORIENT].Y.Low )
      ||         
      ( Data.Y > AxesHyst[_ORIENT].Y.High )
      ||         
      ( Data.Z < AxesHyst[_ORIENT].Z.Low )
      ||         
      ( Data.Z > AxesHyst[_ORIENT].Z.High )
     )
  {
    return false; //невалидная выборка
  }
  return true;    //валидная выборка
}

static void motion_tl_init()
{
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC); //UM2277, rev.6, page 3: the CRC module in STM32 microcontroller \
                                                       (in RCC peripheral clock enable register) has to be enabled \
                                                       before using the library
  MotionTL_Initialize();

  constexpr uint16_t ACCEL_BUF_SIZE = 100U;

  constexpr char *acc_orientation = "enu";
  MotionTL_SetOrientation_Acc(acc_orientation);

  constexpr uint8_t LIB_VERSION_BUF_SIZE  = 35;
  char LibAcVersion[LIB_VERSION_BUF_SIZE] = {0};
  MotionTL_GetLibVersion(LibAcVersion);

  //набрать массив с данными в определенных  в TMemsOrient положениях
  static TAccelData Data;
  static uint32_t SampleCtr = 0U;
  static uint8_t Orient = TMemsOrient::_X_UP;
  static MTL_acc_cal_t acc_cal;
  MTL_cal_result_t CalResult;

  do
  {
    for ( ; Orient < TMemsOrient::_MAX; ++Orient )
    {
      SampleCtr = 0U;

      while (SampleCtr < ACCEL_BUF_SIZE)
      {
        xSemaphoreTake(RawDataMemsRdySemphr, portMAX_DELAY); //waiting external interrupt
          AccMagGyro.collect_acc_data();
          Data = *AccMagGyro.get_acc_data();

          //сохранение выборок по 3-м осям
          AccelBuf[SampleCtr] = Data;

          if ( !chk_sample(AccelBuf[SampleCtr++], (TMemsOrient)Orient) ) //если выборка невалидная
          {
            --SampleCtr; //следующая итерация перезапишет невалидную выборку
          }
      }

      float _AccelBuf[ACCEL_BUF_SIZE][3];
      memcpy(_AccelBuf, AccelBuf, sizeof _AccelBuf);  

      MotionTL_CalibratePosition(_AccelBuf, ACCEL_BUF_SIZE, (MTL_cal_position_t)Orient);

      memset(AccelBuf, 0U, sizeof AccelBuf); //обнуление массива с выборками
    }
    Orient = TMemsOrient::_X_UP;
  } while ( ( CalResult = MotionTL_GetCalValues(&acc_cal) ) != MTL_cal_result_t::CAL_PASS );
  
  Flash.write_acc(*(TAccCalData *)&acc_cal);
  //в MainBuf калибровочные данные появятся после перезагрузки
}
