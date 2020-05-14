#include <limits>

#include "lev_gauge_to_pc.h"
#include "rtos_headers.h"

#include "Led.h"
#include "AD8402.h"

static BaseType_t TmrHigherPriorityTaskWoken;
static BaseType_t Int1HigherPriorityTaskWoken;
float TmpParam;
bool UIF = false;

constexpr uint16_t TX_SIZE = 30000U;
uint8_t Data[ TX_SIZE ];
uint16_t nCapDiff = 0U;              //текущее количество захваченных значений

//----- временные переменные для отладки ----------------------------------------------------------------
//----- //временные переменные для отладки --------------------------------------------------------------

TExchngToPC ExchngToPC = {
                          UsartExt_HW,
                          TmrFreg_HW,
                          TmrCmp_HW,
                          TmrDiff_HW,
                          Adc_HW,
                         };

TExchngToPC::TParamHandle ParamHandle[] =
{//       Cmd              RxVal           Range                    Addr                                        BytesQty
  { PC_CMD::SET_FREG,      350.f,  {    10.f,  1000.f }, { (uint8_t *)&ExchngToPC.Tx_Freq_hz,      sizeof(ExchngToPC.Tx_Freq_hz)      } }, //SET_FREG
//  { PC_CMD::DUTY_CYCLE,      0.5f, {     0.f,   100.f }, { (uint8_t *)&ExchngToPC.Tx_DutyCycle_pc, sizeof(ExchngToPC.Tx_DutyCycle_pc) } }, //DUTY_CYCLE
  { PC_CMD::DUTY_CYCLE,      10.f, {     0.f,   100.f }, { (uint8_t *)&ExchngToPC.Tx_DutyCycle_pc, sizeof(ExchngToPC.Tx_DutyCycle_pc) } }, //DUTY_CYCLE
  { PC_CMD::SET_POS_POT_1,   0.f,  {     0.f,   255.f }, { (uint8_t *)&ExchngToPC.Tx_PosPot_1,     sizeof(ExchngToPC.Tx_PosPot_1)     } }, //SET_POS_POT_1
  { PC_CMD::SET_POS_POT_2,   0.f,  {     0.f,   255.f }, { (uint8_t *)&ExchngToPC.Tx_PosPot_2,     sizeof(ExchngToPC.Tx_PosPot_2)     } }, //SET_POS_POT_2
  { PC_CMD::TEST,            0.f,  { FLT_MIN, FLT_MAX }, { (uint8_t *)&ExchngToPC.Tx_Test,         sizeof(ExchngToPC.Tx_Test)         } }, //TEST
  { PC_CMD::START_ADC,       0.f,  {     1.f, 30000.f }, { (uint8_t *)Data,                        0U                                 } }, //START_ADC
  { PC_CMD::CMP_CAPTURE,     0.f,  { FLT_MIN, FLT_MAX }, { (uint8_t *)&ExchngToPC.Tx_Test,         sizeof(ExchngToPC.Tx_Test)         } }, //CMP_CAPTURE
  { PC_CMD::DIFF_CAPTURE,    0.f,  {     1.f, 65536.f }, { (uint8_t *)&ExchngToPC.Tx_Test,         sizeof(ExchngToPC.Tx_Test)         } }, //DIFF_CAPTURE
};

//----- Реализация класса TExchngToPC -----------------------------------------------------
TExchngToPC::TExchngToPC(
                         const TUsart_HW &_Usart_HW,
                         const TTmr_HW   &_TmrFreg_HW,
                         const TTmr_HW   &_TmrCmp_HW,
                         const TTmr_HW   &_TmrDiff_HW,
                         const TAdc_HW   &_Adc_HW
                        )
:
TExtMaster( _Usart_HW ),
Tx_Freq_hz( 0.f ),
Tx_DutyCycle_pc( 0.f ),
Tx_PosPot_1( 0U ),
Tx_PosPot_2( 0U ),
Tx_Test( 0U ),
TmrFreg( _TmrFreg_HW ),
TmrCmp( _TmrCmp_HW ),
TmrDiff( _TmrDiff_HW ),
Adc( _Adc_HW ),
UpdDemandFlag( false ),
TmrFreq( 100000U ),
AdcCalibFactorS( 0U )
{

}

TExchngToPC::~TExchngToPC()
{

}

TExchngToPC::TPwm TExchngToPC::cnt_pwm()
{
  TPwm Pwm{ 0U, 0U, 0U };
  
  LL_RCC_ClocksTypeDef RCC_Clocks;
  LL_RCC_GetSystemClocksFreq( &RCC_Clocks );

  float Freq = ParamHandle[0].RxVal;                                      //требуемая частота ( математическое округление, т.к усечение дробной части )
  Tx_Freq_hz = Freq;                                                      //реально установленное значение частоты
  float Arr  = TmrFreg.ARR_MAX;                                            
  TmrFreq    = ( (uint32_t)Arr + 1U ) * Freq;                          
                                                                          
  if ( TmrFreq >= RCC_Clocks.PCLK2_Frequency )                             
  {
    TmrFreq = RCC_Clocks.PCLK2_Frequency;
    Pwm.PSC = 0U;
    Arr     = TmrFreq / Freq - 1.f;
  }
  else
  {
    do
    {
      uint16_t Presc = ++Pwm.PSC;
      ++Presc;
      TmrFreq        = ( RCC_Clocks.PCLK2_Frequency + Presc / 2 ) / Presc;
      Arr            = TmrFreq / Freq - 1.f;
    } while ( Arr > TmrFreg.ARR_MAX );    
  }
   
  Pwm.ARR = Arr + 0.5f;
                                                                           
  float DC = ParamHandle[1].RxVal / 100.f;                                //беспроцентная величина [ 0..1 ]
  Pwm.CCR  = ( TmrFreq / Freq ) * DC + 0.5f;
  
  
  Tx_DutyCycle_pc = ( Pwm.CCR * 100.f * Tx_Freq_hz ) / TmrFreq;           //реально установленное значение коэффициента заполнения
  
  return Pwm;
}

void TExchngToPC::set_pwm( const TPwm &Pwm )
{
  LL_TIM_SetPrescaler    ( TmrFreg.Nbr, Pwm.PSC );
  LL_TIM_SetAutoReload   ( TmrFreg.Nbr, Pwm.ARR );
  LL_TIM_OC_SetCompareCH1( TmrFreg.Nbr, Pwm.CCR );
}

void TExchngToPC::ctrl_pwm( TExchngToPC::TParamHandle *ParamHandle )
{
  ParamHandle->RxVal = chk_range( ParamHandle->RxVal, ParamHandle->Range );

  set_upd_demand_flag( true );
  xSemaphoreTake( Tmr_TrigSem, portMAX_DELAY );
  
    TPwm Pwm = cnt_pwm();
    set_pwm( Pwm );       //для сохранения максимальной разрядности необходимо модифицировать все параметры таймера \                            
}

void TExchngToPC::tmr_freg_init()
{
  TmrFreg.Pin.en_clk( TmrFreg.Pin.ClkPortMask );   //тактирование GPIO, к которому подключен выход таймера
  TmrFreg.Clk.en_periph( TmrFreg.Clk.PeriphMask ); //тактирование таймера

  //настройка вывода FREG
  LL_GPIO_SetPinMode( 
                     TmrFreg.Pin.Gpio,
                     TmrFreg.Pin.Nbr,
                     LL_GPIO_MODE_ALTERNATE
                    );
  LL_GPIO_SetPinSpeed(
                      TmrFreg.Pin.Gpio,
                      TmrFreg.Pin.Nbr,
                      LL_GPIO_SPEED_FREQ_MEDIUM
                     );
  LL_GPIO_SetPinPull(
                     TmrFreg.Pin.Gpio,
                     TmrFreg.Pin.Nbr,
                     LL_GPIO_PULL_NO
                    );
  LL_GPIO_SetAFPin_8_15(
                        TmrFreg.Pin.Gpio,
                        TmrFreg.Pin.Nbr,
                        TmrFreg.Pin.AlternateMask        //TIM15_CH1
                       );

  LL_TIM_SetSlaveMode(
                      TmrFreg.Nbr,
                      LL_TIM_SLAVEMODE_DISABLED
                     );

  TPwm Pwm = cnt_pwm();

  LL_TIM_SetPrescaler( TmrFreg.Nbr, Pwm.PSC );
  LL_TIM_EnableARRPreload( TmrFreg.Nbr );
//  LL_TIM_DisableARRPreload( TmrFreg.Nbr );
  LL_TIM_SetAutoReload(
                       TmrFreg.Nbr,
                       Pwm.ARR
                      );

  //ШИМ-режим выбирается независимо для каждого канала
  LL_TIM_OC_ConfigOutput(
                         TmrFreg.Nbr,
                         TmrFreg.Ch,
                         LL_TIM_OCPOLARITY_HIGH
                        );
  LL_TIM_OC_SetMode(
                    TmrFreg.Nbr,
                    TmrFreg.Ch,
                    LL_TIM_OCMODE_PWM1
                   );
  LL_TIM_OC_EnablePreload(
                          TmrFreg.Nbr,
                          TmrFreg.Ch
                         );

  LL_TIM_OC_SetCompareCH1(
                          TmrFreg.Nbr,
                          Pwm.CCR
                         );
  LL_TIM_CC_EnableChannel(TmrFreg.Nbr, TmrFreg.Ch);

  //Preload-регистры включены => \
    необходимо программно сгенерировать событие UEV \
    для записи значений в рабочие (shadow) регистры   
  //Бит UG сбрасывается автоматически аппаратно
  LL_TIM_GenerateEvent_UPDATE( TmrFreg.Nbr );

//  //настройка прерывания вывода LED для подсчета зондирующих импульсов
//  Hw[_NBR].Led.EnableITСС(Hw[_NBR].TIMx);
  LL_TIM_EnableIT_CC1( TmrFreg.Nbr );

//  //настройка NVIC 
//  NVIC_SetPriority(Hw[_NBR].IRQn, 1);
//  NVIC_EnableIRQ(Hw[_NBR].IRQn);

    // Настройка NVIC
  NVIC_SetPriority(
                   TmrFreg.IRQ,
                   NVIC_EncodePriority( NVIC_GetPriorityGrouping(), 5, 0 )
                  );                                                       //обработчик вызывает API функцию RTOS => приоритет д.б. \
                                                                             логически ниже или равен, но численно больше или равен,\
                                                                             установленному  в макросе \
                                                                             configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
  NVIC_EnableIRQ( TIM1_BRK_TIM15_IRQn );

  LL_TIM_EnableCounter( TmrFreg.Nbr );
  LL_TIM_EnableAllOutputs( TmrFreg.Nbr );
}

void TExchngToPC::tmr_cmp_init()
{
  TmrCmp.Pin.en_clk   ( TmrCmp.Pin.ClkPortMask );       //тактирование GPIO, к которому подключен выход таймера
  TmrCmp.Clk.en_periph( TmrCmp.Clk.PeriphMask );        //тактирование таймера

  //настройка вывода CMP
  LL_GPIO_SetPinMode( 
                     TmrCmp.Pin.Gpio,
                     TmrCmp.Pin.Nbr,
                     LL_GPIO_MODE_ALTERNATE
                    );
  LL_GPIO_SetPinSpeed(
                      TmrCmp.Pin.Gpio,
                      TmrCmp.Pin.Nbr,
                      LL_GPIO_SPEED_FREQ_HIGH
                     );
  LL_GPIO_SetPinPull(
                     TmrCmp.Pin.Gpio,
                     TmrCmp.Pin.Nbr,
                     LL_GPIO_PULL_NO
                    );
  LL_GPIO_SetAFPin_0_7(
                        TmrCmp.Pin.Gpio,
                        TmrCmp.Pin.Nbr,
                        TmrCmp.Pin.AlternateMask        //TIM2_CH2
                       );

  LL_TIM_SetSlaveMode(
                      TmrCmp.Nbr,
                      LL_TIM_SLAVEMODE_DISABLED
                     );

//  TPwm Pwm = cnt_pwm();

  LL_TIM_SetPrescaler( TmrCmp.Nbr, 0U );
  LL_TIM_EnableARRPreload( TmrCmp.Nbr );

  LL_TIM_SetAutoReload(
                       TmrCmp.Nbr,
                       TmrCmp.ARR_MAX
                      );
  LL_TIM_IC_SetActiveInput(                              //канал конфигурируется как вход и 
                           TmrCmp.Nbr,                   //CCR1 становится read-only
                           TmrCmp.Ch,
                           LL_TIM_ACTIVEINPUT_DIRECTTI   
                          );
  LL_TIM_IC_SetFilter(
                      TmrCmp.Nbr,
                      TmrCmp.Ch,
                      LL_TIM_IC_FILTER_FDIV1_N2
                     );
  LL_TIM_IC_SetPolarity(
                        TmrCmp.Nbr,
                        TmrCmp.Ch,
                        LL_TIM_IC_POLARITY_RISING
                       );
  LL_TIM_IC_SetPrescaler(
                         TmrCmp.Nbr,                     //захват на каждом валидном переходе, т.е. предделитель отключен
                         TmrCmp.Ch,                      //предделитель сбрасывается как только CCxE=0 в регистре CCER (захват отключается)
                         LL_TIM_ICPSC_DIV1
                        );
  LL_TIM_CC_EnableChannel(
                          TmrCmp.Nbr,
                          TmrCmp.Ch
                         );
  

//  LL_TIM_EnableCounter( TmrCmp.Nbr );
//  LL_TIM_EnableAllOutputs( TmrCmp.Nbr );
}

void TExchngToPC::tmr_diff_init()
{
  TmrDiff.Pin.en_clk   ( TmrDiff.Pin.ClkPortMask );       //тактирование GPIO, к которому подключен выход таймера
  TmrDiff.Clk.en_periph( TmrDiff.Clk.PeriphMask );        //тактирование таймера

  //настройка вывода CMP
  LL_GPIO_SetPinMode( 
                     TmrDiff.Pin.Gpio,
                     TmrDiff.Pin.Nbr,
                     LL_GPIO_MODE_ALTERNATE
                    );
  LL_GPIO_SetPinSpeed(
                      TmrDiff.Pin.Gpio,
                      TmrDiff.Pin.Nbr,
                      LL_GPIO_SPEED_FREQ_HIGH
                     );
  LL_GPIO_SetPinPull(
                     TmrDiff.Pin.Gpio,
                     TmrDiff.Pin.Nbr,
                     LL_GPIO_PULL_NO
                    );
  LL_GPIO_SetAFPin_0_7(
                        TmrDiff.Pin.Gpio,
                        TmrDiff.Pin.Nbr,
                        TmrDiff.Pin.AlternateMask        //TIM2_CH2
                       );

  LL_TIM_SetSlaveMode(
                      TmrDiff.Nbr,
                      LL_TIM_SLAVEMODE_DISABLED
                     );

//  TPwm Pwm = cnt_pwm();

  LL_TIM_SetPrescaler( TmrDiff.Nbr, 0U );
//  LL_TIM_EnableARRPreload( TmrDiff.Nbr );
  LL_TIM_DisableARRPreload( TmrDiff.Nbr );
  LL_TIM_SetAutoReload(
                       TmrDiff.Nbr,
                       TmrDiff.ARR_MAX
                      );
  LL_TIM_IC_SetActiveInput(                              //канал конфигурируется как вход и 
                           TmrDiff.Nbr,                   //CCR1 становится read-only
                           TmrDiff.Ch,
                           LL_TIM_ACTIVEINPUT_DIRECTTI   
                          );
  LL_TIM_IC_SetFilter(
                      TmrDiff.Nbr,
                      TmrDiff.Ch,
                      LL_TIM_IC_FILTER_FDIV1_N2
                     );
  LL_TIM_IC_SetPolarity(
                        TmrDiff.Nbr,
                        TmrDiff.Ch,
                        LL_TIM_IC_POLARITY_RISING
                       );
  LL_TIM_IC_SetPrescaler(
                         TmrDiff.Nbr,                     //захват на каждом валидном переходе, т.е. предделитель отключен
                         TmrDiff.Ch,                      //предделитель сбрасывается как только CCxE=0 в регистре CCER (захват отключается)
                         LL_TIM_ICPSC_DIV1
                        );
  LL_TIM_CC_EnableChannel(
                          TmrDiff.Nbr,
                          TmrDiff.Ch
                         );
}

void TExchngToPC::set_diff_freq( uint16_t Divider )
{
  if ( Divider != 0U )
  {
    LL_TIM_SetPrescaler( TmrDiff.Nbr, Divider - 1U );
  }  
}

void TExchngToPC::adc_init()
{                                                                        
  Adc.Pin.en_clk( Adc.Pin.ClkPortMask );                                //тактирование GPIO, к которому подключен вход АЦП
  Adc.Clk.en_periph( Adc.Clk.PeriphMask );                              //тактирование АЦП
  Adc.sel_clk_src( Adc.SrcMask );                                       //выбор источника тактирования АЦП ( !!! по сбросу "No clock selected" )

  LL_GPIO_SetPinMode( Adc.Pin.Gpio, Adc.Pin.Nbr, LL_GPIO_MODE_ANALOG );
  
  /*
  *  АЦП по умолчанию в глубоко отключенном режиме
  */
  LL_ADC_DisableDeepPowerDown( Adc.Nbr );                               //выход из глубоко отключенного режима
  LL_ADC_EnableInternalRegulator( Adc.Nbr );                            //включить внутренний регулятор напряжения АЦП
   
  vTaskDelay( pdMS_TO_TICKS( 100U ) );                                  //программа должная ждать время включения регулятора напряжения для АЦП. TADCVREG_STUP = 20мкс (макс.)
  
  LL_ADC_SetChannelSingleDiff( Adc.Nbr, Adc.Ch, LL_ADC_SINGLE_ENDED );  //конфигурация канала как single-ended, выполняется только при ADEN=0
    
  adc_cal();
}

void TExchngToPC::adc_cal()
{
  if ( LL_ADC_IsEnabled( Adc.Nbr ) == true )
  {
    LL_ADC_Disable( Adc.Nbr );
    
    do {} while ( LL_ADC_IsDisableOngoing( Adc.Nbr ) == true );
    do {} while ( LL_ADC_IsEnabled( Adc.Nbr ) == true );
  }  
  
//  LL_ADC_SetCommonClock( Adc.Nbr, LL_ADC_CLOCK_SYNC_PCLK_DIV1,  );
  
  LL_ADC_StartCalibration( Adc.Nbr, LL_ADC_SINGLE_ENDED ); //ADCALDIF определяет к какому режиму входа применяется калибровка
                                                           //калибровочный коэффициент для преобразования однопроводного входа отличается от дифференциального
  
  do {} while ( LL_ADC_IsCalibrationOnGoing( Adc.Nbr ) == true );
  
  /*
  * Когда АЦП отключается ( ADEN = 0 ) на длительное время, необходимо сделать новый цикл калибровки
  * Перекалибровка нужна при сбросе питания АЦП ( Пр. вход в STANDBY или VBAT режим )
  * Калибровочный коэффициент может быть сохранен для дальнейшей перезаписи при старте следующей работы с АЦП
  * Перезапись можно делать при включенном АЦП ( ADEN = 1 ), но он не должен быть в процессе преобразования ( т.е ADSTART д.б. = 0 )   
  */
  AdcCalibFactorS = LL_ADC_GetCalibrationFactor( Adc.Nbr, LL_ADC_SINGLE_ENDED );
}

void TExchngToPC::init_diff_exti( uint32_t Trigger )
{
  Adc.DIFF.en_clk( Adc.DIFF.ClkPortMask );                              //включение тактирования GPIO для вывода внешнего прерывания
  Adc.Exti.en_syscfg_clk( Adc.Exti.SysCfgMask );                        //включение тактирования SYSCFG
                                                                        
  LL_GPIO_InitTypeDef GPIO_InitStruct;                                  //сконфигурировать вывод порта как вход (п. 8.3.8 RM0394 Rev 4)
  do
  {
    GPIO_InitStruct.Pin  = Adc.DIFF.Nbr;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  } while ( SUCCESS != LL_GPIO_Init( Adc.DIFF.Gpio, &GPIO_InitStruct ) );
  
  /*
   * System configuration controller (SYSCFG)
   * В том числе управляет соединением линии внешнего прерывания EXTI к GPIO
   */
  if ( Adc.Exti.SysCfgPort != LL_SYSCFG_GetEXTISource( Adc.Exti.SysCfgLine ) ) //проверка соединения линии внешнего прерывания EXTI к порту
  {
    LL_SYSCFG_SetEXTISource( Adc.Exti.SysCfgPort, Adc.Exti.SysCfgLine ); //настроить EXTI в SYSCFG
  }
  
  /*
  * Cконфигурировать линию как источник прерывания:
  *   сконфигурировать бит маски в регистре EXTI_IMR - по умолч. все линии замаскированы
  *	  сконфигурировать биты Trigger Selection (EXTI_RTSR и EXTI_FTSR)
  *	  сконфигурировать биты разрешения и маски, которые управляют NVIC IRQ каналом, отображенным на EXTI, так чтобы прерываниие с одной из линий EXTI могло быть корректно подтверждено
   */
  LL_EXTI_InitTypeDef EXTI_InitStruct;
  do
  {
    EXTI_InitStruct.Line_0_31   = Adc.Exti.Line;
    EXTI_InitStruct.LineCommand = ENABLE; //новое состояние выбранных EXTI линий
    EXTI_InitStruct.Mode        = Adc.Exti.Mode;
//    EXTI_InitStruct.Trigger     = Adc.Exti.Trigger;
    EXTI_InitStruct.Trigger     = Trigger;
  } while ( SUCCESS != LL_EXTI_Init( &EXTI_InitStruct ) );

//  // Настройка NVIC
  NVIC_SetPriority( Adc.Exti.IRQ, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0) ); //обработчик вызывает API функцию RTOS => приоритет д.б. логически ниже,
                                                                                           //но численно больше, установленного в макросе configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY  
                                                                                           //3 - максимальный уровень приоритета в программе ввиду важности своевременной обработки INT'a от микросхемы
  NVIC_EnableIRQ(Adc.Exti.IRQ);
}

void TExchngToPC::deinit_diff_exti()
{
//  LL_GPIO_DeInit( Adc.DIFF.Gpio ); //регистры GPIO возвращаются к значениям по умолчанию
  LL_EXTI_DeInit();
}

void TExchngToPC::en_adc()
{
  LL_ADC_ClearFlag_ADRDY( Adc.Nbr );
  LL_ADC_Enable( Adc.Nbr );                             //бит ADEN не может быть установлен при ADCAL = 1 и в течение \
                                                          4 тактов АЦП после аппаратного сброса бита ADCAL ( окончание калибровки )
  if ( LL_ADC_IsActiveFlag_ADRDY( Adc.Nbr )  == true )
  {
    LL_ADC_ClearFlag_ADRDY( Adc.Nbr );
  }
}

void TExchngToPC::dis_adc()
{
  if ( LL_ADC_REG_IsConversionOngoing( Adc.Nbr ) == true )
  {
    //есть продолжающееся преобразование АЦП из регулярной группы
    LL_ADC_REG_StopConversion( Adc.Nbr );
    do {} while( LL_ADC_REG_IsStopConversionOngoing( Adc.Nbr ) == true );
  }

  if ( LL_ADC_INJ_IsConversionOngoing( Adc.Nbr ) == true )
  {
    //есть продолжающееся преобразование АЦП из инжектированной группы
    LL_ADC_INJ_StopConversion( Adc.Nbr );
    do {} while( LL_ADC_INJ_IsConversionOngoing( Adc.Nbr ) == true );
  }
  
  LL_ADC_Disable( Adc.Nbr );
  do {} while ( LL_ADC_IsEnabled( Adc.Nbr ) == true );
  do {} while ( LL_ADC_IsDisableOngoing( Adc.Nbr ) == true );
}
  
//----- Обработчики приема -------------------------------------------------------------------------------------
void TExchngToPC::rx_set_freg( TExchngToPC::TParamHandle *ParamHandle )
{
  ctrl_pwm( ParamHandle );
}

void TExchngToPC::rx_set_duty_cycle( TExchngToPC::TParamHandle *ParamHandle )
{
  ctrl_pwm( ParamHandle );
}

void TExchngToPC::rx_set_pos_pot_1( TExchngToPC::TParamHandle *ParamHandle )
{
  ParamHandle->RxVal = chk_range( ParamHandle->RxVal, ParamHandle->Range );
  *ParamHandle->SetVal.Addr = ( uint8_t )ParamHandle->RxVal;
  AD8402.set_code( ParamHandle->RxVal, T_AD8402::TChannel::__ONE );
}

void TExchngToPC::rx_set_pos_pot_2( TExchngToPC::TParamHandle *ParamHandle )
{
  ParamHandle->RxVal = chk_range( ParamHandle->RxVal, ParamHandle->Range );
  *ParamHandle->SetVal.Addr = ( uint8_t )ParamHandle->RxVal;
  AD8402.set_code( ParamHandle->RxVal, T_AD8402::TChannel::__TWO );
}

void TExchngToPC::rx_start_adc( TExchngToPC::TParamHandle *ParamHandle )
{
  ParamHandle->RxVal = chk_range( ParamHandle->RxVal, ParamHandle->Range );
    
    UCG1.off(); //на время оцифровки пробросить сигнал с генератора
    
    //оцифровка до получения количества выборок == принятому значению в параметре
    en_adc();
    
    /*
    *  Биты управления кроме DIFSEL, ADCAL, ADEN можно записывать только если АЦП включен
    */
    //выбор общего числа преобразований в регулярной группе
    //каждому номеру преобразования можно поставить в соответствие свой номер канала АЦП
    //SQR-регистры не должны быть модифицированы когда происходят регулярные преобразования
    
    //остановить регулярные преобразования - при установке ADSTP текущее продолжающееся преобразование обрывается \
      с частичным отбрасыванием результата ( DR регистр не обновляется в текущем преобразовании )
    LL_ADC_REG_StopConversion( Adc.Nbr );
    do {} while( LL_ADC_REG_IsStopConversionOngoing( Adc.Nbr ) == true );
    do {} while( LL_ADC_REG_IsConversionOngoing( Adc.Nbr ) == true );
    
    LL_ADC_REG_SetSequencerLength( Adc.Nbr, LL_ADC_REG_SEQ_SCAN_DISABLE );
    LL_ADC_REG_SetSequencerRanks( Adc.Nbr, LL_ADC_REG_RANK_1, Adc.Ch );
    
    //перед запуском преобразования АЦП нужно установить непосредственную связь между источником напряжения (на входе АЦП) и \
      встроенным конденсатором выборки АЦП. \
      это время выборки должно быть достаточным для источника напряжения чтобы зарядить встроенные конденсаторы до \
      уровня входного напряжения. \
      Internal sample and hold capacitor: Cadc = 5pF ( DS11453 Rev 3 )
    //время выборки каждого канала программируется битами SMP в регистре SMPR1.
    //вычисление общего времени преобразования : \
      Tconv = Sampling time + 12.5 ADC clock cycles
    
    //I/O analog switches voltage booster - описание того что делать когда Vdda становится слишком низким    
    LL_ADC_SetChannelSamplingTime( Adc.Nbr, Adc.Ch, LL_ADC_SAMPLINGTIME_2CYCLES_5 ); //установка времени выборки канала. \
                                                                                       во время выборки биты выбора канала не должны изменяться
    LL_ADC_REG_SetContinuousMode( Adc.Nbr, LL_ADC_REG_CONV_CONTINUOUS ); //запрещено включать вместе прерывистый и непрерывный режимы работы
    LL_ADC_REG_SetTriggerSource( Adc.Nbr, LL_ADC_REG_TRIG_SOFTWARE );    //источник запуска группы регулярных преобразований    
    //возможно выполнить более быстрое преобразование снижением разрешения АЦП
    LL_ADC_SetResolution( Adc.Nbr, LL_ADC_RESOLUTION_8B );               //время одного преобразования (Fadc=8MHz, RES=8bits) =  1,375us
    
    LL_ADC_SetDataAlignment( Adc.Nbr, LL_ADC_DATA_ALIGN_RIGHT );         //выравнивание данных   
    LL_ADC_REG_StartConversion( Adc.Nbr );                               //запуск группы регулярных преобразований АЦП \
                                                                           т.к. был выбран программный триггер, то преобразование запускается немедленно
    
    init_diff_exti( Adc.Exti.Trigger );                                         //инициализация вывода DIFF, предназначенного для запуска преобразования АЦП
    xSemaphoreTake( DiffExti_TrigSem, portMAX_DELAY );                          //ожидание заднего фронта на DIFF
      deinit_diff_exti();
    
    for ( uint16_t Ctr = 0U; Ctr < ParamHandle->RxVal ; ++Ctr )
    {
//      do {} while ( LL_ADC_IsActiveFlag_EOC( Adc.Nbr ) == false );
      
//      Data[ Ctr ] = LL_ADC_REG_ReadConversionData8( Adc.Nbr );
      Data[ Ctr ] = 77;
//      LL_ADC_ClearFlag_EOC( Adc.Nbr );      
    }
    
    //устанавливается бит ADSTP => продолжающееся регулярное преобразование обрывается с частичной потерей результата. \
      по завершении процедуры биты ADSTP/ADSTART (для регулярного преобразования) сбрасываются аппаратно и программа \
      должна опрашивать ADSTART пока этот бит не сбросится что даст понять что АЦП полностью остановлен.
    LL_ADC_REG_StopConversion( Adc.Nbr ); 
    do {} while ( LL_ADC_REG_IsStopConversionOngoing( Adc.Nbr ) == true ); //опрос ADSTP
    do {} while ( LL_ADC_REG_IsConversionOngoing( Adc.Nbr ) == true );     //опрос ADSTART
    
    
    dis_adc();    
    
    UCG1.on();
}

void TExchngToPC::rx_cmp_capture( TExchngToPC::TParamHandle *ParamHandle )
{
  constexpr uint16_t MAX_N_CAP = 1000U;
  uint16_t nCap = 0U;          //текущее количество захваченных значений
  
  init_diff_exti( LL_EXTI_TRIGGER_FALLING );                                  //инициализация вывода DIFF, предназначенного для запуска захвата
  xSemaphoreTake( DiffExti_TrigSem, portMAX_DELAY );                          //ожидание заднего фронта на DIFF
    deinit_diff_exti();
    UCG1.off();                                                               //на время захвата пробросить сигнал с генератора     
//    init_diff_exti( LL_EXTI_TRIGGER_RISING );                                 //инициализация вывода DIFF, предназначенного для прекращения захвата
    
    uint32_t *pTmrCmp = reinterpret_cast< uint32_t * >( Data );
    
    LL_TIM_SetCounter( TmrCmp.Nbr, 0U );
    LL_TIM_EnableCounter( TmrCmp.Nbr );
    LL_TIM_EnableAllOutputs( TmrCmp.Nbr );
//    do
//    {
//      if ( LL_TIM_IsActiveFlag_CC2( TmrCmp.Nbr ) == true )
//      {
//        pTmrCmp[ nCap++ ] = LL_TIM_IC_GetCaptureCH2( TmrCmp.Nbr );
//      }
//    } while (
//             nCap < MAX_N_CAP
//             &&
//             xSemaphoreTake( DiffExti_TrigSem, 0U ) == pdFAIL
//            );

    LL_TIM_DisableCounter( TmrCmp.Nbr );
    deinit_diff_exti();
    UCG1.on();
    
    ParamHandle->RxVal = nCap * 4U;
}

void TExchngToPC::rx_diff_capture( TExchngToPC::TParamHandle *ParamHandle )
{
  set_diff_freq( ParamHandle->RxVal );
  
  constexpr uint16_t MAX_N_CAP = 100U;
  
//  UCG1.off();                                                               //на время захвата пробросить сигнал с генератора
  
  uint16_t *pTmrDiff = reinterpret_cast< uint16_t * >( Data );
  
  LL_TIM_GenerateEvent_UPDATE( TmrDiff.Nbr );
  LL_TIM_ClearFlag_UPDATE( TmrDiff.Nbr );
  LL_TIM_EnableCounter( TmrDiff.Nbr );
  LL_TIM_SetCounter( TmrDiff.Nbr, 0U );
  LL_TIM_ClearFlag_UPDATE( TmrDiff.Nbr );
  
  if ( LL_TIM_IsActiveFlag_UPDATE( TmrDiff.Nbr ) == true )
  {
    UIF = true;
  }
  
//  LL_TIM_EnableAllOutputs( TmrDiff.Nbr );
  do
  {
    if ( LL_TIM_IsActiveFlag_CC3( TmrDiff.Nbr ) == true )
    {
      pTmrDiff[ nCapDiff++ ] = LL_TIM_IC_GetCaptureCH3( TmrDiff.Nbr );
    }
  } while (
           nCapDiff < MAX_N_CAP
           &&
           LL_TIM_IsActiveFlag_UPDATE( TmrDiff.Nbr ) == false //нет переполнения таймера
          );
  
  if ( LL_TIM_IsActiveFlag_UPDATE( TmrDiff.Nbr ) == true )
  {
    UIF = true;
  }
  LL_TIM_ClearFlag_UPDATE( TmrDiff.Nbr );
  LL_TIM_DisableCounter( TmrDiff.Nbr );
  deinit_diff_exti();
//  UCG1.on();
  
  ParamHandle->RxVal = nCapDiff * 2U;
}
//----- //Обработчики приема -----------------------------------------------------------------------------------


//----- Обработчики передачи -----------------------------------------------------------------------------------
void TExchngToPC::tx_set_freg( TExchngToPC::TParamHandle *ParamHandle )
{  
  tx_data(
          ParamHandle->SetVal.Addr,
          ParamHandle->SetVal.BytesQty,
          TWrapTxSign::__ALL_WRAP_TX
         );
}

void TExchngToPC::tx_set_duty_cycle( TExchngToPC::TParamHandle *ParamHandle )
{
  tx_data(
          ParamHandle->SetVal.Addr,
          ParamHandle->SetVal.BytesQty,
          TWrapTxSign::__ALL_WRAP_TX
         );
}

void TExchngToPC::tx_set_pos_pot_1( TExchngToPC::TParamHandle *ParamHandle )
{
  tx_data(
          ParamHandle->SetVal.Addr,
          ParamHandle->SetVal.BytesQty,
          TWrapTxSign::__ALL_WRAP_TX
         );
}

void TExchngToPC::tx_set_pos_pot_2( TExchngToPC::TParamHandle *ParamHandle )
{
  tx_data(
          ParamHandle->SetVal.Addr,
          ParamHandle->SetVal.BytesQty,
          TWrapTxSign::__ALL_WRAP_TX
         );
}

void TExchngToPC::tx_test( TExchngToPC::TParamHandle *ParamHandle )
{
  tx_data(
          ParamHandle->SetVal.Addr,
          ParamHandle->SetVal.BytesQty,
          TWrapTxSign::__ALL_WRAP_TX
         );
}

void TExchngToPC::tx_adc( TExchngToPC::TParamHandle *ParamHandle )
{
  tx_data(
          ParamHandle->SetVal.Addr,
          0U,
          TWrapTxSign::__START_WRAP_TX
         );
  tx_data(
          ParamHandle->SetVal.Addr,
          ParamHandle->RxVal,
          TWrapTxSign::__NO_WRAP_TX
         );
  tx_data(
          ParamHandle->SetVal.Addr,
          0U,
          TWrapTxSign::__FINISH_WRAP_TX
         );
}

void TExchngToPC::tx_cmp_capture( TExchngToPC::TParamHandle *ParamHandle )
{
  tx_data(
          ParamHandle->SetVal.Addr,
          0U,
          TWrapTxSign::__START_WRAP_TX
         );
  tx_data(
          ParamHandle->SetVal.Addr,
          ParamHandle->RxVal,
          TWrapTxSign::__NO_WRAP_TX
         );
  tx_data(
          ParamHandle->SetVal.Addr,
          0U,
          TWrapTxSign::__FINISH_WRAP_TX
         );
}

void TExchngToPC::tx_diff_capture( TExchngToPC::TParamHandle *ParamHandle )
{
  tx_data(
          ParamHandle->SetVal.Addr,
          0U,
          TWrapTxSign::__START_WRAP_TX
         );
  tx_data(
          ParamHandle->SetVal.Addr,
          ParamHandle->RxVal,
          TWrapTxSign::__NO_WRAP_TX
         );
  tx_data(
          ParamHandle->SetVal.Addr,
          0U,
          TWrapTxSign::__FINISH_WRAP_TX
         );
}
//----- //Обработчики передачи ---------------------------------------------------------------------------------


float TExchngToPC::chk_range( float Val, const TRange &Range )
{
  if (
      iserr( Val )
      ||
      Val < Range.Min
     )
  {
    return Range.Min;
  }
  
  if ( Val > Range.Max )
  {
    return Range.Max;
  }
  
  return Val;
}

bool TExchngToPC::iserr( const float Val )
{
	if ( (*(__packed uint32_t*)&Val ) == 0xFFFFFFFF )
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool TExchngToPC::get_upd_demand_flag()
{
  bool Flag = false;
  
  __disable_irq();
    Flag = UpdDemandFlag;
  __enable_irq();
  
  return Flag;
}

void TExchngToPC::set_upd_demand_flag( bool Flag )
{
  __disable_irq();
    UpdDemandFlag = Flag;
  __enable_irq();
}

//----- //Реализация класса TExchngToPC ---------------------------------------------------


//----- Задача RTOS -----------------------------------------------------------------------
void lev_gauge_to_pc( void *Params ) //обмен уровнемера с ПК
{
  constexpr uint16_t START_DLY_MS = 100U;
  
  vTaskDelay( pdMS_TO_TICKS( START_DLY_MS ) );
  UCG1.init();
  UCG1.on();
  
  typedef void (TExchngToPC::*TFnct)( TExchngToPC::TParamHandle * );
  
  struct TCmdHandler
  {
    TExchngToPC::TParamHandle *ParamHandle;
    TFnct rxHandler;                        //обработчик приема
    TFnct txHandler;                        //обработчик передачи
  };

  TCmdHandler CmdHandler[] =
  {
    { &ParamHandle[0], &TExchngToPC::rx_set_freg       , &TExchngToPC::tx_set_freg       }, //SET_FREG
    { &ParamHandle[1], &TExchngToPC::rx_set_duty_cycle , &TExchngToPC::tx_set_duty_cycle }, //SET_DUTY_CYCLE
    { &ParamHandle[2], &TExchngToPC::rx_set_pos_pot_1  , &TExchngToPC::tx_set_pos_pot_1  }, //SET_POS_POT_1
    { &ParamHandle[3], &TExchngToPC::rx_set_pos_pot_2  , &TExchngToPC::tx_set_pos_pot_2  }, //SET_POS_POT_2
    { &ParamHandle[4], nullptr                         , &TExchngToPC::tx_test           }, //TEST
    { &ParamHandle[5], &TExchngToPC::rx_start_adc      , &TExchngToPC::tx_adc            }, //START_ADC
    { &ParamHandle[6], &TExchngToPC::rx_cmp_capture    , &TExchngToPC::tx_cmp_capture    }, //CMP_CAPTURE
    { &ParamHandle[7], &TExchngToPC::rx_diff_capture   , &TExchngToPC::tx_diff_capture   }, //DIFF_CAPTURE
  };
  
  AD8402.init();
  ExchngToPC.pin_clk_config();
  ExchngToPC.hw_init();
  
  ExchngToPC.tmr_freg_init();
  ExchngToPC.tmr_cmp_init();
  ExchngToPC.tmr_diff_init();
  ExchngToPC.adc_init();
   
  for ( ;; )
  {
    ExchngToPC.parse_pkt();                                   //ожидание валидного пакета
    for ( auto &item : CmdHandler )
    {
      TExchngToPC::TData Data = ExchngToPC.get_data();

      if ( item.ParamHandle->Cmd == Data.Cmd )
      {
        if ( item.rxHandler != nullptr )
        {
           item.ParamHandle->RxVal = Data.ParamVal;
          ( ExchngToPC.*item.rxHandler )( item.ParamHandle );
        }
        
        ( ExchngToPC.*item.txHandler )( item.ParamHandle );

        break;
      }
    };
      
   //если команда в принятом пакете не была найдена, то ответ не отправляется
    
//    vTaskDelay( pdMS_TO_TICKS( 2U ) );
  }  
}
//----- //Задача RTOS ---------------------------------------------------------------------

extern "C" void TIM1_BRK_TIM15_IRQHandler(void)
{
  if (
      LL_TIM_IsActiveFlag_CC1( TIM15 )
      &&
      LL_TIM_IsEnabledIT_CC1( TIM15 )
     )
  {
    LL_TIM_ClearFlag_CC1( TIM15 );
//    Led.free_toggle();
    
    if ( ExchngToPC.get_upd_demand_flag() == true )
    {
      ExchngToPC.set_upd_demand_flag( false );
      TmrHigherPriorityTaskWoken = pdFALSE;
      if ( xSemaphoreGiveFromISR( Tmr_TrigSem, &TmrHigherPriorityTaskWoken ) == pdFAIL ) //отправить семафор окончания записи
      {
        //семафор уже был доступен, т.е. ранее отдан другой задачей или прерыванием
      }
      if ( TmrHigherPriorityTaskWoken == pdPASS )
      {
         portYIELD_FROM_ISR(TmrHigherPriorityTaskWoken);  //принудительное переключение контекста для разблокировки задачи - обработчика
      } 
    }
  }
}

extern "C" void EXTI15_10_IRQHandler(void)
{
//    Do.closed();
//  RelFour.on();
//		RelThree.on();
  if ( LL_EXTI_IsActiveFlag_0_31( ExchngToPC.Adc.Exti.Line ) )
  {
    LL_EXTI_ClearFlag_0_31( ExchngToPC.Adc.Exti.Line  );
	 
	 Int1HigherPriorityTaskWoken = pdFALSE;
//   RelFour.on();
	 if ( xSemaphoreGiveFromISR( DiffExti_TrigSem, &Int1HigherPriorityTaskWoken ) == pdFAIL ) //отправить семафор окончания записи
    {
      //семафор уже был доступен, т.е. ранее отдан другой задачей или прерыванием
//   RelFour.toggle();
      
    }  
	 if ( Int1HigherPriorityTaskWoken == pdTRUE )
	 {
   
      portYIELD_FROM_ISR( Int1HigherPriorityTaskWoken ); //принудительное переключение контекста для разблокировки задачи - обработчика
	                                                       //максимально быстро перейти к считыванию данных с датчиков микросхемы MPU-9250
	                                                       //для FreeRTOS время от выдачи семафора до перехода к задаче на stm32f3 - 7мкс
	 }	 
  }
//  Do.open();
}
