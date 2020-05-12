#include "main.h"
#include "lib.h"
#include "rtos_headers.h"
#include "discrete_out.h"

#include "MPU-9250_acc_gyro_mag_driver_hl.h"

MPU_9250::TAccGyroMagDriver_HL AccMagGyro = { I2C_HW };

static BaseType_t Int1HigherPriorityTaskWoken;

namespace MPU_9250
{
  //Модифицировать константу под схемотехнику
  extern const TIntHw IntHw = 
  {
	  GPIOB,
	  LL_AHB2_GRP1_PERIPH_GPIOB,
	  LL_APB2_GRP1_PERIPH_SYSCFG,
	  LL_GPIO_PIN_12,
	  LL_GPIO_MODE_INPUT,
	  LL_GPIO_PULL_NO,
	  LL_SYSCFG_EXTI_PORTB,
	  LL_SYSCFG_EXTI_LINE12,
    LL_EXTI_LINE_12,
	  LL_EXTI_MODE_IT,
	  LL_EXTI_TRIGGER_RISING,
	  EXTI15_10_IRQn
  };

//  TAccGyroMagDriver_HL::TAccGyroMagDriver_HL( I2C_TypeDef *I2Cx ) 
  TAccGyroMagDriver_HL::TAccGyroMagDriver_HL( const TI2C_HW &_I2C_HW ) 
  : 
  I2C::TI2C
  ( 
   _I2C_HW, 
   I2C::TMode::_MASTER, 
	 I2C::TRate::_FAST, 
	 I2C::TAddressing::_7_BIT 
  ), //настройки, по которым будет работать I2C драйвер
  SampleCount(0),
  MagWorkFlag(false),
  GyroSensitivity(sensitivity_gyro::_3_FS_SEL),
  AccelSensitivity(sensitivity_accel::_3_FS_SEL),
  MagSensitivity(sensitivity_mag::_16_BIT_SENS),
  GyroSampleRate_us(),
  AccelSampleRate_us(),
  MagSampleRate_us(MAG_MODE2_RATE_us)
  {

  }
  
  TAccGyroMagDriver_HL::~TAccGyroMagDriver_HL()
  {
  
  }

  void TAccGyroMagDriver_HL::init_driver()
  {
    pin_clk_config();
    i2c_hw_init();
  }
  
  void TAccGyroMagDriver_HL::init_aux()
  {
	  HW.INT.en_clk( HW.INT.ClkPortMask ); //включение тактирования GPIO для вывода с INT, если не включено
	  LL_APB2_GRP1_EnableClock( LL_APB2_GRP1_PERIPH_SYSCFG ); //включение тактирования SYSCFG, если не включено
    
    LL_GPIO_InitTypeDef GPIO_InitStruct; //сконфигурировать вывод порта как вход (п. 11.3.8 17-Jan-2017 Rev.8)
	  do
	  {
	    GPIO_InitStruct.Pin  = HW.INT.Nbr;
	    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  } while ( SUCCESS != LL_GPIO_Init( HW.INT.Gpio, &GPIO_InitStruct ) );
	  
	  /*
	   * System configuration controller (SYSCFG)
	   * В том числе управляет соединением линии внешнего прерывания EXTI к GPIO
	   */
	  if ( HW.Exti.SysCfgPort != LL_SYSCFG_GetEXTISource( HW.Exti.SysCfgLine ) ) //проверка соединения линии внешнего прерывания EXTI к порту
	  {
	    LL_SYSCFG_SetEXTISource(HW.Exti.SysCfgPort, HW.Exti.SysCfgLine); //настроить EXTI в SYSCFG
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
	    EXTI_InitStruct.Line_0_31   = HW.Exti.Line;
	    EXTI_InitStruct.LineCommand = ENABLE; //новое состояние выбранных EXTI линий
	    EXTI_InitStruct.Mode        = HW.Exti.Mode;
	    EXTI_InitStruct.Trigger     = HW.Exti.Trigger;
	  } while (SUCCESS != LL_EXTI_Init(&EXTI_InitStruct));

    // Настройка NVIC
    NVIC_SetPriority( HW.Exti.IRQ, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0) ); //обработчик вызывает API функцию RTOS => приоритет д.б. логически ниже,
	                                                                                          //но численно больше, установленного в макросе configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY  
	                                                                                          //3 - максимальный уровень приоритета в программе ввиду важности своевременной обработки INT'a от микросхемы
    NVIC_EnableIRQ(HW.Exti.IRQ);
  }
	 
  void TAccGyroMagDriver_HL::init_chip()
  {
#ifndef __DEBUG__
	 uint8_t AccGyroAddr = get_acc_gyro_addr(_HIGH);
#else
	 uint8_t AccGyroAddr = get_acc_gyro_addr(_LOW);
#endif
	 
    //----- Настройка вывода INT микросхемы ------------------------------------------------------------
	 uint8_t TxByte;
	 uint8_t RegAddr = int_logic_level(nullptr); //получить адрес регистра микросхемы с необходимой настройкой
	 VLogicLevelINT LogicLevelINT = VLogicLevelINT::_ACTIVE_HIGH;
	 VPinConfigINT PinConfigINT = VPinConfigINT::_PUSH_PULL;
	 VLatchINT LatchINT = VLatchINT::_PULSE_50us;

//    RelFour.on();
	 read_single_byte(&TxByte, RegAddr, AccGyroAddr); //считать содержимое регистра микросхемы в TxByte
//    RelFour.off(); 

	 int_logic_level(&TxByte, LogicLevelINT); //модифицировать TxByte записью только необходимых настроек
	 int_pin_config(&TxByte, PinConfigINT); //модифицировать TxByte записью только необходимых настроек
	 int_latch_ctrl(&TxByte, LatchINT); //модифицировать TxByte записью только необходимых настроек

//	 RelFour.on();
	 write_single_byte(&TxByte, RegAddr, AccGyroAddr); //записать модифицированный TxByte в микросхему	 
//	 RelFour.off();

    //----- Настройка источника прерывания, выдаваемого на вывод INT -----------------------------------
	 RegAddr = raw_data_ready_int_ctrl(nullptr); //получить адрес регистра микросхемы с необходимой настройкой
	 VEnDis DataReadyINT = VEnDis::_ENABLE;

//    RelFour.on();
	 read_single_byte(&TxByte, RegAddr, AccGyroAddr); //считать содержимое регистра микросхемы в TxByte
//    RelFour.off(); 
	 
	 raw_data_ready_int_ctrl(&TxByte, DataReadyINT); //модифицировать TxByte записью только необходимых настроек
//	 RelFour.on();
	 write_single_byte(&TxByte, RegAddr, AccGyroAddr); //записать модифицированный TxByte в микросхему
//	 RelFour.off();
    //----- Настройка режима I2C Master микросхемы -----------------------------------------------------
	 RegAddr = i2c_master_if_module_ctrl(nullptr); //получить адрес регистра микросхемы с необходимой настройкой
	 VI2CMasterCtrl I2CMasterCtrl = VI2CMasterCtrl::_DISABLE_I2C_MASTER_MODULE; //выводы AUX_DA и AUX_SCL логически управляются \
                                                                                 выводами  SDA/SDI и SCL/ SCL																											
//	 RelFour.on();
	 read_single_byte(&TxByte, RegAddr, AccGyroAddr); //считать содержимое регистра микросхемы в TxByte
//	 RelFour.off();
	 i2c_master_if_module_ctrl(&TxByte, I2CMasterCtrl); //модифицировать TxByte записью только необходимых настроек
	 write_single_byte(&TxByte, RegAddr, AccGyroAddr); //записать модифицированный TxByte в микросхему
	 
    //----- Настройка Pass-Through Mode микросхемы -----------------------------------------------------
    //----- Позволяет пробросить I2C микроконтроллера к магнитометру -----------------------------------
//	 RegAddr = i2c_master_pins_ctrl(nullptr); //получить адрес регистра микросхемы с необходимой настройкой
//	 VEnDis I2CBypassCtrl = VEnDis::_ENABLE; //включить проброс выводов I2C к магнитометру
////	 LatchINT = VLatchINT::_LEVEL_HELD_UNTIL_CLR; 
//	 
//	 read_single_byte(&TxByte, RegAddr, AccGyroAddr); //считать содержимое регистра микросхемы в TxByte
//	 i2c_master_pins_ctrl(&TxByte, I2CBypassCtrl); //модифицировать TxByte записью только необходимых настроек
////	 int_latch_ctrl(&TxByte, LatchINT);
//	 write_single_byte(&TxByte, RegAddr, AccGyroAddr); //записать модифицированный TxByte в микросхему
	 
	 config_gyro();
	 config_acc();
//	 config_mag();
  }

  bool TAccGyroMagDriver_HL::check_id_acc_gyro()
  {
    uint8_t TxByte;
	 uint8_t RegAddr = get_who_am_i(); //получить адрес регистра микросхемы
	 
#ifndef __DEBUG__
	 uint8_t AccGyroAddr = get_acc_gyro_addr(_HIGH);
#else
	 uint8_t AccGyroAddr = get_acc_gyro_addr(_LOW);
#endif
 
	 read_single_byte(&TxByte, RegAddr, AccGyroAddr); //считать содержимое регистра микросхемы в TxByte
	 
	 return TxByte == WHO_AM_I;
  }

  bool TAccGyroMagDriver_HL::check_id_mag()
  {
    uint8_t TxByte;
	 
	 //Для доступа к регистрам магнитометра, необходимо предварительно установить режим Pass-Through
	 
	 uint8_t RegAddr = mag_device_id(); //получить адрес регистра микросхемы
//	 read_single_byte(TAccGyroMagDriver_LL::ProtocolRdOne, &TxByte, RegAddr, 0x06); //считать содержимое регистра микросхемы в TxByte

//    RelFour.on();	
	 read_single_byte(&TxByte, RegAddr, get_mag_addr()); //считать содержимое регистра микросхемы в TxByte	
//    RelFour.off();	 
	 
	 AKM_ID = TxByte;
	 return TxByte == WIA;
  }
  
  void TAccGyroMagDriver_HL::config_gyro()
  {
    uint8_t TxByte;
#ifndef __DEBUG__
	 uint8_t AccGyroAddr = get_acc_gyro_addr(_HIGH);
#else
	 uint8_t AccGyroAddr = get_acc_gyro_addr(_LOW);
#endif
 
	 uint8_t RegAddr = gyro_full_scale_select(nullptr); //получить адрес регистра микросхемы с необходимой настройкой
	 VGyroFullScaleSelect FullScale = VGyroFullScaleSelect::_2000_DPS;
//	 VGyroFullScaleSelect FullScale = VGyroFullScaleSelect::_1000_DPS;
//	 VGyroFullScaleSelect FullScale = VGyroFullScaleSelect::_250_DPS;
	 
	 read_single_byte(&TxByte, RegAddr, AccGyroAddr); //считать содержимое регистра микросхемы в TxByte
	 gyro_full_scale_select(&TxByte, FullScale); //модифицировать TxByte записью только необходимых настроек
	 switch (FullScale)
	 {
	   case VGyroFullScaleSelect::_2000_DPS:
		     GyroSensitivity = sensitivity_gyro::_3_FS_SEL;
		     break;
	   case VGyroFullScaleSelect::_1000_DPS:
		     GyroSensitivity = sensitivity_gyro::_2_FS_SEL;
		     break;
	   case VGyroFullScaleSelect::_500_DPS:
		     GyroSensitivity = sensitivity_gyro::_1_FS_SEL;
		     break;
	   case VGyroFullScaleSelect::_250_DPS:
           GyroSensitivity = sensitivity_gyro::_0_FS_SEL;
           break;
	   default:
		     break;
	 }
	 write_single_byte(&TxByte, RegAddr, AccGyroAddr); //записать модифицированный TxByte в микросхему
   	 
	 //----- Включение возможности выбора полосы пропускания фильтра гироскопа и термодатчика - Fchoice --------------------
	 RegAddr = gyro_filter_choise(nullptr); //получить адрес регистра микросхемы с необходимой настройкой
	 VGyroFilterChoise FilterChoise = VGyroFilterChoise::_11;

	 read_single_byte(&TxByte, RegAddr, AccGyroAddr); //считать содержимое регистра микросхемы в TxByte
	 gyro_filter_choise(&TxByte, FilterChoise); //модифицировать TxByte записью только необходимых настроек
	 write_single_byte(&TxByte, RegAddr, AccGyroAddr); //записать модифицированный TxByte в микросхему

	 //----- Конфигурация полосы пропускания фильтра гироскопа и термодатчика ----------------------------------------------
	 RegAddr = filter_gyro_temp_config(nullptr); //получить адрес регистра микросхемы с необходимой настройкой
	 VFilterGyroTempConfig FilterGyroTempConfig = VFilterGyroTempConfig::_G184_2d9_1_T188_1d9;
	 
	 read_single_byte(&TxByte, RegAddr, AccGyroAddr); //считать содержимое регистра микросхемы в TxByte
	 filter_gyro_temp_config(&TxByte, FilterGyroTempConfig); //модифицировать TxByte записью только необходимых настроек
	 write_single_byte(&TxByte, RegAddr, AccGyroAddr); //записать модифицированный TxByte в микросхему
	   
	 //----- Проверка установленной полосы пропускания фильтра гироскопа и термодатчика ------------------------------------
	 read_single_byte(&TxByte, RegAddr, AccGyroAddr); //считать содержимое регистра микросхемы в TxByte	 
	 
	 filter_gyro_temp_read(TxByte, &FilterGyroTempConfig);
	 switch (FilterGyroTempConfig)
	 {
	   case VFilterGyroTempConfig::_G10_17d85_1_T10_13d4:
	   case VFilterGyroTempConfig::_G184_2d9_1_T188_1d9:
	   case VFilterGyroTempConfig::_G20_9d9_1_T20_8d3:
	   case VFilterGyroTempConfig::_G41_5d9_1_T42_4d8:
	   case VFilterGyroTempConfig::_G5_33d48_1_T5_18d6:
	   case VFilterGyroTempConfig::_G92_3d9_1_T98_2d8:
		     GyroSampleRate_us = GYRO_1_kHz_SAMPLE_RATE_us;
           break;
	   case VFilterGyroTempConfig::_G250_097_8_T4000_004:
	   case VFilterGyroTempConfig::_G3600_0d17_8_T4000_0d04:
		     GyroSampleRate_us = GYRO_8_kHz_SAMPLE_RATE_us;
           break;
	   default:
		     break;
	 } 
   
//   //----- Настройка делителя частоты выборки для 100Гц (при частоте выборки 1000Гц)
//	 RegAddr = sample_rate_divider(); //получить адрес регистра микросхемы с необходимой настройкой
//	 TxByte = 0x09;
//	 write_single_byte(&TxByte, RegAddr, AccGyroAddr); //записать модифицированный TxByte в микросхему
//   TxByte = 0x00;
//	 read_single_byte(&TxByte, RegAddr, AccGyroAddr); //считать содержимое регистра микросхемы в TxByte	 
   
  }
  
  void TAccGyroMagDriver_HL::config_acc()
  {
    uint8_t TxByte;
#ifndef __DEBUG__
	  uint8_t AccGyroAddr = get_acc_gyro_addr(_HIGH);
#else
	  uint8_t AccGyroAddr = get_acc_gyro_addr(_LOW);
#endif
 
	  uint8_t RegAddr = accel_full_scale_select(nullptr); //получить адрес регистра микросхемы с необходимой настройкой
	  VAccelFullScaleSelect FullScale = VAccelFullScaleSelect::_8G;
    
	  read_single_byte(&TxByte, RegAddr, AccGyroAddr); //считать содержимое регистра микросхемы в TxByte
	  accel_full_scale_select(&TxByte, FullScale); //модифицировать TxByte записью только необходимых настроек
    
	  switch (FullScale)
	  {
	    case VAccelFullScaleSelect::_16G:
		      AccelSensitivity = sensitivity_accel::_3_FS_SEL;
		      break;
	    case VAccelFullScaleSelect::_8G:
		      AccelSensitivity = sensitivity_accel::_2_FS_SEL;
		      break;
	    case VAccelFullScaleSelect::_4G:
		      AccelSensitivity = sensitivity_accel::_1_FS_SEL;
		      break;
	    case VAccelFullScaleSelect::_2G:
		      AccelSensitivity = sensitivity_accel::_0_FS_SEL;
		      break;
	    default:
		      break;
	  }

	  write_single_byte(&TxByte, RegAddr, AccGyroAddr); //записать модифицированный TxByte в микросхему
    
//	 //----- Включение возможности выбора полосы пропускания фильтра гироскопа и термодатчика - Fchoice --------------------
//	 RegAddr = gyro_filter_choise(nullptr); //получить адрес регистра микросхемы с необходимой настройкой
//	 VGyroFilterChoise FilterChoise = VGyroFilterChoise::_11;

//	 read_single_byte(&TxByte, RegAddr, AccGyroAddr); //считать содержимое регистра микросхемы в TxByte
//	 gyro_filter_choise(&TxByte, FilterChoise); //модифицировать TxByte записью только необходимых настроек
//	 write_single_byte(&TxByte, RegAddr, AccGyroAddr); //записать модифицированный TxByte в микросхему
//   
//   //----- Настройка делителя частоты выборки для 1000Гц (при частоте выборки 8000Гц)
//	 RegAddr = sample_rate_divider(); //получить адрес регистра микросхемы с необходимой настройкой
//	 TxByte = 0x07;
//	 write_single_byte(&TxByte, RegAddr, AccGyroAddr); //записать модифицированный TxByte в микросхему
   
	  //----- Настройка скорости и полосы пропускания фильтра акселерометра ------------------------------------------------
	  RegAddr = accel_filter_choise(nullptr); //получить адрес регистра микросхемы с необходимой настройкой
	  VAccelFilterChoise AccelFilterChoise = VAccelFilterChoise::_1;
	  VSetAccelFilter SetAccelFilter = VSetAccelFilter::_218_1Hz_1_88ms;
    
	  read_single_byte(&TxByte, RegAddr, AccGyroAddr); //считать содержимое регистра микросхемы в TxByte
	  accel_filter_choise(&TxByte, AccelFilterChoise); //модифицировать TxByte записью только необходимых настроек
	  set_accel_filter(&TxByte, SetAccelFilter); //модифицировать TxByte записью только необходимых настроек
	  write_single_byte(&TxByte, RegAddr, AccGyroAddr); //записать модифицированный TxByte в микросхему
    
	  //----- Проверка установленной полосы пропускания фильтра гироскопа и термодатчика ------------------------------------
	  read_single_byte(&TxByte, RegAddr, AccGyroAddr); //считать содержимое регистра микросхемы в TxByte
    
	  accel_filter_read(TxByte, &SetAccelFilter);
	  switch (SetAccelFilter)
	  {
	    case VSetAccelFilter::_10_2Hz_16_83ms:
	    case VSetAccelFilter::_218_1Hz_1_88ms:
	    case VSetAccelFilter::_21_2Hz_8_87ms:
	    case VSetAccelFilter::_420Hz_1_38ms:
	    case VSetAccelFilter::_44_8Hz_4_88ms:
	    case VSetAccelFilter::_5_05Hz_32_48ms:
	    case VSetAccelFilter::_99Hz_2_88ms:
		       AccelSampleRate_us = ACCEL_1_kHz_SAMPLE_RATE_us;
           break;
	    default:
		       break;
	  }
  }

  void TAccGyroMagDriver_HL::config_mag()
  {
    //----- Перед работой с магнитометром необходимо проверить: включен ли Pass-Through Mode ---------------------------------
	 //----- Если не включен, включить Pass-Through Mode ----------------------------------------------------------------------
	 uint8_t TxByte;
#ifndef __DEBUG__
	 uint8_t SlaveAddr = get_acc_gyro_addr(_HIGH);
#else
	 uint8_t SlaveAddr = get_acc_gyro_addr(_LOW);
#endif
 
	 uint8_t RegAddr = i2c_master_pins_ctrl(nullptr); //получить адрес регистра микросхемы с необходимой настройкой
	 
	 read_single_byte(&TxByte, RegAddr, SlaveAddr); //считать содержимое регистра микросхемы в TxByte
//    RelFour.off();
	 
	 VEnDis I2CBypassCtrl; //включить проброс выводов I2C к магнитометру
	 i2c_master_pins_read(TxByte, &I2CBypassCtrl);
	 
	 switch (I2CBypassCtrl)
	 {
	   case VEnDis::_DISABLE:
	        I2CBypassCtrl = VEnDis::_ENABLE; //включить проброс выводов I2C к магнитометру
	        i2c_master_pins_ctrl(&TxByte, I2CBypassCtrl); //модифицировать TxByte записью только необходимых настроек
	        write_single_byte(&TxByte, RegAddr, SlaveAddr); //записать модифицированный TxByte в микросхему
		     break;
	   case VEnDis::_ENABLE:
		     //Pass-Through Mode включен, можно настраивать магнитометр 
		     break;
	   default:
		     break;
	 }

	 if (!check_id_mag())
	 { 
		MagState = TMagCheck::_MAG_ERROR;
		return;
	 }
	 else
	 {
	   MagState = TMagCheck::_MAG_OK;
	 }
	 
    
	 //----- Настройка разрядности выходных данных и режима работы магнитометра -----------------------------------------------
	 SlaveAddr = get_mag_addr(); 
	 RegAddr   = mag_output_bit_set(nullptr); //получить адрес регистра микросхемы с необходимой настройкой
	 VMagOutputBitSet MagOutputBitSet = VMagOutputBitSet::_16_BIT_OUTPUT;
	 VMagOperationModeSet MagOperationModeSet = VMagOperationModeSet::_CONTINUOUS_MEAS_MODE_2;
	 
	 read_single_byte(&TxByte, RegAddr, SlaveAddr); //считать содержимое регистра микросхемы в TxByte
	 mag_output_bit_set(&TxByte, MagOutputBitSet); //модифицировать TxByte записью только необходимых настроек
	 mag_operation_mode_set(&TxByte, MagOperationModeSet); //модифицировать TxByte записью только необходимых настроек
	 write_single_byte(&TxByte, RegAddr, SlaveAddr); //записать модифицированный TxByte в микросхему
    
	 //----- Проверка установленной разрядности магнитометра -------------------------------------------------------------------
	 read_single_byte(&TxByte, RegAddr, SlaveAddr); //считать содержимое регистра микросхемы в TxByte	 
	 
	 VMagOutputBitSet BIT;
	 VMagOperationModeSet MODE;
	 mag_output_bit_read(TxByte, &BIT);
	 mag_operation_mode_read(TxByte, &MODE);
	 switch (BIT)
	 {
	   case VMagOutputBitSet::_14_BIT_OUTPUT:
		     MagSensitivity = sensitivity_mag::_14_BIT_SENS;
		     break;
	   case VMagOutputBitSet::_16_BIT_OUTPUT:
		     MagSensitivity = sensitivity_mag::_16_BIT_SENS;
		     break;
	   default:
		     break;
	 }
	 
	 switch (MODE)
	 {
	   case VMagOperationModeSet::_CONTINUOUS_MEAS_MODE_1:
		     MagSampleRate_us = MAG_MODE1_RATE_us;
		     break;
	   case VMagOperationModeSet::_CONTINUOUS_MEAS_MODE_2:
		     MagSampleRate_us = MAG_MODE2_RATE_us;
		     break;
	   default:
		     break;
	 }
	 
	 MagWorkFlag = true; //после установки режима, магнитометр начинает измерять
  }
  
  TMagCheck TAccGyroMagDriver_HL::get_mag_state()
  {
    return MagState;
  }
  
  
//  void TAccGyroMagDriver_HL::gyro_offset_take_account(TAxesData &OffsetDPS)
//  {
//    uint8_t TxByte;
//	 uint8_t RegAddr = gyro_full_scale_select(nullptr); //получить адрес регистра микросхемы с необходимой настройкой
//	 uint8_t AccGyroAddr = get_acc_gyro_addr();
//	 
//	 
//	 read_single_byte(TAccGyroMagDriver_LL::ProtocolRdOne, &TxByte, RegAddr, AccGyroAddr); //считать содержимое регистра микросхемы в TxByte
//	 
//	 VGyroFullScaleSelect GYRO_FS_SEL;
//	 gyro_full_scale_read(TxByte, &GYRO_FS_SEL);
//	 
//	 float GyroSensitivity;
//	 
//	 switch (GYRO_FS_SEL)
//	 {
//	   case VGyroFullScaleSelect::_250_DPS:
//		     GyroSensitivity = sensitivity_gyro::_0_FS_SEL;
//		     break;
//	   case VGyroFullScaleSelect::_500_DPS:
//		     GyroSensitivity = sensitivity_gyro::_1_FS_SEL;
//		     break;
//	   case VGyroFullScaleSelect::_1000_DPS:
//		     GyroSensitivity = sensitivity_gyro::_2_FS_SEL;
//		     break;
//	   case VGyroFullScaleSelect::_2000_DPS:
//		     GyroSensitivity = sensitivity_gyro::_3_FS_SEL;
//		     break;
//	   default:
//		     break;
//	 }
//	 
//	 float Ratio = pow(2, static_cast<uint8_t>(GYRO_FS_SEL)) * GyroSensitivity / static_cast<float>(4);
//	 	 
//	 OffsetDPS.X *= Ratio;
//	 OffsetDPS.Y *= Ratio;
//	 OffsetDPS.Z *= Ratio;
//	 uint16_t DataBurst[MAX_AXES_NUM] = 
//	 {
//      rev_half_word(*reinterpret_cast<TSensorData *>(&OffsetDPS.X)),
//      rev_half_word(*reinterpret_cast<TSensorData *>(&OffsetDPS.Y)),
//      rev_half_word(*reinterpret_cast<TSensorData *>(&OffsetDPS.Z))
//	 };
//    write_burst(TAccGyroMagDriver_LL::ProtocolWrMul, 
//	             reinterpret_cast<uint8_t *>(DataBurst), 
//					 MAX_AXES_NUM, 
//					 remove_gyro_bias_x_h(),
//					 AccGyroAddr);
//  }
//  
  uint32_t TAccGyroMagDriver_HL::pow(uint8_t Num, uint8_t Exp)
  {
    uint32_t Product = 1;
	 while (Exp--)
	 {
	   Product *= Num;
	 }
	 return Product;
  }
  
  void TAccGyroMagDriver_HL::collect_all_data()
  {
#ifndef __DEBUG__
	 uint8_t SlaveAddr = get_acc_gyro_addr(_HIGH);
#else
	 uint8_t SlaveAddr = get_acc_gyro_addr(_LOW);
#endif
 
    uint8_t SensorData[MAX_DATA_BYTES]= {0}; //MAX_DATA_BYTES не учитывает магнитометр
	                                          //!!!Не забывать о доступе к массиву из прерываний
	  uint8_t RegAddr = accel_data_x_h(); //получить адрес регистра микросхемы с необходимой настройкой

//    RelFour.on();
    read_burst(SensorData, MAX_DATA_BYTES, RegAddr, SlaveAddr);
	 
    convert_all_data(SensorData);
  }

  void TAccGyroMagDriver_HL::collect_acc_data()
  {
#ifndef __DEBUG__
	 uint8_t SlaveAddr = get_acc_gyro_addr(_HIGH);
#else
	 uint8_t SlaveAddr = get_acc_gyro_addr(_LOW);
#endif

    uint8_t SensorData[ACC_DATA_BYTES]= {0}; //MAX_DATA_BYTES не учитывает магнитометр \
                                               !!!Не забывать о доступе к массиву из прерываний
	  uint8_t RegAddr = accel_data_x_h(); //получить адрес регистра микросхемы с необходимой настройкой
    read_burst(SensorData, ACC_DATA_BYTES, RegAddr, SlaveAddr);
       
    convert_acc_data(SensorData);                                            


  }

  void TAccGyroMagDriver_HL::convert_all_data(uint8_t *SensorData)
  {
    //приведение данных с датчика к формату Little-endian
//	   uint8_t RD_DATA_SIZE = sizeof (TReadData);
    
	   for (uint8_t DataCtr = 0; DataCtr < sizeof (TReadData) / sizeof(uint16_t) ; ++DataCtr)
	   {
	     TSensorData RawData;
		    RawData.Data = ((uint16_t *)SensorData)[DataCtr];
		    ((uint16_t *)SensorData)[DataCtr] = rev_half_word(RawData);
	   }
    
	   //преобразование полученных данных
	   count_temperature(((TAccTempGyroData *)SensorData)->Temp);
	   count_accel( (TRawAxesData *)&(((TAccTempGyroData *)SensorData)->AccelX) );
	   count_gyro( (TRawAxesData *)&(((TAccTempGyroData *)SensorData)->GyroX) );
  }
  
  void TAccGyroMagDriver_HL::convert_acc_data(uint8_t *SensorData)
  {
    //приведение данных с датчика к формату Little-endian
//	   uint8_t RD_DATA_SIZE = sizeof (TAccData);
    
	   for (uint8_t DataCtr = 0; DataCtr < sizeof (TAccData) / sizeof(uint16_t) ; ++DataCtr)
	   {
	     TSensorData RawData;
		    RawData.Data = ((uint16_t *)SensorData)[DataCtr];
		    ((uint16_t *)SensorData)[DataCtr] = rev_half_word(RawData);
	   }
     
	   count_accel( (TRawAxesData *)&(((TAccTempGyroData *)SensorData)->AccelX) );
  }
  
  void TAccGyroMagDriver_HL::collect_mag_data()
  {
    uint8_t SlaveAddr = get_mag_addr();
    uint8_t SensorData[MAX_AXES_NUM * NUM_BYTES_PER_AXIS + 1]= {0}; //MAX_DATA_BYTES не учитывает магнитометр
	                                                                 //!!!Не забывать о доступе к массиву из прерываний
																						  //дополнительно к данным считываем регистр с флагом переполнения, который показывает валидность считанных данных
	 uint8_t RegAddr = mag_lower_x_data(); //получить адрес регистра микросхемы с необходимой настройкой	 

//	 RelFour.on();
	 read_burst(SensorData, sizeof SensorData, RegAddr, SlaveAddr);
//	 RelFour.off();
	 convert_mag_data(SensorData);
  }

  void TAccGyroMagDriver_HL::convert_mag_data(uint8_t *SensorData) 
  {
	 VStatusFlag BITM;
	 mag_mirror_data_bit(((TRawMagData *)SensorData)->ST2, &BITM);
	 
	 VStatusFlag HOFL;
	 mag_overflow(((TRawMagData *)SensorData)->ST2, &HOFL);
	 if (HOFL == VStatusFlag::_NO) //если нет переполнения, т.е. если |X|+|Y|+|Z| < 4912μT (AsahiKASEI AK8963 6.4.3.6. Magnetic Sensor Overflow)
	 {
	   //данные валидны
		count_magneto( (TRawAxesData *)&(((TRawMagData *)SensorData)->MagData) );
	 }
	 else
	 {
	   //данные не валидны, в выборку не попадают
	 }
  }

  void TAccGyroMagDriver_HL::count_temperature(const int16_t Data)
  {
    Temperature = (Data - temp_mpu_9250::ROOM_TEMP_OFFSET) / temp_mpu_9250::SENSITIVITY + 21; //MPU-9250 Register Map and Descriptions Revision: 1.6
  }

  void TAccGyroMagDriver_HL::count_accel(const TRawAxesData *DataPtr)
  {
    Accel.X = static_cast<float>(DataPtr->X) / AccelSensitivity;
    Accel.Y = static_cast<float>(DataPtr->Y) / AccelSensitivity;
    Accel.Z = static_cast<float>(DataPtr->Z) / AccelSensitivity;
  }

  void TAccGyroMagDriver_HL::count_gyro(const TRawAxesData *DataPtr)
  {
    Gyro.X = static_cast<float>(DataPtr->X) / GyroSensitivity;
    Gyro.Y = static_cast<float>(DataPtr->Y) / GyroSensitivity;
    Gyro.Z = static_cast<float>(DataPtr->Z) / GyroSensitivity;
  }

  void TAccGyroMagDriver_HL::count_magneto(const TRawAxesData *DataPtr)
  {
    Magneto.X = static_cast<float>(DataPtr->X) * MagSensitivity;
    Magneto.Y = static_cast<float>(DataPtr->Y) * MagSensitivity;
    Magneto.Z = static_cast<float>(DataPtr->Z) * MagSensitivity;
  }

  const TModel::TAccelData *TAccGyroMagDriver_HL::get_acc_data() const
  {
    return &Accel;
  }

  const TGyroData *TAccGyroMagDriver_HL::get_gyro_data() const
  {
    return &Gyro;
  }

  const TMagData *TAccGyroMagDriver_HL::get_magneto_data() const
  {
    return &Magneto;
  }

  float TAccGyroMagDriver_HL::get_temperature() const
  {
    return Temperature;
  }

  uint32_t TAccGyroMagDriver_HL::get_gyro_sample_rate() const
  {
    return GyroSampleRate_us;
  }

  uint32_t TAccGyroMagDriver_HL::get_accel_sample_rate() const
  {
    return AccelSampleRate_us;
  }

  uint32_t TAccGyroMagDriver_HL::get_mag_sample_rate() const
  {
    return MagSampleRate_us;
  }
}

extern "C" void EXTI15_10_IRQHandler(void)
{
//    Do.closed();
//  RelFour.on();
//		RelThree.on();
  if ( LL_EXTI_IsActiveFlag_0_31( I2C_HW.Exti.Line ) )
  {
    LL_EXTI_ClearFlag_0_31( I2C_HW.Exti.Line  );
	 
	 Int1HigherPriorityTaskWoken = pdFALSE;
//   RelFour.on();
	 if (xSemaphoreGiveFromISR(RawDataMems_RdySem, &Int1HigherPriorityTaskWoken) == pdFAIL) //отправить семафор окончания записи
    {
      //семафор уже был доступен, т.е. ранее отдан другой задачей или прерыванием
//   RelFour.toggle();
      
    }  
	 if (Int1HigherPriorityTaskWoken == pdTRUE)
	 {
   
      portYIELD_FROM_ISR(Int1HigherPriorityTaskWoken); //принудительное переключение контекста для разблокировки задачи - обработчика
	                                                    //максимально быстро перейти к считыванию данных с датчиков микросхемы MPU-9250
	                                                    //для FreeRTOS время от выдачи семафора до перехода к задаче на stm32f3 - 7мкс
	 }	 
  }
//  Do.open();
}


