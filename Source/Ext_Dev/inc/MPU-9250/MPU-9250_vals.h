#ifndef __MPU_9250_VALS_H
#define __MPU_9250_VALS_H

//#include <cstdint>

namespace MPU_9250
{
  using std::uint8_t;
  using std::uint32_t;

//  enum TI2CProtocol  //константы - I2C протокол датчика
//  {
//    ST,    //старт условие
//    SADW,  //slave адрес + записать
//    SADR,  //slave адрес + считать
//    SAK,   //подтверждение от slave
//    SUB,   //передача подадреса
//    DATAS, //передача данных от slave
//    DATAM, //передача данных от master
//    SP,    //стоп условие
//    SR,    //повстарт
//    MAK,   //подтверждение от master
//    NMAK,  //нет подтверждения от slave
//    MAX_P
//  };
  
//  enum I2CModes
//  {
//    STANDARD,                    //до 100кГц
//    FAST,                        //до 400кГц
//  };

  enum VGyroSensitivity : uint32_t  //mLSB/dps
  {
  
  };
  
  enum TCommType : uint8_t
  {
	 _WRITE = 0,
    _READ,
  };
  
  enum TLogicLevelAD0 : uint8_t
  {
    _LOW = 0,
	 _HIGH
  };
  
//  enum class TInternalSensor : uint8_t
//  {
//    _ACCEL_GYRO,
//	 _MAGNETO
//  };
//  
  enum class VEnDis : uint8_t
  {
    _DISABLE,
	 _ENABLE
  };
  
  enum class VLogicLevelINT : bool
  {
    _ACTIVE_HIGH,
	 _ACTIVE_LOW
  };

  enum class VPinConfigINT : bool
  {
    _PUSH_PULL,
	 _OPEN_DRAIN
  };

  enum class VI2CMasterCtrl : bool
  {
    _DISABLE_I2C_MASTER_MODULE,
	 _ENABLE_I2C_MASTER_MODULE
  };

  enum class VLatchINT : bool
  {
	 _PULSE_50us,
    _LEVEL_HELD_UNTIL_CLR
  };

  enum class VFifoModeConfig : bool
  {
    _FIFO_FULL_YES_WRITE = 0,
    _FIFO_FULL_NO_WRITE
  };

  enum class VFsyncPinConfig : uint8_t
  {
    _DISABLED,
	 _TEMP_OUT_L_0,
	 _GYRO_XOUT_L_0,
	 _GYRO_YOUT_L_0,
	 _GYRO_ZOUT_L_0,
	 _ACCEL_XOUT_L_0,
	 _ACCEL_YOUT_L_0,
	 _ACCEL_ZOUT_L_0
  };
  
  enum class VFilterGyroTempConfig : uint8_t
  { // Gyroscope: 1)Bandwidth 2)Delay 3)Fs. Temperature Sensor: 1)Bandwidth 2)Delay.
    _G250_097_8_T4000_004,
    _G184_2d9_1_T188_1d9,
    _G92_3d9_1_T98_2d8,
    _G41_5d9_1_T42_4d8,
    _G20_9d9_1_T20_8d3,
    _G10_17d85_1_T10_13d4,
    _G5_33d48_1_T5_18d6,
    _G3600_0d17_8_T4000_0d04,
  };
  
  enum class VGyroFullScaleSelect : uint8_t
  {
    _250_DPS,
    _500_DPS,
    _1000_DPS,
    _2000_DPS
  };
  
  enum class VGyroFilterChoise : uint8_t
  {
    _x0 = 1, //значения FCHOICE - это инверсия Fchoice_b
	 _01 = 2,
	 _11 = 0
  };
  
  enum class VAccelFullScaleSelect : uint8_t
  {
    _2G,
    _4G,
    _8G,
    _16G,
  };
  
  enum class VAccelFilterChoise : uint8_t
  {
    _0 = 1, //значение accel_fchoice - инверсия accel_fchoice_b
	  _1 = 0
  };
  
  enum class VSetAccelFilter : uint8_t
  { // 3dB_BW(Hz)___Delay (ms) 
    _218_1Hz_1_88ms = 0, 
    _99Hz_2_88ms    = 2, 
    _44_8Hz_4_88ms  = 3, 
    _21_2Hz_8_87ms  = 4, 
    _10_2Hz_16_83ms = 5, 
    _5_05Hz_32_48ms = 6, 
    _420Hz_1_38ms   = 7,
  };
  
  enum class VSetAccelWakeupFreq : uint8_t
  {
    _0_24Hz,
    _0_49Hz,
    _0_98Hz,
    _1_95Hz,
    _3_91Hz,
    _7_81Hz,
    _15_63Hz,
    _31_25Hz,
    _62_50Hz,
    _125Hz,
    _250Hz,
    _500Hz,
  };
  
  enum class VConfigInternalClk : uint8_t
  { //I2C Master Clock Speed_____8MHz Closk Divider
    _348kHz_23,	 
    _333kHz_24,
    _320kHz_25,
    _308kHz_26,
    _296kHz_27,
    _286kHz_28,
    _276kHz_29,
    _267kHz_30,
    _258kHz_31,
    _500kHz_16,
    _471kHz_17,
    _444kHz_18,
    _421kHz_19,
    _400kHz_20,
    _381kHz_21,
    _364kHz_22	 
  };
  
  enum class VI2CTransferType : uint8_t
  {
	 _TRANSFER_IS_WRITE,
    _TRANSFER_IS_READ
  };
  
  enum class VStatusFlag : bool
  {
    _NO,
	 _YES
  };
  
  enum class VSetReset : uint8_t
  {
	 _RESET,
    _SET
  };
	
  enum class VSlvNumBytesRead : uint8_t
  {
    _0_BYTES_TO_READ,
    _1_BYTES_TO_READ,
    _2_BYTES_TO_READ,
    _3_BYTES_TO_READ,
    _4_BYTES_TO_READ,
    _5_BYTES_TO_READ,
    _6_BYTES_TO_READ,
    _7_BYTES_TO_READ,
    _8_BYTES_TO_READ,
    _9_BYTES_TO_READ,
    _10_BYTES_TO_READ,
    _11_BYTES_TO_READ,
    _12_BYTES_TO_READ,
    _13_BYTES_TO_READ,
    _14_BYTES_TO_READ,
    _15_BYTES_TO_READ,
  };
  
  enum class VNumSamplesCtrl : uint8_t
  {
    _ONE_PLUS_0_SAMPLES,
    _ONE_PLUS_1_SAMPLES,
    _ONE_PLUS_2_SAMPLES,
    _ONE_PLUS_3_SAMPLES,
    _ONE_PLUS_4_SAMPLES,
    _ONE_PLUS_5_SAMPLES,
    _ONE_PLUS_6_SAMPLES,
    _ONE_PLUS_7_SAMPLES,
    _ONE_PLUS_8_SAMPLES,
    _ONE_PLUS_9_SAMPLES,
    _ONE_PLUS_10_SAMPLES,
    _ONE_PLUS_11_SAMPLES,
    _ONE_PLUS_12_SAMPLES,
    _ONE_PLUS_13_SAMPLES,
    _ONE_PLUS_14_SAMPLES,
    _ONE_PLUS_15_SAMPLES,
    _ONE_PLUS_16_SAMPLES,
    _ONE_PLUS_17_SAMPLES,
    _ONE_PLUS_18_SAMPLES,
    _ONE_PLUS_19_SAMPLES,
    _ONE_PLUS_20_SAMPLES,
    _ONE_PLUS_21_SAMPLES,
    _ONE_PLUS_22_SAMPLES,
    _ONE_PLUS_23_SAMPLES,
    _ONE_PLUS_24_SAMPLES,
    _ONE_PLUS_25_SAMPLES,
    _ONE_PLUS_26_SAMPLES,
    _ONE_PLUS_27_SAMPLES,
    _ONE_PLUS_28_SAMPLES,
    _ONE_PLUS_29_SAMPLES,
    _ONE_PLUS_30_SAMPLES,
    _ONE_PLUS_31_SAMPLES,
  };
  
  enum class VClockSrcChoise : uint8_t
  {
    _INTERNAL_20MHz  = 0,
	 _PLL_OR_INTERNAL = 1,
	 _STOP_CLK        = 7
  };
  
  enum class VMagOperationModeSet : uint8_t
  {
    _POWER_DOWN             = 0,
	 _SINGLE_MEAS            = 1, 
	 _CONTINUOUS_MEAS_MODE_1 = 2,
	 _CONTINUOUS_MEAS_MODE_2 = 6,
	 _EXTERNAL_TRIG_MEAS     = 4,
	 _SELF_TEST              = 8,
	 _FUSE_ROM_ACCESS        = 15
  };
  
  enum class VMagOutputBitSet : uint8_t
  {
    _14_BIT_OUTPUT = 0,
    _16_BIT_OUTPUT = 1,
  };
  
  enum class VMagSoftResetCtrl : uint8_t
  {
    _NORMAL = 0,
    _RESET  = 1,
  };
  
  enum class VMagSelfTestCtrl : uint8_t
  {
    _NORMAL             = 0,
	 _GENERATE_MAG_FIELD = 1
  }; 
}

#endif
