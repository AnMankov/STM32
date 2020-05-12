#ifndef __LSM6DS3_VALS_H
#define __LSM6DS3_VALS_H

#include <cstdint>

namespace LSM6DS3
{
  using std::uint8_t;
  
  enum TI2CProtocol  //константы - I2C протокол датчика
  {
    ST,    //старт условие
    SADW,  //slave адрес + записать
    SADR,  //slave адрес + считать
    SAK,   //подтверждение от slave
    SUB,   //передача подадреса
    DATAS, //передача данных от slave
    DATAM, //передача данных от master
    SP,    //стоп условие
    SR,    //повстарт
    MAK,   //подтверждение от master
    NMAK,  //нет подтверждения от slave
    MAX_P
  };
  
  enum I2CModes
  {
    STANDARD,                    //до 100кГц
    FAST,                        //до 400кГц
  };
  
  enum class VLinearAccelerationSensitivity : std::uint16_t
  {
    _2_uG  = 61,
    _4_uG  = 122,
    _8_uG  = 244,
    _16_uG = 488
  };
  
  enum class VAngularRateSensitivity : std::uint32_t
  {
    _FS_125_udps_per_LSB  = 4375,
    _FS_245_udps_per_LSB  = 8750,
    _FS_500_udps_per_LSB  = 17500,
    _FS_1000_udps_per_LSB = 35000,
    _FS_2000_udps_per_LSB = 70000
  };
  
  enum class VSwitch : bool
  {
    _DISABLED = 0,
	 _ENABLED
  };
  
  enum class VAccAntiAliasingBW : uint8_t
  {
    _400_Hz = 0,
	 _200_Hz,
	 _100_Hz,
	 _50_Hz
  };
  
  enum class VAccFullScale : uint8_t
  {
    _2_uG = 0,
    _16_uG,
    _4_uG,
    _8_uG
  };
  
  enum class VAccDataRatePowerMode : uint8_t
  {
    _POWER_DOWN = 0,
	 _12_5_Hz,
	 _26_Hz,
	 _52_Hz,
	 _104_Hz,
	 _208_Hz,
	 _416_Hz,
	 _833_Hz,
	 _1660_Hz,
	 _3330_Hz,
	 _6660_Hz,
  };
  
  enum class VGyroFullScale : uint8_t
  {
    _245_dps = 0,
	 _500_dps,
	 _1000_dps,
	 _2000_dps
  };
  
  enum class VGyroDataRate : uint8_t
  {
    _POWER_DOWN = 0,
	 _12_5_Hz,
	 _26_Hz,
	 _52_Hz,
	 _104_Hz,
	 _208_Hz,
	 _416_Hz,
	 _833_Hz,
	 _1660_Hz
  };
//-----------------------------------------------------------------------------------------------

//----- Константы для регистра CTRL3_C (12h) ----------------------------------------------------  
  enum class VSoftReset : bool
  {
    _NORMAL = 0,
	 _RESET
  };
  
  enum class VEndian : bool
  {
    _LSB_LOWER = 0,
	 _MSB_LOWER
  };
  
  enum class VPushPullOpenDrain : bool
  {
    _PUSH_PULL = 0,
	 _OPEN_DRAIN
  };
  
  enum class VIntActivationLevel : bool
  {
    _ACTIVE_HIGH = 0,
	 _ACTIVE_LOW
  };
  
  enum class VBlockDataUpdate : bool
  {
    _CONTINUOUS_UPDATE = 0,
	 _NOT_UNTIL_READ
  };
  
  enum class VRebootMemoryContent : bool
  {
    _NORMAL = 0,
	 _REBOOT
  };
//-----------------------------------------------------------------------------------------------

//----- Константы для регистра CTRL4_C (13h) ----------------------------------------------------  
  enum class VDisableI2C : bool
  {
    _I2C_SPI_ENABLED = 0,
	 _I2C_DISABLED_SPI_ENABLED
  };
  
  enum class VDataReadyMask : bool
  {
    _DISABLED = 0,
	 _ENABLED
  };

  enum class VDividedIntSignals : bool
  {
    _DIVIDED_INT1_INT2 = 0,
	 _ONLY_INT1
  };

  enum class VGyroSleep : bool
  {
    _DISABLED = 0,
	 _ENABLED
  };

  enum class VAccBw : bool
  {
    _BY_ODR = 0,
	 _BY_BW_XL
  };
//-----------------------------------------------------------------------------------------------

//----- Константы для регистра CTRL5_C (14h) ----------------------------------------------------  
  enum class VAccSelfTest : uint8_t
  {
    _DISABLED = 0,
	 _POSITIVE_SIGN,
	 _NEGATIVE_SIGN,
	 _NOT_ALLOWED
  };
  
  enum class VGyroSelfTest : uint8_t
  {
    _DISABLED = 0,
	 _POSITIVE_SIGN,
	 _NOT_ALLOWED,
	 _NEGATIVE_SIGN
  };

  enum class VBurstModeRead : uint8_t
  {
    _NO_ROUNDING = 0,
	 _ACCEL_ONLY,
	 _GYRO_ONLY,
	 _GYRO_AND_ACCEL,
	 _ACCEL_AND_SENSORHUB1_TO_SENSORHUB6,
	 _GYRO_AND_ACCEL_AND_SENSORHUB1_TO_SENSORHUB6_AND_SENSORHUB7_TO_SENSORHUB2,
	 _GYRO_AND_ACCEL_AND_SENSORHUB1_TO_SENSORHUB6
  };
//-----------------------------------------------------------------------------------------------

//----- Константы для регистра CTRL6_C (15h) ----------------------------------------------------  
  enum class VAccHighPerform : bool
  {
    _HP_ENABLED = 0,
	 _HP_DISABLED
  };
  
  enum class VGyroLevelLatch : bool
  {
    _LATCH_DISABLED = 0,
	 _LATCH_ENABLED
  };

  enum class VGyroDataLevelTrig : bool
  {
    _TRIG_DISABLED = 0,
	 _TRIG_ENABLED
  };

  enum class VGyroDataEdgeTrig : bool
  {
    _TRIG_DISABLED = 0,
	 _TRIG_ENABLED
  };
//-----------------------------------------------------------------------------------------------

//----- Константы для регистра CTRL7_G (16h) ----------------------------------------------------
  enum class VSrcRegRoundingFunc : bool
  {
    _DISABLED = 0,
	 _ENABLED
  };
  
  enum class VGyroDigHpFilter : bool
  {
    _FILTER_RESET_OFF = 0,
	 _FILTER_RESET_ON
  };

  enum class VGyroHpFilterCutoff : uint8_t
  {
    _0_0081_Hz = 0,
	 _0_0324_Hz,
	 _2_07_Hz,
	 _16_32_Hz
  };

  enum class VGyroDigHighPassFilter : bool
  {
    _HPF_DISABLED = 0,
	 _HPF_ENABLED
  };

  enum class VGyroHighPerform : bool
  {
    _HP_ENABLED = 0,
	 _HP_DISABLED
  };
//-----------------------------------------------------------------------------------------------

//----- Константы для регистра CTRL8_XL (17h) ----------------------------------------------------
  enum class VLPFon6D : bool
  {
    _OFF = 0,
	 _ON
  };
  
  enum class VAccHPSlopeFilter : bool
  {
    _SLOPE_FILTER = 0,
	 _HIGH_PASS_FILTER
  };

  enum class VAccHPFConfigCutoffSet : uint8_t
  {
    _SLOPE_XL_4 = 0,
	 _SLOPE_XL_100,
	 _SLOPE_XL_9,
	 _SLOPE_XL_400
  };

  enum class VAccLPF2 : bool
  {
    _OFF = 0,
	 _ON
  };
//-----------------------------------------------------------------------------------------------

//----- Константы для регистра CTRL9_XL (18h) ---------------------------------------------------
  enum class VAccOutX : bool
  {
    _DISABLED = 0,
	 _ENABLED
  };
  
  enum class VAccOutY : bool
  {
    _DISABLED = 0,
	 _ENABLED
  };

  enum class VAccOutZ : bool
  {
    _DISABLED = 0,
	 _ENABLED
  };
//-----------------------------------------------------------------------------------------------

//----- Константы для регистра CTRL10_C (19h) ---------------------------------------------------
  enum class VGyroOutX : bool
  {
    _DISABLED = 0,
	 _ENABLED
  };
  
  enum class VGyroOutY : bool
  {
    _DISABLED = 0,
	 _ENABLED
  };

  enum class VGyroOutZ : bool
  {
    _DISABLED = 0,
	 _ENABLED
  };
//-----------------------------------------------------------------------------------------------

//----- Константы для регистра STATUS_REG (1Eh) -------------------------------------------------
  enum class VAccDataAvailable : bool
  {
    _NO_DATA = 0,
	 _NEW_DATA
  };
  
  enum class VGyroDataAvailable : bool
  {
    _NO_DATA = 0,
	 _NEW_DATA
  };

  enum class VTemperDataAvailable : bool
  {
    _NO_DATA = 0,
	 _NEW_DATA
  };
//-----------------------------------------------------------------------------------------------
}

#endif
