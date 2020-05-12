#ifndef _LSM303DLHC_VAL_DESCR
#define _LSM303DLHC_VAL_DESCR

#include <cstdint>

namespace LSM303DLHC
{
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
//--------------------------------------------------------------------------  
  enum class CAccelPwrRate : std::uint8_t
  {
    _PDM = 0,
	 _1_HZ,      
	 _10_HZ,     
	 _25_HZ,     
	 _50_HZ,     
	 _100_HZ,    
	 _200_HZ,    
	 _400_HZ,    
	 _1620_HZ_LP,
	 _1344_5376_HZ,	 // 1344 - для режима Normal, 5376 - для режима Low-power
  };

  enum class CAccelAxesEn : std::uint8_t
  {
    _all_DIS = 0,
	 _x_EN_y_z_DIS,
	 _y_EN_x_z_DIS,
	 _x_y_EN_z_DIS,
	 _z_EN_x_y_DIS,
	 _x_z_EN_y_DIS,
	 _y_z_EN_x_DIS,
	 _all_EN,
  };

  enum class CAccelMode : std::uint8_t
  {
    _NORMAL = 0,
	 _LOW_POWER
  };
//--------------------------------------------------------------------------  
  enum class CAccelDataUpdate : std::uint8_t
  {
    _CONTINUOUS = 0,
	 _AFTER_READ
  };
  
  enum class CAccelEndianess : std::uint8_t
  {
    _LSB_LOW_ADDR = 0,
	 _MSB_LOW_ADDR
  };
    
  enum class CAccelScale : std::uint8_t
  {
    _2G = 0,
	 _4G,
	 _8G,
	 _16G
  };
  
  enum class CAccelHighResolution : std::uint8_t
  {
    _ENABLE = 0,
	 _DISABLE
  };
//--------------------------------------------------------------------------
    
}

#endif
