#ifndef __MPU_9250_ACC_GYRO_MAG_DRIVER_HL_H
#define __MPU_9250_ACC_GYRO_MAG_DRIVER_HL_H

#include <cstdint>
//#include <vector>

//#include "data_types.h"
#include "I2C_driver.h"
#include "MPU-9250_acc_gyro_mag_driver_ll.h"
#include "I2C_Protocol.h"
#include "model.h"

#pragma anon_unions

namespace MPU_9250
{
  constexpr uint8_t MAX_AXES_NUM       = 3U;
  constexpr uint8_t NUM_BYTES_PER_AXIS = 2U;
  constexpr uint8_t MAX_NUM_SENSORS    = 2U;
  constexpr uint8_t MAX_DATA_BYTES     = MAX_AXES_NUM * NUM_BYTES_PER_AXIS * MAX_NUM_SENSORS + 2; // + 2 байта термодатчика
  constexpr uint8_t ACC_DATA_BYTES     = MAX_AXES_NUM * NUM_BYTES_PER_AXIS;
  
  constexpr uint8_t MAG_MODE1_RATE_HZ = 8U;
  constexpr uint8_t MAG_MODE2_RATE_HZ = 100U;
  constexpr uint32_t MAG_MODE1_RATE_us = 1000000.0 / MAG_MODE1_RATE_HZ;
  constexpr uint32_t MAG_MODE2_RATE_us = 1000000.0 / MAG_MODE2_RATE_HZ;
  
  constexpr float GYRO_32_kHz_SAMPLE_RATE_us = 1000.0 / 32;
  constexpr float GYRO_8_kHz_SAMPLE_RATE_us  = 1000.0 / 8;
  constexpr float GYRO_1_kHz_SAMPLE_RATE_us  = 1000.0 / 1;
  
  constexpr float ACCEL_4_kHz_SAMPLE_RATE_us = 1000.0 / 4;
  constexpr float ACCEL_1_kHz_SAMPLE_RATE_us = 1000.0 / 1;
    
  enum TAxisFlag : bool
  {
    _DISABLE = 0,
    _ENABLE
  };
  
  enum TSensor
  {
    _GYRO = 0,
	  _ACCEL,
	  _MAGNETO,
	  _TEMPERATURE
  };
  
  struct TIntHw
  {
    GPIO_TypeDef *GPIOx;     //порт
    uint32_t PinClkMask;     //маска для разрешения тактирования порта
    uint32_t PeriphClkMask;  //маска для разрешения тактирования периферии
    uint32_t PinMask;        //маска номера используемых выводов
	  uint32_t PinMode;        //маска режима работы используемого вывода
	  uint32_t PullType;       //маска типа подтяжки используемого вывода
    uint32_t SysCfgPort;     //SYSCFG маска для выбора порта конфигурируемой EXTI линии	
	  uint32_t SysCfgLine;     //SYSCFG маска для выбора конфигурируемой EXTI линии
	  uint32_t ExtiLine;       //маска подключаемой EXTI линии
	  uint32_t ExtiMode;       //маска режима для EXTI линии
	  uint32_t ExtiTrigger;    //маска активного состояния сигнала при срабатывании (low/high) для EXTI линии
	  IRQn_Type IntNum;
//    TI2CClkSrc ClkSrc;     //маска для выбора источника тактирования
//    char *Sign;            //подпись
  };
  
  struct TAccTempGyroData
  {
    uint16_t AccelX;
    uint16_t AccelY;
    uint16_t AccelZ;
	  int16_t Temp;
	  uint16_t GyroX;
	  uint16_t GyroY;
	  uint16_t GyroZ;
  };
  
  struct TRawAxesData
  {
    int16_t X;
    int16_t Y;
    int16_t Z;
  };
  
  struct TRawMagData
  {
    TRawAxesData MagData;
	  uint8_t ST2;
  };
    
  union TSensorData
  {
    struct
	  {
	    uint8_t HighByte;
	    uint8_t LowByte;
	  };
	  uint16_t Data;
  };
      
  enum class TMagCheck : uint8_t
  {
    _MAG_ERROR = 0,
	  _MAG_OK
  };

  struct TGyroData
  {
    float X;                    //пересчитанное значение угловой скорости по оси X
    float Y;                    //пересчитанное значение угловой скорости по оси Y
    float Z;                    //пересчитанное значение угловой скорости по оси Z
  };
  
  struct TMagData
  {
    float X;                   //пересчитанное значение плотости магнитного потока по оси X			
    float Y;                   //пересчитанное значение плотости магнитного потока по оси Y			
    float Z;                   //пересчитанное значение плотости магнитного потока по оси Z
  };

  class TAccGyroMagDriver_HL final : public I2C::TI2C, public TAccGyroMagDriver_LL
  {
  public:
	  TAccGyroMagDriver_HL( const TI2C_HW &_I2C_HW ); //инициализация интерфейса микроконтроллера;
    ~TAccGyroMagDriver_HL();
    
	  const TModel::TAccelData *get_acc_data() const;
	  const TGyroData *get_gyro_data() const;
	  const TMagData *get_magneto_data() const;
	  float get_temperature() const;
	  
	  uint32_t get_gyro_sample_rate() const;
	  uint32_t get_accel_sample_rate() const;
	  uint32_t get_mag_sample_rate() const;
	  
	  TMagCheck get_mag_state();	 
//	 void gyro_offset_take_account(TAxesData &);
	  void init_driver();
    void init_aux();  //инициализация вспомогательных выводов микроконтроллера (INT)
	  void init_chip();  //инициализация микросхемы
    
	  bool check_id_acc_gyro();
	 
	 
//    const TNum HwIfaceNum = {TNum::ONE}; //номер аппаратного интерфейса, к которому подключена микросхема
	  void collect_all_data(); //сбор данных с датчиков \
	                             запускать по срабатыванию входа внешнего прерывания
    
    void collect_acc_data(); //сбор данных только с акселерометра \
	                             запускать по срабатыванию входа внешнего прерывания                              
                            
	  void collect_mag_data(); //сбор данных с магнитометра \
	                             выполняется после проверки DRDY магнитометра
		 							 
	  uint8_t SampleCount; //счетчик выборок измеряемых данных
	  bool    MagWorkFlag; //Флаг работы магнитометра
	  uint8_t AKM_ID;      //идентификатор магнитометра AK8963 (AsahiKASEI)
  protected:
  private:
	 
	  bool check_id_mag();
	  
	  void config_gyro();
	  void config_acc();
	  void config_mag();
	  	 
    uint16_t rev_half_word(TSensorData Src){return (Src.HighByte << 8) + Src.LowByte;} //обмен байтами в полуслове
	  uint32_t pow(uint8_t Num, uint8_t Exp);
	  
     
	  void convert_all_data(uint8_t *SensorData);
    void convert_mag_data(uint8_t *SensorData);
    void convert_acc_data(uint8_t *SensorData);
	  
	  void count_temperature(const int16_t Data);
	  void count_accel(const TRawAxesData *);
	  void count_gyro(const TRawAxesData *);
	  void count_magneto(const TRawAxesData *);
	  
	  float    GyroSensitivity;  //чувствительность гироскопа (зависит от выбранной рабочей шкалы микросхемы)
	  uint16_t AccelSensitivity; //чувствительность акселерометра (зависит от выбранной рабочей шкалы микросхемы)
	  float    MagSensitivity;   //чувствительность магнитометра (зависит от выбранной рабочей шкалы микросхемы)
	  
	  uint32_t GyroSampleRate_us;  //время появления выборки гироскопа
	  uint32_t AccelSampleRate_us; //время появления выборки акселерометра
	  uint32_t MagSampleRate_us;   //время появления выборки магнитометра
	  
	  TMagCheck MagState; //проверка инициализации магнитометра
	  
	  //----- Данные с последними выборками -------------------------------------------
	  float Temperature;
	  
	  TModel::TAccelData Accel;
	  TGyroData Gyro;
	  TMagData Magneto;
	  
	  TMagData MagnetoOffset; //Hard iron смещение магнитометра в [uT]/50
	  //-------------------------------------------------------------------------------
  };
  
  extern const TIntHw IntHw;
}

#endif
