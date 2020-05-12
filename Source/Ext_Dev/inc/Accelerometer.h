#ifndef __ACCELEROMETER_H
#define __ACCELEROMETER_H

#include "I2C_driver.h"
#include "LSM6DS3_acc_gyro_driver.h"

//struct TAccelAuxHw
//{
//  GPIO_TypeDef *GPIOx;     //порт
//  uint32_t PinClkMask;     //маска для разрешения тактирования
//  TI2CClkSrc   ClkSrc;     //маска для выбора источника тактирования
//  uint32_t PinMask;        //маска используемых выводов
//  char *Sign;              //подпись
//};

enum class TIntNum : unsigned char
{
  _ONE,
  _TWO
};

class TAccelerometer final : public I2C::TI2C, public LSM6DS3::TAccGyroDriver
{
public:
  TAccelerometer();
  ~TAccelerometer();
  
  void init();         //инициализация всей аппаратной части: интерфейс в микроконтроллере; микросхема
  void get_X_data();   //получить данные по оси X
  void get_Y_data();   //получить данные по оси Y  
  void get_Z_data();   //получить данные по оси Z
  void get_all_data(); //получить данные с 3-х осей
  
  const TNum HwIfaceNum = {TNum::ONE}; //номер аппаратного интерфейса, закрепленного за акселерометром
protected:
private:
  void form_protocol(std::vector<I2C::TI2CProtocol> &Dest, const LSM6DS3::TI2CProtocol *Src, const std::uint8_t); //формирует протокол для I2C драйвера
  void init_auxiliary(); // инициализация вспомогательных выводов микроконтроллера (для INT1 и INT2)
  
//  TAccelAuxHw AccelAuxGpio;
};

#endif
