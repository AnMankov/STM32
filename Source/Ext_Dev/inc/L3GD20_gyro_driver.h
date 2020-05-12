#ifndef __L3GD20_GYRO_DRIVER_H
#define __L3GD20_GYRO_DRIVER_H

#include <vector>
#include <string>
#include "Magnetometer.h"

#include "data_types.h"

namespace L3GD20
{
  /* Драйвер гироскопа L3GD20
     Использованная документация: L3GD20H Datasheet, 05-Mar-2013, Rev 2*/ 
  enum TI2CProtocol
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

  class TGyroDriver
  {
  public:
    TGyroDriver();
    ~TGyroDriver();
  //  const std::string str("TL3GD20_Gyro");
  protected:
    std::vector<TI2CProtocol> ProtocolWrOne; //вектор с протоколом записи одного байта в L3GD20
    std::vector<TI2CProtocol> ProtocolWrMul; //вектор с протоколом записи нескольких байтов в L3GD20
    std::vector<TI2CProtocol> ProtocolRdOne; //вектор с протоколом считывания одного байта из L3GD20
    std::vector<TI2CProtocol> ProtocolRdMul; //вектор с протоколом считывания нескольких байтов из L3GD20
  private: 
  };
}

#endif //__L3GD20_GYRO_DRIVER_H
