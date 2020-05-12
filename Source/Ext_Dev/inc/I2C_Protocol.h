#ifndef _I2C_PROTOCOL_H
#define _I2C_PROTOCOL_H

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
  
#endif
