#ifndef __RTOS_TASKS_MASTER_H
#define __RTOS_TASKS_MASTER_H

#include "main.h"
uint8_t MainBuf[MAIN_BUF_SIZE];

constexpr uint8_t F_BUF_SIZE = 5U;

struct TConnect
{
  TConnect() = default;
  TConnect(TDevConnect _Flags)
  :
  Flags(_Flags)
  {

  }
  ~TConnect(){}

  void set_flag(TDevConnectVal Val, TDevice BitNbr)
  {
    if ( check_flag_nbr(BitNbr) )
    {
      uint8_t MASK = (1U << BitNbr - 1U);
      switch (Val)
      {
        case _CONNECTED:
             Flags.Reg &= ~MASK;
             break;
        case _NOT_CONNECTED:
             Flags.Reg |= MASK;
             break;
        default:
             break;
      }
    }
    else
    {
      //№бита не из диапазона игнорируется
    }
  }

  TDevConnectVal get_flag(TDevice BitNbr)
  {    
    if ( check_flag_nbr(BitNbr) )
    {
      uint8_t MASK = (1U << BitNbr - 1U);
      
      return (TDevConnectVal)(Flags.Reg & MASK);
    }
    else                     //если номер флага не валидный
    {
      return _NOT_CONNECTED; 
    }
  }

  TDevConnect get_flags()
  {
    return Flags;
  }
  
  void reset_flags() //сброс всех флагов в _NOT_CONNECTED
  {
    if (TDevConnectVal::_NOT_CONNECTED == 1U)
    {
      Flags.Reg = 0xFF;
    }
    else
    {
      Flags.Reg = 0x00;
    }
  }
  
  bool check_flag_nbr(TDevice BitNbr)                //валидность номера флага
  {
    return ( BitNbr > _PLATF && BitNbr < _MAX_DEV );
  }
private:
  TDevConnect Flags; //Флаги связи платформы с устройствами \
                       должны обновляться по приему нового пакета \
                       с данными от каждого устройства
};

//----- Задачи FreeRTOS --------------------------------------------------------
//void vUSART(void *);         //передача данных системы по USART на платформе
void vUSART(void *);         //обслуживание интерфейса USART
void vMemsHandler(void *);    //получение и обработка данных MEMS датчика
void vCANTx(void *);         //передача сообщений по CAN
void vCANRx0(void *);        //получение сообщений из FIFO0 CAN
void vCANRx1(void *);        //получение сообщений из FIFO1 CAN на платформе
void vResultCnt(void *);
void vLineByteState(void *); //передача пакета состояния по Линии
void vLineData(void *);      //получение/передача пакетов по Линии
void vWriteFlash(void *);    //запись настроек во Flash микроконтроллера
//------------------------------------------------------------------------------

//----- Таймера FreeRTOS -------------------------------------------------------
void vCheckConnectTimerCallback(TimerHandle_t);
//------------------------------------------------------------------------------

#endif //__RTOS_TASKS_MASTER_H
