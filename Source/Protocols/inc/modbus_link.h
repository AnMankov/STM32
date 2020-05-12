#ifndef __MODBUS_LINK_H
#define __MODBUS_LINK_H

//----- Кнальный уровень протокола MODBUS -------
//----- Низкоуровневый обмен --------------------

#include "usart_driver_.h"

//необходим массив структур с кодами \
MODBUS Exception Codes \
и обработчиками

//в классе необходимо учитывать реентерабельность \
одновременно могут работать 2 экземпляра класса TModbusApp \
один и тот же код может выполняться в разных задачах


class TModbusLink : public TUsart
{
public:  
  typedef void (TModbusLink::*TFnct)( const uint8_t *, uint16_t );
  
  enum TState : uint8_t
  {
    __IDLE                       = 0U,
    __CHECKING_REQUEST           = 1U,
    __FORMATTING_NORMAL_REPLY    = 2U,
    __FORMATTING_ERROR_REPLY     = 3U,
    __PROCESSING_REQUIRED_ACTION = 4U,
  };
    
  enum TFrameState : uint8_t
  {
    _FRAME_OK  = 0U,
    _FRAME_NOK = 1U,
  };
  
  struct TIxSlice
  {
    uint8_t Tail; //индекс элемента, с которого будет осуществляться считывание (хвост кольцевого буфера)
    uint8_t Head; //индекс элемента, в который будет осуществляться запись (на 1 больше того, что уже реально записано) (голова кольцевого буфера)   
  };
  
  struct TRange
  {
    uint8_t Min;
    uint8_t Max;
  };
  
  enum TProcType : uint8_t
  {
    __MASTER = 0,
    __SLAVE  = 1,
		
		__MAX    = __SLAVE + 1,
  };
  
  enum TExceptCodes : uint8_t
  {
    __ILLEGAL_FNCT          = 1U,
    __ILLEGAL_DATA_ADDR     = 2U,
    __ILLEGAL_DATA_VAL      = 3U,
    __SERVER_DEVICE_FAILURE = 4U,
    __ACKNOWLEDGE           = 5U,
    __SERVER_DEVICE_BUSY    = 6U,
  };

public:
  TModbusLink(
              const TUsart_HW &Usart_HW,
              TProcType _ProcType,        
              SemaphoreHandle_t *_RtoTrigSem,
              SemaphoreHandle_t *_CommErrSem 
             );
  ~TModbusLink();
  
  void init_tmr();
	void init_dma();
  void set_addr( uint8_t _Addr );
  uint8_t get_addr();
	
//  unsigned short CRC16(                                     // The function returns the CRC as a unsigned short type
//                       const unsigned char *puchMsg, 
//                       unsigned short usDataLen 
//                      );
                      
  void start_transmit( uint8_t NbrBytes );
  
  constexpr static uint16_t RX_BUF_SIZE = 255U;             //размер буфера приема
  constexpr static uint16_t TX_BUF_SIZE = 20U;              //размер буфера передачи
  constexpr static uint16_t RX_BUF_MASK = RX_BUF_SIZE - 1U;
  
  uint8_t RxBuf[RX_BUF_SIZE];                               //буфер приема, на который настроен DMA
  uint8_t TxBuf[TX_BUF_SIZE];                               //буфер передачи, на который настроен DMA
  uint8_t HandleBuf[RX_BUF_SIZE];                           //буфер в котором осуществляется обработка
  
  uint8_t StrIx;
  
  SemaphoreHandle_t *RtoTrigSem;
  SemaphoreHandle_t *CommErrSem;
  
  static bool Tmr_1_5Ch_Trig;
  
  constexpr static uint8_t _1_5_CH_BITS_NBR = 17U;
  constexpr static uint8_t _3_5_CH_BITS_NBR = 39U;
  
  constexpr static TRange ValidAddr = { 1U, 247 }; //диапазон допустимых адресов Slave-устройства
  constexpr static uint8_t DEF_ADDR = 1U;          //адрес устройства в сети Modbus по умолчанию
  
  TState State;
  uint8_t CNDTR;
  
protected:
  TIxSlice IxSlice;
  TProcType ProcType;
  bool EOF;
  
  void stop_receive();
  void start_handle();
  bool handle_frame( const uint8_t *Buf, uint16_t BUF_SIZE );     //обработка PDU принятого кадра
  
  uint16_t get_rx_dma_data_qty();                                 //обработка кадра (CRC, Slave addr)
  
  void clr_eof();
  
private:

  TFrameState chk_frame( const uint8_t *Buf, uint16_t BUF_SIZE ); //проверка адреса и контрольной суммы принятого кадра
  uint16_t _2_ch_cnt();                                           //вычисление интервала тишины, пропорционального времени 2-х символов на \
                                                                    текущей скорости работы

  constexpr static TRange BAUD_RATE_RANGE  = { 0U, 9U };
  constexpr static TRange PARITY_AND_RANGE = { 0U, 3U };
  
	uint8_t Addr;
};

bool get_eof( uint8_t StrIx, TModbusLink::TProcType ProcType );
void set_eof( uint8_t StrIx, TModbusLink::TProcType ProcType, bool NewVal ); //для вызова из методов класса

#endif //__MODBUS_LINK_H
