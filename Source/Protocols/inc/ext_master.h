#ifndef __EXT_MASTER_H
#define __EXT_MASTER_H

//----- Прикладной уровень протокола работы с внешним мастером -------

#include "usart_driver_.h"
class TExtMaster : public TUsart
{
public:
  __packed struct TData
  {
    uint8_t Cmd;
    float   ParamVal;
  };
  
  __packed struct TReqPkt
  {
    uint8_t PreAmble;
    TData   Data;
    uint8_t PostAmble;
  };
  
  __packed struct TStartPkt
  {
    uint8_t PreAmble;
    uint8_t Cmd;
  };
  
  __packed struct TFinishPkt
  {
    uint8_t PostAmble;
  };
  
  __packed struct TWrapPkt
  {
    TStartPkt  StartPkt;
    TFinishPkt FinishPkt;
  };
  
  enum TWrapTxSign : uint8_t
  {
    __NO_WRAP_TX     = 0U,
    __START_WRAP_TX  = 1U,
    __FINISH_WRAP_TX = 2U,
    __ALL_WRAP_TX    = 3U,
  };

public:
  TExtMaster(
             const TUsart_HW &Usart_HW 
            );
  ~TExtMaster();
  
  void parse_pkt();
  TData get_data();
  void tx_data( const uint8_t *Data, uint16_t Size, TWrapTxSign WrapTxSign );
  
protected:
private:
  constexpr static uint8_t PREAMBLE     = 0xA5;
  constexpr static uint8_t POSTAMBLE    = 0x5A;
  constexpr static uint8_t REQ_PKT_SIZE = sizeof(TReqPkt);
  constexpr static uint8_t TX_BUF_SIZE  = 0x10;
  
  uint8_t Rx[ REQ_PKT_SIZE ]; //буфер приема
  uint8_t Tx[ TX_BUF_SIZE ];  //буфер передачи
  TReqPkt *ReqPkt;
  
  const uint8_t *no_wrap_tx( const uint8_t *Data, uint16_t Size );     //только передача данных
  const uint8_t *start_wrap_tx( const uint8_t *Data, uint16_t Size );  //передача только начала обертки (без данных)
  const uint8_t *finish_wrap_tx( const uint8_t *Data, uint16_t Size ); //передача только конца обертки
  const uint8_t *all_pkt_tx( const uint8_t *Data, uint16_t Size );     //передача всего пакета
  
};

#endif //__EXT_MASTER_H
