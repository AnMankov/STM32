#ifndef __MODBUS_DATA_LINK_H
#define __MODBUS_DATA_LINK_H

#include "hard_config.h"
#include "usart_driver_.h"

namespace TNode
{
  enum T
  {
    MASTER = 0,
    SLAVE  = 1,
  };
};

class TModbusDataLink : public TUsart
{
public:
  enum class TMState : uint8_t //состояния мастера, независящие от используемого режима передачи
  {
    IDLE                     = 0,
    WAITING_TURNAROUND_DELAY = 1,
    WAITING_FOR_REPLAY       = 2,
    PROCESSING_REPLAY        = 3,
    PROCESSING_ERROR         = 4,
  };
  
  enum TSState : uint8_t //состояния слейва, независящие от используемого режима передачи
  {
    IDLE                    = 0,
    CHECKING_REQ            = 1,
    FORMATTING_NORMAL_REPLY = 2,
    PROCESSING_REQ_ACT      = 3,
    FORMATTING_ERROR_REPLY  = 4,
  };

public:
  TModbusDataLink(
                  TNode::T,
                  USART_TypeDef *
                 );
  ~TModbusDataLink();

protected:
  void StateHandler();
private:
  TNode::T Node;
};

#endif //__MODBUS_DATA_LINK_H
