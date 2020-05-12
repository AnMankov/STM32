#include "modbus_data_link.h"

TModbusDataLink::TModbusDataLink(
                                 TNode::T _Node,
                                 USART_TypeDef *Usart
                                )
:
TUsart(Usart),
Node(_Node)
{

}

TModbusDataLink::~TModbusDataLink()
{

}

void TModbusDataLink::StateHandler()
{
  switch (Node)
  {
    case TNode::MASTER:
         break;
    case TNode::SLAVE:
         break;
    default:
         break;
  }
  
}
