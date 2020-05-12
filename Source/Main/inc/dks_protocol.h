#ifndef __DKS_PROTOCOL_H
#define __DKS_PROTOCOL_H

#include "data_types.h"

namespace dks_tx_proc
{
  const uint8_t ID_BURST = 0xAA;                //Идентифкатор пакета 
  const uint8_t ID_ACK   = 0x55;                //Идентифкатор подтверждения пакета
//  const uint8_t LEN_DATA = sizeof(TPcSendData); //Длина пакета 
//  const uint8_t LEN_DATA = sizeof(TAccelData); //Длина пакета 
//  const uint16_t LEN_DATA = sizeof(TSys); //Длина пакета 
  const uint16_t LEN_DATA = 0U; //Длина пакета 
  
  enum TTxPhase
  {
    _PH_ID_BURST,
	  _PH_ID_ACK,
    _PH_LEN_DATA,
    _PH_DATA,
    _PH_CRC,
    _PH_IDLE,
    _PH_ALL,
  };
}

//namespace dks_rx_proc
//{
//  const uint8_t ID_BURST = 0xA5;               //Идентифкатор пакета
//  const uint8_t ID_ACK   = 0x5A;               //Идентифкатор подтверждения пакета
//  const uint8_t LEN_DATA = sizeof(TPcReceive); //Длина пакета 
//  
//  enum TRxPhase
//  {
//    _PH_ID_BURST,
//    _PH_ID_ACK,
//    _PH_LEN_DATA,
//    _PH_DATA,
//    _PH_CRC,
//    _PH_IDLE,
//    _PH_ALL,
//  };
//}
                                              
#endif //__DKS_PROTOCOL_H
