#ifndef __DMA_TYPES_H
#define __DMA_TYPES_H

#include "lib.h"

struct TChannelSets
{
  uint32_t PeriphAddr;
  uint32_t MemAddr;
  uint32_t Direction;
  uint32_t Mode;
  uint32_t NbData;
};

struct TChannel
{
  uint32_t  Nbr;
  IRQn_Type IRQ;
};

typedef void ( *TFnct )( uint32_t );
typedef void ( *TClrFlagFnct )( DMA_TypeDef * );

struct TPeriph
{
  DMA_TypeDef *Nbr;
  TFnct en_clk;
  uint32_t ClkMask;
  TClrFlagFnct clr_flag_tc_fnct;
};

struct TEof
{
  DMA_TypeDef *DMA;
  uint32_t     TxChannel;
  bool         Flag;
};

struct TDmaDuplex
{
  TPeriph  Periph;
	TChannel RxChannel;
	TChannel TxChannel;
};

struct TDmaAdc
{
  TPeriph  Periph;
  TChannel Ch;
};

#endif //__DMA_TYPES_H
