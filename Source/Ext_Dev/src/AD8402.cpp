#include "AD8402.h"

T_AD8402 AD8402{
                Pot_HW
               };

T_AD8402::TResist T_AD8402::Resist[ T_AD8402::TChannel::__CHANNEL_NBR ] =
{// Val,   Chn
  { 0.f, __ONE },
  { 0.f, __TWO },
};

T_AD8402::T_AD8402( const TPot_HW &Pot_HW )
:
HW( Pot_HW ),
Rwb( 0.f )
{

}

T_AD8402::~T_AD8402()
{
	
}

void T_AD8402::init()
{
  HW.CS.en_clk( HW.CS.ClkPortMask );
  HW.SHDN.en_clk( HW.SHDN.ClkPortMask );
  HW.RS.en_clk( HW.RS.ClkPortMask );
  HW.CLK.en_clk( HW.CLK.ClkPortMask );
  HW.SDI.en_clk( HW.SDI.ClkPortMask );

  LL_GPIO_InitTypeDef GPIO_InitStruct;
  
  do
  {
    GPIO_InitStruct.Pin        = HW.CS.Nbr;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
  } while ( SUCCESS != LL_GPIO_Init( HW.CS.Gpio, &GPIO_InitStruct ) );
  
  do
  {
    GPIO_InitStruct.Pin        = HW.SHDN.Nbr;
  } while ( SUCCESS != LL_GPIO_Init( HW.SHDN.Gpio, &GPIO_InitStruct ) );
  
  do
  {
    GPIO_InitStruct.Pin        = HW.RS.Nbr;
  } while ( SUCCESS != LL_GPIO_Init( HW.RS.Gpio, &GPIO_InitStruct ) );
  
  do
  {
    GPIO_InitStruct.Pin        = HW.CLK.Nbr;
  } while ( SUCCESS != LL_GPIO_Init( HW.CLK.Gpio, &GPIO_InitStruct ) );
  
  do
  {
    GPIO_InitStruct.Pin        = HW.SDI.Nbr;
  } while ( SUCCESS != LL_GPIO_Init( HW.SDI.Gpio, &GPIO_InitStruct ) );
  
  latch_set();
  shdn_clr(); //КЗ W и B => Rpot = Rw_ohm
  rs_set();
  clk_clr();
  sdi_clr(); 
}

void T_AD8402::set_code( uint8_t Code, TChannel Chn )
{
  for ( auto &item : Resist ) 
  {
    if ( Chn == item.Chn )
    {
      item.Val = ( (float)Code / ( 1 << RESOLUTION ) ) * Rab_ohm + Rw_ohm;
    }
  }
  
  shdn_set();
  latch_clr();
  
  uint8_t Ctr = DATA_WORD_SIZE;
  //----- проталкивание адреса (номера канала) - реверс уже сделан (TIMING DIAGRAMS, datasheet Rev.E, Page 10)
  uint8_t Data = static_cast<uint8_t>( Chn );
  while ( Ctr--> 8U )
  {
    data_bit_ctrl( &Data );
    clk_clr();
    clk_set();
  }
  
  //----- проталкивание данных
  Data = reverse_bits( Code );
  
//  ++Ctr;
  
  do
  {
    data_bit_ctrl( &Data );
    clk_clr();
    clk_set();
  } while ( Ctr--> 0U );
  
  sdi_clr(); 
  clk_clr();  
  latch_set();
}

void T_AD8402::data_bit_ctrl( uint8_t *Data )
{
  typedef void ( T_AD8402::*TFnct )();
  
  struct TBitCtrl
  {
    TFnct   Fnct;
    uint8_t Remainder;
  };
  
  TBitCtrl BitCtrl[] =
  {
    { &T_AD8402::sdi_set, 1U },
    { &T_AD8402::sdi_clr, 0U },
  };
  
  uint8_t Remainder = *Data % 2;
  
  for ( auto item : BitCtrl )
  {
    if ( Remainder == item.Remainder )
    {
      ( this->*item.Fnct )();
      
      break;
    }
    
  }
  
  *Data >>= 1U;
}

uint8_t T_AD8402::reverse_bits( uint8_t Nbr )
{
   Nbr = (Nbr & 0x55) << 1 | (Nbr & 0xAA) >> 1;
   Nbr = (Nbr & 0x33) << 2 | (Nbr & 0xCC) >> 2;
   Nbr = (Nbr & 0x0F) << 4 | (Nbr & 0xF0) >> 4;
   
   return Nbr;
}


void T_AD8402::latch_set()
{
  LL_GPIO_SetOutputPin( HW.CS.Gpio, HW.CS.Nbr );
}

void T_AD8402::shdn_set()
{
  LL_GPIO_SetOutputPin( HW.SHDN.Gpio, HW.SHDN.Nbr );
}

void T_AD8402::rs_set()
{
  LL_GPIO_SetOutputPin( HW.RS.Gpio, HW.RS.Nbr );
}

void T_AD8402::clk_set()
{
  LL_GPIO_SetOutputPin( HW.CLK.Gpio, HW.CLK.Nbr );
}

void T_AD8402::sdi_set()
{
  LL_GPIO_SetOutputPin( HW.SDI.Gpio, HW.SDI.Nbr );
}

void T_AD8402::latch_clr()
{
  LL_GPIO_ResetOutputPin( HW.CS.Gpio, HW.CS.Nbr );
}

void T_AD8402::shdn_clr()
{
  LL_GPIO_ResetOutputPin( HW.SHDN.Gpio, HW.SHDN.Nbr );
}

void T_AD8402::rs_clr()
{
  LL_GPIO_ResetOutputPin( HW.RS.Gpio, HW.RS.Nbr );
}

void T_AD8402::clk_clr()
{
  LL_GPIO_ResetOutputPin( HW.CLK.Gpio, HW.CLK.Nbr );
}

void T_AD8402::sdi_clr()
{
  LL_GPIO_ResetOutputPin( HW.SDI.Gpio, HW.SDI.Nbr );
}

float T_AD8402::get_resist( TChannel Chn )
{
  struct TChnIx
  {
    TChannel Chn;
    uint8_t  Ix;
  };
  
  TChnIx ChnIx[] =
  {
   { TChannel::__ONE, 0 },
   { TChannel::__TWO, 1 },
  };

  uint8_t Ix = 0U;

  for ( auto item : ChnIx )
  {
    if ( item.Chn == Chn )
    {
      Ix = item.Ix;
    }

    break;
  }

  return Resist[ Ix ].Val;
}
