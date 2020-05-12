#ifndef __AD8402_H
#define __AD8402_H

#pragma anon_unions

#include "hard_config.h"

class T_AD8402
{
public:
  enum TChannel : uint8_t
  {
	  __ONE         = 0U,
	  __TWO         = 2U, //сделан реверс для отправки старшим вперед
    
    __CHANNEL_NBR = 2U,
  };

  __packed union TDataWord
  {
	  __packed struct
	  {
	    uint8_t Data;
      uint8_t Addr : 2;
	  };
    
    uint16_t Word;	
  };

  struct TResist
  {
    float    Val; //Ohm
	  TChannel Chn;
  };
	
public:	
  T_AD8402( const TPot_HW & );
  ~T_AD8402();
  
  void init();                        //аппаратная инициализация
  void set_code( uint8_t, TChannel );
  float get_resist( TChannel );

protected:

private:
  constexpr static uint32_t Rab_ohm        = 100000U;
  constexpr static uint32_t Rw_ohm         =     50U;
  constexpr static uint8_t  RESOLUTION     =      8U;
  
  constexpr static uint8_t  DATA_WORD_SIZE =     10U;
//  constexpr static uint8_t  DATA_WORD_SIZE =     10U;
  
  TPot_HW HW;
  
  float Rwb;
  static TResist Resist[ TChannel::__CHANNEL_NBR ];
  
  void latch_set(); //cs
  void shdn_set();
  void rs_set();
  void clk_set();
  void sdi_set();

  void latch_clr(); //cs
  void shdn_clr();
  void rs_clr();
  void clk_clr();
  void sdi_clr();
  
  void data_bit_ctrl( uint8_t *Data );
  uint8_t reverse_bits( uint8_t Nbr );                //реверс битов в байте
};

extern T_AD8402 AD8402;

#endif //__AD8402_H

