#ifndef __SPI_DRIVER_H
#define __SPI_DRIVER_H

#include "hard_config.h"
#include "lib.h"

class TSPI
{
public:
  enum TSpiState : uint8_t
  {
    __BSY   = 0U,   //SPI занят обменом
    __START = 1U,   //SPI начал обработку нового кадра
    __FREE  = 2U,   //SPI готов начать новый обмен
  };
  
  enum TRate : uint8_t
  {
    __MIN          = 0U,
    __MAX          = 1U,
    __PCLK_DIV_2   = 2U,
    __PCLK_DIV_4   = 3U,
    __PCLK_DIV_8   = 4U,
    __PCLK_DIV_16  = 5U,
    __PCLK_DIV_32  = 6U,
    __PCLK_DIV_64  = 7U,
    __PCLK_DIV_128 = 8U,
    __PCLK_DIV_256 = 9U,
  };

public:
  TSPI( 
       const TSpi_HW &_HW,
       TRate Rate
      );
  ~TSPI();
  
protected:
  void pin_clk_config(); //инициализация выводов SPI
  void hw_init();
  
  void write_frame( const uint8_t *Buf, uint8_t Size ); 
  void read_frame();
  
  TSpiState write_byte( uint8_t Byte );

  TSpi_HW HW;
  TRate Rate;
private:
  void dma_init();
  TSpiState check_busy();
};
  
#endif //__SPI_DRIVER_H
