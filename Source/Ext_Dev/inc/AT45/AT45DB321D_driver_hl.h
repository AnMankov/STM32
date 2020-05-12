#ifndef __AT45DB321D_DRIVER_HL
#define __AT45DB321D_DRIVER_HL

#include "AT45DB321D_driver_ll.h"

class AT45DB321D_DRIVER_HL final : public AT45DB321D_DRIVER_LL, public TSPI
{
public:
  AT45DB321D_DRIVER_HL( 
                       const TSpi_HW &_HW,
                       TRate _Rate
                      );
  ~AT45DB321D_DRIVER_HL();
  
  void init_driver();
  bool check_chip();
  
  void read( uint8_t *Buf, uint8_t Size );        //чтение из флэш
  void write( const uint8_t *Buf, uint8_t Size ); //запись во флэш

private:
  static constexpr uint16_t __BUF_SIZE = 528U;
  __packed struct TBuf
  {
    uint8_t  Buf[ __BUF_SIZE ];
    uint16_t Ix;
  };

private:
  TBuf Rx;
  TBuf Tx; 
};

#endif //__AT45DB321D_DRIVER_HL
