#ifndef __AT45DB321D_DRIVER_LL
#define __AT45DB321D_DRIVER_LL

#include <cstdint>

#include "AT45DB321D_cmd.h"
#include "AT45DB321D_types.h"
#include "SPI_driver.h"

/*
*  Максимальное значение Vpor = 2.5В
*  После достижения Vcc значения Vpor, должно пройти Tpuw = 20мс до начала выполнения операций чтения / записи
**/

class AT45DB321D_DRIVER_LL : public AT45DB321D_CMD
{
public:
  static constexpr uint8_t __ADDR_BYTES_QTY = 3U;
  typedef uint8_t (&AddrByte)[__ADDR_BYTES_QTY];

public:
  AT45DB321D_DRIVER_LL();
  ~AT45DB321D_DRIVER_LL();
 
protected:
  /*  rd-функции: - из Src извлекается нужное поле и записывается в значение на которое указывает второй аргумент, 
  *               - возвращается Opcode
  *               - при втором аргументе == nullptr никаких действий функция не делает, кроме возврата Opcode
  *
  *   wr-функции: - в Dest записывается нужное поле из второго аргумента,
  *               - возвращается Opcode
  *               - при втором аргументе == nullptr никаких действий функция не делает, кроме возврата Opcode
  **/ 
  
  //

//-----  Status Register ---------------------------------------------------------------------------------------------------------
  uint8_t status_rd( uint8_t Src, TStatusVal * );
  uint8_t buf_cmp_rd( uint8_t Src, TBufCmpVal * );
  uint8_t protect_rd( uint8_t Src, TProtectVal * );
  uint8_t page_size_rd( uint8_t Src, TPageSizeVal * );
  
  uint8_t status_wr( uint8_t *Dest, TStatusVal );
  uint8_t buf_cmp_wr( uint8_t *Dest, TBufCmpVal );
  uint8_t protect_wr( uint8_t *Dest, TProtectVal );
  uint8_t page_size_wr( uint8_t *Dest, TPageSizeVal );
//--------------------------------------------------------------------------------------------------------------------------------


private:
  template<typename T_Data, typename T_Reg>                   //для ro регистров
  void read( const uint8_t Byte, T_Data *Data, T_Reg Reg )
  {
    if (Data != nullptr)
    {
      *Data = static_cast<T_Data>( ((T_Reg *)&Byte)->Dest );
    }
  }
  
  template<typename T_Mode, typename T_Reg>                  //для r/w регистров
  void write(uint8_t *const Byte, T_Mode Mode, T_Reg Reg)
  {
    if (Byte != nullptr)
    {
      ((T_Reg *)Byte)->Dest = static_cast<uint8_t>(Mode);
    }
  }
};

#endif //__AT45DB321D_DRIVER_LL
