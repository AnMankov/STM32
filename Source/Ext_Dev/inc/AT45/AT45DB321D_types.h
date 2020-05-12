#ifndef __AT45DB321D_TYPES_LL
#define __AT45DB321D_TYPES_LL

//-----  Status Register ---------------------------------------------------------------------------------------------------------
enum TStatusVal : uint8_t
{
  __BUSY = 0U,
  __RDY  = 1U, //микросхема готова принять следующую команду
};

enum TBufCmpVal : uint8_t
{
  __MATCH     = 0U, //данные в странице main memory совпадают с данными в буфере
  __NOT_MATCH = 1U, //как минимум 1 бит данных в странице main memory отличается от данных в буфере
};

enum TProtectVal : uint8_t
{
  __DIS = 0U,
  __EN  = 1U,
};

enum TPageSizeVal : uint8_t
{
  __STANDARD = 0U, //размер страницы = 528 байт
  __BINARY   = 1U, //размер страницы = 512 байт
};

constexpr uint8_t DEV_DENSITY_VAL = 0x0D;

__packed struct TStatusReg
{
  TPageSizeVal PageSize   : 1;
  TProtectVal  Protect    : 1; //защита сектора
  uint8_t      DevDensity : 4; //д.б. == DEV_DENSITY_VAL и != density code из JEDEC device ID
  TBufCmpVal   BufCmp     : 1;
  TStatusVal   Status     : 1;
};

__packed struct TStatusField
{
  uint8_t Placebo : 7;
  uint8_t Dest    : 1;
};

__packed struct TBufCmpField
{
  uint8_t Placebo1 : 6;
  uint8_t Dest     : 1;
  uint8_t Placebo2 : 1;
};

__packed struct TProtectField
{
  uint8_t Placebo1 : 1;
  uint8_t Dest     : 1;
  uint8_t Placebo2 : 6;
};

__packed struct TPageSizeField
{
  uint8_t Dest    : 1;
  uint8_t Placebo : 7;
};
//--------------------------------------------------------------------------------------------------------------------------------

#endif //__AT45DB321D_TYPES_LL
