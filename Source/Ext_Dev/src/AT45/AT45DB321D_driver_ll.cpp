#include "AT45DB321D_driver_ll.h"


AT45DB321D_DRIVER_LL::AT45DB321D_DRIVER_LL()
:
AT45DB321D_CMD()
{

}

AT45DB321D_DRIVER_LL::~AT45DB321D_DRIVER_LL()
{

}

//-----  Status Register ---------------------------------------------------------------------------------------------------------
uint8_t AT45DB321D_DRIVER_LL::status_rd( uint8_t Src, TStatusVal *Dest )
{
  TStatusField Dummy;
  read( Src, Dest, Dummy );
  
  return static_cast<uint8_t>( AT45DB321D_CMD::Additional::__STATUS_RD );
}

uint8_t AT45DB321D_DRIVER_LL::buf_cmp_rd( uint8_t Src, TBufCmpVal *Dest )
{
  TBufCmpField Dummy;
  read( Src, Dest, Dummy );
  
  return static_cast<uint8_t>( AT45DB321D_CMD::Additional::__STATUS_RD );
}

uint8_t AT45DB321D_DRIVER_LL::protect_rd( uint8_t Src, TProtectVal *Dest )
{
  TProtectField Dummy;
  read( Src, Dest, Dummy );
  
  return static_cast<uint8_t>( AT45DB321D_CMD::Additional::__STATUS_RD );
}

uint8_t AT45DB321D_DRIVER_LL::page_size_rd( uint8_t Src, TPageSizeVal *Dest )
{
  TPageSizeField Dummy;
  read( Src, Dest, Dummy );
  
  return static_cast<uint8_t>( AT45DB321D_CMD::Additional::__STATUS_RD );
}
//------------------------------------------------------------------------
uint8_t AT45DB321D_DRIVER_LL::status_wr( uint8_t *Dest, TStatusVal Src )
{
  TStatusField Dummy;
  write(Dest, Src, Dummy);
  
  return static_cast<uint8_t>( AT45DB321D_CMD::Additional::__STATUS_RD );
}

uint8_t AT45DB321D_DRIVER_LL::buf_cmp_wr( uint8_t *Dest, TBufCmpVal Src )
{
  TBufCmpField Dummy;
  write(Dest, Src, Dummy);
  
  return static_cast<uint8_t>( AT45DB321D_CMD::Additional::__STATUS_RD );
}

uint8_t AT45DB321D_DRIVER_LL::protect_wr( uint8_t *Dest, TProtectVal Src )
{
  TProtectField Dummy;
  write(Dest, Src, Dummy);
  
  return static_cast<uint8_t>( AT45DB321D_CMD::Additional::__STATUS_RD );
}

uint8_t AT45DB321D_DRIVER_LL::page_size_wr( uint8_t *Dest, TPageSizeVal Src )
{
  TPageSizeField Dummy;
  write(Dest, Src, Dummy);
  
  return static_cast<uint8_t>( AT45DB321D_CMD::Additional::__STATUS_RD );
}
//--------------------------------------------------------------------------------------------------------------------------------
