#ifndef __AT45DB321D_CMD
#define __AT45DB321D_CMD

#include <stdint.h>

class AT45DB321D_CMD
{
public:
  AT45DB321D_CMD();
  ~AT45DB321D_CMD();

protected:
  enum Read : uint8_t
  {
    __MAIN_MEM_PAGE      = 0xD2,
    __CONT_ARR_LEGACY    = 0xE8,
    __CONT_ARR_LOW_FREQ  = 0x03,
    __CONT_ARR_HIGH_FREQ = 0x0B,
    __BUF_1_LOW_FREQ     = 0xD1,
    __BUF_2_LOW_FREQ     = 0xD3,
    __BUF_1              = 0xD4,
    __BUF_2              = 0xD6,    
  };
  
  struct TAddrBytes
  {
    uint8_t _0;
    uint8_t _1;
    uint8_t _2;
  };
    
//  __package union __03_H
//  {    
//    TAddrBytes AddrBytes;
//    
//    __package struct StandardPageSize
//    {
//      uint8_t BA0_BA7;
//      uint8_t BA8_BA9  : 2;
//      uint8_t PA0_PA5  : 6;
//      uint8_t PA6_PA12 : 7;
//      uint8_t Res      : 1;
//    };
//    
//    __package struct BinaryPageSize
//    {
//      
//    };
//  };
  
  enum Program_Erase : uint8_t
  {
    __BUF_1_WR               = 0x84,
    __BUF_2_WR               = 0x87,
    __BUF_1_MM_BUIT_IN_ERASE = 0x83,
    __BUF_2_MM_BUIT_IN_ERASE = 0x86,
    __BUF_1_MM               = 0x88,
    __BUF_2_MM               = 0x89,
    __PAGE_ERASE             = 0x81,
    __BLOCK_ERASE            = 0x50,
    __SECTOR_ERASE           = 0x7C,
    __CHIP_ERASE_1           = 0xC7,
    __CHIP_ERASE_2           = 0x94,
    __CHIP_ERASE_3           = 0x80,
    __CHIP_ERASE_4           = 0x9A,
    __MM_PAGE_PROGRAM_BUF_1  = 0x82,
    __MM_PAGE_PROGRAM_BUF_2  = 0x85,
  };
  
  enum Protect_Security : uint8_t
  {
    __EN_SECTOR_PROTECT_1    = 0x3D,
    __EN_SECTOR_PROTECT_2    = 0x2A,
    __EN_SECTOR_PROTECT_3    = 0x7F,
    __EN_SECTOR_PROTECT_4    = 0xA9,
                             
    __DIS_SECTOR_PROTECT_1   = 0x3D,
    __DIS_SECTOR_PROTECT_2   = 0x2A,
    __DIS_SECTOR_PROTECT_3   = 0x7F,
    __DIS_SECTOR_PROTECT_4   = 0x9A, 
    
    __ERASE_SECTOR_PROTECT_1 = 0x3D,
    __ERASE_SECTOR_PROTECT_2 = 0x2A,
    __ERASE_SECTOR_PROTECT_3 = 0x7F,
    __ERASE_SECTOR_PROTECT_4 = 0xCF, 
    
    __PROG_SECTOR_PROTECT_1  = 0x3D,
    __PROG_SECTOR_PROTECT_2  = 0x2A,
    __PROG_SECTOR_PROTECT_3  = 0x7F,
    __PROG_SECTOR_PROTECT_4  = 0xFC,
    
    __RD_SECTOR_PROTECT      = 0x32,
    
    __SECTOR_LOCKDOWN_1      = 0x3D,
    __SECTOR_LOCKDOWN_2      = 0x2A,
    __SECTOR_LOCKDOWN_3      = 0x7F,
    __SECTOR_LOCKDOWN_4      = 0x30,
    
    __RD_SECTOR_LOCKDOWN     = 0x35,
    
    __PROG_SECURITY          = 0x9B,
    __PROG_SECURITY_1        = 0x00,
    __PROG_SECURITY_2        = 0x00,
    __PROG_SECURITY_3        = 0x00,
    
    __RD_SECURITY            = 0x77,
  };
  
  enum Additional : uint8_t
  {
    __MM_PAGE_BUF_1_TRANSFER  = 0x53,
    __MM_PAGE_BUF_2_TRANSFER  = 0x55,
    __MM_PAGE_BUF_1_COMPARE   = 0x60,
    __MM_PAGE_BUF_2_COMPARE   = 0x61,
    __AUTO_PAGE_REWRITE_BUF_1 = 0x58,
    __AUTO_PAGE_REWRITE_BUF_2 = 0x59,
    __DEEP_POWER_DOWN         = 0xB9,
    __RESUME_DEEP_POWER_DOWN  = 0xAB,
    __STATUS_RD               = 0xD7,
    __MANUFACT_DEV_ID_RD      = 0x9F,
  };
  
  enum Legacy : uint8_t
  {
    __BUF_1_RD      = 0x54,
    __BUF_2_RD      = 0x56,
    __MM_PAGE_RD    = 0x52,
    __CONT_ARR_RD   = 0x68,
    __STATUS_REG_RD = 0x57,
  };
};

#endif //__AT45DB321D_CMD
