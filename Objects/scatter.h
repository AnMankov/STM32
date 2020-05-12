#ifndef __SCATTER_H
#define __SCATTER_H

//----- макросы загрузчика ---------------------------
#define BOOT_ROM_START 0x08000000
#define BOOT_ROM_SIZE  0x2000

#define BOOT_RAM_START 0x20000000
#define BOOT_RAM_SIZE  0xA00
#define BOOT_RAM_END   0x20000A00
//----------------------------------------------------

//----- отладочные макросы ---------------------------
#define DEBUG_ROM_START 0x08000000
#define DEBUG_ROM_SIZE  0x0001EC00
//#define DEBUG_RAM_START 0x20000A00
#define DEBUG_RAM_START 0x20000000
//#define DEBUG_RAM_SIZE  0x5C00
//#define DEBUG_RAM_SIZE  0xB800
#define DEBUG_RAM_SIZE  0xC000
//----------------------------------------------------

//----- макросы для основного приложения -------------
#define APP_ROM_START 0x08002000
#define APP_ROM_SIZE  0x0001EC00
#define APP_RAM_START 0x20000A00
#define APP_RAM_SIZE  0xB600
//----------------------------------------------------

#ifdef DEBUG
  #define ROM_START DEBUG_ROM_START
  #define ROM_SIZE  DEBUG_ROM_SIZE 
  #define RAM_START DEBUG_RAM_START
  #define RAM_SIZE  DEBUG_RAM_SIZE 
#else
  #ifdef APP
    #define ROM_START APP_ROM_START
    #define ROM_SIZE  APP_ROM_SIZE 
    #define RAM_START APP_RAM_START
    #define RAM_SIZE  APP_RAM_SIZE
  #endif
#endif

#define SRAM2_START    0x10000000
#define SRAM2_END      0x10004000
#define SRAM2_SIZE     0x4000

#define STACK_SIZE     0x3000
#define STACK_START    ( SRAM2_START + STACK_SIZE )
#define HEAP_SIZE      ( SRAM2_END - STACK_START )

//----- секция с настройками -------------------------
#define SETS_ROM_START 0x0803F800
#define SETS_ROM_SIZE  0x400

#define BOOT_SIGN      (uint16_t)0xA55A
//----------------------------------------------------


#endif //__SCATTER_H
