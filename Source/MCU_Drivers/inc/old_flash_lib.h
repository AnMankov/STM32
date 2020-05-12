#ifndef __OLD_FLASH_LIB_H
#define __OLD_FLASH_LIB_H

#include <stdint.h>
#include "stm32l4xx.h"
#include "stm32l431xx.h"

constexpr uint32_t FLASH_FLAG_BSY    = FLASH_SR_BSY;                      /*!< FLASH Busy flag */
constexpr uint32_t FLASH_FLAG_WRPERR = FLASH_SR_WRPERR;                  /*!< FLASH Write protected error flag */
constexpr uint32_t FLASH_SR_PGERR    = FLASH_SR_PGAERR;                   /*!< Programming Error */
constexpr uint32_t FLASH_KEY1_Pos    = 0U;                              
constexpr uint32_t FLASH_KEY1_Msk    = 0x45670123U << FLASH_KEY1_Pos;   /*!< 0x45670123 */
constexpr uint32_t FLASH_KEY1        = FLASH_KEY1_Msk;                    /*!< FPEC Key1 */
constexpr uint32_t FLASH_KEY2_Pos    = 0U;                              
constexpr uint32_t FLASH_KEY2_Msk    = 0xCDEF89ABU << FLASH_KEY2_Pos;   /*!< 0xCDEF89AB */
constexpr uint32_t FLASH_KEY2        = FLASH_KEY2_Msk;                    /*!< FPEC Key2 */
constexpr uint32_t FLASH_OPTKEY1_Pos = 0U;
constexpr uint32_t FLASH_OPTKEY1_Msk = 0x08192A3BU << FLASH_KEY1_Pos;
constexpr uint32_t FLASH_OPTKEY1     = FLASH_KEY1_Msk;
constexpr uint32_t FLASH_OPTKEY2_Pos = 0U;
constexpr uint32_t FLASH_OPTKEY2_Msk = 0x4C5D6E7FU << FLASH_KEY2_Pos;
constexpr uint32_t FLASH_OPTKEY2     = FLASH_KEY2_Msk;

enum FLASH_Status
{
  FLASH_BUSY = 1,
  FLASH_ERROR_WRP,
  FLASH_ERROR_PROGRAM,
  FLASH_COMPLETE,
  FLASH_TIMEOUT
};

enum class TRdpVal: uint8_t
{
  OB_RDP_Level_0 = 0xAA,
  OB_RDP_Level_1 = 0xBB,
};

__packed struct TOptr
{
  TRdpVal RDP : 8;
  uint8_t Reserved0; //реализовать при необходимости
  uint8_t Reserved1; //реализовать при необходимости
  uint8_t Reserved2; //реализовать при необходимости
};

FLASH_Status FLASH_GetStatus();
void FLASH_Unlock();
void FLASH_OB_Unlock();
void FLASH_Lock();

#endif //__OLD_FLASH_LIB_H
