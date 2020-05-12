#include "old_flash_lib.h"

FLASH_Status FLASH_GetStatus()
{
  FLASH_Status FLASHstatus = FLASH_COMPLETE;
  
  if((FLASH->SR & FLASH_FLAG_BSY) == FLASH_FLAG_BSY) 
  {
    FLASHstatus = FLASH_BUSY;
  }
  else 
  {  
    if((FLASH->SR & (uint32_t)FLASH_FLAG_WRPERR)!= (uint32_t)0x00)
    { 
      FLASHstatus = FLASH_ERROR_WRP;
    }
    else 
    {
      if((FLASH->SR & (uint32_t)(FLASH_SR_PGERR)) != (uint32_t)0x00)
      {
        FLASHstatus = FLASH_ERROR_PROGRAM; 
      }
      else
      {
        FLASHstatus = FLASH_COMPLETE;
      }
    }
  }
  /* Return the FLASH Status */
  return FLASHstatus;
}

void FLASH_Unlock()
{
  if((FLASH->CR & FLASH_CR_LOCK) != RESET)
  {
    /* Authorize the FLASH Registers access */
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
  }  
}

void FLASH_OB_Unlock()
{
  if((FLASH->CR & FLASH_CR_OPTLOCK) == SET)
  { 
    /* Unlocking the option bytes block access */
    FLASH->OPTKEYR = FLASH_OPTKEY1;
    FLASH->OPTKEYR = FLASH_OPTKEY2;
  }
}

void FLASH_Lock()
{
  FLASH->CR |= FLASH_CR_LOCK;//блокировка регистра CR
}
