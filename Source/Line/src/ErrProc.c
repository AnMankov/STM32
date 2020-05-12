// Информация о файле: $HeadURL: http://s005.sensor:18080/svn/Sens/trunk/SensProc/ErrProc.c $
// $Revision: 8 $  $Date: 2015-12-25 16:23:16 +0300 (РџС‚, 25 РґРµРє 2015) $  $Author: bryakin $

#include "ErrProc.h"
#include "sensline.h"
#include "sys_error_codes.h"


// Определяет не сбрасываемые ошибки
const uint32_t NoResetError=0xF000;

uint32_t errtimes[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

void SetErr(uint32_t newerr) {
  uint32_t ebit=1;
  uint8_t n;
  for (n=0;n<16;n++) {
    if (newerr&ebit) {
      errtimes[n]=zgetssec();
    }
    ebit<<=1;
  }
}

// 
uint16_t ErrProc(void) {
  uint32_t ebit=1;
  uint32_t rez=0;
  uint8_t n;
  for (n=0;n<16;n++) {
    if (errtimes[n]) {
      rez|=ebit;
      if ((tdlt(errtimes[n])>ERRORTIME)&&(!(NoResetError&ebit))) {
        errtimes[n]=0;
      }
    }
    ebit<<=1;
  }
  return rez;
}
