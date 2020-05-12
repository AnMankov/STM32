// Информация о файле: $HeadURL: http://s005.sensor:18080/svn/Sens/trunk/SensProc/ErrProc.h $
// $Revision: 8 $  $Date: 2015-12-25 16:23:16 +0300 (РџС‚, 25 РґРµРє 2015) $  $Author: bryakin $

#ifndef ERRPROC_H
#define ERRPROC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void SetErr(uint32_t newerr); // Добавляет новую ошибку при ее возникновении
uint16_t ErrProc(void); // Вызывать постоянно. Возвращает текущий код ошибки 

#ifdef __cplusplus
}
#endif

#endif
