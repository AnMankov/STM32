#ifndef __FLASH_DRIVER_H
#define __FLASH_DRIVER_H

#include "lib.h"

extern unsigned int Image$$SETS_ROM$$Base;

/* Для данных, хранимых во Flash отводим страницу 64 (Page 63)
*/
class TFlash
{
public:
  TFlash(); //при создании объекта происходит считывание настроек в ОЗУ \
              доступ к настройкам - get_settings()
  ~TFlash();
	
  void get_sets( uint8_t *Dest, uint16_t BytesQty );
  void write_sets( const uint8_t *Src, uint16_t BytesQty );
	void write_data( const uint8_t *Src );
  
  constexpr static uint8_t  MAX_PROG_TIME_MS = 10U;
  constexpr static uint32_t PAGE_SIZE        = 0x800;
  
	uint8_t DataPageCtr;
	constexpr static uint8_t  DATA_PAGE_QTY = 95U;
//	constexpr static uint8_t  DATA_PAGE_QTY = 2U;

protected:
private:                                                                                
  void erase_page( uint32_t ) const;                                                    //стереть страницу с настройками
  bool write_page( const uint8_t *Src, uint16_t BytesQty, uint32_t ) const;             //записать страницу с настройками
  void rdp_ob() const;                                                                  //установка бита защиты от чтения, если не установлен
  bool prog( const uint32_t *pData, uint32_t StartAddr, const uint32_t EndAddr ) const;       
  void clear_all_error() const;                                                         //процедура сброса всех ошибок
                                                                                                 
  constexpr static uint32_t FLASH_USER_START_ADDR = (uint32_t) &Image$$SETS_ROM$$Base;  //начальный адрес места хранения настроек
	
	constexpr static uint32_t DATA_START_ADDR = 0x0800E000;
	
  
  void init_tmr();
  void start_tmr() const;
  void stop_tmr() const;
  
  TIM_TypeDef *Tmr;
};

extern TFlash Flash;

#endif
