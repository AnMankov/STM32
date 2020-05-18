#ifndef __USART_DRIVER_H
#define __USART_DRIVER_H

#include "RS-485_phy.h"
#include "hard_config.h"
#include "rtos_headers.h"
#include "lib.h"


class TUsart : public T_RS_485_phy
{
//типы класса
public:
enum class TMode : uint8_t
{
  _STANDARD,
  _LIN,
  _SMARTCARD,
  _SINGLE_WIRE_HALF_DUPLEX,
  _SYNCHRONOUS,
  _IRDA,
  _RS485_DE,
  _DUAL_CLOCK_DOMAIN
};

enum class TBaudRate : uint32_t
{
  _1200    = 1200U,   
  _2400    = 2400U,   
  _4800    = 4800U,   
  _9600    = 9600U,   
  _14400   = 14400U,  
  _19200   = 19200U,  
  _38400   = 38400U,  
  _56000   = 56000U,  
  _57600   = 57600U,  
  _115200  = 115200U, 
  _230400  = 230400U, 
  _256000  = 256000U,
  _460800  = 460800U,
  _500000  = 500000U,
  _921600  = 921600U, 
  _2000000 = 2000000U,
  _3000000 = 3000000U,
  _4000000 = 4000000U,
  _5000000 = 5000000U,
  _6000000 = 6000000U,
  _7000000 = 7000000U,
  _9000000 = 9000000U 
};

enum TParity : uint32_t
{
  _NONE       = 0U,
  _EVEN       = 1U,
  _ODD        = 2U,
	
	_MAX_PARITY = _ODD + 1,
};

enum TStops : uint32_t
{
  _STOPBITS_1 = 0,
  _STOPBITS_2 = 1,
	
	_MAX_STOPS  = _STOPBITS_2 + 1,
};

struct TSets
{
  TBaudRate BaudRate;
  TParity   Parity;
  TStops    Stops;
};

static const TSets DEF_SETS;

static constexpr uint32_t MAX_DEDT    = 31U;
static constexpr uint32_t MAX_DEAT    = 31U;
static constexpr uint8_t SYM_BITS_NUM = 11U;
static constexpr uint8_t SYN_NUM      =  2U;

public:
  TUsart(
         const TUsart_HW &_HW,
         const TSets &_Sets = DEF_SETS,
//		     TMode _Mode        = TMode::_STANDARD
		     TMode _Mode        = TMode::_RS485_DE
        );
  ~TUsart();

  void pin_clk_config();
  void hw_init( uint8_t TimeoutBitsQty = 0U );                //TimeoutBitsQty зависит от используемого протокола                                                            
  void set_sets( const TSets &_Sets );                        
                                                              
protected:                                                    
  const TUsart_HW &HW;                                        
  TSets Sets;                                                 //настройки для конкретного аппаратного модуля должны быть в одном экземпляре
  TMode Mode;                                                 
                                                              
  void set_485_hd( uint32_t Fck, uint32_t OvS );              //RS485 driver enable using USART (half-duplex)
	void set_232( uint32_t Fck, uint32_t OvS );                 
                                                              
  void en_eob_detect( uint8_t Timeout );                      //Timeout - количество бит, соответствующее требуемому времени таймаута
	void dis_eob_detect();
  const TSets &get_sets() const;
  
  void dis_if();
  bool chk_sets( const TSets & );
  
  void read_burst(uint8_t *DataBurst, uint8_t Size);          //многобайтное считывание
  void write_burst( const uint8_t *DataBurst, uint16_t Size ); //многобайтная запись
  uint8_t read_byte();                                        //считывание одного байта
  void enable_rx();
  void disable_rx();
private:
  void set_interrupt();
  void clr_error_flags();
};

#endif //__USART_DRIVER_H
