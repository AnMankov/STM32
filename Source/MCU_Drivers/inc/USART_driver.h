//--------------------------------------------------------
// Файл описания классов драйвера USART микроконтроллера
//--------------------------------------------------------
#ifndef __LEDS_USART_DRIVER_H
#define __LEDS_USART_DRIVER_H

#include "hard_config.h"
#include "lib.h"

namespace USART
{
  using std::uint32_t;
  using std::uint16_t;
  using std::uint8_t;

  struct TUSARTHw
  {
    USART_TypeDef *USARTx;  //интерфейс
    GPIO_TypeDef *GpioTX;   //порт вывода TX
    GPIO_TypeDef *GpioRX;   //порт вывода RX
    GPIO_TypeDef *GpioDE;   //порт вывода DE
    uint32_t PinClkMask;    //маска для разрешения тактирования порта
    uint32_t ClkSrc;        //маска для выбора источника тактирования
	  uint32_t CheckClkSrc;   //маска для проверки выбранного источника тактирования
    uint32_t PeriphClkMask; //маска для разрешения тактирования периферии
    uint32_t TxPinMask;     //маска вывода TX
    uint32_t RxPinMask;     //маска вывода RX
    uint32_t DEPinMask;     //маска вывода DE
    char *Sign;             //подпись
  };
  
  enum class TProc : uint8_t
  {
    _AUTO = 0,      //автоопределение
    _MODBUS,
    _SENS,
    _OMNICOMM,
    _DKS_TO_PC,
  };
  
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
    _921600  = 921600U,
    _2000000 = 2000000U,
    _3000000 = 3000000U,
    _4000000 = 4000000U,
    _5000000 = 5000000U,
    _6000000 = 6000000U,
    _7000000 = 7000000U,
    _9000000 = 9000000U
  };
  
  enum class TParity : uint32_t
  {
    _NONE = LL_USART_PARITY_NONE,
    _EVEN = LL_USART_PARITY_EVEN,
    _ODD  = LL_USART_PARITY_ODD 
  };
  
  enum class TStops : uint32_t
  {
    _STOPBITS_1 = LL_USART_STOPBITS_1,
    _STOPBITS_2 = LL_USART_STOPBITS_2
  };
  
  struct TSet
  {
    TBaudRate BaudRate;
    TParity   Parity;
    TStops    Stops;
  };

  constexpr uint16_t INBUFFER_SIZE   = 1024U;
  constexpr uint16_t USART_RXBUFSIZE =  256U;
  constexpr uint16_t USART_TXBUFSIZE =  256U;
  constexpr TSet _DEF_SET            = {
                                        TBaudRate::_19200,
//                                        TBaudRate::_921600, 
                                        TParity::_NONE, 
                                        TStops::_STOPBITS_1
                                       };

  //cостояния USART
  enum class TUsartState : uint8_t
  {
    U_IDLE = 0, //тишина
    U_TX   = 1, //идет передача
    U_RX   = 2, //идет прием
    U_WT   = 3  //ожидание обработки принятого пакета
  };

  //имена в структуре оставлены для визуальной совместимости со структурой USART_DATA устройства ПМП-201
  struct USART_DATA 
  {
    uint8_t     rsbuf[USART_RXBUFSIZE]; //буфер приема  
    uint8_t     trbuf[USART_TXBUFSIZE]; //буфер передачи
    TUsartState ustate;                  //состояние
    uint8_t     err;                     //ошибка при приеме
  
    uint16_t    rscnt;                   //количество байт в буфере приёма
    uint16_t    trcnt;                   //количество байт в буфере передачи
  };

  class TUSART : public ADM2587E::TADM2587E
  {
    public:
	    TUSART(
             USART_TypeDef *USARTx,
		         TMode _Mode      = TMode::_RS485_DE,
             TProc Proc       = TProc::_MODBUS,
		         const TSet &_Set = _DEF_SET
            );

	    ~TUSART();

      void tx_single_byte(uint8_t TxByte, TProc Proc);                   //передача одного байта
	    void tx_burst(const uint8_t *DataBurst, uint16_t Size, TProc Proc); //многобайтная передача

	    void rx_single_byte(uint8_t *RxByte);                              //считывание одного байта
	    void rx_burst(uint8_t *DataBurst, uint8_t Size);                   //многобайтное считывание

      void pin_clk_config();
	    void usart_hw_init();

      bool set_cmp(const TSet &) const;                             //сравнение переданных настроек с установленными \
		                                                                  изменять настройки необходимо только после вызова \
                                                                      этой функции и если она вернет false
		  void set_chng(const TSet &);                                  //изменение аппаратных настроек USART
      void hw_reinit();

		  void transmit();                                              //передача пакета
		  void reset_receive();
      void educt_pkg();

		  USART_DATA u;
		  TProc Proc;                                                   //протокол по которому в данный момент работает интерфейс USART

      // Внутренний буфер приема из UART
      uint8_t  inbuffer[INBUFFER_SIZE];
      uint32_t bppoint[16];                                  //указатели на следующую точку окончания пакета в буфере
      uint32_t bpperr[16];                                   //ошибки во время приема
      uint32_t bpphead;                                      //"голова", прибавляющаяся при приеме очередного пакета
      uint32_t bpptail;                                      //"хвост" - указатель на уже обработанную точку
      uint32_t uerr;                                         //ошибки в прошедшей секции

    protected:     

	  private:
      TMode    Mode;                                         //режим работы интерфейса (из поддерживаемых микроконтроллером)
		  TSet     Set;                                          //аппаратные настройки USART
		  TUSARTHw USARTHw;                                      //низкоуровневые константы для настройки USART
		  
      void usart_set_485_hd(uint32_t Fck, uint32_t OvS);     //RS485 driver enable using USART (half-duplex)
		  void usart_set_232(uint32_t Fck, uint32_t OvS);
		  void usart_set_interrupt();
		  void tx_int_set();                                     //настройка прерывания для передачи
		  void rx_int_set();                                     //настройка прерывания для приема
           uint8_t calc_crc(const uint8_t *Add, uint8_t LenData); //подсчет контрольной суммы
		  
		  void sens_set();                                       //специфические настройки для протоколов, применяемых в "СЕНСОР" (СЕНС и Modbus)
		  void sens_usart_set();
		  void sens_dma_set();
		  void sens_timer_set(TIM_TypeDef *, IRQn_Type);         //настройка таймера, необходимого для реализации протоколов		
  };
}

#endif
