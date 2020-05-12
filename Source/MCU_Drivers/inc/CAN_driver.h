#ifndef __CAN_DRIVER_H
#define __CAN_DRIVER_H

#include <cstdint>

#include "data_types.h"
#include "stm32l431xx.h"
#include "Interface.h"
#include "SN65HVD233D.h"

namespace bxCAN //Basic Extended CAN
{
  using std::uint32_t;
  using std::uint8_t;

  struct TCANHw
  {
    //на платформе с F303 1 CAN интерфейс
    GPIO_TypeDef *GpioTx;    //порт вывода Tx
    GPIO_TypeDef *GpioRx;    //порт вывода Rx
    GPIO_TypeDef *GpioRs;    //порт вывода Rs
    GPIO_TypeDef *GpioLbk;   //порт вывода Lbk
	 
    uint32_t TxPinClkMask;   //маска для разрешения тактирования порта с Tx
    uint32_t RxPinClkMask;   //маска для разрешения тактирования порта с Rx
    uint32_t RsPinClkMask;   //маска для разрешения тактирования порта с Rs
    uint32_t LbkPinClkMask;  //маска для разрешения тактирования порта с Lbk	 
	                           //Для CAN предусмотрен только один источник тактирования - шина APB1
    uint32_t PeriphClkMask;  //маска для разрешения тактирования периферии
    uint32_t TxPinMask;      //маска вывода Tx
    uint32_t RxPinMask;      //маска вывода Rx
    uint32_t RsPinMask;      //маска вывода Rs
    uint32_t LbkPinMask;     //маска вывода Lbk
	 
	  uint32_t RsAlterMask;    //маска альтернативной функции вывода Rs
	  uint32_t LbkAlterMask;   //маска альтернативной функции вывода Lbk
	 
    char *Sign;              //подпись
  };
  
  struct T11Id
  {
    uint16_t Val : 11; //0x7FF - MAX
  };
  
  struct T29Id
  {
    uint32_t Val : 29; //0x1FFFFFFF - MAX
  };

  enum class TMode : uint8_t
  {
    _INITIALIZATION,
    _NORMAL,
    _SLEEP
  };
  
  enum class TMsgType : uint8_t
  {
    _DATA,    //сообщение с данными
    _SERVICE, //служебное сообщение
  };
  
  enum class TTestMode : uint8_t
  {
    _SILENT,
    _LOOP_BACK,
    _COMBINED   //"Hot Selftest"
  };

  enum TMailboxNum : uint8_t
  {
    _ONE,
    _TWO,
    _THREE
  };

  enum TDataByte : uint8_t
  {
    _BYTE_0,
    _BYTE_1,
    _BYTE_2,
    _BYTE_3,
    _BYTE_4,
    _BYTE_5,
    _BYTE_6,
    _BYTE_7,
  };
   
  enum class TFilterScale : uint8_t
  {
    _32_Bit = 0,
    _16_Bit,
	 
	  _MAX_FILTER_SCALE
  };
  
  enum class TFilterMode : uint8_t
  {
    _ID_MASK = 0,
    _ID_LIST,
	 
	  _MAX_FILTER_MODE
  };
  
  enum class TIdType : uint8_t
  {
    _STANDARD,
    _EXTENDED 
  };
  
  enum class TFrameType : uint8_t
  {
    _DATA,
    _REMOTE
  };
  
  namespace Filter
  {
    enum TFilterBankNum : uint8_t
    {
      _NULL, _ONE, _TWO, _THREE, _FOUR, _FIVE, _SIX,
      _SEVEN, EIGHT, _NINE, _TEN, _ELEVEN, _TWELVE, _THIRTEEN
    };
  }
  
  struct TDBF
  {
    uint32_t Reserved1 : 15;
    uint32_t DBF       : 1;
    uint32_t Reserved2 : 14;
  };

  struct TRxData
  {
    uint8_t Data0;
    uint8_t Data1;
    uint8_t Data2;
    uint8_t Data3;
  };

  enum class TBaudRate : uint32_t
  {
    _1000    = 1000U,
    _10000   = 10000U, 
    _100000  = 100000U, 
    _500000  = 500000U,  
    _1000000 = 1000000U, 
  };

  enum TLec : uint8_t
  {
    _NO_ERR = 0U,
    _STUFF_ERR,
    _FORM_ERR,
    _ACK_ERR,
    _BIT_RECESS_ERR,
    _BIT_DOMIN_ERR,
    _CRC_ERR,
    _SOFT,

    _MAX_LEC	 
  };

  enum TFifoWarning : uint8_t
  {
    _FULL = 0U,
    _OVR,
    
    _MAX_FIFO_WARNING
  };

  enum TCANError : uint8_t
  {
    _BUS_OFF,
    _ALST,    //потеря арбитража
    _TERR,    //ошибка передачи
    _ERROR_PASSIVE,
    _ERROR_WARNING,
    
    _MAX_CAN_ERROR
  };

  enum TFilterBankNum : uint8_t
  {
    _F_NULL = 0,
    _F_ONE,
    _F_TWO,
    _F_THREE,
    _F_FOUR,
    _F_FIVE,
    _F_SIX,
    _F_SEVEN,
    _F_EIGHT,
    _F_NINE,
    _F_TEN,
    _F_ELEVEN,
    _F_TWELVE,
    _F_THREETEEN,

	  _MAX_FILTER_BANK_NUM
  };

  enum class TRxFifoNum : uint8_t
  {
    _NULL = 0,
	  _ONE  = 1,
    
    _MAX
  };

  enum class TFilterActivate : uint8_t
  {
    _F_NOT_ACTIVE,
    _F_ACTIVE,
  };

  class TCAN final: public SN65HVD233D::TCANPhy
  {
  public:
    static constexpr uint8_t DATA_SIZE = 8U; //размер поля с данными CAN сообщения в байтах

    typedef uint8_t (&TData)[DATA_SIZE];
    typedef GPIO_TypeDef * TPort;
    typedef std::uint32_t TClkMask;
    typedef std::uint32_t TPinMask;
    typedef char * TSign;

    TCAN(TBaudRate _BaudRate = TBaudRate::_500000, TIdType _IdType = TIdType::_STANDARD);
    ~TCAN();

    void pin_clk_config();
    void hw_init();

    void enable_test_mode(TTestMode);
    void disable_all_test_mode();

    void transmit_msg(TData TxData);
    void receive_msg(TData RxData, TRxFifoNum, uint32_t *Id); //получение сообщение из FIFO
//    void receive0_msg(TData RxData); 0
//    void receive1_msg(TData RxData); //получение сообщение из FIFO1

  protected:
  private:
    void set_mode(TMode);
    TMode get_mode();
    void aux_init();      //инициализация вспомогательной периферии для CAN
    void bit_timing_init();
    void filter_init();
    void set_11_id(T11Id, TMailboxNum);
    void set_29_id(T29Id, TMailboxNum);
    void set_data_len_code(TMailboxNum, uint8_t LenData);
    void set_data(TMailboxNum, TData TxData);
    bool check_empty_mailbox(uint32_t EmptyFlagMask);
    void transmit_req(TMailboxNum);
    void error_init();
    void clear_dbg_freeze();
    void filter_bank_modify(TFilterScale, TFilterMode, TFilterBankNum, TRxFifoNum); //настройка одного фильтрующего банка
    void filter_activate(TFilterBankNum BankNum, TFilterActivate Activate);
    void set_interrupt();

    TCANHw CANHw;         //аппаратные настройки интерфейса CAN
    TBaudRate BaudRate;

    TMode Mode;           //после сброса CAN аппаратно переключается в Sleep mode
	  TIdType IdType;       //разрядность идентификатора (11 или 29 бит)       

    TRxData RxData;       //Данные, принятые по CAN

	  TLec Lec;
  };
}

#endif
