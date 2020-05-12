//--------------------------------------------------------
// Файл описания классов для обмена данными с PC
//--------------------------------------------------------
#ifndef __PC_COMM_H
#define __PC_COMM_H

#include <cstdint>

#include "USART_driver.h"
#include "data_types.h"
#include "data_ctrl.h"

namespace pc_comm
{
  using std::uint32_t;
  using std::uint8_t;

  constexpr uint8_t RX_ID = 0xA5; //идентификатор пакета приема
  constexpr uint8_t TX_ID = 0x5A; //идентификатор пакета передачи

  enum class TCommType : uint32_t
  {
    _TX_SINGLE,
    _TX_BURST,
    _RX_SINGLE,
    _RX_BURST,
  };

  class TPCComm final : public USART::TUSART
  {
  public:
    explicit TPCComm(data_ctrl::TDataCtrl &);
    ~TPCComm();

	 void send_results(); //отправка пакета с результатами текущей выборки
	 void send_control(); //отправка пакета управления устройством
	 void receive_control(); //прослушивание входного потока выполняется между отправками

	 TCommType CommType;
  protected:
  private:
    data_ctrl::TDataCtrl &Data;
//	 TResults SendResults;
//	 TControls SendControls;

//	 TResults ReceiveResults;
  };
}

#endif
