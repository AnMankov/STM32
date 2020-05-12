#ifndef __MODBUS_APP_H
#define __MODBUS_APP_H

//----- Прикладной уровень протокола MODBUS -------

#include "modbus_link.h"
#include "dev_handlers.h"

class TModbusApp final : public TModbusLink, public TDevHandlers
{
public:
  enum TMbFnct : uint8_t //список поддерживаемых кодов функций протокола
  {
    __READ_COILS               =   1U,
    __READ_DISCRETE_INPUTS     =   2U,
    __READ_HOLDING_REGISTERS   =   3U,
    __READ_INPUT_REGISTERS     =   4U,
    __WRITE_SINGLE_COIL        =   5U,
    __WRITE_SINGLE_REGISTER    =   6U,
    __WRITE_MULTIPLE_REGISTERS =  16U,
    __REPORT_SERVER_ID         =  17U,
    __BOOT_MODE                = 100U,
  };

  typedef void (*TMbWrapper)();
  
  __packed union THalfWordParse
  {
    __packed struct
    {
      uint8_t Hi;
      uint8_t Lo;
    } Segment;
    uint16_t Val;
  };
  
  __packed union TLittleEndian
  {
    __packed struct
    {
      uint8_t Lo;
      uint8_t Hi;
    } Segment;
    uint16_t Val;
  };

  struct TMbParam
  {
    TMbFnct    MbFnct;
    uint16_t   RegAddr;
    TMbWrapper MbWrapper;
  };

  struct TMbTable
  {
    const TMbParam *pParam;
    uint8_t Size;
  };
  
  __packed struct TRegInfo
  {
    uint16_t StartAddr;
    uint16_t Qty;
  };

public:
  TModbusApp(
             const TUsart_HW &Usart_HW,
             TProcType _ProcType,
						 TPduHandler *_PduHandler,
             SemaphoreHandle_t *_RtoTrigSem,
             SemaphoreHandle_t *_CommErrSem 
            );
  ~TModbusApp();

  //----- конечные автоматы -----------------------
  void fsm();
  //-----------------------------------------------

  void cmp_if_sets(); //сравнить настройки USART'a: текущие, по которым работает интерфейс с теми \
                        которые записаны во Flash \
                        если не совпадают, то обновить настройки USART'a
                      //Функцию необходимо вызывать в отсутствии процесса приема и передачи

  //----- обработчики состояний для мастера -------
  void m_idle();
  void m_waiting_turnaround_delay();
  void m_waiting_for_reply();
  void m_processing_reply( uint8_t *Buf, uint16_t BUF_SIZE );
  void m_processing_error();
  //-----------------------------------------------

  //----- обработчики состояний для слейва --------
  void idle();
  void s_checking_request();
  void s_formatting_normal_reply();
  void s_processing_required_action( uint8_t *Buf, uint16_t BUF_SIZE );
  void s_formatting_error_reply( uint8_t *Buf );
  //-----------------------------------------------

  //----- обработчики функций протокола для мастера
  void m_read_discrete_inputs    ( uint8_t *Buf, uint16_t BUF_SIZE ); //  2
  void m_read_holding_registers  ( uint8_t *Buf, uint16_t BUF_SIZE ); //  3
  void m_read_input_registers    ( uint8_t *Buf, uint16_t BUF_SIZE ); //  4
  void m_write_single_coil       ( uint8_t *Buf, uint16_t BUF_SIZE ); //  5
  void m_write_single_register   ( uint8_t *Buf, uint16_t BUF_SIZE ); //  6
  //-----------------------------------------------

  //----- обработчики функций протокола для слейва
  void s_read_coils              ( uint8_t *Buf, uint16_t BUF_SIZE ); //   1
  void s_read_discrete_inputs    ( uint8_t *Buf, uint16_t BUF_SIZE ); //   2
  void s_read_holding_registers  ( uint8_t *Buf, uint16_t BUF_SIZE ); //   3
  void s_read_input_registers    ( uint8_t *Buf, uint16_t BUF_SIZE ); //   4
  void s_write_single_coil       ( uint8_t *Buf, uint16_t BUF_SIZE ); //   5
  void s_write_single_register   ( uint8_t *Buf, uint16_t BUF_SIZE ); //   6
  void s_write_multiple_register ( uint8_t *Buf, uint16_t BUF_SIZE ); //  16
  void s_report_server_id        ( uint8_t *Buf, uint16_t BUF_SIZE ); //  17
  void s_boot_mode               ( uint8_t *Buf, uint16_t BUF_SIZE ); // 100
  //-----------------------------------------------

  TRegInfo m3RegInfo; //используется для мастера
  TRegInfo m4RegInfo; //используется для мастера
protected:

private:
  void read_bits( uint8_t *, uint16_t, TMbFnct );
  void read_regs( uint8_t *, uint16_t, TMbFnct );
  void write_single( uint8_t *, uint16_t, TMbFnct );
	void write_mul( uint8_t *, uint16_t, TMbFnct );
  bool chk_new_val( uint16_t , TMbFnct );
  bool chk_legal_item(
                      const TPDU  *beg, 
                      const TPDU  *end, 
                      uint16_t     CurAddr, 
                      TMbFnct      MbFnct,
                      TChkFnct     chk_fnct,
                      const TPDU **target
                     );
};


#endif //__MODBUS_APP_H
