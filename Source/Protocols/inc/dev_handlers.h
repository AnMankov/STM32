#ifndef __DEV_HANDLERS_H
#define __DEV_HANDLERS_H

#include "lib.h"
#include "model.h"

//для 4-байтных параметров (float, uint32_t, int32_t - необходимо создавать по 2 функции чтения и записи)

class TDevHandlers
{
public:	
  typedef bool (TDevHandlers::*TFnct)( uint32_t Addr );
  
  typedef bool (TDevHandlers::*TChkFnct)( void * ); 
  
  
  enum TAccessNbr
  {
    __U_U = 0,
    __U_A = 1,
    __U_S = 2,
    __A_U = 3,
    __A_A = 4,
    __A_S = 5,
    __S_U = 6,
    __S_A = 7,
    __S_S = 8,
  };

	struct TPDU
	{
		uint8_t  FnctNbr;
		uint16_t RegAddr;
    void    *Chk;   
		TFnct    Fnct;
	};
  
  struct TPduHandler
  {
    TPDU    *Buf;
    uint16_t BUF_SIZE;
  };
  
  __packed union TFourBytesParse
  {
    float fVal;
    uint32_t uVal;
    __packed struct
    {
      uint16_t Lo;
      uint16_t Hi;
    } Segment;
  };
  
  __packed union TTwoBytesParse
  {
    int16_t  iVal;
    uint16_t uVal;
    __packed struct
    {
      uint8_t Lo;
      uint8_t Hi;
    } Segment;
  };

public:
  TDevHandlers( TPduHandler *_PduHandler );
  ~TDevHandlers();

	TPduHandler *PduHandler;

	bool read_base_pos_err( uint32_t Addr );
	bool read_connect( uint32_t Addr );
	bool read_hc( uint32_t Addr );
	bool read_hc_pos_err( uint32_t Addr );
  bool rdi_placebo( uint32_t Addr );
	bool read_base_mems( uint32_t Addr );
	bool read_hc_mems( uint32_t Addr );
	bool read_open_angle( uint32_t Addr );
	bool read_code_sw( uint32_t Addr );
	bool read_pitch_bias_angle( uint32_t Addr );
	bool read_roll_bias_angle( uint32_t Addr );

	bool read_do( uint32_t Addr );
	bool write_do( uint32_t Addr );
	bool read_thr( uint32_t Addr );
	bool write_thr( uint32_t Addr );
	bool read_hyst( uint32_t Addr );
	bool write_hyst( uint32_t Addr );
	bool read_bias( uint32_t Addr );
	bool write_bias( uint32_t Addr );
	bool read_axis_rotate( uint32_t Addr );
	bool write_axis_rotate( uint32_t Addr );
	bool read_sens_axis_rotate( uint32_t Addr );
	bool write_sens_axis_rotate( uint32_t Addr );
	bool read_u_baud_rate( uint32_t Addr );
	bool write_u_baud_rate( uint32_t Addr );
	bool read_u_par( uint32_t Addr );
	bool write_u_par( uint32_t Addr );
	bool read_mb_addr( uint32_t Addr );
	bool write_mb_addr( uint32_t Addr );
	bool read_prog_nbr( uint32_t Addr );

//----- Float - параметры ------------------------------------------------------
	bool read_adm_pswd_lo( uint32_t Addr );
	bool write_adm_pswd_lo( uint32_t Addr );
	bool read_adm_pswd_hi( uint32_t Addr );
	bool write_adm_pswd_hi( uint32_t Addr );
	bool read_super_pswd_lo( uint32_t Addr );
	bool write_super_pswd_lo( uint32_t Addr );
	bool read_super_pswd_hi( uint32_t Addr );
	bool write_super_pswd_hi( uint32_t Addr );
//------------------------------------------------------------------------------
	bool read_calib_process( uint32_t Addr );
	bool write_calib_process( uint32_t Addr );
  bool read_position_ctr_lo( uint32_t Addr );
  bool read_position_ctr_hi( uint32_t Addr );
  bool read_parameter_lo( uint32_t Addr ); 
  bool read_parameter_hi( uint32_t Addr ); 
  bool write_parameter_lo( uint32_t Addr );
  bool write_parameter_hi( uint32_t Addr );
  
	
//------------------------------------------------------------------------------
	bool read_interconn( uint32_t Addr );
	bool write_interconn( uint32_t Addr );
	bool read_my_angle( uint32_t Addr );
	bool read_sens_angle( uint32_t Addr );
	bool my_read_sens_angle( uint32_t Addr );
	bool read_sample_valid_sign( uint32_t Addr );
	bool my_read_sample_valid_sign( uint32_t Addr );
  bool read_sens_sample_valid_sign( uint32_t Addr );
	bool read_calib( uint32_t Addr );
	bool read_sens_calib( uint32_t Addr );
	bool my_read_sens_calib( uint32_t Addr );
  bool read_sens_pitch_bias_angle( uint32_t Addr );
  bool my_read_sens_pitch_bias_angle( uint32_t Addr );
  bool read_sens_roll_bias_angle( uint32_t Addr );
  bool my_read_sens_roll_bias_angle( uint32_t Addr );
	bool read_sens_code_sw( uint32_t Addr );
	bool my_read_sens_code_sw( uint32_t Addr );
	bool my_read_sens_axis_rotate( uint32_t Addr );
  
  
  
	bool read_sens_prog_nbr( uint32_t Addr );
	bool my_read_sens_prog_nbr( uint32_t Addr );
  bool read_state( uint32_t Addr );
  bool my_write_start_meas_cmd( uint32_t Addr );
  bool write_start_meas_cmd( uint32_t Addr );
  bool my_write_sens_axis_rotate( uint32_t Addr );
  
  bool read_pd_pressure_lo( uint32_t Addr );
  bool read_pd_pressure_hi( uint32_t Addr );
  bool write_pd_pressure_lo( uint32_t Addr );
  bool write_pd_pressure_hi( uint32_t Addr );
  
  void m_read_input_registers_complete();
  bool chk_access( void * );
	
	static const char *ID17[];
  const char __BOOT_MODE[10U];                            //признак перехода в загрузчик
  constexpr static uint16_t __RESP_SUCCESS      = 0xA55A; //признак того, что ответ успешно пришел
  constexpr static uint8_t BASE_TO_SENS_MB_ADDR = 143U;   //неизменный адрес при обмене База<->Датчик

protected:
  const char *get_id_17();

private:
  /*
	 *  float данные протокола Modbus могут быть записаны только в 2 захода, но это
	 *  необходимо делать только АТОМАРНО т.к. другой поток может захватить некорректные данные
	 *  в промежуточном состоянии или запись 2-го регистра (при уже записанном 1-ом во Flash) 
	 *  может быть прервана сбоем по питанию
	 */
  TFourBytesParse AdmPswd;
  TFourBytesParse SuperPswd;
  TFourBytesParse CalibParameter;
  TFourBytesParse PdPressure;
};

extern TDevHandlers::TPduHandler BaseToPcPduHandler;
extern TDevHandlers::TPduHandler BaseToSensPduHandler;
extern TDevHandlers::TPduHandler SensToBasePduHandler;
extern TDevHandlers::TPduHandler SensToPcPduHandler;

#endif //__DEV_HANDLERS_H
