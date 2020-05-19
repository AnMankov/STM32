#ifndef __LEV_GAUGE_TO_HC_H
#define __LEV_GAUGE_TO_HC_H

#include "lib.h"
#include "ext_master.h"

namespace PC_CMD
{
  const uint8_t SET_FREG      = 0x66;
  const uint8_t DUTY_CYCLE    = 0x44;
  const uint8_t SET_POS_POT_1 = 0x50;
  const uint8_t SET_POS_POT_2 = 0x70;
  const uint8_t TEST          = 0x54;
  const uint8_t START_ADC     = 0x41;
  const uint8_t CMP_CAPTURE   = 0x43;
  const uint8_t DIFF_CAPTURE  = 0x64;
};

class TExchngToPC final : public TExtMaster
{
public:
  struct TRange
  {
    float Min;
    float Max;
  };

  struct TSetVal
  {
    uint8_t *Addr;
    uint8_t  BytesQty;
  };
    
  struct TParamHandle
  {
    const uint8_t Cmd;
    float         RxVal;
    TRange        Range;
    TSetVal       SetVal;
  };

  struct TPwm
  {
    uint16_t ARR;
    uint16_t PSC;
    uint16_t CCR;
  };
  
public:
  TExchngToPC(
              const TUsart_HW &_Usart_HW,
              const TTmr_HW   &_TmrFreg_HW,
              const TTmr_HW   &_TmrCmp_HW,
              const TTmr_HW   &_TmrDiff_HW,
              const TAdc_HW   &_Adc_HW
             );
  ~TExchngToPC();
  
  void rx_set_freg( TExchngToPC::TParamHandle *ParamHandle );    
  void rx_set_duty_cycle( TExchngToPC::TParamHandle *ParamHandle );   
  void rx_set_pos_pot_1( TExchngToPC::TParamHandle *ParamHandle );
  void rx_set_pos_pot_2( TExchngToPC::TParamHandle *ParamHandle );
  void rx_start_adc( TExchngToPC::TParamHandle *ParamHandle );
  void rx_cmp_capture( TExchngToPC::TParamHandle *ParamHandle );
  void rx_diff_capture( TExchngToPC::TParamHandle *ParamHandle );
  
  void tx_set_freg( TExchngToPC::TParamHandle *ParamHandle );    
  void tx_set_duty_cycle( TExchngToPC::TParamHandle *ParamHandle );   
  void tx_set_pos_pot_1( TExchngToPC::TParamHandle *ParamHandle );
  void tx_set_pos_pot_2( TExchngToPC::TParamHandle *ParamHandle );
  void tx_test( TExchngToPC::TParamHandle *ParamHandle );
  void tx_adc( TExchngToPC::TParamHandle *ParamHandle );
  void tx_cmp_capture( TExchngToPC::TParamHandle *ParamHandle );
  void tx_diff_capture( TExchngToPC::TParamHandle *ParamHandle );
  
  void init_tmr_freg();
  void init_tmr_cmp();
  void init_tmr_diff();
  void init_adc();
  void init_dma();
  void en_dma_with_adc( uint16_t );
  void dis_dma_with_adc();
  void init_diff_exti(  uint32_t Trigger );
  void deinit_diff_exti();
  void adc_cal();
  void set_diff_freq( uint16_t );
  
  void set_upd_demand_flag( bool Flag );
  bool get_upd_demand_flag();
//  float get_freq();
//  void set_freq( float Val );
//  float get_duty_cycle();
//  void set_duty_cycle( float Val );
  
//  TParams RxSets;     //настройки, пришедшие по интерфейсу
//  TParams ActiveSets; //реально установленные настройки

  float   Tx_Freq_hz;
  float   Tx_DutyCycle_pc;
  uint8_t Tx_PosPot_1;
  uint8_t Tx_PosPot_2;
  uint8_t Tx_Test;
  
  const TTmr_HW &TmrFreg;
  const TTmr_HW &TmrCmp;
  const TTmr_HW &TmrDiff;
  const TAdc_HW &Adc; 
protected:
private:
  float chk_range( float Val, const TRange &Range );
  bool  iserr( const float Val );
  void  set_f_pwm();
  void  set_dc_pwm();
  TPwm  cnt_pwm();
  void  ctrl_pwm( TExchngToPC::TParamHandle * );

  void set_pwm( const TPwm & );
  
  void en_adc();
  void dis_adc();
 
//  TData Data;
  bool UpdDemandFlag;

  uint32_t TmrFreq;
  
  uint8_t AdcCalibFactorS;
};


#endif //__LEV_GAUGE_TO_HC_H
