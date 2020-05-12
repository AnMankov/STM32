#include "dev_handlers.h"
#include "prog_ver.h"
#include "rtos_headers.h"
#include "discrete_out.h"

//const char *TDevHandlers::ID17_1 = "DKS1";
//const char *TDevHandlers::ID17_2 = "DKS2";

const char *TDevHandlers::ID17[] =
{
  "DKS1",
  "DKS2",
};

//TDevHandlers::TAccessLevel AccessLevel[] =
//{
//  { TModel::TAccess::__USER,  TModel::TAccess::__USER  }, //[ __U_U ]
//  { TModel::TAccess::__USER,  TModel::TAccess::__ADMIN }, //[ __U_A ]
//  { TModel::TAccess::__USER,  TModel::TAccess::__SUPER }, //[ __U_S ]
//  { TModel::TAccess::__ADMIN, TModel::TAccess::__USER  }, //[ __A_U ]
//  { TModel::TAccess::__ADMIN, TModel::TAccess::__ADMIN }, //[ __A_A ]
//  { TModel::TAccess::__ADMIN, TModel::TAccess::__SUPER }, //[ __A_S ]
//  { TModel::TAccess::__SUPER, TModel::TAccess::__USER  }, //[ __S_U ]
//  { TModel::TAccess::__SUPER, TModel::TAccess::__ADMIN }, //[ __S_A ]
//  { TModel::TAccess::__SUPER, TModel::TAccess::__SUPER }, //[ __S_S ]
//};

static const TModel::TAccess Access[] =
{
  TModel::__USER,  //[ __USER,  ]
  TModel::__ADMIN, //[ __ADMIN, ]
  TModel::__SUPER, //[ __SUPER, ]
};

TDevHandlers::TPDU BaseToPcPDU[] =
{
	{  2, 1000, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_base_pos_err },
	{  2, 1001, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_connect },
	{  2, 1002, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_hc },
	{  2, 1003, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_hc_pos_err },
	{  2, 1004, (void *)&Access[ TModel::__USER  ], &TDevHandlers::rdi_placebo },           //для поддержки всяческих говнотрекеров, с побайтовым вместо побитового опроса
	{  2, 1005, (void *)&Access[ TModel::__USER  ], &TDevHandlers::rdi_placebo },           //для поддержки всяческих говнотрекеров, с побайтовым вместо побитового опроса
	{  2, 1006, (void *)&Access[ TModel::__USER  ], &TDevHandlers::rdi_placebo },           //для поддержки всяческих говнотрекеров, с побайтовым вместо побитового опроса
	{  2, 1007, (void *)&Access[ TModel::__USER  ], &TDevHandlers::rdi_placebo },           //для поддержки всяческих говнотрекеров, с побайтовым вместо побитового опроса
	{  4, 1010, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_open_angle },       
	{  4, 1011, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_state },            //расписанное по битам (1 или 0) 16-ти разрядное значение состояния
	{  4, 1012, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::read_pitch_bias_angle },
	{  4, 1013, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::read_roll_bias_angle },
	{  1, 2000, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_do },
	{  5, 2000, (void *)&Access[ TModel::__ADMIN ], &TDevHandlers::write_do },
	{  3, 2001, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_thr },
	{  6, 2001, (void *)&Access[ TModel::__ADMIN ], &TDevHandlers::write_thr },
	{  3, 2002, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_hyst },
	{  6, 2002, (void *)&Access[ TModel::__ADMIN ], &TDevHandlers::write_hyst },
	{  3, 2003, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_bias },
	{  6, 2003, (void *)&Access[ TModel::__ADMIN ], &TDevHandlers::write_bias },
	{  3, 2004, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_axis_rotate },
	{  6, 2004, (void *)&Access[ TModel::__ADMIN ], &TDevHandlers::write_axis_rotate },
	{  3, 2005, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_sens_axis_rotate },
	{  6, 2005, (void *)&Access[ TModel::__ADMIN ], &TDevHandlers::write_sens_axis_rotate },
	{  3, 2064, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_u_baud_rate },
	{  6, 2064, (void *)&Access[ TModel::__ADMIN ], &TDevHandlers::write_u_baud_rate },
	{  3, 2065, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_u_par },
	{  6, 2065, (void *)&Access[ TModel::__ADMIN ], &TDevHandlers::write_u_par },
	{  3, 2066, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_mb_addr },
	{  6, 2066, (void *)&Access[ TModel::__ADMIN ], &TDevHandlers::write_mb_addr },
	{  4, 2420, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_prog_nbr },
	{  3, 2426, (void *)&Access[ TModel::__ADMIN ], &TDevHandlers::read_adm_pswd_lo },
	{  3, 2427, (void *)&Access[ TModel::__ADMIN ], &TDevHandlers::read_adm_pswd_hi },
	{ 16, 2426, (void *)&Access[ TModel::__ADMIN ], &TDevHandlers::write_adm_pswd_lo },
	{ 16, 2427, (void *)&Access[ TModel::__ADMIN ], &TDevHandlers::write_adm_pswd_hi },
	{  3, 2428, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::read_super_pswd_lo },
	{  3, 2429, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::read_super_pswd_hi },
	{ 16, 2428, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::write_super_pswd_lo },
	{ 16, 2429, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::write_super_pswd_hi },
	{  3, 3000, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_calib_process },
	{  6, 3000, (void *)&Access[ TModel::__USER  ], &TDevHandlers::write_calib_process },
//	{ 16, 3000, (void *)&Access[ TModel::__USER  ], &TDevHandlers::write_calib_process },
//	{  3, 3001, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::read_position_ctr_lo },
//	{  3, 3002, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::read_position_ctr_hi },
	{  3, 3001, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_parameter_lo },
	{  3, 3002, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_parameter_hi },
	{ 16, 3001, (void *)&Access[ TModel::__USER  ], &TDevHandlers::write_parameter_lo },
	{ 16, 3002, (void *)&Access[ TModel::__USER  ], &TDevHandlers::write_parameter_hi },
//	{  4, 7000, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::read_my_angle },
//	{  4, 7001, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::read_sens_angle },
//	{  4, 7002, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::read_code_sw },
//	{  4, 7003, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::read_sens_code_sw },
	{  4, 7000, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_my_angle },     //для тестирования режим доступа - __USER
	{  4, 7001, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_sens_angle },   //для тестирования режим доступа - __USER
	{  4, 7002, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_code_sw },      //для тестирования режим доступа - __USER
	{  4, 7003, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_sens_code_sw }, //для тестирования режим доступа - __USER
	{  4, 7004, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::read_sens_prog_nbr },
	{  4, 7005, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::read_sens_pitch_bias_angle },
	{  4, 7006, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::read_sens_roll_bias_angle }, 
	{  2, 7010, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::read_sample_valid_sign },
	{  2, 7011, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::read_sens_sample_valid_sign },
	{  2, 7012, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::read_calib },
	{  2, 7013, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::read_sens_calib },
	{  2, 7014, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::read_base_mems },
	{  2, 7015, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::read_hc_mems },
	{  3, 8016, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::read_pd_pressure_lo },
	{  3, 8017, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::read_pd_pressure_hi },
};

TDevHandlers::TPDU BaseToSensPDU[] = //в данном обмене база-мастер => обработчики обратных функций (на команды чтения - обработчики записи и наоборот)
{
	{ 4, 7001, (void *)&Access[ TModel::__USER  ], &TDevHandlers::my_read_sens_angle },            //запрос в одном пакете
	{ 4, 7002, (void *)&Access[ TModel::__USER  ], &TDevHandlers::my_read_sample_valid_sign },     //запрос в одном пакете
	{ 4, 7003, (void *)&Access[ TModel::__USER  ], &TDevHandlers::my_read_sens_code_sw },          //запрос в одном пакете
	{ 4, 7004, (void *)&Access[ TModel::__USER  ], &TDevHandlers::my_read_sens_prog_nbr },         //запрос в одном пакете
	{ 4, 7005, (void *)&Access[ TModel::__USER  ], &TDevHandlers::my_read_sens_calib },            //запрос в одном пакете
	{ 4, 7006, (void *)&Access[ TModel::__USER  ], &TDevHandlers::my_read_sens_axis_rotate },      //запрос в одном пакете
	{ 4, 7007, (void *)&Access[ TModel::__USER  ], &TDevHandlers::my_read_sens_pitch_bias_angle }, //запрос в одном пакете
	{ 4, 7008, (void *)&Access[ TModel::__USER  ], &TDevHandlers::my_read_sens_roll_bias_angle },  //запрос в одном пакете
	{ 5, 7010, (void *)&Access[ TModel::__USER  ], &TDevHandlers::my_write_start_meas_cmd },
	{ 6, 7020, (void *)&Access[ TModel::__USER  ], &TDevHandlers::my_write_sens_axis_rotate },
	{ 3, 1016, (void *)&Access[ TModel::__USER  ], &TDevHandlers::write_pd_pressure_lo },
	{ 3, 1017, (void *)&Access[ TModel::__USER  ], &TDevHandlers::write_pd_pressure_hi },
};

TDevHandlers::TPDU SensToBasePDU[] =
{ 
	{  2, 1002, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_hc },
	{  2, 1003, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_hc_pos_err },
  {  4, 1011, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_state },  //расписанное по битам (1 или 0) 16-ти разрядное значение состояния
	{  4, 1012, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::read_pitch_bias_angle },
	{  4, 1013, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::read_roll_bias_angle },
	{  1, 2001, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_interconn },          //на базе не используется, служит для перенастройки типа взаимодействия: Датчик<->База - Датчик<->M.M.
	{  5, 2001, (void *)&Access[ TModel::__USER  ], &TDevHandlers::write_interconn },         //на базе не используется, служит для перенастройки типа взаимодействия: Датчик<->База - Датчик<->M.M.
	{  3, 2004, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_axis_rotate },
	{  6, 2004, (void *)&Access[ TModel::__USER  ], &TDevHandlers::write_axis_rotate },
	{  3, 2064, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_u_baud_rate },        //на базе не используется, служит для перенастройки типа взаимодействия: Датчик<->База - Датчик<->M.M.
	{  6, 2064, (void *)&Access[ TModel::__USER  ], &TDevHandlers::write_u_baud_rate },       //на базе не используется, служит для перенастройки типа взаимодействия: Датчик<->База - Датчик<->M.M.
	{  3, 2065, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_u_par },              //на базе не используется, служит для перенастройки типа взаимодействия: Датчик<->База - Датчик<->M.M.
	{  6, 2065, (void *)&Access[ TModel::__USER  ], &TDevHandlers::write_u_par },             //на базе не используется, служит для перенастройки типа взаимодействия: Датчик<->База - Датчик<->M.M.
	{  3, 2066, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_mb_addr },            //на базе не используется, служит для перенастройки типа взаимодействия: Датчик<->База - Датчик<->M.M.
	{  6, 2066, (void *)&Access[ TModel::__USER  ], &TDevHandlers::write_mb_addr },           //на базе не используется, служит для перенастройки типа взаимодействия: Датчик<->База - Датчик<->M.M.
	{  4, 2420, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_prog_nbr },
	{  3, 3000, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_calib_process },      //на базе не используется, служит для удобства калибровки при первичной прошивке
	{  6, 3000, (void *)&Access[ TModel::__USER  ], &TDevHandlers::write_calib_process },     //на базе не используется, служит для удобства калибровки при первичной прошивке
	{  3, 3001, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_parameter_lo },
	{  3, 3002, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_parameter_hi },
	{ 16, 3001, (void *)&Access[ TModel::__USER  ], &TDevHandlers::write_parameter_lo },
	{ 16, 3002, (void *)&Access[ TModel::__USER  ], &TDevHandlers::write_parameter_hi },
	{  4, 7001, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_my_angle },           //ответ в одном пакете
	{  4, 7002, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_sample_valid_sign },  //ответ в одном пакете
	{  4, 7003, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_code_sw },            //ответ в одном пакете
	{  4, 7004, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_prog_nbr },           //ответ в одном пакете
	{  4, 7005, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_calib },              //ответ в одном пакете
	{  4, 7006, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_axis_rotate },        //ответ в одном пакете
	{  4, 7007, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_pitch_bias_angle },   //ответ в одном пакете
	{  4, 7008, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_roll_bias_angle },    //ответ в одном пакете
	{  5, 7010, (void *)&Access[ TModel::__USER  ], &TDevHandlers::write_start_meas_cmd },
	{  2, 7010, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::read_sample_valid_sign },
	{  2, 7012, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::read_calib },
	{  2, 7015, (void *)&Access[ TModel::__SUPER ], &TDevHandlers::read_hc_mems },
	{  6, 7020, (void *)&Access[ TModel::__USER  ], &TDevHandlers::write_axis_rotate },
};

TDevHandlers::TPDU SensToPcPDU[] =
{
	{  2, 1002, (void *)&Access[ TModel::__USER  ],&TDevHandlers::read_hc },
	{  2, 1003, (void *)&Access[ TModel::__USER  ],&TDevHandlers::read_hc_pos_err },
	{  2, 1004, (void *)&Access[ TModel::__USER  ],&TDevHandlers::rdi_placebo }, //для поддержки всяческих говнотрекеров, с побайтовым вместо побитового опроса
	{  2, 1005, (void *)&Access[ TModel::__USER  ],&TDevHandlers::rdi_placebo }, //для поддержки всяческих говнотрекеров, с побайтовым вместо побитового опроса
	{  2, 1006, (void *)&Access[ TModel::__USER  ],&TDevHandlers::rdi_placebo }, //для поддержки всяческих говнотрекеров, с побайтовым вместо побитового опроса
	{  2, 1007, (void *)&Access[ TModel::__USER  ],&TDevHandlers::rdi_placebo }, //для поддержки всяческих говнотрекеров, с побайтовым вместо побитового опроса
	{  2, 1008, (void *)&Access[ TModel::__USER  ],&TDevHandlers::rdi_placebo }, //для поддержки всяческих говнотрекеров, с побайтовым вместо побитового опроса
	{  2, 1009, (void *)&Access[ TModel::__USER  ],&TDevHandlers::rdi_placebo }, //для поддержки всяческих говнотрекеров, с побайтовым вместо побитового опроса
	{  4, 1011, (void *)&Access[ TModel::__USER  ],&TDevHandlers::read_state },  //расписанное по битам (1 или 0) 16-ти разрядное значение состояния
	{  4, 1012, (void *)&Access[ TModel::__SUPER ],&TDevHandlers::read_pitch_bias_angle },
	{  4, 1013, (void *)&Access[ TModel::__SUPER ],&TDevHandlers::read_roll_bias_angle },
	{  1, 2000, (void *)&Access[ TModel::__USER  ],&TDevHandlers::read_do },
	{  5, 2000, (void *)&Access[ TModel::__ADMIN ],&TDevHandlers::write_do },
	{  1, 2001, (void *)&Access[ TModel::__USER  ],&TDevHandlers::read_interconn },
	{  5, 2001, (void *)&Access[ TModel::__ADMIN ],&TDevHandlers::write_interconn },
	{  3, 2004, (void *)&Access[ TModel::__USER  ],&TDevHandlers::read_axis_rotate },
	{  6, 2004, (void *)&Access[ TModel::__ADMIN ],&TDevHandlers::write_axis_rotate },
	{  3, 2064, (void *)&Access[ TModel::__USER  ],&TDevHandlers::read_u_baud_rate },
	{  6, 2064, (void *)&Access[ TModel::__ADMIN ],&TDevHandlers::write_u_baud_rate },
	{  3, 2065, (void *)&Access[ TModel::__USER  ],&TDevHandlers::read_u_par },
	{  6, 2065, (void *)&Access[ TModel::__ADMIN ],&TDevHandlers::write_u_par },
	{  3, 2066, (void *)&Access[ TModel::__USER  ],&TDevHandlers::read_mb_addr },
	{  6, 2066, (void *)&Access[ TModel::__ADMIN ],&TDevHandlers::write_mb_addr },
	{  4, 2420, (void *)&Access[ TModel::__USER  ],&TDevHandlers::read_prog_nbr },
	{  3, 2426, (void *)&Access[ TModel::__ADMIN ],&TDevHandlers::read_adm_pswd_lo },
	{  3, 2427, (void *)&Access[ TModel::__ADMIN ],&TDevHandlers::read_adm_pswd_hi },
	{ 16, 2426, (void *)&Access[ TModel::__ADMIN ],&TDevHandlers::write_adm_pswd_lo },
	{ 16, 2427, (void *)&Access[ TModel::__ADMIN ],&TDevHandlers::write_adm_pswd_hi },
	{  3, 2428, (void *)&Access[ TModel::__SUPER ],&TDevHandlers::read_super_pswd_lo },
	{  3, 2429, (void *)&Access[ TModel::__SUPER ],&TDevHandlers::read_super_pswd_hi },
	{ 16, 2428, (void *)&Access[ TModel::__SUPER ],&TDevHandlers::write_super_pswd_lo },
	{ 16, 2429, (void *)&Access[ TModel::__SUPER ],&TDevHandlers::write_super_pswd_hi },
	{  3, 3000, (void *)&Access[ TModel::__USER  ],&TDevHandlers::read_calib_process },
	{  6, 3000, (void *)&Access[ TModel::__USER  ],&TDevHandlers::write_calib_process },
//	{ 16, 3000, (void *)&Access[ TModel::__USER  ],&TDevHandlers::write_calib_process },
//	{  3, 3001, (void *)&Access[ TModel::__SUPER ],&TDevHandlers::read_position_ctr_lo },
//	{  3, 3002, (void *)&Access[ TModel::__SUPER ],&TDevHandlers::read_position_ctr_hi },
	{  3, 3001, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_parameter_lo },
	{  3, 3002, (void *)&Access[ TModel::__USER  ], &TDevHandlers::read_parameter_hi },
	{ 16, 3001, (void *)&Access[ TModel::__USER  ], &TDevHandlers::write_parameter_lo },
	{ 16, 3002, (void *)&Access[ TModel::__USER  ], &TDevHandlers::write_parameter_hi },
	{  4, 7001, (void *)&Access[ TModel::__SUPER ],&TDevHandlers::read_my_angle },
	{  4, 7003, (void *)&Access[ TModel::__SUPER ],&TDevHandlers::read_code_sw },
	{  2, 7010, (void *)&Access[ TModel::__SUPER ],&TDevHandlers::read_sample_valid_sign },
	{  2, 7012, (void *)&Access[ TModel::__SUPER ],&TDevHandlers::read_calib },
	{  2, 7015, (void *)&Access[ TModel::__SUPER ],&TDevHandlers::read_hc_mems },
};

TDevHandlers::TPduHandler BaseToPcPduHandler =
{
  BaseToPcPDU,
	sizeof ( BaseToPcPDU ) / sizeof ( BaseToPcPDU[0] )
};

TDevHandlers::TPduHandler BaseToSensPduHandler =
{
  BaseToSensPDU,
	sizeof ( BaseToSensPDU ) / sizeof ( BaseToSensPDU[0] )
};

TDevHandlers::TPduHandler SensToBasePduHandler =
{
  SensToBasePDU,
	sizeof ( SensToBasePDU ) / sizeof ( SensToBasePDU[0] )
};

TDevHandlers::TPduHandler SensToPcPduHandler =
{
  SensToPcPDU,
	sizeof ( SensToPcPDU ) / sizeof ( SensToPcPDU[0] )
};

TDevHandlers::TDevHandlers( TPduHandler *_PduHandler )
:
PduHandler( _PduHandler ),
__BOOT_MODE( "BOOT_MODE" )
{
  AdmPswd.fVal        = 0.0f;
  SuperPswd.fVal      = 0.0f;
  CalibParameter.fVal = 0.0f;
}

TDevHandlers::~TDevHandlers()
{

}

bool TDevHandlers::read_base_pos_err( uint32_t Addr )
{
  uint8_t *ptr = reinterpret_cast<uint8_t *>(Addr);
  
  *ptr = Model.get_base_pos_err();  
  
  return true;
}

bool TDevHandlers::read_connect( uint32_t Addr )
{
  uint8_t *ptr = reinterpret_cast<uint8_t *>(Addr);
  
  *ptr = Model.get_connect();  
  
  return true;
}

bool TDevHandlers::read_hc( uint32_t Addr )
{
  uint8_t *ptr = reinterpret_cast<uint8_t *>(Addr);
  
  *ptr = Model.get_hc_state();  
  
  return true;
}

bool TDevHandlers::read_hc_pos_err( uint32_t Addr )
{
  uint8_t *ptr = reinterpret_cast<uint8_t *>(Addr);
  
  *ptr = Model.get_hc_pos_err();  
  
  return true;
}

bool TDevHandlers::rdi_placebo( uint32_t Addr )
{
  uint8_t *ptr = reinterpret_cast<uint8_t *>(Addr);
  
  *ptr = 0U;  
  
  return true;
}

bool TDevHandlers::read_base_mems( uint32_t Addr )
{
  uint8_t *ptr = reinterpret_cast<uint8_t *>(Addr);
  
  *ptr = Model.get_base_mems();  
  
  return true;
}

bool TDevHandlers::read_hc_mems( uint32_t Addr )
{
  uint8_t *ptr = reinterpret_cast<uint8_t *>(Addr);
  
  *ptr = Model.get_hc_mems();  
  
  return true;
}

bool TDevHandlers::read_open_angle( uint32_t Addr )
{
  int16_t *ptr = reinterpret_cast<int16_t *>(Addr);
  
  *ptr = roundf( Model.get_open_angle() );
  
  return true;
}

bool TDevHandlers::read_code_sw( uint32_t Addr )
{
  uint16_t *ptr = reinterpret_cast<uint16_t *>(Addr);
  
  *ptr = Model.get_code_sw();
  
  return true;
}

bool TDevHandlers::read_pitch_bias_angle( uint32_t Addr )
{
  int16_t *ptr = reinterpret_cast<int16_t *>(Addr);
  
  int8_t Coeff[] =
  {
     1, //TAxisRotate::__0_DEG
     1, //TAxisRotate::__90_DEG
     1, //TAxisRotate::__180_DEG
    -1, //TAxisRotate::__270_DEG
  };
  
  *ptr = Model.get_pitch_bias_angle() * Coeff[ Model.get_axis_rotate() ];
  
  return true;
}

bool TDevHandlers::read_roll_bias_angle( uint32_t Addr )
{
  int16_t *ptr = reinterpret_cast<int16_t *>(Addr);
  
  int8_t Coeff[] =
  {
     1, //TAxisRotate::__0_DEG
     1, //TAxisRotate::__90_DEG
    -1, //TAxisRotate::__180_DEG
     1, //TAxisRotate::__270_DEG
  };
  
  *ptr = Model.get_roll_bias_angle() * Coeff[ Model.get_axis_rotate() ];
  
  return true;
}

bool TDevHandlers::TDevHandlers::read_do( uint32_t Addr )
{
  uint8_t *ptr = reinterpret_cast<uint8_t *>(Addr);
  
  *ptr = Model.get_d_o_sets();  
  
  return true;
}

bool TDevHandlers::write_do( uint32_t Addr )
{
  //проверка входных данных на допустимость не нужна - это делает прикладная часть протокола
  int16_t Val = *reinterpret_cast<uint16_t *>(Addr);
                       
  Model.set_d_o_sets(
                     (Val == 0x0000) 
                     ? TContact::_N_OPENED 
                     : TContact::_N_CLOSED 
                    );
  
  return true;
}

bool TDevHandlers::read_interconn( uint32_t Addr )
{
  uint8_t *ptr = reinterpret_cast<uint8_t *>(Addr);
  
  *ptr = Model.get_interconn();  
  
  return true;
}

bool TDevHandlers::write_interconn( uint32_t Addr )
{
  //проверка входных данных на допустимость не нужна - это делает прикладная часть протокола
  int16_t Val = *reinterpret_cast<uint16_t *>(Addr);

  Model.set_interconn(
                      (Val == 0x0000) 
                      ? TInterconn::__BASE
                      : TInterconn::__OTHER_MASTER 
                     );

  return true;
}

bool TDevHandlers::read_thr( uint32_t Addr )
{
  uint16_t *ptr = reinterpret_cast<uint16_t *>(Addr);
  
  *ptr = Model.get_thr();
  
  return true;
}

bool TDevHandlers::write_thr( uint32_t Addr )
{
  int16_t Val = *reinterpret_cast<uint16_t *>(Addr);
  
  return Model.set_thr( Val );
}

bool TDevHandlers::read_hyst( uint32_t Addr )
{
  uint16_t *ptr = reinterpret_cast<uint16_t *>(Addr);
  
  *ptr = Model.get_hyst();
  
  return true;
}

bool TDevHandlers::write_hyst( uint32_t Addr )
{
  int16_t Val = *reinterpret_cast<uint16_t *>(Addr);
  
  return Model.set_hyst( Val );
}

bool TDevHandlers::read_bias( uint32_t Addr )
{
  uint16_t *ptr = reinterpret_cast<uint16_t *>(Addr);
  
  *ptr = Model.get_bias();
  
  return true;
}

bool TDevHandlers::write_bias( uint32_t Addr )
{
  int16_t Val = *reinterpret_cast<uint16_t *>(Addr);
  
  return Model.set_bias( Val );
}

bool TDevHandlers::read_axis_rotate( uint32_t Addr )
{
  uint16_t *ptr = reinterpret_cast<uint16_t *>(Addr);
  
  *ptr = Model.get_axis_rotate();
  
  return true;
}

bool TDevHandlers::write_axis_rotate( uint32_t Addr )
{
  int16_t Val = *reinterpret_cast<uint16_t *>(Addr);
  
  return Model.set_axis_rotate( Val );
}

bool TDevHandlers::read_u_baud_rate( uint32_t Addr )
{
  uint16_t *ptr = reinterpret_cast<uint16_t *>(Addr);
  
  *ptr = Model.get_u_baud_rate();
  
  return true;
}

bool TDevHandlers::write_u_baud_rate( uint32_t Addr )
{
  int16_t Val = *reinterpret_cast<uint16_t *>(Addr);
  
  return Model.set_u_baud_rate( Val );
}

bool TDevHandlers::read_u_par( uint32_t Addr )
{
  uint16_t *ptr = reinterpret_cast<uint16_t *>(Addr);
  
  *ptr = Model.get_u_par();
  
  return true;
}

bool TDevHandlers::write_u_par( uint32_t Addr )
{
  int16_t Val = *reinterpret_cast<uint16_t *>(Addr);
  
  return Model.set_u_par( Val );
}

bool TDevHandlers::read_mb_addr( uint32_t Addr )
{
  uint16_t *ptr = reinterpret_cast<uint16_t *>(Addr);
  
  *ptr = Model.get_mb_addr();
  
  return true;
}

bool TDevHandlers::write_mb_addr( uint32_t Addr )
{
  int16_t Val = *reinterpret_cast<uint16_t *>(Addr);
  
  return Model.set_mb_addr( Val );
}

bool TDevHandlers::read_prog_nbr( uint32_t Addr )
{
  uint16_t *ptr = reinterpret_cast<uint16_t *>(Addr);
  
  *ptr = __PROG_VERSION >> 16U;
  
  return true;
}

//----- Float - параметры ------------------------------------------------------
bool TDevHandlers::read_adm_pswd_lo( uint32_t Addr )
{
  float Data = Model.get_psw_admin();
  uint16_t *ptr = reinterpret_cast<uint16_t *>(Addr);
  
  *ptr = ((TFourBytesParse *)&Data)->Segment.Lo;
  
  return true;
}

bool TDevHandlers::write_adm_pswd_lo( uint32_t Addr )
{
	AdmPswd.Segment.Lo = *reinterpret_cast<uint16_t *>(Addr);
	
	return true;
}

bool TDevHandlers::read_adm_pswd_hi( uint32_t Addr )
{
  float Data = Model.get_psw_admin();
  uint16_t *ptr = reinterpret_cast<uint16_t *>(Addr);
  
  *ptr = ((TFourBytesParse *)&Data)->Segment.Hi;
  
  return true;
}

bool TDevHandlers::write_adm_pswd_hi( uint32_t Addr )
{
	AdmPswd.Segment.Hi = *reinterpret_cast<uint16_t *>(Addr);
	Model.set_psw_admin( AdmPswd.fVal );
	
	return true;
}

bool TDevHandlers::read_super_pswd_lo( uint32_t Addr )
{
  float Data = Model.get_psw_super();
  uint16_t *ptr = reinterpret_cast<uint16_t *>(Addr);
  
  *ptr = ((TFourBytesParse *)&Data)->Segment.Lo;
  
  return true;
}

bool TDevHandlers::write_super_pswd_lo( uint32_t Addr )
{
	SuperPswd.Segment.Lo = *reinterpret_cast<uint16_t *>(Addr);
	
	return true;
}

bool TDevHandlers::read_super_pswd_hi( uint32_t Addr )
{
  float Data = Model.get_psw_super();
  uint16_t *ptr = reinterpret_cast<uint16_t *>(Addr);
  
  *ptr = ((TFourBytesParse *)&Data)->Segment.Hi;
  
  return true;
}

bool TDevHandlers::write_super_pswd_hi( uint32_t Addr )
{
	SuperPswd.Segment.Hi = *reinterpret_cast<uint16_t *>(Addr);
	Model.set_psw_super( SuperPswd.fVal );
	
	return true;
}
//------------------------------------------------------------------------------

bool TDevHandlers::read_calib_process( uint32_t Addr )
{
  uint16_t *ptr = reinterpret_cast<uint16_t *>(Addr);
  
  *ptr = Model.get_calib_process();
  
  return true;
}

bool TDevHandlers::write_calib_process( uint32_t Addr )
{
  int16_t Val = *reinterpret_cast<uint16_t *>(Addr);
  
  return Model.set_calib_cmd( Val );
}

bool TDevHandlers::read_position_ctr_lo( uint32_t Addr )
{
  uint32_t Data = Model.get_calib_position_ctr();
  uint16_t *ptr = reinterpret_cast<uint16_t *>(Addr);
  
  *ptr = ((TFourBytesParse *)&Data)->Segment.Lo;
  
  return true;
}

bool TDevHandlers::read_position_ctr_hi( uint32_t Addr )
{
  uint32_t Data = Model.get_calib_position_ctr();
  uint16_t *ptr = reinterpret_cast<uint16_t *>(Addr);
  
  *ptr = ((TFourBytesParse *)&Data)->Segment.Hi;
  
  return true;
}

bool TDevHandlers::read_parameter_lo( uint32_t Addr )
{
  float Data = Model.get_calib_parameter();
  uint16_t *ptr = reinterpret_cast<uint16_t *>(Addr);
  
  *ptr = ((TFourBytesParse *)&Data)->Segment.Lo;
  
  return true;
}

bool TDevHandlers::write_parameter_lo( uint32_t Addr )
{
	CalibParameter.Segment.Lo = *reinterpret_cast<uint16_t *>(Addr);
	
	return true;
}

bool TDevHandlers::read_parameter_hi( uint32_t Addr )
{
  float Data = Model.get_calib_parameter();
  uint16_t *ptr = reinterpret_cast<uint16_t *>(Addr);
  
  *ptr = ((TFourBytesParse *)&Data)->Segment.Hi;
  
  return true;
}

bool TDevHandlers::write_parameter_hi( uint32_t Addr )
{
	CalibParameter.Segment.Hi = *reinterpret_cast<uint16_t *>(Addr);
  
  const TModel::TAuthorize *Authorize = Model.get_authorize_item_ptr();
	
  if ( Authorize != nullptr ) //если ожидается ввод пароля
  {
    Model.set_calib_parameter( CalibParameter.fVal );
    
    if ( (Model.*Authorize->get_psw)() == CalibParameter.fVal ) //если значение параметра равно текущему установленному паролю
    {
      Model.set_access( Authorize->Access ); //установка запрошенного режима доступа
      Model.set_calib_process( TModel::TCalibProcess::__PERFORMED );
    }
    else
    {
      Model.set_calib_process( TModel::TCalibProcess::__DENY );
    }    
  }
  else
  {
    Model.set_calib_process( TModel::TCalibProcess::__DENY );
  }
  
  Model.set_authorize_item_ptr( nullptr ); //сброс флага ожидания пароля
  	
	return true;
}

//------------------------------------------------------------------------------
bool TDevHandlers::read_my_angle( uint32_t Addr )
{
  int16_t *ptr = reinterpret_cast<int16_t *>(Addr);
  
  *ptr = Model.get_my_angle();
  
  return true;
}

bool TDevHandlers::read_sens_angle( uint32_t Addr )
{
  int16_t *ptr = reinterpret_cast<int16_t *>(Addr);
  
  *ptr = Model.get_sens_angle();
  
  return true;
}

bool TDevHandlers::my_read_sens_angle( uint32_t Addr )
{
  int16_t Val = *reinterpret_cast<uint16_t *>(Addr);
  
  Model.set_sens_angle( Val );
  
  return true;
}

bool TDevHandlers::read_sample_valid_sign( uint32_t Addr )
{
  int16_t *ptr = reinterpret_cast<int16_t *>(Addr);
  
  *ptr = Model.get_sample_valid_sign();
  
  return true;
}

bool TDevHandlers::my_read_sample_valid_sign( uint32_t Addr )
{
  int16_t Val = *reinterpret_cast<uint16_t *>(Addr);
  
  Model.set_sens_sample_valid_sign( static_cast<TModel::TValidSign>( Val ) );
  
  return true;
}

bool TDevHandlers::read_sens_sample_valid_sign( uint32_t Addr )
{
  int16_t *ptr = reinterpret_cast<int16_t *>(Addr);
  
  *ptr = Model.get_sens_sample_valid_sign();
  
  return true;
}

bool TDevHandlers::read_calib( uint32_t Addr )
{
  uint8_t *ptr = reinterpret_cast<uint8_t *>(Addr);
  
  *ptr = Model.get_acc_calib();  
  
  return true;
}

bool TDevHandlers::read_sens_calib( uint32_t Addr )
{
  uint8_t *ptr = reinterpret_cast<uint8_t *>(Addr);
  
  *ptr = Model.get_hc_acc_calib();  
  
  return true;
}

bool TDevHandlers::my_read_sens_calib( uint32_t Addr )
{
  int16_t Val = *reinterpret_cast<uint16_t *>(Addr);
  
  Model.set_hc_acc_calib( static_cast<TModel::TCalib>( Val ) );
  
  return true;
}

bool TDevHandlers::read_sens_code_sw( uint32_t Addr )
{
  uint16_t *ptr = reinterpret_cast<uint16_t *>(Addr);
  
  *ptr = Model.get_hc_code_sw();
  
  return true;
}

bool TDevHandlers::my_read_sens_code_sw( uint32_t Addr )
{
  uint16_t Val = *reinterpret_cast<uint16_t *>(Addr);
  
  Model.set_hc_code_sw( static_cast<TModel::TCalib>( Val ) );
  
  return true;
}

bool TDevHandlers::read_sens_prog_nbr( uint32_t Addr )
{
  uint16_t *ptr = reinterpret_cast<uint16_t *>(Addr);
  
  *ptr = Model.get_hc_prog_nbr();
  
  return true;
}

bool TDevHandlers::read_sens_axis_rotate( uint32_t Addr )
{
  uint16_t *ptr = reinterpret_cast<uint16_t *>(Addr);
  
  *ptr = Model.get_sens_axis_rotate();
  
  return true;
}

bool TDevHandlers::write_sens_axis_rotate( uint32_t Addr )
{
  bool    Res = false;
  int16_t Val = *reinterpret_cast<uint16_t *>(Addr);
  
  if ( Model.get_sens_axis_rotate() == Val ) //попытка записать уже установленное значение
  {
    Res = true;
  }
  else if ( Model.set_sens_buf_axis_rotate( Val ) ) //установка значения буфера, чтобы считывающий с датчика поток, его не переписал
  {
    constexpr uint32_t WAIT_RESP_MS = 20U;
    
    xSemaphoreGive( SensWrAxisRotateSem ); //необходимо отправить датчику запрос на запись регистра
    Res = ( xSemaphoreTake( SensAxisRotateOkSem, pdMS_TO_TICKS( WAIT_RESP_MS ) ) == pdPASS ) //ожидание успешного ответа от датчика в течение WAIT_RESP_MS
        ? Model.set_sens_axis_rotate( Val ), true
        : false;
  }
  else //попытка записать невалидное значение
  {
    Res = false;
  }
  
  return Res;
}

bool TDevHandlers::my_read_sens_axis_rotate( uint32_t Addr )
{
  uint16_t Val = *reinterpret_cast<uint16_t *>(Addr);

  return Model.set_sens_axis_rotate( Val );
}

bool TDevHandlers::read_sens_pitch_bias_angle( uint32_t Addr )
{
  int16_t *ptr = reinterpret_cast<int16_t *>(Addr);
  
  *ptr = Model.get_sens_pitch_bias_angle();
  
  return true;
}

bool TDevHandlers::my_read_sens_pitch_bias_angle( uint32_t Addr )
{
  int16_t Val = *reinterpret_cast<uint16_t *>(Addr);
  
  Model.set_sens_pitch_bias_angle( Val );
  
  return true;
}

bool TDevHandlers::read_sens_roll_bias_angle( uint32_t Addr )
{
  int16_t *ptr = reinterpret_cast<int16_t *>(Addr);
  
  *ptr = Model.get_sens_roll_bias_angle();
  
  return true;
}

bool TDevHandlers::my_read_sens_roll_bias_angle( uint32_t Addr )
{
  int16_t Val = *reinterpret_cast<int16_t *>(Addr);
  
  Model.set_sens_roll_bias_angle( Val );
  
  return true;
}

bool TDevHandlers::read_state( uint32_t Addr ) 
{
  uint16_t *ptr = reinterpret_cast<uint16_t *>(Addr);
  
  uint32_t Bits = 0U; //то, что расписываем по битам
  uint8_t Tmp = Model.get_dev_state();
  
  for ( uint32_t Mul = 1U; Tmp > 0U && Mul <= 1000U; Mul *= 10U, Tmp /= 2U )
  {
    if ( Tmp % 2U == true ) //если число нечетное
    {
      Bits += Mul;
    }
    else
    {
    
    }
  }
  
  *ptr = Bits;
  
  return true;
}

bool TDevHandlers::my_read_sens_prog_nbr( uint32_t Addr )
{
  int16_t Val = *reinterpret_cast<uint16_t *>(Addr);

  Model.set_hc_prog_nbr( Val );

  return true;
}

bool TDevHandlers::my_write_start_meas_cmd( uint32_t Addr )
{
  uint16_t *ptr = reinterpret_cast<uint16_t *>(Addr);
  
  if ( *ptr == __RESP_SUCCESS ) //приема ответа
  {
    //ответ от датчика пришел и не важно, что находилось в поле с данными принятого пакета
    xSemaphoreGive( SensDevOkSem );
  }
  else //отправка запроса
  {
    TModel::TMeas Cmd = Model.get_start_meas_cmd();
    
    *ptr = ( Cmd == TModel::TMeas::__START_MEAS )
         ? 0x00FF                                 //старший байт перед младшим 
         : 0x0000;
  }
  
  return true;
}

bool TDevHandlers::my_write_sens_axis_rotate( uint32_t Addr )
{
  uint16_t *ptr = reinterpret_cast<uint16_t *>(Addr);
  
  if ( *ptr == Model.get_sens_buf_axis_rotate() ) //приема ответа
  {
    xSemaphoreGive( SensAxisRotateOkSem ); //отправка семафора задаче, ожидающей успешной установки значения
  }
  else //отправка запроса
  {
    TTwoBytesParse Tmp;
    Tmp.uVal = Model.get_sens_buf_axis_rotate(); //буферное значение под настройку
        
    *ptr = ( Tmp.Segment.Lo << 8U ) + Tmp.Segment.Hi;
  }
  
  return true;
}

bool TDevHandlers::write_start_meas_cmd( uint32_t Addr )
{
  //проверка входных данных на допустимость не нужна - это делает прикладная часть протокола
  int16_t Val = *reinterpret_cast<uint16_t *>(Addr);

  TModel::TMeas Meas;
  
  Meas = ( Val == 0x0000 ) ? TModel::TMeas::__STOP_MEAS
                           : TModel::TMeas::__START_MEAS;

  if ( Meas == TModel::TMeas::__START_MEAS )
  {
//    Do.toggle();
//    Do.closed();
  }
  
  Model.set_start_meas_cmd( Meas );

  return true;
}

bool TDevHandlers::read_pd_pressure_lo( uint32_t Addr )
{
  float Data = Model.get_pd_pressure();
  uint16_t *ptr = reinterpret_cast<uint16_t *>(Addr);
  
  *ptr = ((TFourBytesParse *)&Data)->Segment.Lo;
  
  return true;
}

bool TDevHandlers::read_pd_pressure_hi( uint32_t Addr )
{
  float Data = Model.get_pd_pressure();
  uint16_t *ptr = reinterpret_cast<uint16_t *>(Addr);
  
  *ptr = ((TFourBytesParse *)&Data)->Segment.Hi;
  
  return true;
}

bool TDevHandlers::write_pd_pressure_lo( uint32_t Addr )
{
	PdPressure.Segment.Lo = *reinterpret_cast<uint16_t *>(Addr);
	
	return true;
}

bool TDevHandlers::write_pd_pressure_hi( uint32_t Addr )
{
	PdPressure.Segment.Hi = *reinterpret_cast<uint16_t *>(Addr);
	Model.set_pd_pressure( PdPressure.fVal );
	
	return true;
}

//----- Callback'и -------------------------------------------------------------------------------
void TDevHandlers::m_read_input_registers_complete()
{
//  Do.open();
  xSemaphoreGive( SensDevResSem );
}

bool TDevHandlers::chk_access( void *param_ptr )
{
  bool Res = false;
  
  TModel::TAccess Cur = Model.get_access();
  TModel::TAccess Need = *reinterpret_cast< TModel::TAccess * >( param_ptr );
  
  return ( Cur >= Need );
}
//------------------------------------------------------------------------------------------------

//----- Вспомогательные функции ------------------------------------------------------------------
const char *TDevHandlers::get_id_17()
{
  return ID17[ Model.get_dev_type() ];
}
//------------------------------------------------------------------------------------------------
