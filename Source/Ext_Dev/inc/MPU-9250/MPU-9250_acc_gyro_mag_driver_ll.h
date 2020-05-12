#ifndef __MPU_9250_ACC_GYRO_MAG_DRIVER_LL_H
#define __MPU_9250_ACC_GYRO_MAG_DRIVER_LL_H

#include <cstdint>
//#include <vector>

#include "MPU-9250_register_map.h"
#include "MPU-9250_types.h"
#include "MPU-9250_vals.h"
#include "I2C_Protocol.h"

namespace std
{
  typedef decltype(nullptr) nullptr_t;
}

namespace MPU_9250
{
  using std::uint8_t;

  class TAccGyroMagDriver_LL
  {
  public:
    TAccGyroMagDriver_LL();
    ~TAccGyroMagDriver_LL();
	 
//	 std::vector<TI2CProtocol> ProtocolWrOne;
//	 std::vector<TI2CProtocol> ProtocolWrMul;
//	 std::vector<TI2CProtocol> ProtocolRdOne;
//	 std::vector<TI2CProtocol> ProtocolRdMul;
	 const uint8_t WIA      = 0x48; //Идентификатор магнитометра
	 
  protected:	 
    const uint8_t WHO_AM_I = 0x71; //Идентификатор акселерометра с гироскопом
	 
	 uint8_t get_acc_gyro_addr(TLogicLevelAD0 Level) const; //AD0 - определяется аппаратным подключением вывода 9 микросхемы MPU-9250
	 uint8_t get_mag_addr() const; //По этому адресу к магнитометру можно обратиться, когда микросхема будет настроена в режим Pass-Through
	 

//----- REGISTER MAP FOR ACCELEROMETER AND GYROSCOPE -----------------------------------

	 // Registers 0 to 2 – Gyroscope Self-Test Registers 
	 uint8_t get_st_x_gyro(){return reg_gyro_acc::SELF_TEST_X_GYRO;}
    uint8_t get_st_y_gyro(){return reg_gyro_acc::SELF_TEST_Y_GYRO;}
    uint8_t get_st_z_gyro(){return reg_gyro_acc::SELF_TEST_Z_GYRO;}
    
	 // Registers 13 to 15 – Accelerometer Self-Test Registers
	 uint8_t get_st_x_accel(){return reg_gyro_acc::SELF_TEST_X_ACCEL;}
    uint8_t get_st_y_accel(){return reg_gyro_acc::SELF_TEST_Y_ACCEL;}
    uint8_t get_st_z_accel(){return reg_gyro_acc::SELF_TEST_Z_ACCEL;} 

	 //  Registers 19 to 24 – Gyro Offset Registers
	 uint8_t remove_gyro_bias_x_h(){return reg_gyro_acc::XG_OFFSET_H;}
	 uint8_t remove_gyro_bias_x_l(){return reg_gyro_acc::XG_OFFSET_L;}
	 uint8_t remove_gyro_bias_y_h(){return reg_gyro_acc::YG_OFFSET_H;}
	 uint8_t remove_gyro_bias_y_l(){return reg_gyro_acc::YG_OFFSET_L;}
	 uint8_t remove_gyro_bias_z_h(){return reg_gyro_acc::ZG_OFFSET_H;}
	 uint8_t remove_gyro_bias_z_l(){return reg_gyro_acc::ZG_OFFSET_L;}
	 
	 // Register 25 – Sample Rate Divider
	 uint8_t sample_rate_divider(){return reg_gyro_acc::SMPLRT_DIV;} 
	 
	 // Register 26 – Configuration
	 uint8_t fifo_mode_config(uint8_t *const Byte = nullptr, VFifoModeConfig FIFO_MODE = VFifoModeConfig::_FIFO_FULL_YES_WRITE); //вкл/выкл дополнительную запись в fifo, когда fifo заполнено (запись с заменой) 
	 uint8_t fsync_pin_config(uint8_t *const Byte = nullptr, VFsyncPinConfig EXT_SYNC_SET = VFsyncPinConfig::_DISABLED); //выбор защелкиваемого параметра по входу FSYNC
	 uint8_t filter_gyro_temp_config(uint8_t *const Byte = nullptr, VFilterGyroTempConfig DLPF_CFG = VFilterGyroTempConfig::_G3600_0d17_8_T4000_0d04); //выбор значений фильтрации для гироскопа и термодатчика
    uint8_t filter_gyro_temp_read(uint8_t Byte = 0, VFilterGyroTempConfig *DLPF_CFG = nullptr);

	 // Register 27 – Gyroscope Configuration
	 uint8_t enable_x_gyro_self_test(uint8_t *const Byte = nullptr, VEnDis XGYRO_Cten = VEnDis::_DISABLE); //вкл/выкл самотестирования оси X гироскопа
	 uint8_t enable_y_gyro_self_test(uint8_t *const Byte = nullptr, VEnDis YGYRO_Cten = VEnDis::_DISABLE); //вкл/выкл самотестирования оси Y гироскопа
	 uint8_t enable_z_gyro_self_test(uint8_t *const Byte = nullptr, VEnDis ZGYRO_Cten = VEnDis::_DISABLE); //вкл/выкл самотестирования оси Z гироскопа
	 uint8_t gyro_full_scale_select(uint8_t *const Byte = nullptr, VGyroFullScaleSelect GYRO_FS_SEL = VGyroFullScaleSelect::_2000_DPS); //выбор предела измерения гироскопа
	 uint8_t gyro_full_scale_read(const uint8_t Byte, VGyroFullScaleSelect *GYRO_FS_SEL);
	 uint8_t gyro_filter_choise(uint8_t *const Byte = nullptr, VGyroFilterChoise Fchoice_b = VGyroFilterChoise::_11);

	 // Register 28 – Accelerometer Configuration
	 uint8_t enable_x_accel_self_test(uint8_t *const Byte = nullptr, VEnDis ax_st_en = VEnDis::_DISABLE); //вкл/выкл самотестирования оси X акселерометра
	 uint8_t enable_y_accel_self_test(uint8_t *const Byte = nullptr, VEnDis ay_st_en = VEnDis::_DISABLE); //вкл/выкл самотестирования оси Y акселерометра
	 uint8_t enable_z_accel_self_test(uint8_t *const Byte = nullptr, VEnDis az_st_en = VEnDis::_DISABLE); //вкл/выкл самотестирования оси Z акселерометра
	 uint8_t accel_full_scale_select(uint8_t *const Byte = nullptr, VAccelFullScaleSelect ACCEL_FS_SEL = VAccelFullScaleSelect::_16G); //выбор предела измерения акселерометра

	 // Register 29 – Accelerometer Configuration 2
    uint8_t accel_filter_choise(uint8_t *const Byte = nullptr, VAccelFilterChoise accel_fchoice_b = VAccelFilterChoise::_0);
    uint8_t accel_filter_read(uint8_t Byte = 0, VSetAccelFilter *A_DLPFCFG = nullptr);
	 uint8_t set_accel_filter(uint8_t *const Byte = nullptr, VSetAccelFilter A_DLPFCFG = VSetAccelFilter::_218_1Hz_1_88ms);

	 // Register 30 - Low Power Accelerometer ODR Control
	 uint8_t set_accel_wakeup_freq(uint8_t *const Byte = nullptr, VSetAccelWakeupFreq lposc_clksel = VSetAccelWakeupFreq::_0_24Hz);

	 // Register 31 – Wake-on Motion Threshold
	 uint8_t set_wakeon_motion_thr(){return reg_gyro_acc::WOM_THR;}

	 // Register 35 – FIFO Enable
	 uint8_t write_temp_to_fifo(uint8_t *const Byte = nullptr,   VEnDis TEMP_OUT  = VEnDis::_DISABLE);
	 uint8_t write_gyro_x_to_fifo(uint8_t *const Byte = nullptr, VEnDis GYRO_XOUT = VEnDis::_DISABLE);
	 uint8_t write_gyro_y_to_fifo(uint8_t *const Byte = nullptr, VEnDis GYRO_YOUT = VEnDis::_DISABLE);
	 uint8_t write_gyro_z_to_fifo(uint8_t *const Byte = nullptr, VEnDis GYRO_ZOUT = VEnDis::_DISABLE);
	 uint8_t write_accel_to_fifo(uint8_t *const Byte = nullptr,  VEnDis ACCEL = VEnDis::_DISABLE);
	 uint8_t write_slave2_to_fifo(uint8_t *const Byte = nullptr, VEnDis SLV_2 = VEnDis::_DISABLE);
	 uint8_t write_slave1_to_fifo(uint8_t *const Byte = nullptr, VEnDis SLV_1 = VEnDis::_DISABLE);
	 uint8_t write_slave0_to_fifo(uint8_t *const Byte = nullptr, VEnDis SLV_0 = VEnDis::_DISABLE);
	 
	 // Register 36 – I2C Master Control
    uint8_t multi_master_enable(uint8_t *const Byte = nullptr,        VEnDis MULT_MST_EN   = VEnDis::_DISABLE);
	 uint8_t delay_data_ready_int(uint8_t *const Byte = nullptr,       VEnDis WAIT_FOR_ES   = VEnDis::_DISABLE);
	 uint8_t write_data_slv3_to_fifo(uint8_t *const Byte = nullptr,    VEnDis SLV_3_FIFO_EN = VEnDis::_DISABLE);
	 uint8_t ctrl_slave_read_transition(uint8_t *const Byte = nullptr, VEnDis I2C_MST_P_NSR = VEnDis::_DISABLE);
	 uint8_t config_internal_clk(uint8_t *const Byte = nullptr, VConfigInternalClk I2C_MST_CLK = VConfigInternalClk::_400kHz_20);
	 
	 // Register 37 - I2C_SLV0_ADDR
	 uint8_t slv0_direct_transfer(uint8_t *const Byte = nullptr, VI2CTransferType I2C_SLV0_RNW = VI2CTransferType::_TRANSFER_IS_WRITE);
	 uint8_t slv0_phy_address(uint8_t *const Byte = nullptr, const uint8_t Addr_7Bit = 0);
	 
	 // Register 38 - I2C_SLV0_REG 
	 uint8_t slv0_reg_address_begin(){return reg_gyro_acc::I2C_SLV0_REG;}
	 
	 // Register 39 - I2C_SLV0_CTRL
	 uint8_t slv0_enable(uint8_t *const Byte = nullptr, VEnDis I2C_SLV0_EN = VEnDis::_DISABLE);
	 uint8_t slv0_swap_bytes(uint8_t *const Byte = nullptr, VEnDis I2C_SLV0_BYTE_SW = VEnDis::_DISABLE);
	 uint8_t slv0_write_reg_enable(uint8_t *const Byte = nullptr, VEnDis I2C_SLV0_REG_DIS = VEnDis::_DISABLE);
	 uint8_t slv0_group_data(uint8_t *const Byte = nullptr, VEnDis I2C_SLV0_GRP = VEnDis::_DISABLE);
	 uint8_t slv0_num_bytes_read(uint8_t *const Byte = nullptr, VSlvNumBytesRead I2C_SLV0_LENG = VSlvNumBytesRead::_0_BYTES_TO_READ);
	 
//	 // Register 40 - I2C_SLV1_ADDR
//    uint8_t slv1_transfer_type(uint8_t *const Byte = nullptr, );
//	 uint8_t slv1_phy_address(uint8_t *const Byte = nullptr, );
//	 
//	 // Register 41 - I2C_SLV1_REG
//	 uint8_t slv1_reg_address_begin(uint8_t *const Byte = nullptr, );
//	 
//	 // Register 42 - I2C_SLV1_CTRL
//	 uint8_t slv1_enable(uint8_t *const Byte = nullptr, );
//	 uint8_t slv1_swap_bytes(uint8_t *const Byte = nullptr, );
//	 uint8_t slv1_transaction_ctrl(uint8_t *const Byte = nullptr, );
//	 uint8_t slv1_determine_address(uint8_t *const Byte = nullptr, );
//	 uint8_t slv1_num_bytes_read(uint8_t *const Byte = nullptr, );
	 
//	 // Register 43 - I2C_SLV2_ADDR
//    uint8_t slv2_transfer_type(uint8_t *const Byte = nullptr, );
//	 uint8_t slv2_phy_address(uint8_t *const Byte = nullptr, );
//	 
//    // Register 44 - I2C_SLV2_REG
//	 uint8_t slv2_reg_address_begin(uint8_t *const Byte = nullptr, );
//	 
//	 // Register 45 - I2C_SLV2_CTRL
//	 uint8_t slv2_enable(uint8_t *const Byte = nullptr, );
//	 uint8_t slv2_swap_bytes(uint8_t *const Byte = nullptr, );
//	 uint8_t slv2_transaction_ctrl(uint8_t *const Byte = nullptr, );
//	 uint8_t slv2_determine_address(uint8_t *const Byte = nullptr, );
//	 uint8_t slv2_num_bytes_read(uint8_t *const Byte = nullptr, );

//	 // Register 46 - I2C_SLV3_ADDR
//    uint8_t slv3_transfer_type(uint8_t *const Byte = nullptr, );
//	 uint8_t slv3_phy_address(uint8_t *const Byte = nullptr, );

//	 // Register 47 - I2C_SLV3_REG
//	 uint8_t slv3_reg_address_begin(uint8_t *const Byte = nullptr, );

//	 // Register 48 - I2C_SLV3_CTRL
//	 uint8_t slv3_enable(uint8_t *const Byte = nullptr, );
//	 uint8_t slv3_swap_bytes(uint8_t *const Byte = nullptr, );
//	 uint8_t slv3_transaction_ctrl(uint8_t *const Byte = nullptr, );
//	 uint8_t slv3_determine_address(uint8_t *const Byte = nullptr, );
//	 uint8_t slv3_num_bytes_read(uint8_t *const Byte = nullptr, );

//	 // Register 49 - I2C_SLV4_ADDR
//    uint8_t slv4_transfer_type(uint8_t *const Byte = nullptr, );
//	 uint8_t slv4_phy_address(uint8_t *const Byte = nullptr, );

//	 // Register 50 - I2C_SLV4_REG
//	 uint8_t slv4_reg_address_begin(uint8_t *const Byte = nullptr, );

//	 // Register 51 - I2C_SLV4_DO
//	 uint8_t slv4_data_wr(uint8_t *const Byte = nullptr, );

//	 // Register 52 - I2C_SLV4_CTRL
//	 uint8_t slv4_enable(uint8_t *const Byte = nullptr, );
//	 uint8_t slv4_complete_and_int(uint8_t *const Byte = nullptr, );
//	 uint8_t slv4_transaction_ctrl(uint8_t *const Byte = nullptr, );
//	 uint8_t num_samples_ctrl(uint8_t *const Byte = nullptr, );
//	 
//	 // Register 53 - I2C_SLV4_DI
//    uint8_t slv4_data_rd(uint8_t *const Byte = nullptr, );

	 // Register 54 – I2C Master Status
	 uint8_t status_fsync_interrupt(const uint8_t Byte = 0, VStatusFlag *PASS_THROUGH = nullptr);
	 uint8_t slv4_transfer_complete(const uint8_t Byte = 0, VStatusFlag *I2C_SLV4_DONE = nullptr);
	 uint8_t slv_lost_arbitration(const uint8_t Byte = 0, VStatusFlag *I2C_LOST_ARB = nullptr);
	 uint8_t slv4_rx_nack(const uint8_t Byte = 0, VStatusFlag *I2C_SLV4_NACK = nullptr);
	 uint8_t slv3_rx_nack(const uint8_t Byte = 0, VStatusFlag *I2C_SLV3_NACK = nullptr);
	 uint8_t slv2_rx_nack(const uint8_t Byte = 0, VStatusFlag *I2C_SLV2_NACK = nullptr);
	 uint8_t slv1_rx_nack(const uint8_t Byte = 0, VStatusFlag *I2C_SLV1_NACK = nullptr);
	 uint8_t slv0_rx_nack(const uint8_t Byte = 0, VStatusFlag *I2C_SLV0_NACK = nullptr);

	 // Register 55 – INT Pin / Bypass Enable Configuration
	 uint8_t int_logic_level(uint8_t *const Byte = nullptr, VLogicLevelINT ACTL = VLogicLevelINT::_ACTIVE_HIGH);
	 uint8_t int_pin_config(uint8_t *const Byte = nullptr, VPinConfigINT OPEN = VPinConfigINT::_PUSH_PULL);
	 uint8_t int_latch_ctrl(uint8_t *const Byte = nullptr, VLatchINT LATCH_INT_EN = VLatchINT::_PULSE_50us);
	 uint8_t int_status_ctrl(uint8_t *const Byte = nullptr, VEnDis INT_ANYRD_2CLEAR = VEnDis::_DISABLE);
	 uint8_t fsync_level_ctrl(uint8_t *const Byte = nullptr, VEnDis ACTL_FSYNC = VEnDis::_DISABLE);
    uint8_t fsync_enable(uint8_t *const Byte = nullptr, VEnDis FSYNC_INT_MODE_EN = VEnDis::_DISABLE);
	 uint8_t i2c_master_pins_ctrl(uint8_t *const Byte = nullptr, VEnDis BYPASS_EN = VEnDis::_DISABLE);
	 uint8_t i2c_master_pins_read(const uint8_t Byte = 0, VEnDis *BYPASS_EN = nullptr);

	 // Register 56 – Interrupt Enable
    uint8_t wake_on_motion_int_ctrl(uint8_t *const Byte = nullptr, VEnDis WOM_EN = VEnDis::_DISABLE);
    uint8_t fifo_ovrf_int_ctrl(uint8_t *const Byte = nullptr, VEnDis FIFO_OVERFLOW_EN = VEnDis::_DISABLE);
    uint8_t fsync_int_ctrl(uint8_t *const Byte = nullptr, VEnDis FSYNC_INT_EN = VEnDis::_DISABLE);
	 uint8_t raw_data_ready_int_ctrl(uint8_t *const Byte = nullptr, VEnDis RAW_RDY_EN = VEnDis::_DISABLE);

	 // Register 58 – Interrupt Status
	 uint8_t wake_on_motion_int_status(const uint8_t Byte = 0, VStatusFlag *WOM_INT = nullptr);
	 uint8_t fifo_ovrf_int_status(const uint8_t Byte = 0, VStatusFlag *FIFO_OVERFLOW_INT = nullptr);
	 uint8_t fsync_int_status(const uint8_t Byte = 0, VStatusFlag *FSYNC_INT = nullptr);
	 uint8_t raw_data_ready_int_status(const uint8_t Byte = 0, VStatusFlag *RAW_DATA_RDY_INT = nullptr);

	 // Registers 59 to 64 – Accelerometer Measurements
	 uint8_t accel_data_x_h(){return reg_gyro_acc::ACCEL_XOUT_H;}
	 uint8_t accel_data_x_l(){return reg_gyro_acc::ACCEL_XOUT_L;}
	 uint8_t accel_data_y_h(){return reg_gyro_acc::ACCEL_YOUT_H;}
	 uint8_t accel_data_y_l(){return reg_gyro_acc::ACCEL_YOUT_L;}
	 uint8_t accel_data_z_h(){return reg_gyro_acc::ACCEL_ZOUT_H;}
	 uint8_t accel_data_z_l(){return reg_gyro_acc::ACCEL_ZOUT_L;}
	 
	 // Registers 65 and 66 – Temperature Measurement
	 uint8_t temp_data_h(){return reg_gyro_acc::TEMP_OUT_H;}
	 uint8_t temp_data_l(){return reg_gyro_acc::TEMP_OUT_L;}
	 
	 // Registers 67 to 72 – Gyroscope Measurements
	 uint8_t gyro_data_x_h(){return reg_gyro_acc::GYRO_XOUT_H;}
	 uint8_t gyro_data_x_l(){return reg_gyro_acc::GYRO_XOUT_L;}
	 uint8_t gyro_data_y_h(){return reg_gyro_acc::GYRO_YOUT_H;}
	 uint8_t gyro_data_y_l(){return reg_gyro_acc::GYRO_YOUT_L;}
	 uint8_t gyro_data_z_h(){return reg_gyro_acc::GYRO_ZOUT_H;}
	 uint8_t gyro_data_z_l(){return reg_gyro_acc::GYRO_ZOUT_L;}
	 
	 // Registers 73 to 96 – External Sensor Data
	 uint8_t ext_sens_data_0() {return reg_gyro_acc::EXT_SENS_DATA_00;}
	 uint8_t ext_sens_data_1() {return reg_gyro_acc::EXT_SENS_DATA_01;}
	 uint8_t ext_sens_data_2() {return reg_gyro_acc::EXT_SENS_DATA_02;}
	 uint8_t ext_sens_data_3() {return reg_gyro_acc::EXT_SENS_DATA_03;}
	 uint8_t ext_sens_data_4() {return reg_gyro_acc::EXT_SENS_DATA_04;}
	 uint8_t ext_sens_data_5() {return reg_gyro_acc::EXT_SENS_DATA_05;}
	 uint8_t ext_sens_data_6() {return reg_gyro_acc::EXT_SENS_DATA_06;}
	 uint8_t ext_sens_data_7() {return reg_gyro_acc::EXT_SENS_DATA_07;}
	 uint8_t ext_sens_data_8() {return reg_gyro_acc::EXT_SENS_DATA_08;}
	 uint8_t ext_sens_data_9() {return reg_gyro_acc::EXT_SENS_DATA_09;}
	 uint8_t ext_sens_data_10(){return reg_gyro_acc::EXT_SENS_DATA_10;}
	 uint8_t ext_sens_data_11(){return reg_gyro_acc::EXT_SENS_DATA_11;}
	 uint8_t ext_sens_data_12(){return reg_gyro_acc::EXT_SENS_DATA_12;}
	 uint8_t ext_sens_data_13(){return reg_gyro_acc::EXT_SENS_DATA_13;}
	 uint8_t ext_sens_data_14(){return reg_gyro_acc::EXT_SENS_DATA_14;}
	 uint8_t ext_sens_data_15(){return reg_gyro_acc::EXT_SENS_DATA_15;}
	 uint8_t ext_sens_data_17(){return reg_gyro_acc::EXT_SENS_DATA_17;}
	 uint8_t ext_sens_data_18(){return reg_gyro_acc::EXT_SENS_DATA_18;}
	 uint8_t ext_sens_data_19(){return reg_gyro_acc::EXT_SENS_DATA_19;}
	 uint8_t ext_sens_data_20(){return reg_gyro_acc::EXT_SENS_DATA_20;}
	 uint8_t ext_sens_data_21(){return reg_gyro_acc::EXT_SENS_DATA_21;}
	 uint8_t ext_sens_data_22(){return reg_gyro_acc::EXT_SENS_DATA_22;}
	 uint8_t ext_sens_data_23(){return reg_gyro_acc::EXT_SENS_DATA_23;}
	 
	 // Register 99 – I2C Slave 0 Data Out
	 uint8_t data_out_slv0_wr(){return reg_gyro_acc::I2C_SLV0_DO;}
	 
	 // Register 100 – I2C Slave 1 Data Out
	 uint8_t data_out_slv1_wr(){return reg_gyro_acc::I2C_SLV1_DO;}
	 
	 // Register 101 – I2C Slave 2 Data Out
	 uint8_t data_out_slv2_wr(){return reg_gyro_acc::I2C_SLV2_DO;}
	 
	  // Register 102 – I2C Slave 3 Data Out
	 uint8_t data_out_slv3_wr(){return reg_gyro_acc::I2C_SLV3_DO;}

	 // Register 103 – I2C Master Delay Control
	 uint8_t delay_shadow_ext_sens_ctrl(uint8_t *const Byte = nullptr, VEnDis DELAY_ES_SHADOW = VEnDis::_DISABLE);
	 uint8_t slv4_dly_access_ctrl(uint8_t *const Byte = nullptr,       VEnDis I2C_SLV4_DLY_EN = VEnDis::_DISABLE);
	 uint8_t slv3_dly_access_ctrl(uint8_t *const Byte = nullptr,       VEnDis I2C_SLV3_DLY_EN = VEnDis::_DISABLE);
	 uint8_t slv2_dly_access_ctrl(uint8_t *const Byte = nullptr,       VEnDis I2C_SLV2_DLY_EN = VEnDis::_DISABLE);
	 uint8_t slv1_dly_access_ctrl(uint8_t *const Byte = nullptr,       VEnDis I2C_SLV1_DLY_EN = VEnDis::_DISABLE);
	 uint8_t slv0_dly_access_ctrl(uint8_t *const Byte = nullptr,       VEnDis I2C_SLV0_DLY_EN = VEnDis::_DISABLE);
	 
	 // Register 104 – Signal Path Reset
	 uint8_t reset_gyro_signal_path(uint8_t *const Byte = nullptr,  VEnDis GYRO_RST = VEnDis::_DISABLE);
	 uint8_t reset_accel_signal_path(uint8_t *const Byte = nullptr, VEnDis ACCEL_RST = VEnDis::_DISABLE);
	 uint8_t reset_temp_signal_path(uint8_t *const Byte = nullptr,  VEnDis TEMP_RST = VEnDis::_DISABLE);
	 
	 // Register 105 – Accelerometer Interrupt Control 
	 uint8_t accel_wake_on_motion_ctrl(uint8_t *const Byte = nullptr,  VEnDis ACCEL_INTEL_EN = VEnDis::_DISABLE);
	 uint8_t accel_compare_samples_ctrl(uint8_t *const Byte = nullptr, VEnDis ACCEL_INTEL_MODE = VEnDis::_DISABLE);
	 
	 // Register 106 – User Control
	 uint8_t fifo_mode_ctrl(uint8_t *const Byte = nullptr,            VEnDis FIFO_EN = VEnDis::_DISABLE);
	 uint8_t i2c_master_if_module_ctrl(uint8_t *const Byte = nullptr, VI2CMasterCtrl I2C_MST_EN = VI2CMasterCtrl::_DISABLE_I2C_MASTER_MODULE);
	 uint8_t interface_mode_ctrl(uint8_t *const Byte = nullptr,       VEnDis I2C_IF_DIS = VEnDis::_DISABLE);
	 uint8_t reset_fifo_module(uint8_t *const Byte = nullptr,         VEnDis FIFO_RST = VEnDis::_DISABLE);
	 uint8_t reset_i2c_master_module(uint8_t *const Byte = nullptr,   VEnDis I2C_MST_RST = VEnDis::_DISABLE);
	 uint8_t reset_all_signal_path(uint8_t *const Byte = nullptr,     VEnDis SIG_COND_RST = VEnDis::_DISABLE);
	 
	 // Register 107 – Power Management 1
	 uint8_t reset_and_restore_internal_regs(uint8_t *const Byte = nullptr, VSetReset H_RESET = VSetReset::_RESET);
	 uint8_t sleep_ctrl(uint8_t *const Byte = nullptr,                      VSetReset SLEEP = VSetReset::_RESET);
	 uint8_t cycle_ctrl(uint8_t *const Byte = nullptr,                      VSetReset CYCLE = VSetReset::_RESET);
	 uint8_t gyro_standby_ctrl(uint8_t *const Byte = nullptr,               VSetReset GYRO_STANDBY = VSetReset::_RESET);
	 uint8_t voltage_generator_ctrl(uint8_t *const Byte = nullptr,          VSetReset PD_PTAT = VSetReset::_RESET);
	 uint8_t clock_src_choise(uint8_t *const Byte = nullptr,                VClockSrcChoise CLKSEL = VClockSrcChoise::_INTERNAL_20MHz);
	 
	 // Register 108 – Power Management 2
    uint8_t accel_x_ctrl(uint8_t *const Byte = nullptr, VEnDis DISABLE_XA = VEnDis::_DISABLE );
    uint8_t accel_y_ctrl(uint8_t *const Byte = nullptr, VEnDis DISABLE_YA = VEnDis::_DISABLE );
    uint8_t accel_z_ctrl(uint8_t *const Byte = nullptr, VEnDis DISABLE_ZA = VEnDis::_DISABLE );
    uint8_t gyro_x_ctrl(uint8_t *const Byte = nullptr,  VEnDis DISABLE_XG = VEnDis::_DISABLE  );
    uint8_t gyro_y_ctrl(uint8_t *const Byte = nullptr,  VEnDis DISABLE_YG = VEnDis::_DISABLE  );
    uint8_t gyro_z_ctrl(uint8_t *const Byte = nullptr,  VEnDis DISABLE_ZG = VEnDis::_DISABLE  );
	 
	 // Register 114 and 115 – FIFO Count Registers
	 uint8_t fifo_high_bits_count(uint8_t *const Byte = nullptr, uint8_t FIFO_CNT = 0);
	 uint8_t fifo_low_bits_count(){return reg_gyro_acc::FIFO_COUNTL;}
	 
	 // Register 116 – FIFO Read Write
	 uint8_t r_w_fifo_data(){return reg_gyro_acc::FIFO_R_W;}
	 
	 // Register 117 – Who Am I
    uint8_t get_who_am_i(){return reg_gyro_acc::WHO_AM_I;} // 0x71
	 
	 // Registers 119, 120, 122, 123, 125, 126 Accelerometer Offset Registers
	 uint8_t accel_upper_x_offset_cancel(){return reg_gyro_acc::XA_OFFSET_H;}
	 uint8_t accel_lower_x_offset_cancel(){return reg_gyro_acc::XA_OFFSET_L;}
	 uint8_t accel_upper_y_offset_cancel(){return reg_gyro_acc::YA_OFFSET_H;}
	 uint8_t accel_lower_y_offset_cancel(){return reg_gyro_acc::YA_OFFSET_L;}
	 uint8_t accel_upper_z_offset_cancel(){return reg_gyro_acc::ZA_OFFSET_H;}
	 uint8_t accel_lower_z_offset_cancel(){return reg_gyro_acc::ZA_OFFSET_L;}
	 
//----- REGISTER MAP FOR MAGNETOMETER ---------------------------------------------------------
	 
	 // WIA: Device ID
	 uint8_t mag_device_id(){return reg_mag::WIA;} // 0x48
	 
	 // INFO: Information
	 uint8_t mag_information(){return reg_mag::INFO;}
	 
	 // ST1: Status 1
	 uint8_t mag_data_ready(const uint8_t Byte = 0,   VStatusFlag *DOR = nullptr);
	 uint8_t mag_data_overrun(const uint8_t Byte = 0, VStatusFlag *DRDY = nullptr);
	 
	 // HXL to HZH: Measurement Data 
	 uint8_t mag_lower_x_data(){return reg_mag::HXL;}
	 uint8_t mag_higher_x_data(){return reg_mag::HXH;}
	 uint8_t mag_lower_y_data(){return reg_mag::HYL;}
	 uint8_t mag_higher_y_data(){return reg_mag::HYH;}
	 uint8_t mag_lower_z_data(){return reg_mag::HZL;}
	 uint8_t mag_higher_z_data(){return reg_mag::HZH;}
	 
	 // ST2: Status 2
	 uint8_t mag_overflow(const uint8_t Byte = 0,        VStatusFlag *HOFL = nullptr);
	 uint8_t mag_mirror_data_bit(const uint8_t Byte = 0, VStatusFlag *BITM = nullptr);
	 
	 // CNTL1: Control 1
	 uint8_t mag_operation_mode_set(uint8_t *const Byte = nullptr, VMagOperationModeSet MODE = VMagOperationModeSet::_POWER_DOWN);
	 uint8_t mag_operation_mode_read(const uint8_t Byte, VMagOperationModeSet *MODE = nullptr);
	 uint8_t mag_output_bit_set(uint8_t *const Byte = nullptr,     VMagOutputBitSet BIT = VMagOutputBitSet::_16_BIT_OUTPUT);
	 uint8_t mag_output_bit_read(const uint8_t Byte, VMagOutputBitSet *BIT = nullptr);
	 
	 // CNTL2: Control 2
	 uint8_t mag_soft_reset_ctrl(uint8_t *const Byte = nullptr, VMagSoftResetCtrl SRST = VMagSoftResetCtrl::_RESET);
	 
	 // ASTC: Self-Test Control
    uint8_t mag_self_test_ctrl(uint8_t *const Byte = nullptr, VMagSelfTestCtrl SELF = VMagSelfTestCtrl::_NORMAL);
	 
	 // I2CDIS: I2C Disable
    uint8_t mag_i2c_disable(uint8_t *const Byte = nullptr); //отключить I2С интерфейс, для повторного включения - 8 раз подряд отправить старт условие
	 
	 // ASAX, ASAY, ASAZ: Sensitivity Adjustment values
	 uint8_t mag_sens_adj_x(){return reg_mag::ASAX;}
	 uint8_t mag_sens_adj_y(){return reg_mag::ASAY;}
	 uint8_t mag_sens_adj_z(){return reg_mag::ASAZ;}
	 
    template<typename T_Data, typename T_Reg>                   //для ro регистров
    void read(const uint8_t Byte, T_Data *Data, T_Reg Reg)
    {
      if (Data != nullptr)
	   {
	     *Data = static_cast<T_Data>(((T_Reg *)&Byte)->Dest);
	   }
    }
  private:

    template<typename T_Mode, typename T_Reg>                  //для r/w регистров
    void operate(uint8_t *const Byte, T_Mode Mode, T_Reg Reg)
    {
      if (Byte != nullptr)
		    {
		      ((T_Reg *)Byte)->Dest = static_cast<uint8_t>(Mode);
		    }
	   }
	 	
//    void fill_in_vector(const TI2CProtocol Src[], std::vector<TI2CProtocol> &Dest); //заполняет вектор элементами массива
//	 const TLogicLevelAD0 Level = {_HIGH};
//-----------------------------------------------------------------------------------------------
    
  };
}

#endif
