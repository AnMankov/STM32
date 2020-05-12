#include "MPU-9250_acc_gyro_mag_driver_ll.h"

namespace MPU_9250
{
//  constexpr TI2CProtocol _ProtocolWrOne[] = {ST, SADW, SAK, SUB, SAK, DATAM, SAK, SP}; 
//  constexpr TI2CProtocol _ProtocolWrMul[] = {ST, SADW, SAK, SUB, SAK, DATAM, SAK, DATAM, SAK, SP}; 
//  constexpr TI2CProtocol _ProtocolRdOne[] = {ST, SADW, SAK, SUB, SAK, SR, SADR, SAK, DATAS, NMAK, SP}; 
//  constexpr TI2CProtocol _ProtocolRdMul[] = {ST, SADW, SAK, SUB, SAK, SR, SADR, SAK, DATAS, MAK, DATAS, NMAK, SP}; 

//  void TAccGyroMagDriver_LL::fill_in_vector(const TI2CProtocol Src[], std::vector<TI2CProtocol> &Dest)
//  {
//    std::vector<TI2CProtocol> Temp(Src, Src + sizeof Src / sizeof Src[0]);
//	   Dest = Temp;
//  }

  TAccGyroMagDriver_LL::TAccGyroMagDriver_LL()
  {  	 
//	   fill_in_vector(_ProtocolWrOne, ProtocolWrOne);
//	   fill_in_vector(_ProtocolWrMul, ProtocolWrMul);
//	   fill_in_vector(_ProtocolRdOne, ProtocolRdOne);
//	   fill_in_vector(_ProtocolRdMul, ProtocolRdMul);
  }

  TAccGyroMagDriver_LL::~TAccGyroMagDriver_LL()
  {
  
  }

  uint8_t TAccGyroMagDriver_LL::get_acc_gyro_addr(TLogicLevelAD0 Level) const
  {
    constexpr uint8_t Addr[] = 
	   {0x68, 0x69};
    
	   return Addr[Level];
  }

  uint8_t TAccGyroMagDriver_LL::get_mag_addr() const
  {
    return 0x0C;  //The slave address for the AK8963 is 0X0C or 12 decimal
  }
//-----------------------------------------------------------------------------------------------
  // Registers 0 to 2 – Gyroscope Self-Test Registers 
  
//  uint8_t TAccGyroMagDriver_LL::get_st_x_gyro(uint8_t *const Byte, VSwitch DRDY_G)
//  {
//    SGyroDataReady Reg;
////    operate(Byte, );
//    return reg_gyro_acc::SELF_TEST_X_GYRO;
//  }

//  uint8_t TAccGyroMagDriver_LL::get_st_y_gyro(uint8_t *const Byte, )
//  {
////    operate(Byte, );
//    return reg_gyro_acc::SELF_TEST_Y_GYRO;
//  }

//  uint8_t TAccGyroMagDriver_LL::get_st_z_gyro(uint8_t *const Byte, )
//  {
////    operate(Byte, );
//    return reg_gyro_acc::SELF_TEST_Z_GYRO;
//  }

//  // Registers 13 to 15 – Accelerometer Self-Test Registers

//  uint8_t TAccGyroMagDriver_LL::get_st_x_accel(uint8_t *const Byte, )
//  {
////    operate(Byte, );
//    return reg_gyro_acc::SELF_TEST_X_ACCEL;
//  }

//  uint8_t TAccGyroMagDriver_LL::get_st_y_accel(uint8_t *const Byte, )
//  {
////    operate(Byte, );
//    return reg_gyro_acc::SELF_TEST_Y_ACCEL;
//  }

//  uint8_t TAccGyroMagDriver_LL::get_st_z_accel(uint8_t *const Byte, )
//  {
////    operate(Byte, );
//    return reg_gyro_acc::SELF_TEST_Z_ACCEL;
//  } 
  
  //  Registers 19 to 24 – Gyro Offset Registers
  
//  uint8_t TAccGyroMagDriver_LL::remove_gyro_bias_x_h(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::XG_OFFSET_H;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::remove_gyro_bias_x_l(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::XG_OFFSET_L;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::remove_gyro_bias_y_h(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::YG_OFFSET_H;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::remove_gyro_bias_y_l(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::YG_OFFSET_L;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::remove_gyro_bias_z_h(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::ZG_OFFSET_H;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::remove_gyro_bias_z_l(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::ZG_OFFSET_L;
//  }
  
  // Register 25 – Sample Rate Divider
  
//  uint8_t TAccGyroMagDriver_LL::sample_rate_divider(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::SMPLRT_DIV;
//  }
  
  // Register 26 – Configuration
  
  uint8_t TAccGyroMagDriver_LL::fifo_mode_config(uint8_t *const Byte, VFifoModeConfig FIFO_MODE)
  {
    TFifoModeConfig Reg;
	  operate(Byte, FIFO_MODE, Reg);
    return reg_gyro_acc::CONFIG;
  }
  
  uint8_t TAccGyroMagDriver_LL::fsync_pin_config(uint8_t *const Byte, VFsyncPinConfig EXT_SYNC_SET)
  {
    TFsyncPinConfig Reg;
	  operate(Byte, EXT_SYNC_SET, Reg);
    return reg_gyro_acc::CONFIG;
  }
  
  uint8_t TAccGyroMagDriver_LL::filter_gyro_temp_config(uint8_t *const Byte, VFilterGyroTempConfig DLPF_CFG)
  {
    TFilterGyroTempConfig Reg;
	  operate(Byte, DLPF_CFG, Reg);
    return reg_gyro_acc::CONFIG;
  }
  
  uint8_t TAccGyroMagDriver_LL::filter_gyro_temp_read(uint8_t Byte, VFilterGyroTempConfig *DLPF_CFG)
  {
    TFilterGyroTempConfig Reg;
    read(Byte, DLPF_CFG, Reg);
    return reg_gyro_acc::CONFIG;
  }
  
  // Register 27 – Gyroscope Configuration
  
  uint8_t TAccGyroMagDriver_LL::enable_x_gyro_self_test(uint8_t *const Byte, VEnDis XGYRO_Cten)
  {
    TEnableXGyroSelfTest Reg;
	 operate(Byte, XGYRO_Cten, Reg);
    return reg_gyro_acc::GYRO_CONFIG;
  }
  
  uint8_t TAccGyroMagDriver_LL::enable_y_gyro_self_test(uint8_t *const Byte, VEnDis YGYRO_Cten)
  {
    TEnableYGyroSelfTest Reg;
	 operate(Byte, YGYRO_Cten, Reg);
    return reg_gyro_acc::GYRO_CONFIG;
  }
  
  uint8_t TAccGyroMagDriver_LL::enable_z_gyro_self_test(uint8_t *const Byte, VEnDis ZGYRO_Cten)
  {
    TEnableZGyroSelfTest Reg;
	 operate(Byte, ZGYRO_Cten, Reg);
    return reg_gyro_acc::GYRO_CONFIG;
  }
  
  uint8_t TAccGyroMagDriver_LL::gyro_full_scale_select(uint8_t *const Byte, VGyroFullScaleSelect GYRO_FS_SEL)
  {
    TGyroFullScaleSelect Reg;
	 operate(Byte, GYRO_FS_SEL, Reg);
    return reg_gyro_acc::GYRO_CONFIG;
  }
    
  uint8_t TAccGyroMagDriver_LL::gyro_full_scale_read(const uint8_t Byte, VGyroFullScaleSelect *GYRO_FS_SEL)
  {
    TGyroFullScaleSelect Reg;
    read(Byte, GYRO_FS_SEL, Reg);
    return reg_gyro_acc::I2C_MST_STATUS;
  }
  
  uint8_t TAccGyroMagDriver_LL::gyro_filter_choise(uint8_t *const Byte, VGyroFilterChoise Fchoice_b)
  {
    TGyroFilterChoise Reg;
	 operate(Byte, Fchoice_b, Reg);
    return reg_gyro_acc::GYRO_CONFIG;
  }
  
  // Register 28 – Accelerometer Configuration
  
  uint8_t TAccGyroMagDriver_LL::enable_x_accel_self_test(uint8_t *const Byte, VEnDis ax_st_en)
  {
    TEnableXAccelSelfTest Reg;
	 operate(Byte, ax_st_en, Reg);
    return reg_gyro_acc::ACCEL_CONFIG;
  }
  
  uint8_t TAccGyroMagDriver_LL::enable_y_accel_self_test(uint8_t *const Byte, VEnDis ay_st_en)
  {
    TEnableYAccelSelfTest Reg;
	 operate(Byte, ay_st_en, Reg);
    return reg_gyro_acc::ACCEL_CONFIG;
  }
  
  uint8_t TAccGyroMagDriver_LL::enable_z_accel_self_test(uint8_t *const Byte, VEnDis az_st_en)
  {
    TEnableZAccelSelfTest Reg;
	 operate(Byte, az_st_en, Reg);
    return reg_gyro_acc::ACCEL_CONFIG;
  }
  
  uint8_t TAccGyroMagDriver_LL::accel_full_scale_select(uint8_t *const Byte, VAccelFullScaleSelect ACCEL_FS_SEL)
  {
    TAccelFullScaleSelect Reg;
	 operate(Byte, ACCEL_FS_SEL, Reg);
    return reg_gyro_acc::ACCEL_CONFIG;
  }
  
  // Register 29 – Accelerometer Configuration 2
  
  uint8_t TAccGyroMagDriver_LL::accel_filter_choise(uint8_t *const Byte, VAccelFilterChoise accel_fchoice_b)
  {
    TAcceFilterChoise Reg;
	 operate(Byte, accel_fchoice_b, Reg);
    return reg_gyro_acc::ACCEL_CONFIG_2;
  }
  
  uint8_t TAccGyroMagDriver_LL::set_accel_filter(uint8_t *const Byte, VSetAccelFilter A_DLPFCFG)
  {
    TSetAccelFilter Reg;
	 operate(Byte, A_DLPFCFG, Reg);
    return reg_gyro_acc::ACCEL_CONFIG_2;
  }
  
  uint8_t TAccGyroMagDriver_LL::accel_filter_read(uint8_t Byte, VSetAccelFilter *A_DLPFCFG)
  {
    TSetAccelFilter Reg;
	 read(Byte, A_DLPFCFG, Reg);
    return reg_gyro_acc::ACCEL_CONFIG_2;
  }
   
  // Register 30 - Low Power Accelerometer ODR Control
  
  uint8_t TAccGyroMagDriver_LL::set_accel_wakeup_freq(uint8_t *const Byte, VSetAccelWakeupFreq lposc_clksel)
  {
    TSetAccelWakeupFreq Reg;
	 operate(Byte, lposc_clksel, Reg);
    return reg_gyro_acc::LP_ACCEL_ODR;
  }
  
  // Register 31 – Wake-on Motion Threshold
  
//  uint8_t TAccGyroMagDriver_LL::set_wakeon_motion_thr(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::WOM_THR;
//  }
  
  // Register 35 – FIFO Enable
  
  uint8_t TAccGyroMagDriver_LL::write_temp_to_fifo(uint8_t *const Byte, VEnDis TEMP_OUT)
  {
    TWrTempToFifo Reg;
	 operate(Byte, TEMP_OUT, Reg);
    return reg_gyro_acc::FIFO_EN;
  }
  
  uint8_t TAccGyroMagDriver_LL::write_gyro_x_to_fifo(uint8_t *const Byte, VEnDis GYRO_XOUT)
  {
    TWrGyroXToFifo Reg;
	 operate(Byte, GYRO_XOUT, Reg);
    return reg_gyro_acc::FIFO_EN;
  }
  
  uint8_t TAccGyroMagDriver_LL::write_gyro_y_to_fifo(uint8_t *const Byte, VEnDis GYRO_YOUT)
  {
    TWrGyroYToFifo Reg;
	 operate(Byte, GYRO_YOUT, Reg);
    return reg_gyro_acc::FIFO_EN;
  }
  
  uint8_t TAccGyroMagDriver_LL::write_gyro_z_to_fifo(uint8_t *const Byte, VEnDis GYRO_ZOUT)
  {
    TWrGyroZToFifo Reg;
	 operate(Byte, GYRO_ZOUT, Reg);
    return reg_gyro_acc::FIFO_EN;
  }
  
  uint8_t TAccGyroMagDriver_LL::write_accel_to_fifo(uint8_t *const Byte, VEnDis ACCEL)
  {
    TWrAccelToFifo Reg;
	 operate(Byte, ACCEL, Reg);
    return reg_gyro_acc::FIFO_EN;
  }
  
  uint8_t TAccGyroMagDriver_LL::write_slave2_to_fifo(uint8_t *const Byte, VEnDis SLV_2)
  {
    TWrSlave2ToFifo Reg;
	 operate(Byte, SLV_2, Reg);
    return reg_gyro_acc::FIFO_EN;
  }
  
  uint8_t TAccGyroMagDriver_LL::write_slave1_to_fifo(uint8_t *const Byte, VEnDis SLV_1)
  {
    TWrSlave1ToFifo Reg;
	 operate(Byte, SLV_1, Reg);
    return reg_gyro_acc::FIFO_EN;
  }
  
  uint8_t TAccGyroMagDriver_LL::write_slave0_to_fifo(uint8_t *const Byte, VEnDis SLV_0)
  {
    TWrSlave0ToFifo Reg;
	 operate(Byte, SLV_0, Reg);
    return reg_gyro_acc::FIFO_EN;
  }
  
  // Register 36 – I2C Master Control
  
  uint8_t TAccGyroMagDriver_LL::multi_master_enable(uint8_t *const Byte, VEnDis MULT_MST_EN)
  {
    TMultiMasterEnable Reg;
	 operate(Byte, MULT_MST_EN, Reg);
    return reg_gyro_acc::I2C_MST_CTRL;
  }
  
  uint8_t TAccGyroMagDriver_LL::delay_data_ready_int(uint8_t *const Byte, VEnDis WAIT_FOR_ES)
  {
    TDelayDataReadyInt Reg;
	 operate(Byte, WAIT_FOR_ES, Reg);
    return reg_gyro_acc::I2C_MST_CTRL;
  }
  
  uint8_t TAccGyroMagDriver_LL::write_data_slv3_to_fifo(uint8_t *const Byte, VEnDis SLV_3_FIFO_EN)
  {
    TWriteDataSlv3ToFifo Reg;
	 operate(Byte, SLV_3_FIFO_EN, Reg);
    return reg_gyro_acc::I2C_MST_CTRL;
  }
  
  uint8_t TAccGyroMagDriver_LL::ctrl_slave_read_transition(uint8_t *const Byte, VEnDis I2C_MST_P_NSR)
  {
    TCtrlSlaveReadTransition Reg;
	 operate(Byte, I2C_MST_P_NSR, Reg);
    return reg_gyro_acc::I2C_MST_CTRL;
  }
  
  uint8_t TAccGyroMagDriver_LL::config_internal_clk(uint8_t *const Byte, VConfigInternalClk I2C_MST_CLK)
  {
    TCfgInternalClk Reg;
    operate(Byte, I2C_MST_CLK, Reg);
    return reg_gyro_acc::I2C_MST_CTRL;
  }
  
  // Register 37 - I2C_SLV0_ADDR
  
  uint8_t TAccGyroMagDriver_LL::slv0_direct_transfer(uint8_t *const Byte, VI2CTransferType I2C_SLV0_RNW)
  {
    TSlv0DirectTransfer Reg;
	 operate(Byte, I2C_SLV0_RNW, Reg);
    return reg_gyro_acc::I2C_SLV0_ADDR;
  }
  
  uint8_t TAccGyroMagDriver_LL::slv0_phy_address(uint8_t *const Byte, const uint8_t Addr_7Bit)
  {
    TSlv0PhyAddress Reg;
	 operate(Byte, Addr_7Bit, Reg);
    return reg_gyro_acc::I2C_SLV0_ADDR;
  }
  
  // Register 38 - I2C_SLV0_REG 
  
//  uint8_t TAccGyroMagDriver_LL::slv0_reg_address_begin(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV0_REG;
//  }
  
  // Register 39 - I2C_SLV0_CTRL
  
  uint8_t TAccGyroMagDriver_LL::slv0_enable(uint8_t *const Byte, VEnDis I2C_SLV0_EN)
  {
    TSlv0Enable Reg;
	 operate(Byte, I2C_SLV0_EN, Reg);
    return reg_gyro_acc::I2C_SLV0_CTRL;
  }
  
  uint8_t TAccGyroMagDriver_LL::slv0_swap_bytes(uint8_t *const Byte, VEnDis I2C_SLV0_BYTE_SW)
  {
    TSlv0SwapBytes Reg;
	 operate(Byte, I2C_SLV0_BYTE_SW, Reg);
    return reg_gyro_acc::I2C_SLV0_CTRL;
  }
  
  uint8_t TAccGyroMagDriver_LL::slv0_write_reg_enable(uint8_t *const Byte, VEnDis I2C_SLV0_REG_DIS)
  {
    TSlv0WriteReg Reg;
	 operate(Byte, I2C_SLV0_REG_DIS, Reg);
    return reg_gyro_acc::I2C_SLV0_CTRL;
  }
  
  uint8_t TAccGyroMagDriver_LL::slv0_group_data(uint8_t *const Byte, VEnDis I2C_SLV0_GRP)
  {
    TSlv0GroupData Reg;
	 operate(Byte, I2C_SLV0_GRP, Reg);
    return reg_gyro_acc::I2C_SLV0_CTRL;
  }
  
  uint8_t TAccGyroMagDriver_LL::slv0_num_bytes_read(uint8_t *const Byte, VSlvNumBytesRead I2C_SLV0_LENG)
  {
    TSlv0NumBytesRead Reg;
	 operate(Byte, I2C_SLV0_LENG, Reg);
    return reg_gyro_acc::I2C_SLV1_CTRL;
  }
//  
//  // Register 40 - I2C_SLV1_ADDR
//  
//  uint8_t TAccGyroMagDriver_LL::slv1_transfer_type(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV1_ADDR;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::slv1_phy_address(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV1_ADDR;
//  }
//  
//  // Register 41 - I2C_SLV1_REG
//  
//  uint8_t TAccGyroMagDriver_LL::slv1_reg_address_begin(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV1_REG;
//  }
//  
//  // Register 42 - I2C_SLV1_CTRL
//  
//  uint8_t TAccGyroMagDriver_LL::slv1_enable(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV1_CTRL;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::slv1_swap_bytes(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV1_CTRL;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::slv1_transaction_ctrl(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV1_CTRL;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::slv1_determine_address(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV1_CTRL;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::slv1_num_bytes_read(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV1_CTRL;
//  }
//  
//  // Register 43 - I2C_SLV2_ADDR
//  
//  uint8_t TAccGyroMagDriver_LL::slv2_transfer_type(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV2_ADDR;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::slv2_phy_address(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV2_ADDR;
//  }
//  
//  // Register 44 - I2C_SLV2_REG
//  
//  uint8_t TAccGyroMagDriver_LL::slv2_reg_address_begin(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV2_REG;
//  }
//  
//  // Register 45 - I2C_SLV2_CTRL
//  
//  uint8_t TAccGyroMagDriver_LL::slv2_enable(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV2_CTRL;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::slv2_swap_bytes(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV2_CTRL;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::slv2_transaction_ctrl(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV2_CTRL;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::slv2_determine_address(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV2_CTRL;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::slv2_num_bytes_read(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV2_CTRL;
//  }
//  
//  // Register 46 - I2C_SLV3_ADDR
//  
//  uint8_t TAccGyroMagDriver_LL::slv3_transfer_type(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV3_ADDR;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::slv3_phy_address(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV3_ADDR;
//  }
//  
//  // Register 47 - I2C_SLV3_REG
//  
//  uint8_t TAccGyroMagDriver_LL::slv3_reg_address_begin(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV3_REG;
//  }
//  
//  // Register 48 - I2C_SLV3_CTRL
//  
//  uint8_t TAccGyroMagDriver_LL::slv3_enable(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV3_CTRL;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::slv3_swap_bytes(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV3_CTRL;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::slv3_transaction_ctrl(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV3_CTRL;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::slv3_determine_address(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV3_CTRL;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::slv3_num_bytes_read(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV3_CTRL;
//  }
//  
//  // Register 49 - I2C_SLV4_ADDR
//  
//  uint8_t TAccGyroMagDriver_LL::slv4_transfer_type(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV4_ADDR;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::slv4_phy_address(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV4_ADDR;
//  }
//  
//  // Register 50 - I2C_SLV4_REG
//  
//  uint8_t TAccGyroMagDriver_LL::slv4_reg_address_begin(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV4_REG;
//  }
//  
//  // Register 51 - I2C_SLV4_DO
//  
//  uint8_t TAccGyroMagDriver_LL::slv4_data_wr(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV4_DO;
//  }
//  
//  // Register 52 - I2C_SLV4_CTRL
//  
//  uint8_t TAccGyroMagDriver_LL::slv4_enable(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV4_CTRL;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::slv4_complete_and_int(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV4_CTRL;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::slv4_transaction_ctrl(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV4_CTRL;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::num_samples_ctrl(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV4_CTRL;
//  }
//  
//  // Register 53 - I2C_SLV4_DI
//  
//  uint8_t TAccGyroMagDriver_LL::slv4_data_rd(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV4_DI;
//  }
//  
  // Register 54 – I2C Master Status
  
  uint8_t TAccGyroMagDriver_LL::status_fsync_interrupt(const uint8_t Byte, VStatusFlag *PASS_THROUGH)
  {
    TStatusFsyncInterrupt Reg;
    read(Byte, PASS_THROUGH, Reg);
    return reg_gyro_acc::I2C_MST_STATUS;
  }
  
  uint8_t TAccGyroMagDriver_LL::slv4_transfer_complete(const uint8_t Byte, VStatusFlag *I2C_SLV4_DONE)
  {
    TSlv4TransferComplete Reg;
    read(Byte, I2C_SLV4_DONE, Reg);
    return reg_gyro_acc::I2C_MST_STATUS;
  }
  
  uint8_t TAccGyroMagDriver_LL::slv_lost_arbitration(const uint8_t Byte, VStatusFlag *I2C_LOST_ARB)
  {
    TSlvLostArbitration Reg;
    read(Byte, I2C_LOST_ARB, Reg);
    return reg_gyro_acc::I2C_MST_STATUS;
  }
  
  uint8_t TAccGyroMagDriver_LL::slv4_rx_nack(const uint8_t Byte, VStatusFlag *I2C_SLV4_NACK)
  {
    TSlv4RxNack Reg;
    read(Byte, I2C_SLV4_NACK, Reg);
    return reg_gyro_acc::I2C_MST_STATUS;
  }
  
  uint8_t TAccGyroMagDriver_LL::slv3_rx_nack(const uint8_t Byte, VStatusFlag *I2C_SLV3_NACK)
  {
    TSlv3RxNack Reg;
    read(Byte, I2C_SLV3_NACK, Reg);
    return reg_gyro_acc::I2C_MST_STATUS;
  }
  
  uint8_t TAccGyroMagDriver_LL::slv2_rx_nack(const uint8_t Byte, VStatusFlag *I2C_SLV2_NACK)
  {
    TSlv2RxNack Reg;
    read(Byte, I2C_SLV2_NACK, Reg);
    return reg_gyro_acc::I2C_MST_STATUS;
  }
  
  uint8_t TAccGyroMagDriver_LL::slv1_rx_nack(const uint8_t Byte, VStatusFlag *I2C_SLV1_NACK)
  {
    TSlv1RxNack Reg;
    read(Byte, I2C_SLV1_NACK, Reg);
    return reg_gyro_acc::I2C_MST_STATUS;
  }
  
  uint8_t TAccGyroMagDriver_LL::slv0_rx_nack(const uint8_t Byte, VStatusFlag *I2C_SLV0_NACK)
  {
    TSlv0RxNack Reg;
    read(Byte, I2C_SLV0_NACK, Reg);
    return reg_gyro_acc::I2C_MST_STATUS;
  }
  
  // Register 55 – INT Pin / Bypass Enable Configuration
  
  uint8_t TAccGyroMagDriver_LL::int_logic_level(uint8_t *const Byte, VLogicLevelINT ACTL)
  {
    TIntLogicLevel Reg;
    operate(Byte, ACTL, Reg);
    return reg_gyro_acc::INT_PIN_CFG;
  }
  
  uint8_t TAccGyroMagDriver_LL::int_pin_config(uint8_t *const Byte, VPinConfigINT OPEN)
  {
    TIntPinConfig Reg;
    operate(Byte, OPEN, Reg);
    return reg_gyro_acc::INT_PIN_CFG;
  }
  
  uint8_t TAccGyroMagDriver_LL::int_latch_ctrl(uint8_t *const Byte, VLatchINT LATCH_INT_EN)
  {
    TIntLatchCtrl Reg;
    operate(Byte, LATCH_INT_EN, Reg);
    return reg_gyro_acc::INT_PIN_CFG;
  }
  
  uint8_t TAccGyroMagDriver_LL::int_status_ctrl(uint8_t *const Byte, VEnDis INT_ANYRD_2CLEAR)
  {
    TIntStatusCtrl Reg;
    operate(Byte, INT_ANYRD_2CLEAR, Reg);
    return reg_gyro_acc::INT_PIN_CFG;
  }
  
  uint8_t TAccGyroMagDriver_LL::fsync_level_ctrl(uint8_t *const Byte, VEnDis ACTL_FSYNC)
  {
    TFsyncLevelCtrl Reg;
    operate(Byte, ACTL_FSYNC, Reg);
    return reg_gyro_acc::INT_PIN_CFG;
  }
  
  uint8_t TAccGyroMagDriver_LL::fsync_enable(uint8_t *const Byte, VEnDis FSYNC_INT_MODE_EN)
  {
    TFsyncEnable Reg;
    operate(Byte, FSYNC_INT_MODE_EN, Reg);
    return reg_gyro_acc::INT_PIN_CFG;
  }
  
  uint8_t TAccGyroMagDriver_LL::i2c_master_pins_ctrl(uint8_t *const Byte, VEnDis BYPASS_EN)
  {
    TI2CMasterPinsCtrl Reg;
    operate(Byte, BYPASS_EN, Reg);
    return reg_gyro_acc::INT_PIN_CFG;
  }
  
  uint8_t TAccGyroMagDriver_LL::i2c_master_pins_read(const uint8_t Byte, VEnDis *BYPASS_EN)
  {
    TI2CMasterPinsCtrl Reg;
    read(Byte, BYPASS_EN, Reg);
    return reg_gyro_acc::INT_PIN_CFG;
  }
  
  // Register 56 – Interrupt Enable
  
  uint8_t TAccGyroMagDriver_LL::wake_on_motion_int_ctrl(uint8_t *const Byte, VEnDis WOM_EN)
  {
    TWakeOnMotionIntCtrl Reg;
    operate(Byte, WOM_EN, Reg);
    return reg_gyro_acc::INT_ENABLE;
  }
  
  uint8_t TAccGyroMagDriver_LL::fifo_ovrf_int_ctrl(uint8_t *const Byte, VEnDis FIFO_OVERFLOW_EN)
  {
    TFifoOvrfIntCtrl Reg;
    operate(Byte, FIFO_OVERFLOW_EN, Reg);
    return reg_gyro_acc::INT_ENABLE;
  }
  
  uint8_t TAccGyroMagDriver_LL::fsync_int_ctrl(uint8_t *const Byte, VEnDis FSYNC_INT_EN)
  {
    TFsyncIntCtrl Reg;
    operate(Byte, FSYNC_INT_EN, Reg);
    return reg_gyro_acc::INT_ENABLE;
  }
  
  uint8_t TAccGyroMagDriver_LL::raw_data_ready_int_ctrl(uint8_t *const Byte, VEnDis RAW_RDY_EN)
  {
    TRawDataReadyIntCtrl Reg;
    operate(Byte, RAW_RDY_EN, Reg);
    return reg_gyro_acc::INT_ENABLE;
  }
  
  // Register 58 – Interrupt Status
  
  uint8_t TAccGyroMagDriver_LL::wake_on_motion_int_status(const uint8_t Byte, VStatusFlag *WOM_INT)
  {
    TWakeOnMotionIntStatus Reg;
    read(Byte, WOM_INT, Reg);
    return reg_gyro_acc::INT_STATUS;
  }
  
  uint8_t TAccGyroMagDriver_LL::fifo_ovrf_int_status(const uint8_t Byte, VStatusFlag *FIFO_OVERFLOW_INT)
  {
    TFifoOvrfIntStatus Reg;
    read(Byte, FIFO_OVERFLOW_INT, Reg);
    return reg_gyro_acc::INT_STATUS;
  }
  
  uint8_t TAccGyroMagDriver_LL::fsync_int_status(const uint8_t Byte, VStatusFlag *FSYNC_INT)
  {
    TFsyncIntStatus Reg;
    read(Byte, FSYNC_INT, Reg);
    return reg_gyro_acc::INT_STATUS;
  }
  
  uint8_t TAccGyroMagDriver_LL::raw_data_ready_int_status(const uint8_t Byte, VStatusFlag *RAW_DATA_RDY_INT)
  {
    TRawDataReadyIntStatus Reg;
    read(Byte, RAW_DATA_RDY_INT, Reg);
    return reg_gyro_acc::INT_STATUS;
  }
  
  // Registers 59 to 64 – Accelerometer Measurements
  
//  uint8_t TAccGyroMagDriver_LL::accel_data_x_h(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::ACCEL_XOUT_H;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::accel_data_x_l(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::ACCEL_XOUT_L;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::accel_data_y_h(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::ACCEL_YOUT_H;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::accel_data_y_l(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::ACCEL_YOUT_L;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::accel_data_z_h(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::ACCEL_ZOUT_H;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::accel_data_z_l(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::ACCEL_ZOUT_L;
//  }
//  
  // Registers 65 and 66 – Temperature Measurement
  
//  uint8_t TAccGyroMagDriver_LL::temp_data_h(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::TEMP_OUT_H;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::temp_data_l(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::TEMP_OUT_L;
//  }
//  
  // Registers 67 to 72 – Gyroscope Measurements
  
//  uint8_t TAccGyroMagDriver_LL::gyro_data_x_h(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::GYRO_XOUT_H;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::gyro_data_x_l(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::GYRO_XOUT_L;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::gyro_data_y_h(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::GYRO_YOUT_H;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::gyro_data_y_l(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::GYRO_YOUT_L;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::gyro_data_z_h(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::GYRO_ZOUT_H;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::gyro_data_z_l(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::GYRO_ZOUT_L;
//  }
  
  // Registers 73 to 96 – External Sensor Data
  
//  uint8_t TAccGyroMagDriver_LL::ext_sens_data_0(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::EXT_SENS_DATA_00;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::ext_sens_data_1(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::EXT_SENS_DATA_01;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::ext_sens_data_2(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::EXT_SENS_DATA_02;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::ext_sens_data_3(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::EXT_SENS_DATA_03;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::ext_sens_data_4(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::EXT_SENS_DATA_04;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::ext_sens_data_5(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::EXT_SENS_DATA_05;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::ext_sens_data_6(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::EXT_SENS_DATA_06;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::ext_sens_data_7(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::EXT_SENS_DATA_07;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::ext_sens_data_8(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::EXT_SENS_DATA_08;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::ext_sens_data_9(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::EXT_SENS_DATA_09;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::ext_sens_data_10(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::EXT_SENS_DATA_10;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::ext_sens_data_11(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::EXT_SENS_DATA_11;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::ext_sens_data_12(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::EXT_SENS_DATA_12;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::ext_sens_data_13(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::EXT_SENS_DATA_13;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::ext_sens_data_14(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::EXT_SENS_DATA_14;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::ext_sens_data_15(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::EXT_SENS_DATA_15;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::ext_sens_data_16(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::EXT_SENS_DATA_16;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::ext_sens_data_17(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::EXT_SENS_DATA_17;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::ext_sens_data_18(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::EXT_SENS_DATA_18;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::ext_sens_data_19(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::EXT_SENS_DATA_19;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::ext_sens_data_20(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::EXT_SENS_DATA_20;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::ext_sens_data_21(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::EXT_SENS_DATA_21;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::ext_sens_data_22(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::EXT_SENS_DATA_22;
//  }
//
//  uint8_t TAccGyroMagDriver_LL::ext_sens_data_23(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::EXT_SENS_DATA_23;
//  }
  
  // Register 99 – I2C Slave 0 Data Out
  
//  uint8_t TAccGyroMagDriver_LL::data_out_slv0_wr(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV0_DO;
//  }
//  
//  // Register 100 – I2C Slave 1 Data Out
//  
//  uint8_t TAccGyroMagDriver_LL::data_out_slv1_wr(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV1_DO;
//  }
//  
//  // Register 101 – I2C Slave 2 Data Out
//  
//  uint8_t TAccGyroMagDriver_LL::data_out_slv2_wr(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV2_DO;
//  }
//  
//  // Register 102 – I2C Slave 3 Data Out
//  
//  uint8_t TAccGyroMagDriver_LL::data_out_slv3_wr(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::I2C_SLV3_DO;
//  }
  
  // Register 103 – I2C Master Delay Control
  
  uint8_t TAccGyroMagDriver_LL::delay_shadow_ext_sens_ctrl(uint8_t *const Byte, VEnDis DELAY_ES_SHADOW)
  {
    TDelayShadowExtSense Reg;
    operate(Byte, DELAY_ES_SHADOW, Reg);
    return reg_gyro_acc::I2C_MST_DELAY_CTRL;
  }
  
  uint8_t TAccGyroMagDriver_LL::slv4_dly_access_ctrl(uint8_t *const Byte, VEnDis I2C_SLV4_DLY_EN)
  {
    TSlv4DlyAccessCtrl Reg;
    operate(Byte, I2C_SLV4_DLY_EN, Reg);
    return reg_gyro_acc::I2C_MST_DELAY_CTRL;
  }
  
  uint8_t TAccGyroMagDriver_LL::slv3_dly_access_ctrl(uint8_t *const Byte, VEnDis I2C_SLV3_DLY_EN)
  {
    TSlv3DlyAccessCtrl Reg;
    operate(Byte, I2C_SLV3_DLY_EN, Reg);
    return reg_gyro_acc::I2C_MST_DELAY_CTRL;
  }
  
  uint8_t TAccGyroMagDriver_LL::slv2_dly_access_ctrl(uint8_t *const Byte, VEnDis I2C_SLV2_DLY_EN)
  {
    TSlv2DlyAccessCtrl Reg;
    operate(Byte, I2C_SLV2_DLY_EN, Reg);
    return reg_gyro_acc::I2C_MST_DELAY_CTRL;
  }
  
  uint8_t TAccGyroMagDriver_LL::slv1_dly_access_ctrl(uint8_t *const Byte, VEnDis I2C_SLV1_DLY_EN)
  {
    TSlv1DlyAccessCtrl Reg;
    operate(Byte, I2C_SLV1_DLY_EN, Reg);
    return reg_gyro_acc::I2C_MST_DELAY_CTRL;
  }
  
  uint8_t TAccGyroMagDriver_LL::slv0_dly_access_ctrl(uint8_t *const Byte, VEnDis I2C_SLV0_DLY_EN)
  {
    TSlv0DlyAccessCtrl Reg;
    operate(Byte, I2C_SLV0_DLY_EN, Reg);
    return reg_gyro_acc::I2C_MST_DELAY_CTRL;
  }
  
  // Register 104 – Signal Path Reset
  
  uint8_t TAccGyroMagDriver_LL::reset_gyro_signal_path(uint8_t *const Byte, VEnDis GYRO_RST)
  {
    TResetGyroSignalPath Reg;
    operate(Byte, GYRO_RST, Reg);
    return reg_gyro_acc::SIGNAL_PATH_RESET;
  }
  
  uint8_t TAccGyroMagDriver_LL::reset_accel_signal_path(uint8_t *const Byte, VEnDis ACCEL_RST)
  {
    TResetAccelSignalPath Reg;
    operate(Byte, ACCEL_RST, Reg);
    return reg_gyro_acc::SIGNAL_PATH_RESET;
  }
  
  uint8_t TAccGyroMagDriver_LL::reset_temp_signal_path(uint8_t *const Byte, VEnDis TEMP_RST)
  {
    TResetTempSignalPath Reg;
    operate(Byte, TEMP_RST, Reg);
    return reg_gyro_acc::SIGNAL_PATH_RESET;
  }
  
  // Register 105 – Accelerometer Interrupt Control
  
  uint8_t TAccGyroMagDriver_LL::accel_wake_on_motion_ctrl(uint8_t *const Byte, VEnDis ACCEL_INTEL_EN)
  {
    TAccelWakeOnMotionCtrl Reg;
    operate(Byte, ACCEL_INTEL_EN, Reg);
    return reg_gyro_acc::MOT_DETECT_CTRL;
  }
  
  uint8_t TAccGyroMagDriver_LL::accel_compare_samples_ctrl(uint8_t *const Byte, VEnDis ACCEL_INTEL_MODE)
  {
    TAccelCompareSamplesCtrl Reg;
    operate(Byte, ACCEL_INTEL_MODE, Reg);
    return reg_gyro_acc::MOT_DETECT_CTRL;
  }
  
  // Register 106 – User Control
  
  uint8_t TAccGyroMagDriver_LL::fifo_mode_ctrl(uint8_t *const Byte, VEnDis FIFO_EN)
  {
    TFifoModeCtrl Reg;
    operate(Byte, FIFO_EN, Reg);
    return reg_gyro_acc::USER_CTRL;
  }
  
  uint8_t TAccGyroMagDriver_LL::i2c_master_if_module_ctrl(uint8_t *const Byte, VI2CMasterCtrl I2C_MST_EN)
  {
    TI2CMasterIFModuleCtrl Reg;
    operate(Byte, I2C_MST_EN, Reg);
    return reg_gyro_acc::USER_CTRL;
  }
  
  uint8_t TAccGyroMagDriver_LL::interface_mode_ctrl(uint8_t *const Byte, VEnDis I2C_IF_DIS)
  {
    TInterfaceModeCtrl Reg;
    operate(Byte, I2C_IF_DIS, Reg);
    return reg_gyro_acc::USER_CTRL;
  }
  
  uint8_t TAccGyroMagDriver_LL::reset_fifo_module(uint8_t *const Byte, VEnDis FIFO_RST)
  {
    TResetFifoModule Reg;
    operate(Byte, FIFO_RST, Reg);
    return reg_gyro_acc::USER_CTRL;
  }
  
  uint8_t TAccGyroMagDriver_LL::reset_i2c_master_module(uint8_t *const Byte, VEnDis I2C_MST_RST)
  {
    TResetI2CMasterModule Reg;
    operate(Byte, I2C_MST_RST, Reg);
    return reg_gyro_acc::USER_CTRL;
  }
  
  uint8_t TAccGyroMagDriver_LL::reset_all_signal_path(uint8_t *const Byte, VEnDis SIG_COND_RST)
  {
    TResetAllSignalPath Reg;
    operate(Byte, SIG_COND_RST, Reg);
    return reg_gyro_acc::USER_CTRL;
  }
  
  // Register 107 – Power Management 1  
  
  uint8_t TAccGyroMagDriver_LL::reset_and_restore_internal_regs(uint8_t *const Byte, VSetReset H_RESET)
  {
    TResetAndRestoreInternalRegs Reg;
    operate(Byte, H_RESET, Reg);
    return reg_gyro_acc::PWR_MGMT_1;
  }
  
  uint8_t TAccGyroMagDriver_LL::sleep_ctrl(uint8_t *const Byte, VSetReset SLEEP)
  {
    TSleepCtrl Reg;
    operate(Byte, SLEEP, Reg);
    return reg_gyro_acc::PWR_MGMT_1;
  }
  
  uint8_t TAccGyroMagDriver_LL::cycle_ctrl(uint8_t *const Byte, VSetReset CYCLE)
  {
    TCycleCtrl Reg;
    operate(Byte, CYCLE, Reg);
    return reg_gyro_acc::PWR_MGMT_1;
  }
  
  uint8_t TAccGyroMagDriver_LL::gyro_standby_ctrl(uint8_t *const Byte, VSetReset GYRO_STANDBY)
  {
    TGyroStandbyCtrl Reg;
    operate(Byte, GYRO_STANDBY, Reg);
    return reg_gyro_acc::PWR_MGMT_1;
  }
  
  uint8_t TAccGyroMagDriver_LL::voltage_generator_ctrl(uint8_t *const Byte, VSetReset PD_PTAT)
  {
    TVoltageGeneratorCtrl Reg;
    operate(Byte, PD_PTAT, Reg);
    return reg_gyro_acc::PWR_MGMT_1;
  }
  
  uint8_t TAccGyroMagDriver_LL::clock_src_choise(uint8_t *const Byte, VClockSrcChoise CLKSEL)
  {
    TClockSrcChoise Reg;
    operate(Byte, CLKSEL, Reg);
    return reg_gyro_acc::PWR_MGMT_1;
  }
  
  // Register 108 – Power Management 2
  
  uint8_t TAccGyroMagDriver_LL::accel_x_ctrl(uint8_t *const Byte, VEnDis DISABLE_XA)
  {
    TAccelXCtrl Reg;
    operate(Byte, DISABLE_XA, Reg);
    return reg_gyro_acc::PWR_MGMT_2;
  }
  
  uint8_t TAccGyroMagDriver_LL::accel_y_ctrl(uint8_t *const Byte, VEnDis DISABLE_YA)
  {
    TAccelYCtrl Reg;
    operate(Byte, DISABLE_YA, Reg);
    return reg_gyro_acc::PWR_MGMT_2;
  }
  
  uint8_t TAccGyroMagDriver_LL::accel_z_ctrl(uint8_t *const Byte, VEnDis DISABLE_ZA)
  {
    TAccelZCtrl Reg;
    operate(Byte, DISABLE_ZA, Reg);
    return reg_gyro_acc::PWR_MGMT_2;
  }
  
  uint8_t TAccGyroMagDriver_LL::gyro_x_ctrl(uint8_t *const Byte, VEnDis DISABLE_XG)
  {
    TGyroXCtrl Reg;
    operate(Byte, DISABLE_XG, Reg);
    return reg_gyro_acc::PWR_MGMT_2;
  }
  
  uint8_t TAccGyroMagDriver_LL::gyro_y_ctrl(uint8_t *const Byte, VEnDis DISABLE_YG)
  {
    TGyroYCtrl Reg;
    operate(Byte, DISABLE_YG, Reg);
    return reg_gyro_acc::PWR_MGMT_2;
  }
  
  uint8_t TAccGyroMagDriver_LL::gyro_z_ctrl(uint8_t *const Byte, VEnDis DISABLE_ZG)
  {
    TGyroZCtrl Reg;
    operate(Byte, DISABLE_ZG, Reg);
    return reg_gyro_acc::PWR_MGMT_2;
  }
  
  // Register 114 and 115 – FIFO Count Registers
  
  uint8_t TAccGyroMagDriver_LL::fifo_high_bits_count(uint8_t *const Byte, uint8_t FIFO_CNT)
  {
    TFifoHighBits Reg;
    operate(Byte, FIFO_CNT, Reg);
    return reg_gyro_acc::FIFO_COUNTH;
  }
  
//  uint8_t TAccGyroMagDriver_LL::fifo_low_bits_count(uint8_t *const Byte, uint8_t FIFO_CNT)
//  {
//     Reg;
//    operate(Byte, FIFO_CNT, Reg);
//    return reg_gyro_acc::FIFO_COUNTL;
//  }
  
  // Register 116 – FIFO Read Write
  
//  uint8_t TAccGyroMagDriver_LL::r_w_fifo_data(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::FIFO_R_W;
//  }
  
  // Register 117 – Who Am I
  
//  uint8_t TAccGyroMagDriver_LL::get_who_am_i(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::WHO_AM_I;
//  } // 0x71
  
  // Registers 119, 120, 122, 123, 125, 126 Accelerometer Offset Registers
  
//  uint8_t TAccGyroMagDriver_LL::accel_upper_x_offset_cancel(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::XA_OFFSET_H;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::accel_lower_x_offset_cancel(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::XA_OFFSET_L;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::accel_upper_y_offset_cancel(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::YA_OFFSET_H;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::accel_lower_y_offset_cancel(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::YA_OFFSET_L;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::accel_upper_z_offset_cancel(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::ZA_OFFSET_H;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::accel_lower_z_offset_cancel(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_gyro_acc::ZA_OFFSET_L;
//  }
  
  // REGISTER MAP FOR MAGNETOMETER
  
  // WIA: Device ID
  
//  uint8_t TAccGyroMagDriver_LL::mag_device_id(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_mag::WIA;
//  } // 0x48
//  
//  // INFO: Information
//  
//  uint8_t TAccGyroMagDriver_LL::mag_information(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_mag::INFO;
//  }
  
  // ST1: Status 1
  
  uint8_t TAccGyroMagDriver_LL::mag_data_ready(const uint8_t Byte, VStatusFlag *DOR)
  {
    TDataReady Reg;
	 read(Byte, DOR, Reg);
    return reg_mag::ST1;
  }
  
  uint8_t TAccGyroMagDriver_LL::mag_data_overrun(const uint8_t Byte, VStatusFlag *DRDY)
  {
    TDataOverrun Reg;
    read(Byte, DRDY, Reg);
    return reg_mag::ST1;
  }
  
// HXL to HZH: Measurement Data
//  
//  uint8_t TAccGyroMagDriver_LL::mag_lower_x_data(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_mag::HXL;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::mag_higher_x_data(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_mag::HXH;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::mag_lower_y_data(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_mag::HYL;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::mag_higher_y_data(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_mag::HYH;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::mag_lower_z_data(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_mag::HZL;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::mag_higher_z_data(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_mag::HZH;
//  }
  
  // ST2: Status 2
  
  uint8_t TAccGyroMagDriver_LL::mag_overflow(const uint8_t Byte, VStatusFlag *HOFL)
  {
	 TMagSensorOvrf Reg;
	 read(Byte, HOFL, Reg);
    return reg_mag::ST2;
  }
  
  uint8_t TAccGyroMagDriver_LL::mag_mirror_data_bit(const uint8_t Byte, VStatusFlag *BITM)
  {
    TOutBitSetting Reg;
	 read(Byte, BITM, Reg);
    return reg_mag::ST2;
  }
  
  // CNTL1: Control 1
  
  uint8_t TAccGyroMagDriver_LL::mag_operation_mode_set(uint8_t *const Byte, VMagOperationModeSet MODE)
  {
    TMagMode Reg;
    operate(Byte, MODE, Reg);
    return reg_mag::CNTL1;
  }
  
  uint8_t TAccGyroMagDriver_LL::mag_operation_mode_read(const uint8_t Byte, VMagOperationModeSet *MODE)
  {
    TMagMode Reg;
    read(Byte, MODE, Reg);
    return reg_mag::CNTL1;
  }
  
  uint8_t TAccGyroMagDriver_LL::mag_output_bit_set(uint8_t *const Byte, VMagOutputBitSet BIT)
  {
    TMagBit Reg;
    operate(Byte, BIT, Reg);
    return reg_mag::CNTL1;
  }
  
  uint8_t TAccGyroMagDriver_LL::mag_output_bit_read(const uint8_t Byte, VMagOutputBitSet *BIT)
  {
    TMagBit Reg;
    read(Byte, BIT, Reg);
    return reg_mag::CNTL1;
  }
  
  // CNTL2: Control 2
  
  uint8_t TAccGyroMagDriver_LL::mag_soft_reset_ctrl(uint8_t *const Byte, VMagSoftResetCtrl SRST)
  {
    TSoftReset Reg;
    operate(Byte, SRST, Reg);
    return reg_mag::CNTL2;
  }
  
  // ASTC: Self-Test Control
  
  uint8_t TAccGyroMagDriver_LL::mag_self_test_ctrl(uint8_t *const Byte, VMagSelfTestCtrl SELF)
  {
    TSelfTestControl Reg;
    operate(Byte, SELF, Reg);
    return reg_mag::ASTC;
  }
  
  // I2CDIS: I2C Disable
  
  uint8_t TAccGyroMagDriver_LL::mag_i2c_disable(uint8_t *const Byte)
  {	 
    *Byte = 0x1B;
    return reg_mag::I2CDIS;
  }
  
  // ASAX, ASAY, ASAZ: Sensitivity Adjustment values
//  
//  uint8_t TAccGyroMagDriver_LL::mag_sens_adj_x(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_mag::ASAX;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::mag_sens_adj_y(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_mag::ASAY;
//  }
//  
//  uint8_t TAccGyroMagDriver_LL::mag_sens_adj_z(uint8_t *const Byte, )
//  {
//    operate(Byte, );
//    return reg_mag::ASAZ;
//  }
}
