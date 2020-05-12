#ifndef __MPU_9250_REGISTER_MAP_H
#define __MPU_9250_REGISTER_MAP_H

#include <cstdint>

namespace MPU_9250
{
  using std::uint8_t;
  using std::uint16_t;
  
  namespace reg_gyro_acc
  {
    const uint8_t SELF_TEST_X_GYRO   = uint8_t(0x00);
    const uint8_t SELF_TEST_Y_GYRO   = uint8_t(0x01);
    const uint8_t SELF_TEST_Z_GYRO   = uint8_t(0x02);
    const uint8_t SELF_TEST_X_ACCEL  = uint8_t(0x0D);
    const uint8_t SELF_TEST_Y_ACCEL  = uint8_t(0x0E);
    const uint8_t SELF_TEST_Z_ACCEL  = uint8_t(0x0F);
    const uint8_t XG_OFFSET_H        = uint8_t(0x13);
    const uint8_t XG_OFFSET_L        = uint8_t(0x14);
    const uint8_t YG_OFFSET_H        = uint8_t(0x15);
    const uint8_t YG_OFFSET_L        = uint8_t(0x16);
    const uint8_t ZG_OFFSET_H        = uint8_t(0x17);
    const uint8_t ZG_OFFSET_L        = uint8_t(0x18);
    const uint8_t SMPLRT_DIV         = uint8_t(0x19);
    const uint8_t CONFIG             = uint8_t(0x1A);
    const uint8_t GYRO_CONFIG        = uint8_t(0x1B);
    const uint8_t ACCEL_CONFIG       = uint8_t(0x1C);
    const uint8_t ACCEL_CONFIG_2     = uint8_t(0x1D);
    const uint8_t LP_ACCEL_ODR       = uint8_t(0x1E);
    const uint8_t WOM_THR            = uint8_t(0x1F);
    const uint8_t FIFO_EN            = uint8_t(0x23);
    const uint8_t I2C_MST_CTRL       = uint8_t(0x24);
    const uint8_t I2C_SLV0_ADDR      = uint8_t(0x25);
    const uint8_t I2C_SLV0_REG       = uint8_t(0x26);
    const uint8_t I2C_SLV0_CTRL      = uint8_t(0x27);
    const uint8_t I2C_SLV1_ADDR      = uint8_t(0x28);
    const uint8_t I2C_SLV1_REG       = uint8_t(0x29);
    const uint8_t I2C_SLV1_CTRL      = uint8_t(0x2A);
    const uint8_t I2C_SLV2_ADDR      = uint8_t(0x2B);
    const uint8_t I2C_SLV2_REG       = uint8_t(0x2C);
    const uint8_t I2C_SLV2_CTRL      = uint8_t(0x2D);
    const uint8_t I2C_SLV3_ADDR      = uint8_t(0x2E);
    const uint8_t I2C_SLV3_REG       = uint8_t(0x2F);
    const uint8_t I2C_SLV3_CTRL      = uint8_t(0x30);
    const uint8_t I2C_SLV4_ADDR      = uint8_t(0x31);
    const uint8_t I2C_SLV4_REG       = uint8_t(0x32);
    const uint8_t I2C_SLV4_DO        = uint8_t(0x33);
    const uint8_t I2C_SLV4_CTRL      = uint8_t(0x34);
    const uint8_t I2C_SLV4_DI        = uint8_t(0x35);
    const uint8_t I2C_MST_STATUS     = uint8_t(0x36);
    const uint8_t INT_PIN_CFG        = uint8_t(0x37);
    const uint8_t INT_ENABLE         = uint8_t(0x38);
    const uint8_t INT_STATUS         = uint8_t(0x3A);
    const uint8_t ACCEL_XOUT_H       = uint8_t(0x3B);
    const uint8_t ACCEL_XOUT_L       = uint8_t(0x3C);
    const uint8_t ACCEL_YOUT_H       = uint8_t(0x3D);
    const uint8_t ACCEL_YOUT_L       = uint8_t(0x3E);
    const uint8_t ACCEL_ZOUT_H       = uint8_t(0x3F);
    const uint8_t ACCEL_ZOUT_L       = uint8_t(0x40);
    const uint8_t TEMP_OUT_H         = uint8_t(0x41);
    const uint8_t TEMP_OUT_L         = uint8_t(0x42);
    const uint8_t GYRO_XOUT_H        = uint8_t(0x43);
    const uint8_t GYRO_XOUT_L        = uint8_t(0x44);
    const uint8_t GYRO_YOUT_H        = uint8_t(0x45);
    const uint8_t GYRO_YOUT_L        = uint8_t(0x46);
    const uint8_t GYRO_ZOUT_H        = uint8_t(0x47);
    const uint8_t GYRO_ZOUT_L        = uint8_t(0x48);
    const uint8_t EXT_SENS_DATA_00   = uint8_t(0x49);
    const uint8_t EXT_SENS_DATA_01   = uint8_t(0x4A);
    const uint8_t EXT_SENS_DATA_02   = uint8_t(0x4B);
    const uint8_t EXT_SENS_DATA_03   = uint8_t(0x4C);
    const uint8_t EXT_SENS_DATA_04   = uint8_t(0x4D);
    const uint8_t EXT_SENS_DATA_05   = uint8_t(0x4E);
    const uint8_t EXT_SENS_DATA_06   = uint8_t(0x4F);
    const uint8_t EXT_SENS_DATA_07   = uint8_t(0x50);
    const uint8_t EXT_SENS_DATA_08   = uint8_t(0x51);
    const uint8_t EXT_SENS_DATA_09   = uint8_t(0x52);
    const uint8_t EXT_SENS_DATA_10   = uint8_t(0x53);
    const uint8_t EXT_SENS_DATA_11   = uint8_t(0x54);
    const uint8_t EXT_SENS_DATA_12   = uint8_t(0x55);
    const uint8_t EXT_SENS_DATA_13   = uint8_t(0x56);
    const uint8_t EXT_SENS_DATA_14   = uint8_t(0x57);
    const uint8_t EXT_SENS_DATA_15   = uint8_t(0x58);
    const uint8_t EXT_SENS_DATA_16   = uint8_t(0x59);
    const uint8_t EXT_SENS_DATA_17   = uint8_t(0x5A);
    const uint8_t EXT_SENS_DATA_18   = uint8_t(0x5B);
    const uint8_t EXT_SENS_DATA_19   = uint8_t(0x5C);
    const uint8_t EXT_SENS_DATA_20   = uint8_t(0x5D);
    const uint8_t EXT_SENS_DATA_21   = uint8_t(0x5E);
    const uint8_t EXT_SENS_DATA_22   = uint8_t(0x5F);
    const uint8_t EXT_SENS_DATA_23   = uint8_t(0x60);
    const uint8_t I2C_SLV0_DO        = uint8_t(0x63);
    const uint8_t I2C_SLV1_DO        = uint8_t(0x64);
    const uint8_t I2C_SLV2_DO        = uint8_t(0x65);
    const uint8_t I2C_SLV3_DO        = uint8_t(0x66);
    const uint8_t I2C_MST_DELAY_CTRL = uint8_t(0x67);
    const uint8_t SIGNAL_PATH_RESET  = uint8_t(0x68);
    const uint8_t MOT_DETECT_CTRL    = uint8_t(0x69);
    const uint8_t USER_CTRL          = uint8_t(0x6A);
    const uint8_t PWR_MGMT_1         = uint8_t(0x6B);
    const uint8_t PWR_MGMT_2         = uint8_t(0x6C);
    const uint8_t FIFO_COUNTH        = uint8_t(0x72);
    const uint8_t FIFO_COUNTL        = uint8_t(0x73);
    const uint8_t FIFO_R_W           = uint8_t(0x74);
    const uint8_t WHO_AM_I           = uint8_t(0x75);
    const uint8_t XA_OFFSET_H        = uint8_t(0x77);
    const uint8_t XA_OFFSET_L        = uint8_t(0x78);
    const uint8_t YA_OFFSET_H        = uint8_t(0x7A);
    const uint8_t YA_OFFSET_L        = uint8_t(0x7B);
    const uint8_t ZA_OFFSET_H        = uint8_t(0x7D);
    const uint8_t ZA_OFFSET_L        = uint8_t(0x7E);	 
  }
  
  namespace reg_mag
  {    
    const uint8_t WIA                = uint8_t(0x00);
    const uint8_t INFO               = uint8_t(0x01);
    const uint8_t ST1                = uint8_t(0x02);
    const uint8_t HXL                = uint8_t(0x03);
    const uint8_t HXH                = uint8_t(0x04);
    const uint8_t HYL                = uint8_t(0x05);
    const uint8_t HYH                = uint8_t(0x06);
    const uint8_t HZL                = uint8_t(0x07);
    const uint8_t HZH                = uint8_t(0x08);
    const uint8_t ST2                = uint8_t(0x09);
    const uint8_t CNTL1              = uint8_t(0x0A);
    const uint8_t CNTL2              = uint8_t(0x0B);
    const uint8_t ASTC               = uint8_t(0x0C);
    const uint8_t TS1                = uint8_t(0x0D);
    const uint8_t TS2                = uint8_t(0x0E);
    const uint8_t I2CDIS             = uint8_t(0x0F);
    const uint8_t ASAX               = uint8_t(0x10);
    const uint8_t ASAY               = uint8_t(0x11);
    const uint8_t ASAZ               = uint8_t(0x12);	
  }
  
  namespace sensitivity_gyro
  {
    const float _0_FS_SEL = 131.0f;
    const float _1_FS_SEL =  65.5f;
    const float _2_FS_SEL =  32.8f;
    const float _3_FS_SEL =  16.4f;
  }
  
  namespace sensitivity_accel
  {
    const uint16_t _0_FS_SEL = 16384;
    const uint16_t _1_FS_SEL =  8192;
    const uint16_t _2_FS_SEL =  4096;
    const uint16_t _3_FS_SEL =  2048;
  }
  
  namespace sensitivity_mag
  {
    const float _14_BIT_SENS = 0.6f;  //в мкТл
    const float _16_BIT_SENS = 0.15f; //в мкТл
  }
  namespace temp_mpu_9250
  {
    const uint8_t ROOM_TEMP_OFFSET  = 21;
	  const float SENSITIVITY        = 340.0f;
  } 
}

#endif







