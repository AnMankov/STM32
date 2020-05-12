#ifndef __LSM6DS3_REGS_H
#define __LSM6DS3_REGS_H

#include <cstdint>

namespace LSM6DS3
{
  using std::uint8_t;

//----- Объявления типов для аппаратного регистра INT1_CTRL (0Dh) -------------------
  struct SGyroDataReady
  {
    uint8_t Reserved1 :1;
    uint8_t Dest      :1; //INT1_DRDY_G
    uint8_t Reserved2 :6;
  };
  
  struct SAccDataReady
  {
    uint8_t Dest      :1; //INT1_DRDY_XL
    uint8_t Reserved1 :7;
  };
//----------------------------------------------------------------------------------

//----- Объявления типов для аппаратного регистра CTRL1_XL (10h) -------------------
  struct SAccAntiAliasingBW
  {
    uint8_t Dest      :2; //BW_XL [1:0]
    uint8_t Reserved1 :6;
  };
  
  struct SAccFullScale
  {
    uint8_t Reserved1 :2;
    uint8_t Dest      :2; //FS_XL [1:0]
    uint8_t Reserved2 :4;
  };
  
  struct SAccDataRatePowerMode
  {
    uint8_t Reserved1 :4;
    uint8_t Dest      :4; //ODR_XL [3:0]
  };
//----------------------------------------------------------------------------------

//----- Объявления типов для аппаратного регистра CTRL2_G (11h) --------------------
  struct SGyroFullScale125
  {
    uint8_t Reserved1 :1;
    uint8_t Dest      :1; //FS_125
    uint8_t Reserved2 :6;
  };

  struct SGyroFullScale
  {
    uint8_t Reserved1 :2;
    uint8_t Dest      :2; //FS_G [1:0]
    uint8_t Reserved2 :4;
  };

  struct SGyroDataRate
  {
    uint8_t Reserved  :4;
    uint8_t Dest      :4; //ODR_G [3:0]
  };
//----------------------------------------------------------------------------------


//----- Объявления типов для аппаратного регистра CTRL3_C (12h) --------------------

  struct SSoftReset
  {
    uint8_t Dest      :1; //SW_RESET
    uint8_t Reserved  :7;
  };

  struct SEndian
  {
    uint8_t Reserved1 :1;
    uint8_t Dest      :1; //BLE
    uint8_t Reserved2 :6;
  };

  struct SAutoIncrement
  {
    uint8_t Reserved1 :2;
    uint8_t Dest      :1; //IF_INC
    uint8_t Reserved2 :5;
  };

  struct SPushPullOpenDrain
  {
    uint8_t Reserved1 :4;
    uint8_t Dest      :1; //PP_OD
    uint8_t Reserved2 :3;
  };

  struct SIntActivationLevel
  {
    uint8_t Reserved1 :5;
    uint8_t Dest      :1; //H_LACTISE
    uint8_t Reserved2 :2;
  };

  struct SBlockDataUpdate
  {
    uint8_t Reserved1 :6;
    uint8_t Dest      :1; //BDU
    uint8_t Reserved2 :1;
  };

  struct SRebootMemoryContent
  {
    uint8_t Reserved1 :7;
    uint8_t Dest      :1; //BOOT
  };
//----------------------------------------------------------------------------------


//----- Объявления типов для аппаратного регистра CTRL4_C (13h) --------------------
  struct SDisableI2C
  {
    uint8_t Reserved1 :2;
    uint8_t Dest      :1; //I2C_disable
    uint8_t Reserved2 :5;
  };

  struct SDataReadyMask
  {
    uint8_t Reserved1 :3;
    uint8_t Dest      :1; //DRDY_MASK
    uint8_t Reserved2 :4;
  };

  struct SDividedIntSignals
  {
    uint8_t Reserved1 :5;
    uint8_t Dest      :1; //INT2_on_INT1
    uint8_t Reserved2 :2;
  };

  struct SGyroSleep
  {
    uint8_t Reserved1 :6;
    uint8_t Dest      :1; //SLEEP_G
    uint8_t Reserved2 :1;
  };

  struct SAccBw
  {
    uint8_t Reserved  :7;
    uint8_t Dest      :1; //XL_BW_SCAL_ODR
  };
//----------------------------------------------------------------------------------


//----- Объявления типов для аппаратного регистра CTRL5_C (14h) --------------------
  struct SAccSelfTest
  {
    uint8_t Dest      :2; //ST_XL [1:0]
    uint8_t Reserved  :6;
  };

  struct SGyroSelfTest
  {
    uint8_t Reserved1 :2;
    uint8_t Dest      :2; //ST_G [1:0]
    uint8_t Reserved2 :4;
  };

  struct SBurstModeRead
  {
    uint8_t Reserved  :5;
    uint8_t Dest      :3; //ROUNDING[2:0]
  };
//----------------------------------------------------------------------------------


//----- Объявления типов для аппаратного регистра CTRL6_C (15h) --------------------
  struct SAccHighPerform
  {
    uint8_t Reserved1 :4;
    uint8_t Dest      :1; //XL_HM_MODE
    uint8_t Reserved2 :3;
  };

  struct SGyroLevelLatch
  {
    uint8_t Reserved1 :5;
    uint8_t Dest      :1; //LVL2_EN
    uint8_t Reserved2 :2;
  };

  struct SGyroDataLevelTrig
  {
    uint8_t Reserved1 :6;
    uint8_t Dest      :1; //LVLen
    uint8_t Reserved2 :1;
  };

  struct SGyroDataEdgeTrig
  {
    uint8_t Reserved  :7;
    uint8_t Dest      :1; //TRIG_EN
  };
//----------------------------------------------------------------------------------


//----- Объявления типов для аппаратного регистра CTRL7_G (16h) --------------------
  struct SSrcRegRoundingFunc
  {
    uint8_t Reserved1 :2;
    uint8_t Dest      :1; //ROUNDING_STATUS
    uint8_t Reserved2 :5;
  };

  struct SGyroDigHpFilter
  {
    uint8_t Reserved1 :3;
    uint8_t Dest      :1; //HP_G_RST
    uint8_t Reserved2 :4;
  };

  struct SGyroHpFilterCutoff
  {
    uint8_t Reserved1 :4;
    uint8_t Dest      :2; //HPCF_G[1:0]
    uint8_t Reserved2 :2;
  };

  struct SGyroDigHighPassFilter
  {
    uint8_t Reserved1 :6;
    uint8_t Dest      :1; //HP_G_EN
    uint8_t Reserved2 :1;
  };

  struct SGyroHighPerform
  {
    uint8_t Reserved  :7;
    uint8_t Dest      :1; //G_HM_MODE
  };
//----------------------------------------------------------------------------------


//----- Объявления типов для аппаратного регистра CTRL8_XL (17h) --------------------
  struct SLPFon6D
  {
    uint8_t Dest      :1; //LOW_PASS_ON_6D
    uint8_t Reserved  :7;
  };

  struct SAccHPSlopeFilter
  {
    uint8_t Reserved1 :2;
    uint8_t Dest      :1; //HP_SLOPE_XL_EN
    uint8_t Reserved2 :5;
  };

  struct SAccHPFConfigCutoffSet
  {
    uint8_t Reserved1 :5;
    uint8_t Dest      :2; //HPCF_XL[1:0]
    uint8_t Reserved2 :1;
  };

  struct SAccLPF2
  {
    uint8_t Reserved  :7;
    uint8_t Dest      :1; //LPF2_XL_EN
  };
//----------------------------------------------------------------------------------


//----- Объявления типов для аппаратного регистра CTRL9_XL (18h) -------------------
  struct SAccOutX
  {
    uint8_t Reserved1 :3;
    uint8_t Dest      :1; //Xen_XL
    uint8_t Reserved2 :4;
  };

  struct SAccOutY
  {
    uint8_t Reserved1 :4;
    uint8_t Dest      :1; //Yen_XL
    uint8_t Reserved2 :3;
  };

  struct SAccOutZ
  {
    uint8_t Reserved1 :5;
    uint8_t Dest      :1; //Zen_XL
    uint8_t Reserved2 :2;
  };
//----------------------------------------------------------------------------------


//----- Объявления типов для аппаратного регистра CTRL10_C (19h) -------------------
  struct SGyroOutX
  {
    uint8_t Reserved1 :3;
    uint8_t Dest      :1; //Xen_G
    uint8_t Reserved2 :4;
  };

  struct SGyroOutY
  {
    uint8_t Reserved1 :4;
    uint8_t Dest      :1; //Yen_G
    uint8_t Reserved2 :3;
  };

  struct SGyroOutZ
  {
    uint8_t Reserved1 :5;
    uint8_t Dest      :1; //Zen_G
    uint8_t Reserved2 :2;
  };
//----------------------------------------------------------------------------------


//----- Объявления типов для аппаратного регистра STATUS_REG (1Eh) -----------------
  struct SAccDataAvailable
  {
    uint8_t Dest      :1; //XLDA
    uint8_t Reserved  :7;
  };

  struct SGyroDataAvailable
  {
    uint8_t Reserved1 :1;
    uint8_t Dest      :1; //GDA
    uint8_t Reserved2 :6;
  };

  struct STemperDataAvailable
  {
    uint8_t Reserved1 :2;
    uint8_t Dest      :1; //TDA
    uint8_t Reserved2 :5;
  };
//----------------------------------------------------------------------------------
}

#endif
