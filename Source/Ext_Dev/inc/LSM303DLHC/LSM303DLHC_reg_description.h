#ifndef _LSM303DLHC_REG_DESCR
#define _LSM303DLHC_REG_DESCR

#include <cstdint>

#pragma anon_unions

namespace LSM303DLHC
{
//--------------------------------------------------------------------------
  struct TAccelPwrRate
  {
    std::uint8_t Reserved :4;
    std::uint8_t Dest     :4; 
  };  // CTRL_REG1_A (20h)
  
  struct TAccelAxesEn  
  {
    std::uint8_t Dest     :3;
    std::uint8_t Reserved :5;
  };  // CTRL_REG1_A (20h)
  
  struct TAccelPwrMode
  {
    std::uint8_t Reserved1 :3;
    std::uint8_t Dest      :1;
    std::uint8_t Reserved2 :4;
  };  // CTRL_REG1_A (20h)
//--------------------------------------------------------------------------
  struct TAccelDataUpdate
  {
    std::uint8_t Reserved :7;
    std::uint8_t Dest     :1; 
  };  // CTRL_REG4_A (23h)
  
  struct TAccelEndianess  
  {
    std::uint8_t Reserved1 :6;
    std::uint8_t Dest      :1;
    std::uint8_t Reserved2 :1;
  };  // CTRL_REG4_A (23h)
  
  struct TAccelScale
  {
    std::uint8_t Reserved1 :4;
    std::uint8_t Dest      :2;
    std::uint8_t Reserved2 :2;
  };  // CTRL_REG4_A (23h)
  
  struct TAccelHighResolution
  {
    std::uint8_t Reserved1 :3;
    std::uint8_t Dest      :1;
    std::uint8_t Reserved2 :4;
  };  // CTRL_REG4_A (23h)
//--------------------------------------------------------------------------
 










































 
  struct CTRL_REG1_A
  {
    union
    {
      struct
  	 {
  	   std::uint8_t Xen    :1; // 0/1 - выключить/включить ось X
        std::uint8_t Yen    :1; // 0/1 - выключить/включить ось Y
        std::uint8_t Zen    :1; // 0/1 - выключить/включить ось Z
  	 };
  	 std::uint8_t Axes     :3; //управление всеми осями
    };
    
    
    std::uint8_t LPen       :1; // 0/1 - normal - высокое разрешение/low-power - низкое потребление
    union                        // режим работы, скорость выдачи данных
    {
  	 struct
  	 {
  		std::uint8_t ODR0   :1;
  		std::uint8_t ODR1   :1;
  		std::uint8_t ODR2   :1;
  		std::uint8_t ODR3   :1;
  	 };
  	 std::uint8_t ODR      :4;
    }; 
  };
  	
  struct CTRL_REG2_A
  {
    std::uint8_t HPIS1   :1;
    std::uint8_t HPIS2   :1;
    std::uint8_t HPCLICK :1;
    std::uint8_t FDS     :1;
    std::uint8_t HPCF    :2;
    std::uint8_t HPM     :2;
  };
  
  struct CTRL_REG3_A
  {
    std::uint8_t Reserved   :1;
    std::uint8_t I1_OVERRUN :1;
    std::uint8_t I1_WTM     :1;
    std::uint8_t I1_DRDY2   :1;
    std::uint8_t I1_DRDY1   :1;
    std::uint8_t I1_AOI2    :1;
    std::uint8_t I1_AOI1    :1;
    std::uint8_t I1_CLICK   :1;
  };
  
  struct CTRL_REG4_A
  {
    std::uint8_t SIM      :1;
    std::uint8_t Reserved :2;
    std::uint8_t HR       :1;
    std::uint8_t FS0      :1;
    std::uint8_t FS1      :1;
    std::uint8_t BLE      :1;
    std::uint8_t BDU      :1;
  };
  
  struct CTRL_REG5_A
  {
    std::uint8_t D4D_INT2 :1;
    std::uint8_t LIR_INT2 :1;
    std::uint8_t D4D_INT1 :1;
    std::uint8_t LIR_INT1 :1;
    std::uint8_t Reserved :2;
    std::uint8_t FIFO_EN  :1;
    std::uint8_t BOOT     :1;
  };
  
  struct CTRL_REG6_A
  {
    std::uint8_t Reserved0  :1;
    std::uint8_t H_LACTIVE  :1;
    std::uint8_t Reserved1  :1;
    std::uint8_t P2_ACT     :1;
    std::uint8_t BOOT_I1    :1;
    std::uint8_t I2_INT2    :1;
    std::uint8_t I2_INT1    :1;
    std::uint8_t I2_CLICKen :1;
  };
  
  struct REFERENCE_A
  {
      std::uint8_t Ref        :8;
  };
  
  struct FIFO_CTRL_REG_A
  {
    std::uint8_t FTH :5;
    std::uint8_t TR  :1;
    std::uint8_t FM  :2;
  };
  
  struct INT1_CFG_A
  {
    std::uint8_t XLIE_XDOWNE :1;
    std::uint8_t XHIE_XUPE   :1;
    std::uint8_t YLIE_YDOWNE :1;
    std::uint8_t YHIE_YUPE   :1;
    std::uint8_t ZLIE_ZDOWNE :1;
    std::uint8_t ZHIE_ZUPE   :1;
    std::uint8_t _6D         :1;
    std::uint8_t AOI         :1;
  };
  
  struct INT1_THS_A
  {
    std::uint8_t THS      :7;
    std::uint8_t Reserved :1;
  };
  
  struct INT1_DURATION_A
  {
    std::uint8_t D        :7;
    std::uint8_t Reserved :1;
  };
  
  struct INT2_CFG_A
  {
    std::uint8_t XLIE :1;
    std::uint8_t XHIE :1;
    std::uint8_t YLIE :1;
    std::uint8_t YHIE :1;
    std::uint8_t ZLIE :1;
    std::uint8_t ZHIE :1;
    std::uint8_t _6D  :1;
    std::uint8_t AOI  :1;
  };
  
  struct INT2_THS_A
  {
    std::uint8_t THS      :7;
    std::uint8_t Reserved :1;
  };
  
  struct INT2_DURATION_A
  {
    std::uint8_t D        :7;
    std::uint8_t Reserved :1;
  };
  
  struct CLICK_CFG_A
  {
    std::uint8_t XS       :1;
    std::uint8_t XD       :1;
    std::uint8_t YS       :1;
    std::uint8_t YD       :1;
    std::uint8_t ZS       :1;
    std::uint8_t ZD       :1;
    std::uint8_t Reserved :2;
  };
  
  struct CLICK_SRC_A
  {
    std::uint8_t X        :1;
    std::uint8_t Y        :1;
    std::uint8_t Z        :1;
    std::uint8_t Sign     :1;
    std::uint8_t SCLICK   :1;
    std::uint8_t DCLICK   :1;
    std::uint8_t IA       :1;
    std::uint8_t Reserved :1;
  };
  
  struct CLICK_THS_A
  {
    std::uint8_t Ths      :7;
    std::uint8_t Reserved :1;
  };
  
  struct TIME_LIMIT_A
  {
    std::uint8_t TLI      :7;
    std::uint8_t Reserved :1;
  };
  
  struct TIME_LATENCY_A
  {
    std::uint8_t TLA      :8;
  };
  
  struct TIME_WINDOW_A
  {
    std::uint8_t TW       :8;
  };
  
  struct CRA_REG_M
  {
    std::uint8_t Reserved0 :2;
    std::uint8_t DO        :3;
    std::uint8_t Reserved1 :2;
    std::uint8_t TEMP_EN   :1;
  };
  
  struct CRB_REG_M
  {
    std::uint8_t Reserved :5;
    std::uint8_t GN       :3;
  };
  
  struct MR_REG_M
  {
    std::uint8_t MD       :2;
    std::uint8_t Reserved :6;
  };
  
  //---------------------------------------------------------------------------------
  /* Объединение всех RW - регистров акселерометра LSM303DLHC */
  struct TAccRwRegs
  {
  
  };
  
  /* Объединение всех RW - регистров магнитометра LSM303DLHC */
  struct TMagRwRegs
  {
  
  };
  
  /* Объединение всех RW - регистров микросхемы LSM303DLHC */
  struct TRwRegs
  {
  
  };
  //---------------------------------------------------------------------------------
  struct STATUS_REG_A
  {
    std::uint8_t XDA   :1;
    std::uint8_t YDA   :1;
    std::uint8_t ZDA   :1;
    std::uint8_t ZYXDA :1;
    std::uint8_t XOR   :1;
    std::uint8_t YOR   :1;
    std::uint8_t ZOR   :1;
    std::uint8_t ZYXOR :1;
  };
  
  struct OUT_X_L_A
  {
    std::uint8_t X_L_A :8;
  };
  
  struct OUT_X_H_A
  {
    std::uint8_t X_H_A :8;
  };
  
  struct OUT_Y_L_A
  {
    std::uint8_t Y_L_A :8;
  };
  
  struct OUT_Y_H_A
  {
    std::uint8_t Y_H_A :8;
  };
  
  struct OUT_Z_L_A
  {
    std::uint8_t Z_L_A :8;
  };
  
  struct OUT_Z_H_A
  {
    std::uint8_t Z_H_A :8;
  };
  
  struct FIFO_SRC_REG_A
  {
    std::uint8_t FSS       :5; 
    std::uint8_t EMPTY     :1;
    std::uint8_t OVRN_FIFO :1;
    std::uint8_t WTM       :1;
  };
  
  struct INT1_SRC_A
  {
    std::uint8_t XL       :1;
    std::uint8_t XH       :1;
    std::uint8_t YL       :1;
    std::uint8_t YH       :1;
    std::uint8_t ZL       :1;
    std::uint8_t ZH       :1;
    std::uint8_t IA       :1;
    std::uint8_t Reserved :1;
  };
  
  struct INT2_SRC_A
  {
    std::uint8_t XL       :1;
    std::uint8_t XH       :1;
    std::uint8_t YL       :1;
    std::uint8_t YH       :1;
    std::uint8_t ZL       :1;
    std::uint8_t ZH       :1;
    std::uint8_t IA       :1;
    std::uint8_t Reserved :1;
  };
  
  struct OUT_X_H_M
  {
    std::uint8_t X_H_M :8;
  };
  
  struct OUT_X_L_M
  {
    std::uint8_t X_L_M :8;
  };
  
  struct OUT_Z_H_M
  {
    std::uint8_t Z_H_M :8;
  };
  
  struct OUT_Z_L_M
  {
    std::uint8_t Z_L_M :8;
  };
  
  struct OUT_Y_H_M
  {
    std::uint8_t Y_H_M :8;
  };
  
  struct OUT_Y_L_M
  {
    std::uint8_t Y_L_M :8;
  };
  
  struct SR_REG_M
  {
    std::uint8_t DRDY     :1;
    std::uint8_t LOCK     :1;
    std::uint8_t Reserved :1;
  };
  
  struct IRA_REG_M
  {
  
  };
  
  struct IRB_REG_M
  {
  
  };
  
  struct IRC_REG_M
  {
  
  };
  
  //---------------------------------------------------------------------------------
  /* Объединение всех RO - регистров акселерометра LSM303DLHC */
  struct TAccRoRegs
  {
  
  };
  
  /* Объединение всех RO - регистров магнитометра LSM303DLHC */
  struct TMagRoRegs
  {
  
  };
  
  /* Объединение всех RO - регистров LSM303DLHC */
  struct TRo_Regs
  {
  
  };
  //---------------------------------------------------------------------------------
  /* Регистры встроенного термодатчика */
  struct TEMP_OUT_H_M
  {
    std::uint8_t TEMP_H :8;
  };
  
  struct TEMP_OUT_L_M
  {
    std::uint8_t Reserved :4;
    std::uint8_t TEMP_L   :4;
  };
}
//---------------------------------------------------------------------------------
#endif //LSM303DLHC_REGISTER_DESCRIPTION
