#ifndef __MPU_9250_TYPES_H
#define __MPU_9250_TYPES_H

#pragma anon_unions

#include <cstdint>

namespace MPU_9250
{
  using std::uint8_t;
  using std::uint16_t;
  
  // Register 26 – Configuration
  struct TFifoModeConfig
  {
    uint8_t Reserved1 : 6;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 1;
  };

  struct TFsyncPinConfig
  {
    uint8_t Reserved1 : 3;
    uint8_t Dest      : 3;
    uint8_t Reserved2 : 2;
  };

  struct TFilterGyroTempConfig
  {
    uint8_t Dest      : 3;
    uint8_t Reserved1 : 5;
  };

  // Register 27 – Gyroscope Configuration  
  struct TEnableXGyroSelfTest
  {
    uint8_t Reserved  : 7;
    uint8_t Dest      : 1;
  };

  struct TEnableYGyroSelfTest
  {
    uint8_t Reserved1 : 6;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 1;
  };
  
  struct TEnableZGyroSelfTest
  {
    uint8_t Reserved1 : 5;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 2;
  };
  
  struct TGyroFullScaleSelect
  {
    uint8_t Reserved1 : 3;
    uint8_t Dest      : 2;
    uint8_t Reserved2 : 3;
  };
  
  struct TGyroFilterChoise
  {
    uint8_t Dest      : 2;
    uint8_t Reserved1 : 6;
  };

  //  Register 28 – Accelerometer Configuration
  struct TEnableXAccelSelfTest
  {
    uint8_t Reserved  : 7;
	  uint8_t Dest      : 1;
  };
  struct TEnableYAccelSelfTest
  {
    uint8_t Reserved1 : 6;
	  uint8_t Dest      : 1;
	  uint8_t Reserved2 : 1;
  };

  struct TEnableZAccelSelfTest
  {
    uint8_t Reserved1 : 5;
	  uint8_t Dest      : 1;
	  uint8_t Reserved2 : 2;
  };

  struct TAccelFullScaleSelect
  {
    uint8_t Reserved1 : 3;
	  uint8_t Dest      : 2;
	  uint8_t Reserved2 : 3;
  };

  // Register 29 – Accelerometer Configuration 2
  struct TAcceFilterChoise
  {
    uint8_t Reserved1 : 3;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 4;	 
  };

  struct TSetAccelFilter
  {
    uint8_t Dest      : 3;
    uint8_t Reserved  : 5;	 
  };
  
  // Register 30 – Low Power Accelerometer ODR Control
  struct TSetAccelWakeupFreq
  {
    uint8_t Dest      : 4;
	  uint8_t Reserved  : 4;
  };
  
  // Register 35 – FIFO Enable
  struct TWrTempToFifo
  {
	  uint8_t Reserved  : 7;
    uint8_t Dest      : 1;
  };
  
  struct TWrGyroXToFifo
  {
	  uint8_t Reserved1 : 6;
    uint8_t Dest      : 1;
	  uint8_t Reserved2 : 1;
  };
  
  struct TWrGyroYToFifo
  {
	  uint8_t Reserved1 : 5;
    uint8_t Dest      : 1;
	  uint8_t Reserved2 : 2;
  };
  
  struct TWrGyroZToFifo
  {
    uint8_t Reserved1 : 4;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 3;
  };
  
  struct TWrAccelToFifo
  {
    uint8_t Reserved1 : 3;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 4;
  };
  
  struct TWrSlave2ToFifo
  {
    uint8_t Reserved1 : 2;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 5;
  };
  
  struct TWrSlave1ToFifo
  {
    uint8_t Reserved1 : 1;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 6;
  };
  
  struct TWrSlave0ToFifo
  {
    uint8_t Dest      : 1;
    uint8_t Reserved  : 7;
  };
  
  // Register 36 – I2C Master Control
  struct TMultiMasterEnable
  {
    uint8_t Reserved  : 7;
    uint8_t Dest      : 1;
  };
  
  struct TDelayDataReadyInt
  {
    uint8_t Reserved1 : 6;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 1;
  };
  
  struct TWriteDataSlv3ToFifo
  {
    uint8_t Reserved1 : 5;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 2;
  };
  
  struct TCtrlSlaveReadTransition
  {
    uint8_t Reserved1 : 4;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 3;
  };
  
  struct TCfgInternalClk
  {
    uint8_t Dest      : 4;
    uint8_t Reserved2 : 4;
  };
  
  // Register 37 - I2C_SLV0_ADDR
  struct TSlv0DirectTransfer
  {
    uint8_t Reserved  : 7;
    uint8_t Dest      : 1;
  };
  
  struct TSlv0PhyAddress
  {
    uint8_t Dest      : 7;
    uint8_t Reserved  : 1;
  };
  
  // Register 39 - I2C_SLV0_CTRL
  struct TSlv0Enable
  {
    uint8_t Reserved  : 7;
    uint8_t Dest      : 1;
  };
  
  struct TSlv0SwapBytes
  {
    uint8_t Reserved1 : 6;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 1;
  };
  
  struct TSlv0WriteReg
  {
    uint8_t Reserved1 : 5;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 2;
  };
  
  struct TSlv0GroupData
  {
    uint8_t Reserved1 : 4;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 3;
  };
  
  struct TSlv0NumBytesRead
  {
    uint8_t Dest      : 4;
    uint8_t Reserved  : 4;
  };
   
  // Register 40 - I2C_SLV1_ADDR
  struct TSlv1TransferType
  {
    uint8_t Reserved  : 7;
    uint8_t Dest      : 1;
  };
  
  struct TSlv1PhyAddress
  {
    uint8_t Dest      : 7;
    uint8_t Reserved  : 1;
  };
   
  // Register 42 - I2C_SLV1_CTRL
  struct TSlv1Enable
  {
    uint8_t Reserved  : 7;
    uint8_t Dest      : 1;
  };
  
  struct TSlv1SwapBytes
  {
    uint8_t Reserved1 : 6;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 1;
  };
  
  struct TSlv1TransactionCtrl
  {
    uint8_t Reserved1 : 5;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 2;
  };
  
  struct TSlv1DetermineAddress
  {
    uint8_t Reserved1 : 4;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 3;
  };
  
  struct TSlv1NumBytesRead
  {
    uint8_t Dest      : 4;
    uint8_t Reserved2 : 4;
  };
  
  // Register 43 - I2C_SLV2_ADDR
  struct TSlv2TransferType
  {
    uint8_t Reserved  : 7;
    uint8_t Dest      : 1;
  };
  
  struct TSlv2PhyAddress
  {
    uint8_t Dest      : 7;
    uint8_t Reserved  : 1;
  };
  
  // Register 45 - I2C_SLV2_CTRL
  struct TSlv2Enable
  {
    uint8_t Reserved  : 7;
    uint8_t Dest      : 1;
  };
  
  struct TSlv2SwapBytes
  {
    uint8_t Reserved1 : 6;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 1;
  };
  
  struct TSlv2TransactionCtrl
  {
    uint8_t Reserved1 : 5;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 2;
  };
  
  struct TSlv2DetermineAddress
  {
    uint8_t Reserved1 : 4;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 3;
  };
  
  struct TSlv2NumBytesRead
  {
    uint8_t Dest      : 4;
    uint8_t Reserved2 : 4;
  };
  
  // Register 46 - I2C_SLV3_ADDR
  struct TSlv3TransferType
  {
    uint8_t Reserved  : 7;
    uint8_t Dest      : 1;
  };
  
  struct TSlv3PhyAddress
  {
    uint8_t Dest      : 7;
    uint8_t Reserved  : 1;
  };
  
  // Register 48 - I2C_SLV3_CTRL
  struct TSlv3Enable
  {
    uint8_t Reserved  : 7;
    uint8_t Dest      : 1;
  };
  
  struct TSlv3SwapBytes
  {
    uint8_t Reserved1 : 6;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 1;
  };
  
  struct TSlv3TransactionCtrl
  {
    uint8_t Reserved1 : 5;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 2;
  };
  
  struct TSlv3DetermineAddress
  {
    uint8_t Reserved1 : 4;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 3;
  };
  
  struct TSlv3NumBytesRead
  {
    uint8_t Dest      : 4;
    uint8_t Reserved2 : 4;
  };
  
  // Register 49 - I2C_SLV4_ADDR
  struct TSlv4TransferType
  {
    uint8_t Reserved  : 7;
    uint8_t Dest      : 1;
  };
  
  struct TSlv4PhyAddress
  {
    uint8_t Dest      : 7;
    uint8_t Reserved  : 1;
  };
  
  // Register 52 - I2C_SLV4_CTRL
  struct TSlv4Enable
  {
    uint8_t Reserved  : 7;
    uint8_t Dest      : 1;
  };
  
  struct TSlv4Complete
  {
    uint8_t Reserved1 : 6;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 2;
  };
  
  struct TSlv4TransactionCtrl
  {
    uint8_t Reserved1 : 5;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 2;
  };
  
  struct TNumSamplesCtrl
  {
    uint8_t Dest      : 5;
    uint8_t Reserved  : 3;
  };
  
  // Register 54 – I2C Master Status
  struct TStatusFsyncInterrupt
  {
    uint8_t Reserved1 : 7;
    uint8_t Dest      : 1;
  };
  
  struct TSlv4TransferComplete
  {
    uint8_t Reserved1 : 6;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 1;
  };
  
  struct TSlvLostArbitration
  {
    uint8_t Reserved1 : 5;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 2;
  };
  
  struct TSlv4RxNack
  {
    uint8_t Reserved1 : 4;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 3;
  };
  
  struct TSlv3RxNack
  {
    uint8_t Reserved1 : 3;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 4;
  };
  
  struct TSlv2RxNack
  {
    uint8_t Reserved1 : 2;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 5;
  };
  
  struct TSlv1RxNack
  {
    uint8_t Reserved1 : 1;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 6;
  };
  
  struct TSlv0RxNack
  {
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 7;
  };
  
  // Register 55 – INT Pin / Bypass Enable Configuration
  struct TIntLogicLevel
  {
    uint8_t Reserved1 : 7;
    uint8_t Dest      : 1;
  };
  
  struct TIntPinConfig
  {
    uint8_t Reserved1 : 6;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 1;
  };
  
  struct TIntLatchCtrl
  {
    uint8_t Reserved1 : 5;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 2;
  };
  
  struct TIntStatusCtrl
  {
    uint8_t Reserved1 : 4;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 3;
  };
  
  struct TFsyncLevelCtrl
  {
    uint8_t Reserved1 : 3;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 4;
  };
  
  struct TFsyncEnable
  {
    uint8_t Reserved1 : 2;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 5;
  };
  
  struct TI2CMasterPinsCtrl
  {
    uint8_t Reserved1 : 1;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 6;
  };
  
  // Register 56 – Interrupt Enable
  struct TWakeOnMotionIntCtrl
  {
    uint8_t Reserved1 : 6;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 1;
  };
  
  struct TFifoOvrfIntCtrl
  {
    uint8_t Reserved1 : 4;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 3;
  };
  
  struct TFsyncIntCtrl
  {
    uint8_t Reserved1 : 3;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 4;
  };
  
  struct TRawDataReadyIntCtrl
  {
    uint8_t Dest      : 1;
    uint8_t Reserved1 : 7;
  };
  
  // Register 58 – Interrupt Status
  struct TWakeOnMotionIntStatus
  {
    uint8_t Reserved1 : 6;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 1;
  };
  
  struct TFifoOvrfIntStatus
  {
    uint8_t Reserved1 : 4;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 3;
  };
  
  struct TFsyncIntStatus
  {
    uint8_t Reserved1 : 3;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 4;
  };
  
  struct TRawDataReadyIntStatus
  {
    uint8_t Dest      : 1;
    uint8_t Reserved  : 7;
  };
  
  // Register 103 – I2C Master Delay Control
  struct TDelayShadowExtSense
  {
    uint8_t Reserved  : 7;
    uint8_t Dest      : 1;
  };
  
  struct TSlv4DlyAccessCtrl
  {
    uint8_t Reserved1 : 4;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 3;
  };
  
  struct TSlv3DlyAccessCtrl
  {
    uint8_t Reserved1 : 3;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 4;
  };
  
  struct TSlv2DlyAccessCtrl
  {
    uint8_t Reserved1 : 2;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 5;
  };
  
  struct TSlv1DlyAccessCtrl
  {
    uint8_t Reserved1 : 1;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 6;
  };
  
  struct TSlv0DlyAccessCtrl
  {
    uint8_t Dest      : 1;
    uint8_t Reserved  : 7;
  };
  
  // Register 104 – Signal Path Reset
  struct TResetGyroSignalPath
  {
    uint8_t Reserved1 : 2;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 5;
  };
  
  struct TResetAccelSignalPath
  {
    uint8_t Reserved1 : 1;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 6;
  };
  
  struct TResetTempSignalPath
  {
    uint8_t Dest      : 1;
    uint8_t Reserved  : 7;
  };
	 
  // Register 105 – Accelerometer Interrupt Control 
  struct TAccelWakeOnMotionCtrl
  {
    uint8_t Reserved  : 7;
    uint8_t Dest      : 1;
  };
  
  struct TAccelCompareSamplesCtrl
  {
    uint8_t Reserved1 : 6;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 1;
  };
  	 
  // Register 106 – User Control
  struct TFifoModeCtrl
  {
    uint8_t Reserved1 : 6;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 1;
  };
  
  struct TI2CMasterIFModuleCtrl
  {
    uint8_t Reserved1 : 5;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 2;
  };
  
  struct TInterfaceModeCtrl
  {
    uint8_t Reserved1 : 4;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 3;
  };
  
  struct TResetFifoModule
  {
    uint8_t Reserved1 : 2;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 5;
  };
  
  struct TResetI2CMasterModule
  {
    uint8_t Reserved1 : 1;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 6;
  };
  
  struct TResetAllSignalPath
  {
    uint8_t Dest      : 1;
    uint8_t Reserved  : 7;
  };
  
	 
  // Register 107 – Power Management 1
  struct TResetAndRestoreInternalRegs
  {
    uint8_t Reserved  : 7;
    uint8_t Dest      : 1;
  };
  
  struct TSleepCtrl
  {
    uint8_t Reserved1 : 6;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 1;
  };
  
  struct TCycleCtrl
  {
    uint8_t Reserved1 : 5;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 2;
  };
  
  struct TGyroStandbyCtrl
  {
    uint8_t Reserved1 : 4;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 3;
  };
  
  struct TVoltageGeneratorCtrl
  {
    uint8_t Reserved1 : 3;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 4;
  };
  
  struct TClockSrcChoise
  {
    uint8_t Dest      : 3;
    uint8_t Reserved  : 5;
  };
	 
  // Register 108 – Power Management 2
  struct TAccelXCtrl
  {
    uint8_t Reserved1 : 5;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 2;
  };
  
  struct TAccelYCtrl
  {
    uint8_t Reserved1 : 4;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 3;
  };
  
  struct TAccelZCtrl
  {
    uint8_t Reserved1 : 3;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 4;
  };
  
  struct TGyroXCtrl
  {
    uint8_t Reserved1 : 2;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 5;
  };
  
  struct TGyroYCtrl
  {
    uint8_t Reserved1 : 1;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 6;
  };
  
  struct TGyroZCtrl
  {
    uint8_t Dest      : 1;
    uint8_t Reserved  : 7;
  };
	 
  // Register 114 and 115 – FIFO Count Registers
  struct TFifoHighBits
  {
    uint8_t Dest      : 5;
    uint8_t Reserved  : 3;
  };
  
  // ST1: Status 1
  struct TDataReady
  {
    uint8_t Dest      : 1;
    uint8_t Reserved  : 7;
  };
  
  struct TDataOverrun
  {
    uint8_t Reserved1 : 1;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 6;
  };

  // ST2: Status 2
  struct TMagSensorOvrf
  {
    uint8_t Reserved1 : 3;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 4;
  };
 
  struct TOutBitSetting
  {
    uint8_t Reserved1 : 4;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 3;
  };
  
  // CNTL1: Control 1
  struct TMagMode
  {
    uint8_t Dest      : 4;
    uint8_t Reserved  : 4;
  };
  
  struct TMagBit
  {
    uint8_t Reserved1 : 4;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 3;
  };

  // CNTL2: Control 2
  struct TSoftReset
  {
    uint8_t Dest      : 1;
    uint8_t Reserved  : 7;
  };
  
  // ASTC: Self-Test Control
  struct TSelfTestControl
  {
    uint8_t Reserved1 : 6;
    uint8_t Dest      : 1;
    uint8_t Reserved2 : 1;
  };
  
  struct TAccData
  {
    union
    {
      struct
      {
        uint8_t ACCEL_XOUT_H; 
        uint8_t ACCEL_XOUT_L;
      };
      uint16_t ACCEL_XOUT;
    };
    union
    {
      struct
      {
        uint8_t ACCEL_YOUT_H;
        uint8_t ACCEL_YOUT_L;
      };
      uint16_t ACCEL_YOUT;
    };
    union
    {
      struct
      {
        uint8_t ACCEL_ZOUT_H;
        uint8_t ACCEL_ZOUT_L;
      };
      uint16_t ACCEL_ZOUT;
    };
  };
  
  
  struct TReadData
  {
    struct
    {
      union
      {
        struct
        {
          uint8_t ACCEL_XOUT_H; 
          uint8_t ACCEL_XOUT_L;
        };
        uint16_t ACCEL_XOUT;
      };
      union
      {
        struct
        {
          uint8_t ACCEL_YOUT_H;
          uint8_t ACCEL_YOUT_L;
        };
        uint16_t ACCEL_YOUT;
      };
      union
      {
        struct
        {
          uint8_t ACCEL_ZOUT_H;
          uint8_t ACCEL_ZOUT_L;
        };
        uint16_t ACCEL_ZOUT;
      };
    };
	 
	  struct
	  {
	    union
		  {
		    struct
		    {
	        uint8_t TEMP_OUT_H;  
	        uint8_t TEMP_OUT_L;
		    };
		    uint16_t TEMP_DATA;
		  }; 
	  }; 
	 
	  struct
	  {
	    union
		  {
		    struct 
		    {
	        uint8_t GYRO_XOUT_H; 
	        uint8_t GYRO_XOUT_L;
		    };
		    uint16_t GYRO_XOUT;
		  };
	    union
		  {
		    struct 
		    {
	        uint8_t GYRO_YOUT_H; 
	        uint8_t GYRO_YOUT_L;
		    };
		    uint16_t GYRO_YOUT;
		  };
	    union
		  {
		    struct 
		    {
	        uint8_t GYRO_ZOUT_H; 
	        uint8_t GYRO_ZOUT_L; 
		    };
		    uint16_t GYRO_ZOUT;
		  };  
	  };
  };  
}

#endif
