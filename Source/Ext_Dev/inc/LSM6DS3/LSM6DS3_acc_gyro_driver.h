#ifndef __LSM6DS3_ACC_GYRO_DRIVER_H
#define __LSM6DS3_ACC_GYRO_DRIVER_H

#include <cstdint>

#include "LSM6DS3_addr.h"
#include "LSM6DS3_regs.h"
#include "LSM6DS3_vals.h"

namespace std
{
  typedef decltype(nullptr) nullptr_t;
}

namespace LSM6DS3
{
  using std::uint8_t;
  
  constexpr TI2CProtocol ArrProtocolWrOne[]    = {ST, SADW, SAK, SUB, SAK, DATAM, SAK, SP}; //вектор с протоколом записи одного байта в LSM303DLHC
  constexpr uint8_t PROTOCOL_WR_ONE_SIZE = sizeof ArrProtocolWrOne / sizeof *ArrProtocolWrOne;
  
  constexpr TI2CProtocol ArrProtocolWrMul[]    = {ST, SADW, SAK, SUB, SAK, DATAM, SAK, DATAM, SAK, SP}; //вектор с протоколом записи нескольких байтов в LSM303DLHC
  constexpr uint8_t PROTOCOL_WR_MUL_SIZE = sizeof ArrProtocolWrMul / sizeof *ArrProtocolWrMul;
  
  constexpr TI2CProtocol ArrProtocolRdOne[]    = {ST, SADW, SAK, SUB, SAK, SR, SADR, SAK, DATAS, NMAK, SP}; //вектор с протоколом считывания одного байта из LSM303DLHC
  constexpr uint8_t PROTOCOL_RD_ONE_SIZE = sizeof ArrProtocolRdOne / sizeof *ArrProtocolRdOne;
  
  constexpr TI2CProtocol ArrProtocolRdMul[]    = {ST, SADW, SAK, SUB, SAK, SR, SADR, SAK, DATAS, MAK, DATAS, MAK, DATAS, NMAK, SP}; //вектор с протоколом считывания нескольких байтов из LSM303DLHC
  constexpr uint8_t PROTOCOL_RD_MUL_SIZE = sizeof ArrProtocolRdMul / sizeof *ArrProtocolRdMul;
}

namespace LSM6DS3
{
  using std::uint8_t;
  
  class TAccGyroDriver
  {
  public:
    TAccGyroDriver();
    ~TAccGyroDriver();
  protected:
    const TI2CProtocol ProtocolWrOne[PROTOCOL_WR_ONE_SIZE] = {ST, SADW, SAK, SUB, SAK, DATAM, SAK, SP};                                         //вектор с протоколом записи одного байта в LSM303DLHC
	 const TI2CProtocol ProtocolWrMul[PROTOCOL_WR_MUL_SIZE] = {ST, SADW, SAK, SUB, SAK, DATAM, SAK, DATAM, SAK, SP};                             //вектор с протоколом записи нескольких байтов в LSM303DLHC
	 const TI2CProtocol ProtocolRdOne[PROTOCOL_RD_ONE_SIZE] = {ST, SADW, SAK, SUB, SAK, SR, SADR, SAK, DATAS, NMAK, SP};                         //вектор с протоколом считывания одного байта из LSM303DLHC
	 const TI2CProtocol ProtocolRdMul[PROTOCOL_RD_MUL_SIZE] = {ST, SADW, SAK, SUB, SAK, SR, SADR, SAK, DATAS, MAK, DATAS, MAK, DATAS, NMAK, SP}; //вектор с протоколом считывания нескольких байтов из LSM303DLHC

    const uint8_t I2C_ADDR_0_R = {0xD5U}; //6-бит slave адреса акселерометра + 1 бит SA0 подтяжка к "0" + 1 бит считывания
    const uint8_t I2C_ADDR_0_W = {0xD4U}; //6-бит slave адреса акселерометра + 1 бит SA0 подтяжка к "0" + 1 бит записи
	 
    const uint8_t I2C_ADDR_1_R = {0xD7U}; //6-бит slave адреса акселерометра + 1 бит SA0 подтяжка к "1" + 1 бит считывания
    const uint8_t I2C_ADDR_1_W = {0xD6U}; //6-бит slave адреса акселерометра + 1 бит SA0 подтяжка к "1" + 1 бит записи

	 const I2CModes Mode = {FAST}; //режим работы I2C
	 
//----- Управление регистром INT1_CTRL (0Dh) ----------------------------------------------------    
	 uint8_t gyro_data_ready_int1(uint8_t *const Byte = nullptr, VSwitch DRDY_G = VSwitch::_DISABLED); //подключение сигнала готовности данных гироскопа к выводу INT1
	 uint8_t acc_data_ready_int1(uint8_t *const Byte = nullptr, VSwitch DRDY_XL = VSwitch::_DISABLED); //подключение сигнала готовности данных акселерометра к выводу INT1
//-----------------------------------------------------------------------------------------------

//----- Управление регистром INT2_CTRL (0Eh)-----------------------------------------------------
	 uint8_t gyro_data_ready_int2(uint8_t *const Byte = nullptr, VSwitch DRDY_G = VSwitch::_DISABLED); //подключение сигнала готовности данных гироскопа к выводу INT2
	 uint8_t acc_data_ready_int2(uint8_t *const Byte = nullptr, VSwitch DRDY_XL = VSwitch::_DISABLED); //подключение сигнала готовности данных акселерометра к выводу INT2
//-----------------------------------------------------------------------------------------------

//----- Управление регистром WHO_AM_I (0Fh) -----------------------------------------------------
	 uint8_t who_am_i_rd(); //считывание идентификатора микросхемы (фиксированный 0x69h)
//-----------------------------------------------------------------------------------------------

//----- Управление регистром CTRL1_XL (10h) -----------------------------------------------------
	 uint8_t acc_aa_filter_bw(uint8_t *const Byte = nullptr, VAccAntiAliasingBW BW_XL = VAccAntiAliasingBW::_400_Hz); //выбор полосы сглаживающего фильтра акселерометра
	 uint8_t acc_full_scale(uint8_t *const Byte = nullptr, VAccFullScale FS_XL = VAccFullScale::_2_uG);               //выбор шкалы измерения ускорения
	 uint8_t acc_data_rate_power_mode(uint8_t *const Byte = nullptr, VAccDataRatePowerMode ODR_XL = VAccDataRatePowerMode::_POWER_DOWN); //выбор скорости выходных данных и режима потребления акселерометра
//-----------------------------------------------------------------------------------------------

//----- Управление регистром CTRL2_G (11h) ------------------------------------------------------
	 uint8_t gyro_full_scale_125(uint8_t *const Byte = nullptr, VSwitch FS_125 = VSwitch::_DISABLED);         //установка шкалы измерения угловой скорости - 125dps
	 uint8_t gyro_full_scale(uint8_t *const Byte = nullptr, VGyroFullScale FS_G = VGyroFullScale::_245_dps);  //выбор шкалы измерения угловой скорости
	 uint8_t gyro_data_rate(uint8_t *const Byte = nullptr, VGyroDataRate ODR_G = VGyroDataRate::_POWER_DOWN); //выбор скорости выходных данных гироскопа
//-----------------------------------------------------------------------------------------------

//----- Управление регистром CTRL3_C (12h) ------------------------------------------------------
	 uint8_t software_reset(uint8_t *const Byte = nullptr, VSoftReset SW_RESET = VSoftReset::_NORMAL);                               //программный сброс
	 uint8_t endian_data(uint8_t *const Byte = nullptr, VEndian BLE = VEndian::_LSB_LOWER);                                          //установка порядка байтов
	 uint8_t auto_increment(uint8_t *const Byte = nullptr, VSwitch IF_INC = VSwitch::_ENABLED);                                      //установка автоинкремента
	 uint8_t push_pull_open_drain(uint8_t *const Byte = nullptr, VPushPullOpenDrain PP_OD = VPushPullOpenDrain::_PUSH_PULL);         //установка режима выходов INT1 и INT2 - Push-pull или open-drain
	 uint8_t int_activation_level(uint8_t *const Byte = nullptr, VIntActivationLevel H_LACTIVE = VIntActivationLevel::_ACTIVE_HIGH); //установка уровня сигнала на аппаратном выводе при прерывании
	 uint8_t block_data_update(uint8_t *const Byte = nullptr, VBlockDataUpdate BDU = VBlockDataUpdate::_CONTINUOUS_UPDATE);          //установка режима обновления данныхна аппаратном выводе при прерывании
	 uint8_t reboot_memory_content(uint8_t *const Byte = nullptr, VRebootMemoryContent BOOT = VRebootMemoryContent::_NORMAL);        //перезагрузка содержимого памяти
//-----------------------------------------------------------------------------------------------

//----- Управление регистром CTRL4_C (13h) ------------------------------------------------------
	 uint8_t disable_i2c(uint8_t *const Byte = nullptr, VDisableI2C I2C_disable = VDisableI2C::_I2C_SPI_ENABLED);                         //вкл/выкл I2C
	 uint8_t data_ready_mask(uint8_t *const Byte = nullptr, VDataReadyMask DRDY_MASK = VDataReadyMask::_DISABLED);                        //маскирование DRDY пока не стабилизируются фильтры датчика
	 uint8_t int_available_int1(uint8_t *const Byte = nullptr, VDividedIntSignals INT2_on_INT1 = VDividedIntSignals::_DIVIDED_INT1_INT2); //выбор аппаратного вывода для преры
	 uint8_t gyro_sleep_mode(uint8_t *const Byte = nullptr, VGyroSleep SLEEP_G = VGyroSleep::_DISABLED);                                  //установка гироскопа в sleep
	 uint8_t acc_bw_selection(uint8_t *const Byte = nullptr, VAccBw XL_BW_SCAL_ODR = VAccBw::_BY_ODR);                                    //выбор фактора, определяющего полосу акселерометра
//-----------------------------------------------------------------------------------------------

//----- Управление регистром CTRL5_C (14h) ------------------------------------------------------
	 uint8_t acc_self_test_enable(uint8_t *const Byte = nullptr, VAccSelfTest ST_XL = VAccSelfTest::_DISABLED);      //вкл/выкл самотестирования акселерометра
	 uint8_t gyro_self_test_enable(uint8_t *const Byte = nullptr, VGyroSelfTest ST_G = VGyroSelfTest::_DISABLED);    //вкл/выкл самотестирования гироскопа
	 uint8_t burst_mode_read(uint8_t *const Byte = nullptr, VBurstModeRead ROUNDING = VBurstModeRead::_NO_ROUNDING); //выбор регистров для циклического пакетного считывания
//-----------------------------------------------------------------------------------------------
 
//----- Управление регистром CTRL6_C (15h) ------------------------------------------------------
	 uint8_t acc_high_perform(uint8_t *const Byte = nullptr, VAccHighPerform XL_HM_MODE = VAccHighPerform::_HP_ENABLED);         //вкл/выкл высокопроизводительного режима акселерометра
	 uint8_t gyro_level_latch(uint8_t *const Byte = nullptr, VGyroLevelLatch LVL2_EN = VGyroLevelLatch::_LATCH_DISABLED);        //вкл/выкл защелки по уровню для гироскопа
	 uint8_t gyro_data_level_trig(uint8_t *const Byte = nullptr, VGyroDataLevelTrig LVLen = VGyroDataLevelTrig::_TRIG_DISABLED); //вкл/выкл срабатывания по уровню для гироскопа
	 uint8_t gyro_data_edge_trig(uint8_t *const Byte = nullptr, VGyroDataEdgeTrig TRIG_EN = VGyroDataEdgeTrig::_TRIG_DISABLED);  //вкл/выкл срабатывания по фронту для гироскопа
//-----------------------------------------------------------------------------------------------
 
//----- Управление регистром CTRL7_G (16h) ------------------------------------------------------
	 uint8_t src_reg_rounding_func(uint8_t *const Byte = nullptr, VSrcRegRoundingFunc ROUNDING_STATUS = VSrcRegRoundingFunc::_DISABLED); //вкл/выкл регистры функции округления
	 uint8_t gyro_digit_hpf_reset(uint8_t *const Byte = nullptr, VGyroDigHpFilter HP_G_RST = VGyroDigHpFilter::_FILTER_RESET_OFF);       //сброс цифрового HPF гироскопа
	 uint8_t gyro_hpf_cutoff_freq(uint8_t *const Byte = nullptr, VGyroHpFilterCutoff HPCF_G = VGyroHpFilterCutoff::_0_0081_Hz);          //выбор частоты среза HPF гироскоп
	 uint8_t gyro_digit_hpf_en(uint8_t *const Byte = nullptr, VGyroDigHighPassFilter HP_G_EN = VGyroDigHighPassFilter::_HPF_DISABLED);   //вкл/выкл цифрового HPF гироскопа
	 uint8_t gyro_hp_mode_dis(uint8_t *const Byte = nullptr, VGyroHighPerform G_HM_MODE = VGyroHighPerform::_HP_ENABLED);                //вкл/выкл HP режим для гироскопа
//-----------------------------------------------------------------------------------------------

//----- Управление регистром CTRL8_XL (17h) -----------------------------------------------------
	 uint8_t lpf_on_6d(uint8_t *const Byte = nullptr, VLPFon6D ROUNDING_STATUS = VLPFon6D::_OFF);                                 //вкл/выкл LPF функции 6D
	 uint8_t acc_slopef_hpf(uint8_t *const Byte = nullptr, VAccHPSlopeFilter HP_G_RST = VAccHPSlopeFilter::_SLOPE_FILTER);        //выбор фильтра акселерометра
	 uint8_t acc_cutoff_freq(uint8_t *const Byte = nullptr, VAccHPFConfigCutoffSet HPCF_G = VAccHPFConfigCutoffSet::_SLOPE_XL_4); //конфигурация фильтра и настройка частоты акселерометра
	 uint8_t acc_lpf2_en(uint8_t *const Byte = nullptr, VAccLPF2 HP_G_RST = VAccLPF2::_OFF);                                      //выбор LPF2 акселерометра
//-----------------------------------------------------------------------------------------------

//----- Управление регистром CTRL9_XL (18h) -----------------------------------------------------
	 uint8_t acc_x_axis(uint8_t *const Byte = nullptr, VAccOutX Xen_XL = VAccOutX::_ENABLED); //вкл/выкл оси X акселерометра
	 uint8_t acc_y_axis(uint8_t *const Byte = nullptr, VAccOutY Yen_XL = VAccOutY::_ENABLED); //вкл/выкл оси Y акселерометра
	 uint8_t acc_z_axis(uint8_t *const Byte = nullptr, VAccOutZ Zen_XL = VAccOutZ::_ENABLED); //вкл/выкл оси Z акселерометра
//-----------------------------------------------------------------------------------------------

//----- Управление регистром CTRL10_C (19h) -----------------------------------------------------
	 uint8_t gyro_x_axis(uint8_t *const Byte = nullptr, VGyroOutX Xen_G = VGyroOutX::_ENABLED); //вкл/выкл оси X гироскопа
	 uint8_t gyro_y_axis(uint8_t *const Byte = nullptr, VGyroOutY Yen_G = VGyroOutY::_ENABLED); //вкл/выкл оси Y гироскопа
	 uint8_t gyro_z_axis(uint8_t *const Byte = nullptr, VGyroOutZ Zen_G = VGyroOutZ::_ENABLED); //вкл/выкл оси Z гироскопа
//-----------------------------------------------------------------------------------------------

//----- Управление регистром STATUS_REG (1Eh) ---------------------------------------------------
	 uint8_t acc_data_available(const uint8_t Byte = 0, VAccDataAvailable *XLDA = nullptr);      //новые данные от акселерометра доступны	 
	 uint8_t gyro_data_available(const uint8_t Byte = 0, VGyroDataAvailable *GDA = nullptr);     //новые данные от гироскопа доступны
	 uint8_t temper_data_available(const uint8_t Byte = 0, VTemperDataAvailable *TDA = nullptr); //новые данные от термодатчика доступны
//-----------------------------------------------------------------------------------------------
    
	 uint8_t get_gyro_addr_x_low() {return LSM6DS3_ACC_GYRO_OUTX_L_G;}  //младший байт данных оси X гироскопа
	 uint8_t get_gyro_addr_x_high(){return LSM6DS3_ACC_GYRO_OUTX_H_G;}  //старший байт данных оси X гироскопа
	 uint8_t get_gyro_addr_y_low() {return LSM6DS3_ACC_GYRO_OUTY_L_G;}  //младший байт данных оси Y гироскопа
	 uint8_t get_gyro_addr_y_high(){return LSM6DS3_ACC_GYRO_OUTY_H_G;}  //старший байт данных оси Y гироскопа
	 uint8_t get_gyro_addr_z_low() {return LSM6DS3_ACC_GYRO_OUTZ_L_G;}  //младший байт данных оси Z гироскопа  
	 uint8_t get_gyro_addr_z_high(){return LSM6DS3_ACC_GYRO_OUTZ_H_G;}  //старший байт данных оси Z гироскопа
	 
	 uint8_t get_acc_addr_x_low() {return LSM6DS3_ACC_GYRO_OUTX_L_XL;}  //младший байт данных оси X акселерометра
	 uint8_t get_acc_addr_x_high(){return LSM6DS3_ACC_GYRO_OUTX_H_XL;}  //старший байт данных оси X акселерометра
	 uint8_t get_acc_addr_y_low() {return LSM6DS3_ACC_GYRO_OUTY_L_XL;}  //младший байт данных оси Y акселерометра
	 uint8_t get_acc_addr_y_high(){return LSM6DS3_ACC_GYRO_OUTY_H_XL;}  //старший байт данных оси Y акселерометра
	 uint8_t get_acc_addr_z_low() {return LSM6DS3_ACC_GYRO_OUTZ_L_XL;}  //младший байт данных оси Z акселерометра
	 uint8_t get_acc_addr_z_high(){return LSM6DS3_ACC_GYRO_OUTZ_H_XL;}  //старший байт данных оси Z акселерометра
	 
	 
  private:  
    template<typename T_Mode, typename T_Reg>                  //для r/w регистров
    void operate(uint8_t *const Byte, T_Mode Mode, T_Reg Reg)
    {
      if (Byte != nullptr)
		{
		  ((T_Reg *)Byte)->Dest = static_cast<uint8_t>(Mode);
		}
	 }
	 
    template<typename T_Data, typename T_Reg>                   //для ro регистров
    void read(const uint8_t Byte, T_Data *Data, T_Reg Reg)      //Byte - считанный из RO регистра байт данных; *Data - переменная, в которую вносятся необходимые изменения 
    {
      if (Data != nullptr)
	   {
	     *Data = static_cast<T_Data>(((T_Reg *)Byte)->Dest);
	   }
    }	
  };
  
//  template<typename addr_type,
//           typename reg_type,
//			  const addr_type addr,
//			  const reg_type val = reg_type(0)>
//  class reg_access
//  {
//  public:
//    static void reg_set()
//	 {
//	   *reinterpret_cast<volatile reg_type *>(addr) = val;
//	 }
//	 static void reg_or()
//	 {
//	   *reinterpret_cast<volatile reg_type *>(addr) |= val;
//	 }
//	 static void reg_and()
//	 {
//	 
//	 }
//	 static reg_type reg_get()
//	 {
//	 
//	 }
//	 
//	 static void bit_set()
//	 {
//	 
//	 }
//	 
//	 static void bit_clr()
//	 {
//	 
//	 }
//	 
//	 static void bit_not()
//	 {
//	   *reinterpret_cast<volatile reg_type *>(addr) ^= reg_type(reg_type(1U) << val);
//	 }
//	 
//	 static void bit_get()
//	 {
//	 
//	 }
//	 
//	 static void variable_reg_set(const reg_type)
//	 {
//	 
//	 }
//	 
//  };
  // Пример установки порта в 0:
  // reg_access<std::uint8_t,
  //            std::uint8_t,
  //            mcal::reg::portb,
  //            std::uint8_t(0x00)>::reg_set();
//  reg_access<std::uint8_t,
//             std::uint8_t,
//             mcal::reg::portb,
//             std::uint8_t(0x00)>::

  //Обобщенный шаблонный класс для распределения памяти регистра
//  struct bit8_type
//  {
//    std::uint8_t b0 : 1;
//    std::uint8_t b1 : 1;
//    std::uint8_t b2 : 1;
//    std::uint8_t b3 : 1;
//    std::uint8_t b4 : 1;
//    std::uint8_t b5 : 1;
//    std::uint8_t b6 : 1;
//    std::uint8_t b7 : 1;
//  }
//  
//  template<typename addr_type,
//           typename reg_type,
//			  typename bits_type,
//			  const addr_type addr>
//  class reg_map
//  {
//  public:
//    static reg_type &value()
//	 { 
//	   return *reinterpret_cast<volatile reg_type *>(addr);
//	 }
//	 
//    static bits_type &bits()
//	 {
//	   return *reinterpret_cast<volatile bits_type *>(addr);
//	 }
//  }


  

  // Пример использования для доступа к регистру portb
/*
  reg_map<std::uint8_t,
          std::uint8_t,
			 bit8_type,
			 portb>::value() = uint8_t(0);
			 
  reg_map<std::uint8_t,
          std::uint8_t,
			 bit8_type,
			 portb>::bits().b5 = 1U;
*/  
			
}
 

#endif
