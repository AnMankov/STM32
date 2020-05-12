#ifndef __LSM303DLHC_ACC_MAG_DRIVER_H
#define __LSM303DLHC_ACC_MAG_DRIVER_H

#include <vector>
#include <cstdint>

#include "LSM303DLHC_reg_description.h"
#include "LSM303DLHC_val_description.h"

/* Заголовочный файл драйвера модуля e-compass LSM303DLHC 
 * Использованная документация: LSM303DLHC Datasheet, 05-Nov-2013, Rev 2, Initial release 
 * Работа с драйвером:
 * 1. Создать производный класс, указав в списке наследования TAccMagDriver
 * 2. Запись настройки:
 *   2.1 Получить адрес регистра с настройкой:
 *       - Вызвать ..._select - функцию класса TAccMagDriver со спецификатором protected, ничего не передавая ей в первом параметре
 *       - Возврат функции - необходимый адрес регистра
     2.2 Считать настройку по интерфейсу (данная процедура д.б. обеспечена клиентом класса-драйвера)
	      - Считанную настройку сохранить в переменную типа std::uint8_t (например с именем Byte)
	  2.3 Изменить текущую настройку на требуемую:
	      - Вызвать ..._select - функцию класса TAccMagDriver со спецификатором protected, передав ей в первом параметре указатель на переменную из п.2.2
     2.4 Записать настройку по интерфейсу (данная процедура д.б. обеспечена клиентом класса-драйвера)
	      - Для записи использовать измененную переменную из п.2.2
 * 3. Считывание настройки/данных:
 *    - Повторить п.2.1 - п.2.2 для требуемого регистра   
*/

/* Вспомогательные типы и константы драйвера 
*/
namespace LSM303DLHC
{  
  constexpr TI2CProtocol ArrProtocolWrOne[]    = {ST, SADW, SAK, SUB, SAK, DATAM, SAK, SP}; //вектор с протоколом записи одного байта в LSM303DLHC
  constexpr std::uint8_t PROTOCOL_WR_ONE_SIZE = sizeof ArrProtocolWrOne / sizeof *ArrProtocolWrOne;
  
  constexpr TI2CProtocol ArrProtocolWrMul[]    = {ST, SADW, SAK, SUB, SAK, DATAM, SAK, DATAM, SAK, SP}; //вектор с протоколом записи нескольких байтов в LSM303DLHC
  constexpr std::uint8_t PROTOCOL_WR_MUL_SIZE = sizeof ArrProtocolWrMul / sizeof *ArrProtocolWrMul;
  
  constexpr TI2CProtocol ArrProtocolRdOne[]    = {ST, SADW, SAK, SUB, SAK, SR, SADR, SAK, DATAS, NMAK, SP}; //вектор с протоколом считывания одного байта из LSM303DLHC
  constexpr std::uint8_t PROTOCOL_RD_ONE_SIZE = sizeof ArrProtocolRdOne / sizeof *ArrProtocolRdOne;
  
  constexpr TI2CProtocol ArrProtocolRdMul[]    = {ST, SADW, SAK, SUB, SAK, SR, SADR, SAK, DATAS, MAK, DATAS, MAK, DATAS, NMAK, SP}; //вектор с протоколом считывания нескольких байтов из LSM303DLHC
  constexpr std::uint8_t PROTOCOL_RD_MUL_SIZE = sizeof ArrProtocolRdMul / sizeof *ArrProtocolRdMul; 
}

/* Основной класс драйвера
*/
namespace LSM303DLHC
{
  class TAccMagDriver
  {
  public:
    TAccMagDriver();
    ~TAccMagDriver();
  protected:
    const TI2CProtocol ProtocolWrOne[PROTOCOL_WR_ONE_SIZE] = {ST, SADW, SAK, SUB, SAK, DATAM, SAK, SP};                                         //вектор с протоколом записи одного байта в LSM303DLHC
	 const TI2CProtocol ProtocolWrMul[PROTOCOL_WR_MUL_SIZE] = {ST, SADW, SAK, SUB, SAK, DATAM, SAK, DATAM, SAK, SP};                             //вектор с протоколом записи нескольких байтов в LSM303DLHC
	 const TI2CProtocol ProtocolRdOne[PROTOCOL_RD_ONE_SIZE] = {ST, SADW, SAK, SUB, SAK, SR, SADR, SAK, DATAS, NMAK, SP};                         //вектор с протоколом считывания одного байта из LSM303DLHC
	 const TI2CProtocol ProtocolRdMul[PROTOCOL_RD_MUL_SIZE] = {ST, SADW, SAK, SUB, SAK, SR, SADR, SAK, DATAS, MAK, DATAS, MAK, DATAS, NMAK, SP}; //вектор с протоколом считывания нескольких байтов из LSM303DLHC

    const std::uint8_t ACC_ADDR_W = {0x32U}; //7-битный slave адрес акселерометра + 1 бит записи
    const std::uint8_t ACC_ADDR_R = {0x33U}; //7-битный slave адрес акселерометра + 1 бит считывания
    const std::uint8_t MAG_ADDR_W = {0x3CU}; //7-битный slave адрес магнитометра + 1 бит записи
    const std::uint8_t MAG_ADDR_R = {0x3DU}; //7-битный slave адрес магнитометра + 1 бит считывания
	 
	 const I2CModes Mode = {FAST}; //режим работы I2C
	     	 
	 /*******************************************************************************
    * Input         : ptr to Byte read from CTRL_REG1_A reg, PwrRate - Power mode and ODR selection
    * Return        : Register Address CTRL_REG1_A
    *******************************************************************************/
	 std::uint8_t accel_power_mode_odr_select(std::uint8_t *const Byte = NULL, CAccelPwrRate PwrRate = CAccelPwrRate::_PDM); //выбор режима потребления и скорости выходных данных
	 
	 /*******************************************************************************
    * Input         : ptr to Byte read from CTRL_REG1_A reg, AxesEn - Required axes
    * Return        : Register Address CTRL_REG1_A
    *******************************************************************************/
	 std::uint8_t accel_axes_select(std::uint8_t *const Byte = NULL, CAccelAxesEn AxesEn = CAccelAxesEn::_all_DIS); //выбор рабочих осей
	 
	 /*******************************************************************************
    * Input         : ptr to Byte read from CTRL_REG1_A reg, AccelMode - Required mode/power
    * Return        : Register Address CTRL_REG1_A
    *******************************************************************************/
	 std::uint8_t accel_mode_select(std::uint8_t *const Byte = NULL, CAccelMode AccelMode = CAccelMode::_NORMAL); //выбор режима работы/потребления
	 
	 /*******************************************************************************
    * Input         : 
    * Return        : Register Address CTRL_REG4_A
    *******************************************************************************/
	 std::uint8_t data_update_select(std::uint8_t *const Byte = NULL, CAccelMode AccelMode = CAccelMode::_NORMAL); //выбор режима работы/потребления
    
	 
	 
  private:
    template<typename T_Mode, typename T_Reg>
    void operate(std::uint8_t *const Byte, T_Mode Mode, T_Reg Reg)
    {
      if (Byte != NULL)
		{
		  ((T_Reg *)Byte)->Dest = static_cast<std::uint8_t>(Mode);
		}
	 }
  };
}

//extern LSM303DLHC::TAccMagDriver AccMagDriver;

#endif //__LSM303DLHC_ACC_MAG_DRIVER_H
