#ifndef __I2C_DRIVER_H
#define __I2C_DRIVER_H

#include "hard_config.h"
#include "lib.h"
//#include "data_types.h"

/*******************************************************************
 * Типы используемые в I2C драйвере 
 *******************************************************************
 */
namespace I2C
{
  using std::uint32_t;
  using std::uint8_t;
     
  struct TI2CClkSrc
  {
    uint32_t Pclk1;
    uint32_t Hsi16;
    uint32_t SysClk;
  };
  	 
  struct TI2CHw
  {
    I2C_TypeDef  *I2Cx;      //интерфейс
    GPIO_TypeDef *GpioSDA;   //порт вывода SDA
    GPIO_TypeDef *GpioSCL;   //порт вывода SCL
    uint32_t SDAPinClkMask;  //маска для разрешения тактирования порта с SDA
    uint32_t SCLPinClkMask;  //маска для разрешения тактирования порта с SCL
    TI2CClkSrc ClkSrc;       //маска для выбора источника тактирования
    uint32_t PeriphClkMask;  //маска для разрешения тактирования периферии
    uint32_t SDAPinMask;     //маска вывода SDA
    uint32_t SCLPinMask;     //маска вывода SCL
    char *Sign;              //подпись
  };

  enum class TMode  //режим, в котором должен работать I2C контроллер
  {
    _SLAVE = 0,
	  _MASTER,
	  _SMBUS_HOST,
	  _SMBUS_DEVICE
  };
  
  enum class TRate : uint8_t
  {
    _STANDARD,                //до 100кГц
    _FAST,                    //до 400кГц
    _FAST_MODE_PLUS           //до 1МГц
  };
  
  enum class TAddressing : uint8_t
  {
    _7_BIT,
	 _10_BIT
  };

  enum class TDirect : uint8_t
  {
    _SLAVE_TRANSMITTER,   //Slave режим установлен по умолчанию
	 _SLAVE_RECEIVER,
	 _MASTER_TRANSMITTER,
	 _MASTER_RECEIVER
  };
  
  enum class TOperation : bool
  {
    _READ = 0,
	 _WRITE
  };
}

/*******************************************************************
 * Основной класс I2C драйвера
 *******************************************************************
 */
namespace I2C
{
  using std::uint32_t;
  using std::uint8_t;

//  class TI2C : public TInterface
  class TI2C
  {
  public:
    TI2C(
         const TI2C_HW &_HW,
	       TMode Mode             = TMode::_MASTER,
			   TRate Rate             = TRate::_STANDARD,
			   TAddressing Addressing = TAddressing::_7_BIT 
        );
    ~TI2C();

    static constexpr uint8_t BUF_SIZE = 10;	 
    const char *get_i2c_sign() const; //подписать

	  uint8_t Addr;                //адрес микросхемы
	  uint8_t SlaveRegAddr;        //адрес регистра в slave
	  uint8_t TransferSize;
	  uint8_t RxData[BUF_SIZE];
	  uint8_t TxData[BUF_SIZE];
	  static bool SlaveAddrFlag;   //флаг отправки адреса slave-регистра
	  static TOperation Operation; //флаг текущей операции 

  protected:
    void pin_clk_config(); //инициализация выводов I2C и выбор источника тактирования I2C
    void i2c_hw_init();
	 
    void write_single_byte(uint8_t *TxByte, uint8_t RegAddr, uint8_t SlaveAddr);                  //запись одного байта
	  void write_burst(uint8_t *DataBurst, uint8_t Size, uint8_t StartRegAddr, uint8_t SlaveAddr); //многобайтная запись
    
	  void read_single_byte(uint8_t *RxByte, uint8_t RegAddr, uint8_t SlaveAddr);                  //считывание одного байта
	  void read_burst(uint8_t *DataBurst, uint8_t Size, uint8_t StartRegAddr, uint8_t SlaveAddr);  //многобайтное считывание
		
    TI2C_HW HW;

  private:  
    void i2c_set_interrupt();
    void start_transfer(uint8_t SlaveAddr); //универсальная функция, реализующая общую часть для всех протоколов
    	 
	  TMode Mode;
	  TRate Rate;
	  TAddressing Addressing; //режим адрессации
  };
}
#endif //__I2C_DRIVER_H
