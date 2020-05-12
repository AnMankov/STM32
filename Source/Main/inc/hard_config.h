#ifndef __HARD_CONFIG_H
#define __HARD_CONFIG_H

#include "lib.h"

namespace TZ_Const
{
  constexpr static uint8_t  __POLL_PERIOD_MS = 10U;                      //период опроса в мс
  constexpr static uint16_t __START_HOLD_CTR = 100U  / __POLL_PERIOD_MS; //количество опросов за 0,1сек
  constexpr static uint16_t __ANY_PRESS_CTR  = __START_HOLD_CTR;         //количество опросов за 0,1сек
  constexpr static uint16_t __1_PRESS_CTR    = 1000U / __POLL_PERIOD_MS; //количество опросов за 1сек
  constexpr static uint16_t __3_5_PRESS_CTR  = 3500U / __POLL_PERIOD_MS; //количество опросов за 3,5сек
}

enum TPull : uint8_t
{
  _DOWN = 0,
  _UP   = 1,
};

struct TPin
{
  GPIO_TypeDef *Gpio;
  uint32_t      Nbr;
  uint32_t      ClkPortMask;
  void ( *en_clk )( uint32_t );    //включение тактирования порта с данным вывода
  uint32_t      AlternateMask;
};

struct TClk
{
  void ( *en_periph )( uint32_t ); //включение тактирования периферии
  uint32_t PeriphMask;             //маска тактирования
  uint32_t SrcMask;
  uint32_t FreqMask;
};

struct TPot_HW
{
  TPin CS;
  TPin SHDN;
  TPin RS;
  TPin CLK;
  TPin SDI;
};

struct TTmr_HW
{
  TIM_TypeDef *Nbr;
  IRQn_Type    IRQ;
  uint32_t     Ch;
  uint32_t     ARR_MAX; 
  TClk         Clk;
  TPin         Pin;
};

struct TExti
{
	uint32_t SysCfgPort; 
	uint32_t SysCfgLine; 
	uint32_t Line;   
	uint32_t Mode;   
	uint32_t Trigger;
	IRQn_Type IRQ;
  void ( *en_syscfg_clk )( uint32_t ); //включение тактирования SYSCFG
  uint32_t SysCfgMask;
};

struct TAdc_HW
{
  ADC_TypeDef        *Nbr;
  ADC_Common_TypeDef *CommNbr;
  IRQn_Type           IRQ;
  uint32_t            Ch;
  TClk                Clk;
  void ( *sel_clk_src )( uint32_t ); //выбор источника тактирования
  uint32_t            SrcMask;
  TPin                Pin;
  TPin                DIFF;
  TExti               Exti;
};

struct TUsart_HW
{
  USART_TypeDef *If;
  IRQn_Type      IRQ;
  TClk           Clk;
  TPin           TX;
  TPin           RX;
  TPin           DE;
};

struct TSpi_HW
{
  SPI_TypeDef   *If;
  IRQn_Type      IRQ;
  TClk           Clk; 
  TPin           SO;
  TPin           SI;
  TPin           SCK;
  TPin           CS;
};

struct TI2C_HW
{
  I2C_TypeDef *If;
  IRQn_Type    Ev_IRQ;
  IRQn_Type    Err_IRQ;
  TClk         Clk; 
  TPin         SCL;
  TPin         SDA;
  TPin         INT;
	TExti        Exti;
};

struct TDevAddr_HW
{
  TPin    Pin;
  uint8_t WeightCoeff;
  TPull   Pull;                       //подтяжка по умолчанию
};


extern TUsart_HW    UsartInt_HW;      //USART, через который МК подключен к датчику на крышке (для базового блока) \
                                        и к Modbus Master для датчика
extern TUsart_HW    UsartExt_HW;      //USART, через который МК подключен к внешней сети
extern TPin         DevDetect_HW;     //вывод определения вида устройства БАЗА/ДАТЧИК
extern TI2C_HW      I2C_HW;           //I2C, через который МК подключен к МЭМС
extern TSpi_HW      Spi_HW;           //SPI, через который МК подключена флэш память AT45DB321D

typedef TDevAddr_HW TDevAddr_HWr[4U];
extern TDevAddr_HWr DevAddr_HW;       //кодовый переключатель
                                      
extern TPin         DO_HW;            //вывод дискретного выхода
                                      
extern TPin         Led_HW;           //вывод, к которому подключен светодиод
extern TPin         Button_HW;        //вывод, к которому подключена кнопка
extern TTmr_HW      TmrFreg_HW;
extern TTmr_HW      TmrCmp_HW;
extern TTmr_HW      TmrDiff_HW;
extern TAdc_HW      Adc_HW;
extern TPot_HW      Pot_HW;
extern TPin         UCG1_HW;          //управление генератором GEN1

#endif
