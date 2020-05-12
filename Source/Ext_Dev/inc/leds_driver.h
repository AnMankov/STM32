//--------------------------------------------------------
// Файл описания классов драйвера Светодиодов
//--------------------------------------------------------
#ifndef __LEDS_DRIVER_H
#define __LEDS_DRIVER_H

#include "data_types.h"

//#define LEDS_NUMBER  8U

//enum TLedNum
//{
//  LED_GREEN_ONE = 0,
//  LED_ORANGE_ONE,
//  LED_RED_ONE,
//  LED_BLUE_ONE,
//  LED_GREEN_TWO,
//  LED_ORANGE_TWO,
//  LED_RED_TWO,
//  LED_BLUE_TWO,

//  LED_MAX
//};

//class TLedsDriver
//{
//public:
//  explicit TLedsDriver(const TPinHw &);
//  ~TLedsDriver();

//  void led_on()                    const; //включить
//  void led_off()                   const; //отключить
//  void toggle_led()                const; //переключить

//  const char *get_led_sign() const; //подписать
//private:
//  void hw_init_led_gpio()          const;
//  
//  TPinHw LedGpio;
//};

//class Digit
//{
//public:
//private:
//  unsigned char x;   
//};


class TLed
{
public:
  typedef GPIO_TypeDef * TPort;
  typedef std::uint32_t TClkMask;
  typedef std::uint32_t TPinMask;
  typedef char * TSign;

  TLed(const TPort, const TClkMask, const TPinMask, TSign);
  
  void toggle() const;
  void on() const;
  void off() const;
private:
  TPort GPIOx;        //порт
  TClkMask ClkMask;   //маска для разрешения тактирования
  TPinMask PinMask;   //вывод
  TSign Sign;         //подпись
};









//extern TLedsDriver LedNoActivity;
//extern TLedsDriver LedStationary;
//extern TLedsDriver LedWalking;
//extern TLedsDriver LedFastwalking;
//extern TLedsDriver LedJogging;
//extern TLedsDriver LedBiking;
//extern TLedsDriver LedDriving;

//extern const TLedsDriver *Leds[];
//extern unsigned char LedsNum;





#endif

