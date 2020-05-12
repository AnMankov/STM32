#ifndef __INTERFACE_H
#define __INTERFACE_H

#include <cstdint>

enum class TNum : std::uint8_t //константы - номера аппаратных интерфейсов в микроконтроллере
{
  ONE = 0,
  TWO,
  THREE,
  FOUR,
  FIVE
};

enum class TIFace : std::uint8_t
{
  _I2C,
  _CAN,
  _UART
};

class TInterface
{
public:
  virtual void init_if(TIFace) = 0;
  virtual void send_byte(const std::uint8_t byte) const = 0;
  virtual std::uint8_t recv_byte() const = 0;
protected:
private:
};

#endif
