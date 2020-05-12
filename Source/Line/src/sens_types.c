#include "sens_types.h"
#include "sensline.h"


MySettings HardDSt;
MySettings DSt;

CurData cdt;

uint8_t MainFlags = 0x00;
uint32_t MFtime;

void SetMainFlag(uint8_t Flag) 
{
  MainFlags |= Flag;
  MFtime     = getssec();
}

const MySettings DefDSt = // Настройки текущие
{
/* Обязательные
*/
     1,    //Addr;
  1234,    //PswAdmin;
    31.41, //PswSuper;

/* Настройки устройства
*/
// порог, гистерезис, смещение  
  {25.0f, 10.0f,      0.0f}, //DevSets
  {
//   .Bits.Byte = 0x00, 
   .Bits.Item.Normal    = _N_OPENED, //Нормально разомкнуто
   .Bits.Item.Interconn = __BASE,    //Взаимодействие Датчик<->База
   
   .Reserved2 = 0xFF,
   .Reserved3 = 0x00
  },                      //ADPBITS

/* Настройки интерфейсов
*/
  {
    1, //MBAddr;            //адрес Modbus
    5, //USpeed;            //скорость 5 - 19200
    3, //UPar;              //четность и количество стоп битов: 3 - 8E1
  }
};
