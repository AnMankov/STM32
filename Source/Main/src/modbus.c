#include "modbus.h"
#include "map_reg.h"
#include "main.h"

//данные
uint8_t  mb_adr; 	//адрес ModBus
uint8_t  lstMode; //режим прослушивания 1-включен; 0-выключен

//диагностика
uint16_t dReg;    //диагностический регистр
uint16_t CNT0B;	//счетчик всех обнаруженных пакетов
uint16_t CNT0C;	//счетчик обнаруженных пакетов с ошибочным CRC
uint16_t CNT0D;	//счетчик ответных пакетов с исключениями
uint16_t CNT0E;	//счетчик пакетов, адресуемых нам
uint16_t CNT0F;	//счетчик пакетов, адресуемых нам на которые не было никаких ответов
uint16_t CNT10;	//счетчик пакетов, адресуемых нам с исключениями NAK (05)
uint16_t CNT11;	//счетчик пакетов, адресуемых нам с исключениями Slave Device Busy (06)
uint16_t CNT12;	//счетчик переполнений

//информация об устройстве (DKS-1)
const uint8_t SlaveID[] = {'D','K','S','-','1'};
const uint8_t szSlaveID = sizeof SlaveID;

////////////////////////
//инициализация ModBus
void MB_Init(uint8_t curmbadr) 
{
  lstMode = 0;
  mb_adr = curmbadr;
  
  if (mb_adr > 0xF7)
  {
    mb_adr = 0xF7;
  } 
}

uint8_t get_mb_addr()
{
  return mb_adr;
}

/////////////////////////
//сброс счетчиков ModBus
void MB_ResetCnt(void) 
{
  CNT0B = CNT0C = CNT0D = CNT0E = CNT0F = CNT10 = CNT11 = CNT12 = 0;
  dReg = 0;
}

////////////////////////////
//перестановка байт местами
uint16_t ByteSwap(uint16_t val)
{
	uint16_t res;
	res = val >> 8;
	res |= (val << 8);
	return res;
}

//////////////////////////////////////////
//расчет контрольной суммы пакетов ModBus
uint16_t CalcCRC(uint8_t *curbuf, int16_t cnt) 
{
  uint16_t crc = 0xffff;
  uint16_t n;
  uint16_t m;

  for (n = 0; n < cnt; n++) 
  {
    crc ^= curbuf[n];

    for (m = 0; m < 8; m++) 
    {
      if (crc & 1) 
      {
        crc >>= 1;
        crc ^= 0xA001;
      } 
      else 
      {
        crc >>= 1;
      }
    }
  }

  return crc;
}
//////////////////////////////////////////////////////////////////////////
//функция разбора пакета ModBus (u.rsbuf) и формирования ответа (u.trbuf)
//возвращает количество байт в ответе (u.trcnt)
//возвращает 0 если пакет разобран и требуется ответ
//           1 если пакет разобран, но адрес не наш или ответа не требуется
//          10 если пакет ошибочен
uint16_t MB_Parse(void)
{
  uint16_t crc;      //контрольная сумма
  uint16_t sub_cmd;  //подфункция
  uint16_t st_adr;   //начальный адрес
  uint16_t value;    //значение
  uint16_t quantity; //количество регистров
  uint16_t i;
  uint16_t tmp;

  uint8_t  cmd;      //код команды
  uint8_t  byte_cnt; //количество байт данных
  uint8_t  ex;       //код исключения
  uint8_t  answ;     //тип ответа: 0-как в запросе; 1-вручную; 2-без ответа

  CNT0B++;
  ex   = 0;
  answ = 1;
  
  //ошибка переполнения
  if (Usart.u.err & 0xC8)
  {
    ++CNT12;
    return 10;
  }
  
  //ошибка приема
  if (Usart.u.err || Usart.u.rscnt < 4)
  {
    CNT0C++;
    return 10;
  }

  // Проверка контрольной суммы
  crc = CalcCRC(Usart.u.rsbuf, Usart.u.rscnt - 2);
  
  if (crc != *((__packed uint16_t *)&Usart.u.rsbuf[Usart.u.rscnt - 2]))
  {
    CNT0C++;
    return 10;
  }
  
  // Проверка адреса (0-ой или mb_adr)
  if (Usart.u.rsbuf[0] != mb_adr && Usart.u.rsbuf[0] != 0)
  {
    return 1;
  }
  	
  CNT0E++;
  
  // Режим прослушивания
  if(lstMode)
  {
    if (Usart.u.rsbuf[1] != 0x08 || Usart.u.rsbuf[2] != 0x00 || Usart.u.rsbuf[3] != 0x01)
    {
    	CNT0F++;
    	return 1;
    }
  }
  
  // Разбираем пакет
  cmd                            = Usart.u.rsbuf[1];
  Usart.u.trcnt                  = 0;
  Usart.u.trbuf[Usart.u.trcnt++] = Usart.u.rsbuf[0];
  Usart.u.trbuf[Usart.u.trcnt++] = Usart.u.rsbuf[1];
  
  if (!ex)
  {
    switch (cmd)
    {
      //функция 0x03 - чтение регистров хранения
      //функция 0x04 - чтение входных регистров
      case 3:
      case 4:
           //ошибка приема, неверный размер кадра
           if (Usart.u.rscnt != 8)
           {
           	CNT0C++;
           	Usart.u.trcnt = 0;
           	return 1;
           }
           st_adr   = ByteSwap(*((__packed uint16_t *)(Usart.u.rsbuf + 2)));
           quantity = ByteSwap(*((__packed uint16_t *)(Usart.u.rsbuf + 4)));
           
           if (quantity < 1 || quantity > 0x007D)
           {
             ex = 3;
             break;
           }
           if (((long)st_adr + (long)quantity) > 0xFFFF)
           {
             ex = 2;
             break;
           }
           byte_cnt = 0;
           Usart.u.trcnt++;

           for (i = st_adr; i < (st_adr + quantity); i++) 
			  {
             if (cmd == 3)
             {
				       tmp = GetHoldingRegisters(i);
				     }
             else
				     {
				       tmp = GetInputRegisters(i);
				     }        

             Usart.u.trbuf[Usart.u.trcnt++] = (uint8_t)(tmp); 
             Usart.u.trbuf[Usart.u.trcnt++] = (uint8_t)(tmp >> 8); 
             byte_cnt += 2;
           }
           Usart.u.trbuf[2] = byte_cnt;
           
           break;

      // Функция 0x06 - запись одного регистра
      case 6:
           // Ошибка приема, неверный размер кадра
           if (Usart.u.rscnt != 8)
           {
             CNT0C++;
             Usart.u.trcnt = 0;
             
             return 1;
           }
           st_adr = ByteSwap(*((__packed uint16_t *)(Usart.u.rsbuf + 2)));
           value = *((__packed uint16_t *)(Usart.u.rsbuf + 4));
           
           if (SetHoldingRegisters(st_adr, value))
			  {
             ex = 4;
			  }

           answ = 0;
      break;
      
		  // Функция 0x08 - диагностика SerialLine
      case 8:
           if (Usart.u.rsbuf[2] != 0)
           {
             ex = 0x01;
             
             break;
           }
           
			  Usart.u.trbuf[Usart.u.trcnt++] = Usart.u.rsbuf[2];
           Usart.u.trbuf[Usart.u.trcnt++] = Usart.u.rsbuf[3];
           sub_cmd = ByteSwap(*((__packed uint16_t *)(Usart.u.rsbuf + 2)));           
           
           if (sub_cmd == 0x00 || sub_cmd == 0x01 || sub_cmd == 0x03)
           {
             answ = 0;

             //подфункции
             switch (sub_cmd) 
				     {
               //возврат полученных данных
               case 0x00:
                    break;
               
					     //сброс настроек связи
               case 0x01:
                    //ошибка приема, неверный размер кадра
                    if (Usart.u.rscnt != 8)
                    {
                      CNT0C++;
                      Usart.u.trcnt = 0;
                      
							 return 1;
                    }
                    
                    if (Usart.u.rsbuf[5] || (Usart.u.rsbuf[4] != 0x00 && Usart.u.rsbuf[4] != 0xFF))
                    {
                      ex = 3;
                      break;
                    }   
                    
						  if (Usart.u.rsbuf[4])
						  {
                      MB_ResetCnt();
						  }
                    
                    lstMode = 0;                	 
                    break;    
               //изменить ASCII символ десятичный разделитель
               case 0x03:
                    //ошибка приема, неверный размер кадра
                    if (Usart.u.rscnt != 8)
                    {
                      CNT0C++;
                      Usart.u.trcnt = 0;
                      
                      return 1;
                    }
                    
						  if (Usart.u.rsbuf[5])
                    {
                      ex = 3;   
                      break;
                    }	
                    break;					
             }
           }
           else
           {
             //ошибка приема, неверный размер кадра
             if (Usart.u.rscnt != 8)
             {
               CNT0C++;
               Usart.u.trcnt = 0;
               
               return 1;
             }

             // 2 байта нуля в данных
             if ((Usart.u.rsbuf[4] | Usart.u.rsbuf[5]))
             {
             	ex = 3;
             	
					break;
             }

             //подфункции
             switch (sub_cmd) 
				     {
               //прочитать регистр диагностики
               case 0x02:
               	  Usart.u.trbuf[Usart.u.trcnt++] = (uint8_t)(dReg >> 8);
               	  Usart.u.trbuf[Usart.u.trcnt++] = (uint8_t)(dReg);
						  
                    break;

               //войти в режим прослушивания
               case 0x04:
               	  lstMode = 1;
               	  answ    = 2;
						  
                    break;	

               //сброс счетчиков
               case 0x0A:
               	  MB_ResetCnt();
               	  answ = 0;
						  
                    break;

               //CNT0B
               case 0x0B:
               	  Usart.u.trbuf[Usart.u.trcnt++] = (uint8_t)(CNT0B >> 8); 
               	  Usart.u.trbuf[Usart.u.trcnt++] = (uint8_t)(CNT0B); 
                    
						  break;	

               //CNT0C
               case 0x0C:
               	  Usart.u.trbuf[Usart.u.trcnt++] = (uint8_t)(CNT0C >> 8);
               	  Usart.u.trbuf[Usart.u.trcnt++] = (uint8_t)(CNT0C);
						  
                    break;	

               //CNT0D
               case 0x0D:
               	  Usart.u.trbuf[Usart.u.trcnt++] = (uint8_t)(CNT0D >> 8);
               	  Usart.u.trbuf[Usart.u.trcnt++] = (uint8_t)(CNT0D);
                    
						  break;	

               //CNT0E
               case 0x0E:
               	  Usart.u.trbuf[Usart.u.trcnt++] = (uint8_t)(CNT0E >> 8);
               	  Usart.u.trbuf[Usart.u.trcnt++] = (uint8_t)(CNT0E);
                    
						  break;	

               //CNT0F
               case 0x0F:
                    Usart.u.trbuf[Usart.u.trcnt++] = (uint8_t)(CNT0F >> 8);
                    Usart.u.trbuf[Usart.u.trcnt++] = (uint8_t)(CNT0F);
						  
                    break;	

               //CNT10
               case 0x10:
               	  Usart.u.trbuf[Usart.u.trcnt++] = (uint8_t)(CNT10 >> 8); 
               	  Usart.u.trbuf[Usart.u.trcnt++] = (uint8_t)(CNT10); 
                   
						  break;	

               //CNT11
               case 0x11:
               	  Usart.u.trbuf[Usart.u.trcnt++] = (uint8_t)(CNT11 >> 8); 
               	  Usart.u.trbuf[Usart.u.trcnt++] = (uint8_t)(CNT11); 
                    
						  break;

               //CNT12
               case 0x12:
               	  Usart.u.trbuf[Usart.u.trcnt++] = (uint8_t)(CNT12 >> 8); 
               	  Usart.u.trbuf[Usart.u.trcnt++] = (uint8_t)(CNT12); 
                    
						  break;	

               //очистить счетчик переполнений и флаг
               case 0x14:
               	  CNT12 = 0;
               	  answ  = 0;
                    
						  break;

               default:
               	  ex=0x01;
                    
						  break;
           	 }
           }
           break;
      
		  //функция 0x10 - записать несколько регистров
      case 0x10:
      	  //ошибка приема, неверный размер кадра
      	  if (Usart.u.rscnt < 9)
      	  {
      	    CNT0C++;
      	    Usart.u.trcnt = 0;
      	    
             return 1;
      	  }
				
      	  st_adr   = ByteSwap( *((__packed uint16_t *)(Usart.u.rsbuf + 2)) );
      	  quantity = ByteSwap( *((__packed uint16_t *)(Usart.u.rsbuf + 4)) );     	  
      	  byte_cnt = Usart.u.rsbuf[6];
      	  
      	  if (
                quantity < 1                     || 
                quantity > 0x007B                || 
                byte_cnt != (2 * (long)quantity) ||
                (2 * (long)quantity + 9) != Usart.u.rscnt
				   )
      	  {
      	  	ex = 3;
 
      	  	break;
      	  }
 
      	  if ( ((long)st_adr + (long)quantity) > 0xFFFF )
      	  {
      	    ex = 2;
      	    
				  break;
      	  }	
      	  
			  //запись     	  
           for (i = st_adr; i < (st_adr + quantity); i++)
      	  {
      	    value = *((__packed uint16_t *)(Usart.u.rsbuf + 7 + (i - st_adr) * 2));
      	    
				  if ( SetHoldingRegisters(i, value) )
				  {
      	      ex = 4;
				  }
      	  }
 
      	  for (i = 2; i < 6; i++)
			  {
      	    Usart.u.trbuf[Usart.u.trcnt++] = Usart.u.rsbuf[i];
			  }
 
           break;

      //функция 0x11 - чтение информации об устройстве
      case 0x11:
           //ошибка приема, неверный размер кадра
           if (Usart.u.rscnt != 4)
           {
             CNT0C++;
             Usart.u.trcnt = 0;
             
             return 1;
           }
           
			     //SlaveID
           Usart.u.trbuf[Usart.u.trcnt++] = szSlaveID + 1;
           for (i = 0; i < szSlaveID; i++)
			  {
             Usart.u.trbuf[Usart.u.trcnt++] = SlaveID[i];
			  }

           //устройство работает
           Usart.u.trbuf[Usart.u.trcnt++] = 0xFF;
           
           break;
      default:
           ex = 0x01;
           
			  break;
    }
  }
  
  //отвечаем
  if(answ != 2)
  {
    //обработка исключений
    if (ex)
    {
    	Usart.u.trcnt                  = 0;
    	Usart.u.trbuf[Usart.u.trcnt++] = Usart.u.rsbuf[0];
    	Usart.u.trbuf[Usart.u.trcnt++] = Usart.u.rsbuf[1] | 0x80;
    	Usart.u.trbuf[Usart.u.trcnt++] = ex;
    }
    else if (answ == 0)
    {
      //отвечаем тем же, что и в запросе
      for (Usart.u.trcnt=0; Usart.u.trcnt < (Usart.u.rscnt - 2); Usart.u.trcnt++)
      {
        Usart.u.trbuf[Usart.u.trcnt] = Usart.u.rsbuf[Usart.u.trcnt];    	
      }
    }
    
    //подсчет CRC
    crc = CalcCRC(Usart.u.trbuf, Usart.u.trcnt);
    *((__packed uint16_t *)&Usart.u.trbuf[Usart.u.trcnt]) = crc;
    Usart.u.trcnt += 2; 	
  }
  else
  {
    CNT0F++;
  } 

  return 0;
}
