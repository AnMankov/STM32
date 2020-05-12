#include "SensDev.h"
#include "cmdcodes_1.h"
#include "SensProc.h"
//#include "main.h"
//#include "fasti2c.h"
//#include "sensor.h"
#include "ErrProc.h"
//#include "fram.h"
//#include "app.h"
#include "sensline.h"
#include "sens_types.h"
#include "model.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

//extern SemaphoreHandle_t WriteFlashSetsSemphr; //записать настройки во Flash микроконтроллера
//extern SemaphoreHandle_t WaitCalStartSemphr;   //ожидание окончание калибровки какого-либо из устройств в системе
//extern SemaphoreHandle_t MbCalStartSemphr;     //запуск процесса калибровки через Modbus

//extern SemaphoreHandle_t MagCalStartSemphr;    //запуск процесса калибровки магнитометра
//extern SemaphoreHandle_t HCMagCalStartSemphr;  //запуск процесса калибровки магнитометра на крышке люка
//extern SemaphoreHandle_t LMagCalStartSemphr;   //запуск процесса калибровки магнитометра на замке

//extern TaskHandle_t USARTHandle;               //задача - работа по USART
//extern TaskHandle_t LineDataHandle;            //задача - работа с данными по Линии

//extern bool MagCalReq;       //запрос на калибровку магнитометра, принятый через интерфейсы

//uint8_t cmd_handle(uint8_t *buf, SemaphoreHandle_t Semphr); //обработка команды калибровки
//void form_buf(uint8_t *buf);                                //формирование буфера для отправки данных

uint8_t dbg_val = 0x07;

////////////////////////////////// Параметры СЕНС-ПД
const float cnNumTPnt[3] ={7,3,3};    // Количество калибровочных точек в режиме супер, админ, пользователь соответственно
float NumTPnt = 3;

uint32_t emutime = 0; // Время включения режима эмуляции или 0 если выключен

uint8_t ProgNum[3] = {0x00,0xFC,0x00};    // Версия программы

// Настройки для сохранения
MySettings UserDSt;
MySettings FactoryDSt;

//TTCTab HardTCTab;     						// Таблица термокомпенсации для сохранения 
//TTCTab TCTab;     								// Таблица термокомпенсации текущая 

//const uint8_t bt8=8;
//const uint8_t bt9=9;
//const uint8_t bt24=24;
//const uint8_t bt41=41;
//const uint8_t bt55=55;
//#################### Линия СЕНС #######################
#ifdef __SENS
// Описатели таблиц
SensTable to_levtab=
{
  DSt.LevTab,LEVTABNUM
};

SensTable to_gistab=
{
  DSt.GisTab,GISTABNUM
};

//SensTable to_gistab_=
//{
//  (void*)DSt.GisTab_,GISTABNUM_
//};
#endif
//^^^^^^^^^^^^^^^^^^^^^ Линия СЕНС ^^^^^^^^^^^^^^^^^^^^^^

//SensTable to_temptab=
//{
//  &TCTab,TEMPTABNUM
//};

// СПИСОК ПАРАМЕТРОВ
// Завершает параметр с ID=0
const SensOneParam Params[] =
{
/* Настройки устройства
*/
  {PNT1,    READ_ALL|WRITE_ADM|S_FLOAT, &(DSt.DevSets.Thr)},     //порог срабатывания по углу
  {GIST1,   READ_ALL|WRITE_ADM|S_FLOAT, &(DSt.DevSets.Hyst)},    //гистерезис срабатывания по углу 
  {PNTB,    READ_ALL|WRITE_ADM|S_FLOAT, &(DSt.DevSets.Bias)},    //смещение по углу
  {ADPBITS, READ_ALL|WRITE_ADM|S_BITS,  (void *)&(DSt.AdpBits)}, //

/* Измеряемые
*/
  {LEVEL,       READ_ALL|WRITE_NON|S_FLOAT,  &(cdt.HC1_Angle)},     //угол крышки 1
  {CELS,        READ_ALL|WRITE_NON|S_FLOAT,  &(cdt.HC2_Angle)},     //угол крышки 2
  {PERCENT,     READ_ALL|WRITE_NON|S_FLOAT,  &(cdt.SysStatus)},     //состояние системы
//  {PNT9,        READ_ALL|WRITE_NON|S_FLOAT,   &(cdt.HC1_Angle)},     //угол крышки 1
//  {PNTA,        READ_ALL|WRITE_NON|S_FLOAT,   &(cdt.HC2_Angle)},     //угол крышки 2
//  {LEVEL,       READ_ALL|WRITE_NON|S_FLOAT,   &(cdt.OpenedAngle)},   //угол открытия
//  {CELS,        READ_ALL|WRITE_NON|S_FLOAT,   &(cdt.SealedAngle)},   //угол вскрытия
//  {CELS,        READ_ALL|WRITE_NON|S_FLOAT,   &(cdt.PTemper)},       //температура кристалла датчика на платформе
  {EPRM,        READ_ALL|WRITE_NON|S_FLOAT,   &(cdt.srab)},			       //номер последнего параметра, который вызвал срабатывание

/* Настройки интерфейсов
*/
  {MODADDR,     READ_ALL|WRITE_ADM|S_BYTE,    &(DSt.If.MBAddr)},
  {RS232SPEED,  READ_ALL|WRITE_ADM|S_BYTE,    &(DSt.If.USpeed)},
  {RS232PARITY, READ_ALL|WRITE_ADM|S_BYTE,    &(DSt.If.UPar)},

  // Обязательные для всех устройств
  {NEWVERSION,  READ_ALL|WRITE_NON|S_PROGNUM, (void *)&(ProgNum)},      //версия программы
  {VERSION,     READ_ALL|WRITE_NON|S_PROGNUM, (void *)&(ProgNum)},      //версия программы
  {MYADDR,      READ_ALL|WRITE_ADM|S_ADDR,    (void *)&(DSt.Addr)},     //адрес устройства в линии СЕНС
  {PASSWD1,     READ_ADM|WRITE_ADM|S_FLOAT,   (void *)&(DSt.PswAdmin)}, //пароль администратора
  {PASSWD2,     READ_SUP|WRITE_SUP|S_FLOAT,   (void *)&(DSt.PswSuper)}, //пароль суперадминистратора
  {ERRCODE,     READ_ALL|WRITE_NON|S_PROGNUM, (void *)&cdt.errcode},    //код ошибки
  {M_TPNT,      READ_ALL|WRITE_NON|S_FLOAT,   (void *)&NumTPnt},        //количество калибровочных точек в режиме администратора
//  {IZMPRM,	    READ_ALL|WRITE_NON|S_FLOAT,   (void*)&DSt.izmprm},     //наличие измеряемых параметров в датчике, используется для формирования меню LEV
  {0,0,0}
};

uint32_t emumode; // флаг режима эмуляции

void ReinitMemory(void)
{
/*
//// Начальное заполнение памяти
//	uint32_t n,l;
//	__packed uint8_t * p;
//	l=sizeof(DSt);
//	p=(__packed uint8_t *)(&DSt);
//	for (n=0;n<l;n++) *(p++)=0;
//	DSt.Addr = 1;
//	DSt.optsens = 0x00; // последовательный интерфейс
//	DSt.PswAdmin = 1234;
//	DSt.PswSuper = 31.41;
//	

//	//метод измерения температуры (PNT8,0xE8) (0x00,0x16,0x0A)	

//	// *((uint32_t*)(&HardDSt.tmethod)) = 0x0A160000; // метод измерения температуры
//	*((uint32_t*)(&DSt.tmethod)) = 0x0A101200; // метод измерения температуры
//	*((uint32_t*)(&DSt.izmprm)) = 0x00010000; // наличие измеряемых параметров(сейчас одно давление)
//	DSt.adcgain = 1; // коэффициент усиления АЦП по давлению = 2

//	DSt.kmul = 1; // мультипликативный коэф. PNT2
//	DSt.kadd = 0; // аддитивный коэф. PNT1
//	// В ЕДИНИЦАХ ИЗМЕРЕНИЯ ОТОБРАЖЕНИЯ ДОБАВИЛИСЬ ПРОЦЕНТЫ, 0x5E585B00 изменилось на 0x5F585B00 
////	*((uint32_t*)(&DSt.eizm5)) = 0x5E585B00; // единицы измерения отображения кгс/см^2
////	*((uint32_t*)(&DSt.eizmc)) = 0x5E585B00; // единицы измерения калибровки кгс/см^2
////	*((uint32_t*)(&DSt.eizmd)) = 0x5E585B00; // единицы измерения таблицы кгс/см^2
//	*((uint32_t*)(&DSt.eizm5)) = 0x5F585900; // единицы измерения отображения кПа
//	*((uint32_t*)(&DSt.eizmc)) = 0x5F585900; // единицы измерения калибровки кПа
//	*((uint32_t*)(&DSt.eizmd)) = 0x5F585900; // единицы измерения таблицы кПа
//	DSt.plow = 0; // PRESS_L
//	DSt.phigh = 600; // PRESS_H
//	
//	DSt.pgist = 0.1; // гистерезис

//	DSt.optsens = 0x00; // НЕ ВКЛЮЧАЕМ RS-485
//	// Параметры RS485
//	DSt.MBAddr = 1;
//	DSt.OAddr = 0;  
//	DSt.ProtMod = 0; // автоопределение протокола
//	DSt.UMode = 0;  
//	DSt.UPar = 0; //   битность/четность
//	DSt.USpeed = 5; // 19200
//	
////#################### Линия СЕНС #######################	
//#ifdef __SENS
//	SaveConst(0);
//#endif
////^^^^^^^^^^^^^^^^^^^^^ Линия СЕНС ^^^^^^^^^^^^^^^^^^^^^^
*/
}


//-------------------------------------
// Установить/сбросить режим эмуляции
// state = 1 - установить
//void EmulMode(uint8_t state)
//{
//	if(state)
//	{
//		emutime = zgetssec();		
//	}
//	else
//	{
//		emutime = 0;		
//		
//	}
//}


//-------------------------------------
// СБОРКА СТРУКТУРЫ МЕНЮ
uint8_t Menu(uint8_t mnum, uint8_t *buf)
{
  uint8_t pos    = 3;
//  uint8_t access = 2;
  switch (mnum)
  {
    case (CMD_GETPI):               //получение всех измеряемых параметров
//         buf[pos++] = PNT7;
//         buf[pos++] = PNT8;
//         buf[pos++] = CELS;
         buf[pos++] = LEVEL;
         buf[pos++] = CELS;
         buf[pos++] = PERCENT;
//         buf[pos++] = PERCENT;
//         access = GetAccessMode();  //получаем уровень доступа
//         if(access < 1)             //в режиме суперпользователя
//         {
//           buf[pos++] = PLTIZM;   //код АЦП канала давления
//         }
         break;
    case (CMD_GETPS):               //получение всех установочных параметров
         buf[pos++] = PNT1;
         buf[pos++] = GIST1;
         buf[pos++] = PNT2;
         buf[pos++] = GIST2;
         buf[pos++] = PNTB;
         buf[pos++] = PNTC;
         break;   
    case (CMD_GETTN):               //получение кода программы и всех таблиц
    	   buf[pos++] = VERSION;
//    	   buf[pos++] = LEVTAB;    	   
//			if(HardDSt.pgist == 0)
//    	   {
//           buf[pos++] =  GISTF;	
//    	   }   	   
			if (GetAccessMode() < USERMODE) 
    	   {
    	   	buf[pos++] = MENUMOD;
    	   	buf[pos++] = MENU_PSWD;
    	   }   	   
//			if(HardDSt.optsens & OPT_UART)
//    	   {  
//    	   	buf[pos++] = MENUMOD;
//    	   	buf[pos++] = MENU_SADA;
//    	   }   	   
         buf[pos++] = M_TPNT;
    	   buf[pos++] = MENUMOD;
         buf[pos++] = MENU_SADA;
//    	   if(GetAccessMode() == SUPERMODE)
//			{
//			  NumTPnt = cnNumTPnt[SUPERMODE]; //у суперадмина все калибровочные точки
//			}
//    	   else
//			{
//			  NumTPnt = cnNumTPnt[ADMINMODE]; //пользователь и администратор видит только 3 калибровочные точки
//			}
    	   break;
//    case (MENU_USER):                      //меню быстрого доступа
//    	   buf[pos++] = MENUMOD;
//    	   buf[pos++] = MENU_INPRM;
//    	   buf[pos++] = MENUMOD;
//    	   buf[pos++] = MENU_OUTPRM;
//    	   break;
//    case (MENU_INPRM):                     //вводимые параметры
//    	   if(IsEmuMode())
//			{
//			  buf[pos++] = PRESS;
//			}
//    	   break;
//    case (MENU_OUTPRM):                    //выводимые параметры 
//    	   buf[pos++] = PRESS;
//    	   break;
    case (MENU_PSWD):                      //пароли к режимам !!! УТОЧНИТЬ ПРИ ОТЛАДКЕ
    	   if (GetAccessMode() != USERMODE)
			{
			  buf[pos++] = PASSWD1;
			}
    	   if (GetAccessMode() == SUPERMODE)
			{
			  buf[pos++] = PASSWD2;
			} 
    	   break;
    case (MENU_SADA): //меню настройки адаптера
    	   buf[pos++] = RS232SPEED;
    	   buf[pos++] = RS232PARITY;
    	   buf[pos++] = MODIFY;
    	   buf[pos++] = MODADDR;
    	   break;
  }
 
  buf[0] = --pos;
  return pos;
}


////// Должна заполнить байты 3..N номерами параметров меню
////// полученную длину N-1 записать в buf[0] и вернуть как результат
////// Неизведанные меню - пустые
uint8_t ThePredef(uint8_t mnum,uint8_t * buf) 
{
	return Menu(mnum,buf);
}

//данная функция вызывается для копирования настроек в буфер в ОЗУ
void LoadConst(void) 
{
  Model.get_sets( DSt ); //запись в DSt из Model
}

//Данная функция вызывается когда нужно сохранить изменения в настройках
//При режиме "USER" - после каждого изменения (не рекомендуется использовать)
//При остальных режимах - при возврате к режиму USER
//Передаваемый параметр - код измененного параметра или 0, если сохранять все
void SaveConst(uint8_t pnum)
{
//  xSemaphoreGive( FlashData_WrSem ); //данные в DSt готовы, необходима валидация перед записью\
                                       в Model и во Flash микроконтроллера                                       
}

// Это "железное" сохранение. Выполняется из main после последнего логического изменения данных
// Сохраняет обе копии с подсчетом CRC
void HardSaveConst(uint8_t *Flag) 
{
	
}

// Считывает настройки из памяти в ОЗУ (в HardDSt) -с--п-р-о-в-е-р-к-о-й--CRC-
// Возвращает 0 в случае успеха
// 1 - устранимая ошибка
// 2 - не устранимая ошибка
uint8_t ReadConst(void) 
{
  //должны проверяться 2 банка настроек, хранящихся во внешней памяти (flash/eeprom/fram/...)
  uint8_t res;
//	res = EReadConst();
  return res;
}

// Считывает таблицу термокомпенсации из памяти в ОЗУ (в HardDSt) -с--п-р-о-в-е-р-к-о-й--CRC-
// Возвращает 0 в случае успеха
// 1 - устранимая ошибка
// 2 - не устранимая ошибка
uint8_t ReadTCTab(void) 
{
	uint8_t res;
	return res;
//	return EReadTCTable();
}

//Выполнение команды калибровки, не являющейся основной
//Вход - полный буфер пришедшего пакета
//На выходе может формироваться буфер и возвращается:
//  0 если команда выполнена (буфер формируется сам)
//  1 если команда не выполнена (буфер формируется сам)
//  8 если требуются дополнительные параметры (нужен сформированный буфер)
//255 если команда неизвестна (буфер должен быть оставлен без изменения)
//Номер команды - в buf[3]
uint8_t SetTCPnt(uint8_t *buf)
{
  if (GetAccessMode() >= ADMINMODE)
  {
    return 1; //калибровка доступна только для суперадминистратора
  }
    
  bool CalReq;
	
  taskENTER_CRITICAL();
//    CalReq = MagCalReq;
  taskEXIT_CRITICAL();
  
  if (CalReq) //если калибровка уже запрошена через ЛИН или через Modbus
  {
    return 255U; //команда неизвестна
  }
  return 255U;
//  else
//  {
//    /*
//     * Определяем задачу из которой произошла попытка запустить калибровку
//    */		
//    TaskHandle_t CurTaskHandle = xTaskGetCurrentTaskHandle();
//    
//    if (CurTaskHandle == USARTHandle)
//    {
//      return cmd_handle(buf, NULL);  //при запросе на калибровку через USART по Modbus, сбрасывать таймер обратного отсчета в МСК не нужно
//    }
//    else if (CurTaskHandle == LineDataHandle)
//    {
//      return cmd_handle(buf, WaitCalStartSemphr); //при запрос калибровки через ЛИН, необходимо сбрасывать таймер обратного отсчета в МСК
//    }
//    else
//    {
//      return 255; //SetTCPnt() вызвана некорректно
//    }
//  }
}

//uint8_t cmd_handle(uint8_t *buf, SemaphoreHandle_t Semphr)
//{
//  TDataReq DataReq = *(TDataReq *)buf;

//  if (Semphr == NULL) //если калибровка запрошена через USART по Modbus
//  {
//    switch (DataReq.CalCmd) 
//    {
//      case _PLATFORM_CAL:
//	        form_buf(buf);                       //формирование буфера для отправки ответа
//           xSemaphoreGive(MagCalStartSemphr);   //этот семафор будет ждать задача vAccGyroMag  	 
//           return 0; //команда выполнена
//      case _HC_1_CAL:
//	        form_buf(buf);                       //формирование буфера для отправки ответа
//           xSemaphoreGive(HCMagCalStartSemphr); //этот семафор будет ждать задача передачи данных по CAN на master'e\
//    		                                        передать датчику на крышке задание на калибровку магнитометра	    		
//           return 0; //команда выполнена
//      case _HC_2_CAL:
//	        form_buf(buf);                       //формирование буфера для отправки ответа
//           xSemaphoreGive(LMagCalStartSemphr);  //этот семафор будет ждать задача передачи данных по CAN на master'e\
//    		                                        передать датчику на замке задание на калибровку магнитометра
//           return 0; //команда выполнена
//      default:
//           return 255; //команда неизвестна
//    }
//  }
//  else //калибровка запрошена через ЛИН
//  {
//    switch (DataReq.CalCmd) 
//    {
//      case _PLATFORM_CAL:
//	        form_buf(buf);                       //формирование буфера для отправки ответа
//           xSemaphoreGive(Semphr);              //семафор для сброса обратного отсчета в МСК при нахождении в процессе калибровки
//           xSemaphoreGive(MagCalStartSemphr);   //этот семафор будет ждать задача vAccGyroMag 
//           return 0; //команда выполнена
//      case _HC_1_CAL:
//	        form_buf(buf);                       //формирование буфера для отправки ответа
//           xSemaphoreGive(Semphr);              //семафор для сброса обратного отсчета в МСК при нахождении в процессе калибровки
//           xSemaphoreGive(HCMagCalStartSemphr); //этот семафор будет ждать задача передачи данных по CAN на master'e\
//    		                                         передать датчику на крышке задание на калибровку магнитометра	    		
//           return 0; //команда выполнена
//      case _HC_2_CAL:
//	        form_buf(buf);                       //формирование буфера для отправки ответа
//           xSemaphoreGive(Semphr);              //семафор для сброса обратного отсчета в МСК при нахождении в процессе калибровки
//           xSemaphoreGive(LMagCalStartSemphr);  //этот семафор будет ждать задача передачи данных по CAN на master'e\
//    		                                         передать датчику на замке задание на калибровку магнитометра
//           return 0; //команда выполнена
//      default:
//           return 255; //команда неизвестна
//    }
//  
//  }
//}

//void form_buf(uint8_t *buf)
//{
//  ((TDataAns *)buf)->Pre.Len = ((TDataReq *)buf)->Pre.Len + 1; //к номеру команды добавляется результат выполнения
//  ((TDataAns *)buf)->CalCmd  = ((TDataReq *)buf)->CalCmd;
//  ((TDataAns *)buf)->Res     = _SLOW;                          //запросившее калибровку устройство должно будет ждать отправки успешного результата
//}

// Возвращает 1 если включен режим эмуляции
uint8_t IsEmuMode(void) 
{
  return emutime?1:0;
}


// Вызывать постоянно в основном цикле
void TCPProc(void) 
{
	if(emutime) 
	{
		if (tdlt(emutime) > 600000) emutime = 0;
	}
	
}

void SetProgNum(uint32_t pgn) {
  ProgNum[0]=(pgn>>16)&0xFF;
  ProgNum[1]=(pgn>>24)&0xFF;
  ProgNum[2]=(pgn>>8)&0xFF;
}
