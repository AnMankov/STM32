#include "modbus.h"
#include "map_reg.h"
#include "SensProc.h"
#include "SensDev.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

extern SemaphoreHandle_t SENSMutex;        //мьютекс данных протокола СЕНС
extern SemaphoreHandle_t MbCalStartSemphr; //запуск процесса калибровки через Modbus

float calparam; // Параметр команд калибровки (в основном для паролей)

// Чтение входных регистров
uint16_t GetInputRegisters(uint16_t adr)
{
  return GetHoldingRegisters(adr);
}

//uint16_t tpntflag = 99; //Результат выполнения калибровочных точек

TPntFlag PntFlag = _NO_CMD_EXEC; //Результат выполнения команды калибровки

const SensOneParam *sp;

float flt;

//Возвращает параметр СЕНС
//Возможные типы - S_FLOAT,S_BYTE,S_ADDR
//Для неизвестных типов возвращает 0xFFFF
//Для отсутствующих параметров и при передаче параметра только одним словом возвращается 0x0000
uint16_t GetSensParam(uint8_t pnum, uint8_t hiside) 
{
  uint16_t rez;
  uint8_t type;
  
  const __packed uint16_t *obp;
    
  xSemaphoreTake(SENSMutex, portMAX_DELAY); //захватить мьютекс для атомарной работы с нереентерабельной функцией FindParam
    sp = FindParam(pnum);
  xSemaphoreGive(SENSMutex);                //освободить мьютекс для атомарной работы с нереентерабельной функцией FindParam
   
  if (!sp) 
  {
    return 0xFFFF;
  }
  
  uint8_t GetAccessModeRet; 
  
  xSemaphoreTake(SENSMutex, portMAX_DELAY); //захватить мьютекс для атомарной работы с нереентерабельной функцией GetAccessMode
    GetAccessModeRet = GetAccessMode();
  xSemaphoreGive(SENSMutex);                //освободить мьютекс для атомарной работы с нереентерабельной функцией GetAccessMode
  
  if ( ((sp->Type & 0xC0) >> 6) < (GetAccessModeRet + 1) ) 
  {
    return 0xFFFF; //доступ закрыт
  }
  
  type = (sp->Type & 0x0F);
  
  switch (type) 
  {
    case (S_FLOAT):
          obp = ((uint16_t *)(sp->Pointer));
          if (hiside) 
		    {
            return *(obp + 1);
          } 
		    else 
		    {
            return *(obp);
          }
//		    break;
    case (S_BYTE):
         flt = (float)(*((uint8_t*)(sp->Pointer)));
         obp = ((uint16_t*)(&flt));
         if (hiside) 
		   {
           return *(obp + 1);
         } 
		   else 
		   {
           return *(obp);
         }
//		   break;
    case (S_PROGNUM):
         if (hiside) 
		   {
           //верхняя часть
           rez = (uint16_t)( *((int16_t *)(sp->Pointer) + 1) );
         } 
		   else 
		   {
           rez=(uint16_t)( *((int16_t *)(sp->Pointer)) );
         }
			
         return rez;
    case (S_ADDR):
         rez=(uint16_t)( *((int8_t*)(sp->Pointer)) );
         
			return rez;
    default:
	      break;
  }
  
  return 0xFFFF;
}

// Возвращает 0 если записано
// 1 если параметра нет
// 10 если не записано
uint8_t SetSensParam(uint8_t snum, uint16_t Val, uint8_t hiside) 
{
  uint8_t type;
  __packed uint16_t *obp;
    
  xSemaphoreTake(SENSMutex, portMAX_DELAY); //захватить мьютекс для атомарной работы с нереентерабельной функцией FindParam
    sp = FindParam(snum);
  xSemaphoreGive(SENSMutex);                //освободить мьютекс для атомарной работы с нереентерабельной функцией FindParam
  
  if (!sp) 
  {
    return 1;
  }
  
  uint8_t GetAccessModeRet; 
  
  xSemaphoreTake(SENSMutex, portMAX_DELAY); //захватить мьютекс для атомарной работы с нереентерабельной функцией GetAccessMode
    GetAccessModeRet = GetAccessMode();
  xSemaphoreGive(SENSMutex);                //освободить мьютекс для атомарной работы с нереентерабельной функцией GetAccessMode
  
  if ( ((sp->Type & 0x30) >> 4)<(GetAccessModeRet + 1) ) 
  {
    return 1; //доступ закрыт
  }
  
  type = sp->Type & 0x0F;
 
  switch (type) 
  {
    case (S_FLOAT):
         obp = ((uint16_t *)(sp->Pointer));
         if (hiside) 
		   {
           *(obp + 1) = Val;
         } 
		   else 
		   {
           *(obp) = Val;
         }
         break;
    case (S_BYTE):
         obp=((uint16_t *)(&flt));
         
		   if (hiside) 
		   {
           *(obp + 1) = Val;
           //сохранение в память
           *((uint8_t*)(sp->Pointer)) = (uint8_t)(flt);
         } 
		   else 
		   {
           *(obp) = Val;
           return 0;
         }
         break;
    case (S_PROGNUM):
         if (hiside) 
			{
            *((int16_t *)(sp->Pointer) + 1) = Val;
         } else {
            *((int16_t *)(sp->Pointer))     = Val;
         }
         break;
    case (S_ADDR):
         if (!hiside)
         {
			  *((int8_t *)(sp->Pointer)) = Val;
			}
         break;
    default:
	      return 10;
  }
  
  xSemaphoreTake(SENSMutex, portMAX_DELAY); //захватить мьютекс для атомарной работы с нереентерабельной функцией SaveConst
    SaveConst(0);
  xSemaphoreGive(SENSMutex);                //освободить мьютекс для атомарной работы с нереентерабельной функцией SaveConst
  
  return 0;
}

// Чтение регистров хранения
uint16_t GetHoldingRegistersREV(uint16_t Addr) 
{
  uint8_t IserrRetOA;
  uint8_t IserrRetSA;
  
  uint8_t MBAddr;
  uint8_t USpeed;
  uint8_t UPar;
  uint8_t ProtMod;
   
  xSemaphoreTake(SENSMutex, portMAX_DELAY); //захватить мьютекс для атомарной работы с нереентерабельной функцией iserr
    taskENTER_CRITICAL(); //к HardDSt без критической секции можно обращать только в задаче vLineData
      IserrRetOA = iserr(cdt.HC1_Angle);
      IserrRetSA = iserr(cdt.HC2_Angle);
		MBAddr  = HardDSt.MBAddr;
		USpeed  = HardDSt.USpeed;
		UPar    = HardDSt.UPar;
		ProtMod = HardDSt.ProtMod;
    taskEXIT_CRITICAL();
  xSemaphoreGive(SENSMutex);                //освободить мьютекс для атомарной работы с нереентерабельной функцией iserr

  switch (Addr)
  {
    /*
	  * Измеряемые целочисленные параметры
	  */
    case 0:
	      {			  
			  if ( IserrRetOA || IserrRetSA )
           {
		       return 0;
		     }		  
           return 1;
			}
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
	      return 0x8000; //0x8000 - ошибка значения для измеренных параметров в диапазоне 0-31
    /*
	  * Параметры связи Modbus
	  */
	 case 35:
	      return MBAddr;
	 case 36:
	      return USpeed;
	 case 37:
	      return UPar;
	 case 38:
	      return ProtMod;
  }

  /*
   * Измеряемые параметры (PNT7, PNT8, CELS, ERRCODE)
   */
  if (Addr >= 1000 && Addr <= 1999)
  {
    uint16_t SensNum = ( (Addr - 1000) >> 1 ) + 1;
	 
	 return GetSensParam(SensNum, Addr & 1);
  }

  /*
   *  Настройки устройства
   */
  if (Addr >= 2000 && Addr <= 2499)
  {
    uint16_t SensNum = ( (Addr - 2000 ) >> 1 ) + 32;
	 
	 return GetSensParam(SensNum, Addr & 1);
  }

  if (
      Addr >= 3000 && 
      Addr <  3004
	  )
  {
    __packed uint16_t * obp;
    obp = ((__packed uint16_t*)(&calparam));
    
	 //калибровочные команды
    switch (Addr) 
	 {
      case 3000:
		     return PntFlag;
      case 3001:
		     return 0;
      case 3002:
		     return *obp;     
      case 3003:
		     return *(obp+1);  
    }
  }
  
  return 0xFFFF; //запрашиваемым ячейкам не соответствует ни один параметр 
  
}

uint16_t GetHoldingRegisters(uint16_t Addr)
{
  uint16_t	rev;
  uint16_t  ByteSwapRet;
    
  rev = GetHoldingRegistersREV(Addr);
  
  xSemaphoreTake(SENSMutex, portMAX_DELAY); //захватить мьютекс для атомарной работы с нереентерабельной функцией ByteSwap
    ByteSwapRet = ByteSwap(rev);
  xSemaphoreGive(SENSMutex);                //освободить мьютекс для атомарной работы с нереентерабельной функцией ByteSwap
  
  return ByteSwapRet;
}

//Запись регистров хранения
//Возвращает 0 если записано
uint16_t SetHoldingRegisters(uint16_t Addr, uint16_t beVal)
{
  uint16_t leVal;

  xSemaphoreTake(SENSMutex, portMAX_DELAY); //захватить мьютекс для атомарной работы с нереентерабельной функцией ByteSwap
    leVal = ByteSwap(beVal);
  xSemaphoreGive(SENSMutex);                //освободить мьютекс для атомарной работы с нереентерабельной функцией ByteSwap

//  if (GetReadOnly())
//  {
//    return 1;
//  }

  /*
   * Параметры связи Modbus
   */
  if ( (Addr >= 35) && (Addr <= 38) )
  {
    if (GetAccessMode() >= USERMODE)
    {
	   //парметры интерфейса в USERMODE установить нельзя
	   return 1;
	 }
    switch (Addr)
    {
      case 35:
           taskENTER_CRITICAL(); //к DSt без критической секции можно обращать только в задаче vLineData
	          DSt.MBAddr  = leVal;
           taskEXIT_CRITICAL();
	  		  break;
      case 36:
           taskENTER_CRITICAL(); //к DSt без критической секции можно обращать только в задаче vLineData
	          DSt.USpeed  = leVal;
           taskEXIT_CRITICAL();
	  		  break;
      case 37:
           taskENTER_CRITICAL(); //к DSt без критической секции можно обращать только в задаче vLineData
	          DSt.UPar  = leVal;
           taskEXIT_CRITICAL();
	  		  break;
      case 38:
           taskENTER_CRITICAL(); //к DSt без критической секции можно обращать только в задаче vLineData
	          DSt.ProtMod  = leVal;
           taskEXIT_CRITICAL();
	  		  break;
	   default:
	        return 1;
    }
    xSemaphoreTake(SENSMutex, portMAX_DELAY); //захватить мьютекс для атомарной работы с нереентерабельной функцией SaveConst
      SaveConst(0);
    xSemaphoreGive(SENSMutex);                //освободить мьютекс для атомарной работы с нереентерабельной функцией SaveConst	 
  }
  else if (Addr >= 2000 && Addr <= 2499)
  { 
    /*
     *  Настройки устройства
     */ 
    uint16_t SensNum = ( (Addr - 2000 ) >> 1 ) + 32;
	 
	 return (SetSensParam(SensNum, leVal, Addr & 1) > 10);
  }
  else if (Addr >= 3000 && Addr <= 3004)
  {
    /*
     *  Калибровочные команды
     */ 
//	  _FAILURE     =  0,
//	  _PERFORMING  = 85,
//	  _DONE        = 90,
//	  _NO_CMD_EXEC = 99 
//   _IMPOSSIBLE  = 255
	  
    __packed uint16_t *obp;
    uint8_t ebuf[50];
	 uint8_t povnum;
	 obp = (__packed uint16_t *)&calparam;
	 
	 TDataReq *ReqEbuf = (TDataReq *)ebuf;
	 TDataAns *AnsEbuf = (TDataAns *)ebuf;
	 
	 switch (Addr)
	 {
      case 3000:
		     if (leVal == 0) 
	        {
             PntFlag = _NO_CMD_EXEC;
           } 
	        else 
	        {
             ReqEbuf->Pre.Len  = 3;
             povnum  = 0;
             
				 do 
             {
               ReqEbuf->Pre.Addr = 0x00;
               ReqEbuf->Pre.Cmd  = 0x1E; 
               ReqEbuf->CalCmd   = leVal;
               
               uint8_t SensAnswerPkgRet;
	 	        
//               if (leVal == _PLATFORM_CAL)                  
//	 	         {
//	 	           xSemaphoreGive(MbCalStartSemphr);
//	 	         }
           
               xSemaphoreTake(SENSMutex, portMAX_DELAY); //захватить мьютекс для атомарной работы с нереентерабельной функцией SensAnswerPkg
                 SensAnswerPkgRet = SensAnswerPkg(ebuf);		
	 	         xSemaphoreGive(SENSMutex);                //освободить мьютекс для атомарной работы с нереентерабельной функцией SensAnswerPkg
	 	        
	 	         if (SensAnswerPkgRet) //если пакет не обработан (невозможная ситуация)
	 	         {       
                  povnum   = 10; 
                  PntFlag  = _IMPOSSIBLE;
               }
	 	         else 
	 	         {
                 switch (AnsEbuf->Res)  //Проверка поля результата в сформированном SensAnswerPkg буфере
	 	           {
                   case _SUCCESS:
						      PntFlag  = _FAILURE;                  //0
			   	  	      povnum   = 10;
	 	        	         calparam =  0;
	 	        	         break;
	 	             case _SLOW:
	 	                  PntFlag  = _PERFORMING;               //85
			   	  	      povnum   = 10;
	 	                  break;
                   case _DENY:
	 	                  PntFlag  = _INTERM;                   //1
							   povnum   = 10;
	 	        	         break;
                   case _PARAMS_REQ:
                        //формируем параметры
                        ebuf[4] = ebuf[5];
                        ebuf[0] = 8;
                        FloatToS3B(ebuf + 5, calparam);
                        povnum++;
                        PntFlag = _REQ;                       //7
                        break;
                 }
               }
	          } while (povnum < 2);
				 
				 switch (PntFlag)
				 {
				   case _FAILURE:
                    PntFlag  = _DONE;    //90
					     break;
				   case _INTERM:
					     PntFlag  = _FAILURE; //0
					     break;
				   case _UNKNOWN:            //??? параметр не подошел
//					     break;
				   case _IMPOSSIBLE:
					     PntFlag  = _FAILURE; //0
					     break;
				   default:
					     break;
				 }
           }
		     break;
      case 3001:
		     break;
      case 3002:
		     *obp = leVal;
			  break;
      case 3003:
		     *(obp+1) = leVal;
			  break;
	 }
  }
  
  return 0; //записано, в т.ч. для всего неиспользуемого
}
