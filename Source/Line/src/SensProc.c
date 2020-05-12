// Информация о файле: $HeadURL: http://s005.sensor:18080/svn/Sens/trunk/SensProc/SensProc.c $
// $Revision: 26 $  $Date: 2016-11-21 16:25:33 +0300 (РџРЅ, 21 РЅРѕСЏ 2016) $  $Author: alb $
///////////////////////////////////////////////////////////////
// Обработка пакетов линии СЕНС
// Версия 0.9
//      : При запросе паролей командами 231, 232 восстановлена нумерация запрашивемых переменных
// Версия 0.8
//      : Поменяны местами строки 260, 261 для исключения зависания при обработке пунктов с меню в конце
// Версия 0.7
//   Отличия от 0.6
//      : SetSaveData сразу записывает настройки, так как настройки принимаются сразу после изменения
//   Отличия от 0.5
//      : Изменена функция iserr
//      : В заголовочный файл добавлен прототип функции GetReadOnly
//   Отличия от 0.4
//      : Тип S_PROGNUM теперь 32-х битный, не доступен старший байт
//   Отличия от 0.3
//      : Добавлено поле VarLength в структуру SensTable - указатель на переменную, хранящую текущую длину таблицы
//      : Добавлен тип S_VARFLOATTBL - таблица float с переменной длинной
// Версия 0.3.
//   Отличия от 0.2
//      : Добавлен опрос выключателя защиты записи настроек 
// Версия 0.2. 
//   Отличия от 0.1 
//      : работа с градуировочной таблицей (через файлы grdtab)
//      : можно расширить для прямой работы с таблицами в FRAM     
//      : исправлена процедура записи параметров
// Версия 0.1. Отличия от 0.0 - флаг SaveData теперь можно установить извне (например, при установке tPnt) 
//   Ограничения версии 0:
//   1. Невозможно работать на запись с таблицами, не копируемыми в ОЗУ (градуировочной, например)
//   2. Нет возможности описания меню, вложенных в пункт меню TN или SETT (такова работа процедуры считывания параметра)
#include "SensProc.h"
#include "sensline.h"
#include "cmdcodes_1.h"
#include "sens_types.h"
#include "Sensor.h"
//#include "sensdev.h"

uint8_t Access   = USERMODE; // Текущий режим доступа
uint8_t SaveData = 0;        // необходимость записи параметров


static uint8_t GetReadOnly(void);   // Возвращает 1 если запись запрещена

// Возвращает текущий уровень доступа
uint8_t GetAccessMode(void)
{
  return Access;
}
// Переключает на пользовательский режим без сохранения изменений
void ResetAccess(void)
{
  LoadConst();
  
  Access = USERMODE;
}

// Запись настроек
void SetSaveData(void) { 
  SaveConst(0);
  SaveData=0;
};

float theerr(void) {
  const uint8_t p[4]={0xFF,0xFF,0xFF,0xFF};
  return *((float*)(&p));
}


uint8_t isok(float f) {
  if ((f==0)||iserr(f)) return 0;
  return 1;
}

// Проверяет только 3 байта, без младшего
uint8_t iserr(const float f){
	// 0x7f800000 - бесконечность
	// 0xff800000 - минус бесконечность
	if(((*(__packed uint32_t*)&f) & 0x7fffffff) > (uint32_t)0x7f800000)
		return 1;
	else return 0;
}

const SensOneParam * FindParam(uint8_t idp) {
  const SensOneParam * pp;
  pp=Params;
  while (pp->ID) {
    if (pp->ID==idp) return pp;
    pp++;
  }
  return 0;
};

// Преобразование 3 байта -> float
float S3BToFloat(__packed uint8_t * buf) {
  float r=0;
  __packed uint8_t * p;
  p=(__packed uint8_t*)(&r);
  *(p)=(*buf&0x01)?0xFF:0x00;
  *(p+1)=*(buf++);
  *(p+2)=*(buf++);
  *(p+3)=*(buf++);
  return r;  
}

// Преобразование Float -> 3 байта
void FloatToS3B(__packed uint8_t*buf,float inf) {
  __packed uint8_t * p;
  p=((__packed uint8_t*)(&inf)+1);
  *(buf++)=*(p++);
  *(buf++)=*(p++);
  *(buf++)=*(p++); 
}

uint16_t ReadInt16(__packed uint8_t * buf) {
  uint16_t rez=0;
  __packed uint8_t * p;
  p=(__packed uint8_t*)&rez;
  *(p++)=*(buf++);
  *(p)=*(buf);
  return rez;
}

void WriteInt16(__packed uint8_t * buf,uint16_t zn) {
  __packed uint8_t * p;
  p=(__packed uint8_t*)&zn;
  *(buf++)=*(p++);
  *(buf)=*p;
}

float pp,dlt;

// Возвращает ноль если переход разрешен
// 1 - если надо запросить пароль
// 2 - если переход запрещен
uint8_t CheckPSWD(__packed uint8_t * buf, float sp)
{
  if (sp == 0)
  {
    return 0; // Без пароля
  }

  if (buf[0] < 5)
  {
    return 1; // Не введен пароль
  }

  pp  = S3BToFloat(buf + 5);
  dlt = sp - pp;

  if (dlt < 0)
  {
    dlt = -dlt;
  }

  if (pp < 0) 
  {
    pp = -pp;
  }

  if (dlt < (pp / 10000))
  {
    return 0;
  }

  return 2;
}

/// Обработка пакетов СЕНС
// Формат пакета: 
// 0 байт - длина БЕЗ ДАННОГО БАЙТА
// 1 байт - адрес
// 2 байт - команда
// 3 и далее - сам пакет
// Ответ формируется в том же буфере
// Возвращает 0 если пакет обработан
// В противном случае пакет не искажается
// Размер буфера не менее 65 байт
// Адрес не анализируется
// 

  uint32_t tbladr,tblnum; // Адрес и число ячеек для операций с таблицами

char SensAnswerPkg(uint8_t *buf) 
{
  uint32_t cmd = buf[2] & 0x1F;;
  uint32_t len = buf[0];
  uint8_t  prm;
  
  __packed uint8_t *indt;
  __packed uint8_t *outdt;
  __packed uint8_t *enddt;  //указатель на последний блок буфера
  __packed uint8_t *pdt;    //указатель для работы с данными

  const SensOneParam *fp;
  const SensTable    *tbl;

  if (len < 2)
  {
    return 1;
  } 
  if (buf[2] & 0x80)
  {
    return 2;
  }

  switch (cmd) 
  {
    case (CMD_SETPOINT):
         switch (buf[3]) //выполнение команд - выясняем номер команды
         {
           case (230):   //переход на пользовательский уровень
                Access = USERMODE;
                if (SaveData)
                {
                  SaveConst(0);
                } 
                SaveData=0;
                buf[4] = buf[3];
                buf[3] = 0x5A;
                buf[0] = 4;
                break;
           case (231):  //переход на администраторский уровень
           case (232):  //переход на супервайзерский уровень
                if (!SaveData) 
                {
                  LoadConst();
                }
                switch (CheckPSWD(buf, (buf[3] == 231) ? (HardDSt.PswAdmin) : (HardDSt.PswSuper))) 
                {
                  case (0):
                       Access = ((buf[3] == 231) ? ADMINMODE : SUPERMODE);
                       if (SaveData) 
                       {
                         SaveConst(0);
                       }
                       SaveData = 0;
                       buf[4]   = buf[3];
                       buf[3]   = 0x5A;
                       buf[0]   = 4;
                       break;
                  case (1): //дозапрос пароля
                       buf[0] = 8;
                       buf[4] = buf[3];
                       buf[5] = ((buf[3] == 231) ? PASSWD1 : PASSWD2);
                       buf[3] = 0x20;  // После этих ^^^
                       buf[6] = 0;
                       buf[7] = 0;
                       buf[8] = 0;
                       break;
                  default:  //отказ в выполнении
                       buf[4] = buf[3];
                       buf[3] = 0x00;
                       buf[0] = 4;
                       break;              
                }
                break;
           case (235): //СБРОС ПАРОЛЕЙ 
                if (GetAccessMode() == USERMODE)
                {
                  DSt.PswAdmin = 0;
                }
                if (GetAccessMode() == ADMINMODE)
                {
                  DSt.PswSuper = 31.41;
                }
                SaveConst(0);
                buf[4] = buf[3];
                buf[3] = 0x5A;
                buf[0] = 4;
                break;
           default:
//			       SetTCPnt(buf);
                switch (SetTCPnt(buf)) 
                {
                  case (0):          //выполнена
//                       buf[4] = buf[3];
//                       buf[3] = 0x5A;
//                       buf[0] = 4;
                       break;
                  case (1):          //отказ
                       buf[4] = buf[3];
                       buf[3] = 0x00;
                       buf[0] = 4;
                       break;
                  case (8):
                       return 0;     //дозапрос параметров
                  default:
                       return 4;
                }
         }
         break;
    case (CMD_START):      // Команда принудительного измерения - приравнивается к CMD_GETPI
         len = 4;
         FloatToS3B(buf + 3, CMD_GETPI);   //
    case (CMD_MENULEV):
         if (len < 4) 
         {
           return 3;
         }
         cmd = (uint8_t)S3BToFloat(buf + 3);
    case (CMD_GETPI):
    case (CMD_GETPS):
    case (CMD_GETTN):
         len = ThePredef(cmd, buf); // Считываем соответствующие пункты
    case (CMD_GETPR):
         if (len > 17)
         {
           len = 17;
         } 
         outdt  = buf + 3 + (len - 3) * 4;
         enddt  = outdt;
         indt   = buf + len;
         len   -= 2;           //количество параметров
         buf[0] = len * 4 + 2; //длина ответного пакета
         while (len) 
         {
           // Ищем очередной параметр
           prm = *(indt--);
           if ((len > 1) ? ((*indt) == MENUMOD) : 0) 
           {
             // Это - меню!
             --indt;
             --len;
             // Весь буфер надо сдвинуть на 4 байта ближе к началу
             outdt -= 4;     // vv Строки поменяны местами в версии 0.8
             pdt    = outdt; // ^^
             do 
             {
               *(pdt) = *(pdt + 4);
               ++pdt;
             } while (pdt != enddt);
             enddt  -= 4;
             buf[0] -= 4;
             // Формируем указатель на пункт меню
             *(outdt) = MENUMOD;
             FloatToS3B(outdt + 1, (float)prm);
           } 
           else 
           {
             // Работаем как с обычным параметром - это не меню
             *(outdt) = prm;
             fp = FindParam(prm);
             if (fp == 0) 
             {
               *(outdt+1) = 0xFF;
               *(outdt+2) = 0xFF;
               *(outdt+3) = 0xFF;
             } 
             else 
             {
               if (((fp->Type & 0xC0) >> 6) < (Access + 1)) 
               {
                 // Доступ закрыт
                 *(outdt+1) = 0xFF;
                 *(outdt+2) = 0xFF;
                 *(outdt+3) = 0xFF;
               } 
               else 
               {
                 switch (fp->Type & 0x0F) 
                 {
                   case (S_FLOAT):
                        FloatToS3B(outdt + 1, *((__packed float *)(fp->Pointer)));
                        break;
                   case (S_BYTE):
                        FloatToS3B(outdt+1, *((__packed uint8_t *)(fp->Pointer)));
                        break;
                   case (S_BITS):
                   case (S_PROGNUM):
                        *(outdt + 1) = *((__packed uint8_t *)(fp->Pointer));
                        *(outdt + 2) = *((__packed uint8_t *)(fp->Pointer) + 1);
                        *(outdt + 3) = *((__packed uint8_t *)(fp->Pointer) + 2);
                        break;
                   case (S_ADDR):
                        *(outdt + 1) = *(outdt + 2) = *(outdt + 3) = *((uint8_t *)(fp->Pointer));
                        break;
                   case (S_EIZM):
			   						   FloatToS3B(outdt + 1, *((__packed float *)(fp->Pointer)));
                        break;
                   case (S_TABLE):
                        FloatToS3B(outdt + 1, ((SensTable *)(fp->Pointer))->Length);
                        break;
                   case (S_VARFLOATTBL): 
                        {
                          float pl;
                          if (((SensTable *)(fp->Pointer))->VarLength) 
                          {
                            pl = *((SensTable *)(fp->Pointer))->VarLength;
                          } 
                          else 
                          {
                            pl = 0;
                          }
                          if (pl > ((SensTable *)(fp->Pointer))->Length)
                          {
                            pl = ((SensTable *)(fp->Pointer))->Length;
                          } 
                          FloatToS3B(outdt + 1, pl * 3);
                        }
                        break;
                   case (S_FLOATTBL):
                        FloatToS3B(outdt + 1,((SensTable *)(fp->Pointer))->Length * 3);
                        break;
                   case (S_FRAMTABLE):
                        break;
                   default:
                        *(outdt + 1) = 0xFF;
                        *(outdt + 2) = 0xFF;
                        *(outdt + 3) = 0xFF;
                 }
               }
             }
           }
           outdt -= 4;
           --len;
         }
         break;
    case (CMD_SETPR):
         outdt  = (buf + 3);
         indt   = (buf + 3);
         len    = (buf[0] - 2) / 4;
         buf[0] = 2;               // Длина ответного пакета
         if (GetReadOnly()) 
         {
           break; // Если запись запрещена - не выполняем
         }
         while (len) 
         {
           // Ищем очередной параметр
           prm = *(indt++);
           fp  = FindParam(prm);
           if (fp != 0) 
           {
             if (((fp->Type & 0x30) >> 4) >= (Access + 1)) //по значению больше, а по правам меньше
             {
               *(outdt++) = prm;
               switch (fp->Type & 0x0F) 
               {
                 case (S_FLOAT):
                      *((__packed float*)(fp->Pointer)) = S3BToFloat(indt);
                      buf[0]++;
                      break;
                 case (S_BYTE):
                      *((__packed uint8_t*)(fp->Pointer)) = S3BToFloat(indt);
                      buf[0]++;
                      break;
                 case (S_BITS):
                      *((__packed uint8_t*)(fp->Pointer))   = *(indt);
                      *((__packed uint8_t*)(fp->Pointer)+1) = *(indt + 1);
                      buf[0]++;
                      break;
                 case (S_ADDR):
                      if ((*(indt) == *(indt + 1)) && (*(indt + 1) == *(indt + 2))) 
                      {
                        *((uint8_t *)(fp->Pointer)) = *indt;
                        buf[0]++;
                      }
			   			 break;
                 case (S_EIZM):
//			   					    if ((*(indt) >= MIN_EIZM) && (*(indt) <= MAX_EIZM) \
//			   					    		&& (MIN_EIZM == *(indt + 1)) && (MAX_EIZM == *(indt + 2)))
//			   					    {
//			   					    	*((__packed float *)(fp->Pointer)) = S3BToFloat(indt);
//			   					    	buf[0]++;
//			   					    }
                      break;
                 case (S_TABLE):
                 case (S_FRAMTABLE):
                 case (S_VARFLOATTBL):
                 case (S_FLOATTBL):
                      break;
               }
               SaveData=1;
             }
           }
           indt += 3;
           --len;
         }
         break;
    //таблицы
    case (CMD_GETTBL):
          if (buf[0] >= 7) 
          {
            tbladr = ReadInt16(buf + 4);
            tblnum = ReadInt16(buf + 6);
            prm    = buf[3];
            fp     = FindParam(prm);
            if (((fp != 0) && (tblnum < 59)) ? (((fp->Type & 0xC0) >> 6) >= (Access + 1)) : 0) 
            {
              switch (fp->Type & 0x0F) 
              {
                case (S_TABLE):
                     //да, таблица есть и доступна
                     tbl = (const SensTable*) fp->Pointer;
                     if ((tbladr+tblnum)>tbl->Length)
                     {
                       tblnum=tbl->Length-tbladr;
                     } 
                     //копируем кучу байтов
                     indt  = (uint8_t *)tbl->Data + tbladr;
                     outdt = buf + 6;
                     for (len = 0; len < tblnum; len++) 
                     {
                       *(outdt++) = *(indt++);
                     }
                     buf[0] = tblnum + 5;
                     break;
                case (S_FRAMTABLE):
                     if (!fp->Pointer) 
                     {
                       //buf[0]=GTTableToByte(buf+6,tbladr,tblnum)+5;
                     }
                     break;
                case (S_VARFLOATTBL):
                case (S_FLOATTBL): 
                     {
                       float *flp;
                       uint8_t btflp;
                       //да, таблица есть и доступна, нужно преобразовывать во Float при обращении
                       tbl = (const SensTable *)fp->Pointer;
                       if ((tbladr + tblnum) > (tbl->Length * 3))
                       {
                         tblnum = (tbl->Length * 3) - tbladr;
                       } 
                       //копируем кучу байтов
                       flp   = (float *)(tbl->Data) + tbladr / 3;
                       btflp = tbladr % 3;
                       outdt = buf + 6;
                       for (len = 0; len < tblnum; len++) 
                       {
                         *(outdt++) = *(((uint8_t *)flp) + btflp + 1);
                         ++btflp;
                         if (btflp > 2) 
                         {
                           btflp = 0;
                           ++flp;
                         }
                       }
                       buf[0] = tblnum + 5;
                     } 
                     break;
                default:
                     buf[0] = 5;
              }
            } 
            else 
            {
              buf[0] = 5; // Возвращаем типа пустые данные
            }
          } 
          else 
          {
            return 3;
          }
          break;
    case (CMD_SETTBL):
		     if (GetReadOnly())  // Если запись запрещена - не выполняем 
		     {
		       buf[6] = 0;
		       buf[7] = 0;
		       buf[0] = 7;
		       break;   
		     }
         if (buf[0] >= 6) 
         {
           //ищем очередной параметр
           tbladr  = ReadInt16(buf + 4);
           tblnum  = buf[0];
           tblnum -= 5;
           prm     = buf[3];
           fp      = FindParam(prm);

           if ((fp != 0) ? (((fp->Type & 0x30) >> 4) >= (Access + 1)) : 0) 
           {
             //да, таблица есть и доступна
             switch (fp->Type & 0x0F) 
             {
               case (S_TABLE):
                    tbl = (const SensTable *)fp->Pointer;
                    if ((tbladr + tblnum) >= tbl->Length)
                    {
                      tblnum = tbl->Length-tbladr;          
                    } 
                    //копируем кучу байтов
                    indt  = buf + 6;
                    outdt = (uint8_t *)tbl->Data + tbladr;
                    for (len = 0; len < tblnum; len++)
                    {
                      *(outdt++) = *(indt++);
                    } 
                    WriteInt16(buf + 6, tblnum);
                    buf[0]   = 7;
                    SaveData = 1;
                    break;
               case (S_FRAMTABLE):
                    if (!fp->Pointer) 
                    {
                      //WriteInt16(buf + 6, GTByteToTable(buf + 6, tbladr, tblnum)); // Считывает со смещения adr num байт в градуировочную таблицу из буфера
                    } 
                    else 
                    {
                      buf[6] = 0;
                      buf[7] = 0;
                    }
                    buf[0] = 7;
                    break;
               case (S_VARFLOATTBL):
               case (S_FLOATTBL): 
                    {
                      float *flp;
                      uint8_t btflp;
                      
                      tbl = (const SensTable *)fp->Pointer;
                      if ((tbladr + tblnum) > (tbl->Length * 3))
                      {
                        tblnum=(tbl->Length * 3) - tbladr;
                      } 
                      // Копируем кучу байтов
                      indt  = buf + 6;
                      flp   = (float *)tbl->Data + tbladr / 3;
                      btflp = tbladr % 3;
                      for (len = 0; len < tblnum; len++) 
                      {
                        *(((uint8_t *)flp) + btflp + 1)= *(indt++);
                        ++btflp;                  
                        if (btflp > 2) 
                        {
                          *((uint8_t *)flp) = 0;
                          btflp = 0;
                          ++flp;
                        } 
                      }
                      WriteInt16(buf + 6, tblnum);
                      buf[0]   = 7;
                      SaveData = 1;
                    }
                    break;
               default:
                    buf[6] = 0;
                    buf[7] = 0;
                    buf[0] = 7; // Возвращаем типа ничего не записано              
                    break;
             }
           } 
           else 
           {
             buf[6] = 0;
             buf[7] = 0;
             buf[0] = 7; // Возвращаем типа ничего не записано
           }
         }
         break;    
         // Команда получения байта состояния
    case (CMD_GETSTATE):
//#################### Линия СЕНС #######################
#ifdef __SENS	
         if (!GetBSBuf(buf)) 
         {  // Если нет пакета - формируется пустой
#endif
//^^^^^^^^^^^^^^^^^^^^^ Линия СЕНС ^^^^^^^^^^^^^^^^^^^^^^
           buf[0]  = 3;
           buf[2] |= 0x80;
           buf[3]  = 0;
//#################### Линия СЕНС #######################
#ifdef __SENS	
         }
#endif
//^^^^^^^^^^^^^^^^^^^^^ Линия СЕНС ^^^^^^^^^^^^^^^^^^^^^^			
         break;  
    default:
         return 4;
  }

	if (SaveData /*&&(Access==USERMODE)*/ ) 
	{
    // Данные записываются сразу после изменения. Всегда.
     SaveConst(buf[3]);
     SaveData = 0;
	}
// Тут должен быть сформирован весь буфер. Остается выставить флаг ответа
  buf[2] |= 0x80;
  return 0;
}

//-----------------------------------------
// Возвращает 1 если запись запрещена
static uint8_t GetReadOnly(void) 
{
  return 0x00;
}

