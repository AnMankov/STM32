// Информация о файле: $HeadURL: http://s005.sensor:18080/svn/Sens/trunk/SensLine/sensline.c $
// $Revision: 27 $  $Date: 2017-09-13 15:27:21 +0300 (Ср, 13 сен 2017) $  $Author: alb $
/////////////////////////////////////////////////////////////////////////////
// Реализация транспортного уровня линии СЕНС - исходный текст
// Версия 0.6
//   Отличия 0.6 - Введено определение PINSPEED, так как определения различны у разных контроллеров
//               - Исправлена инициализация портов (убрано прямое упоминание GPIOA)
//               - Введена инициализация структуры TimInitStruct
//   Отличия 0.5 - Введена функция CheckBSTr()
//   Отличия 0.4 - введено определение INPINAF для выбора альтернативной функции входной 
//                 ножки при работе без компаратора
//   Отличия 0.3 - введена функция zgetssec
//   Отличия 0.2 - в строке 264 введено использование PCLK1 для ПМП201 
//   Отличия 0.1 - в строчке 256 PCLK2 заменено на HCLK (из-за контроллера stm32f051). 
//                 В строчке 193 проинвертирован выход компаратора (для контроллера stm32f051)


#include "sensline.h"

#ifndef PINSPEED
  #define PINSPEED GPIO_Speed_Level_2
#endif

///////////////////// Занимает ресурсы:
// TMR3
// Ножку для передачи

// Состояние автомата линии
#define SS_WAIT  0x00
#define SS_P1    0x21
#define SS_P2    0x22
#define SS_P3    0x23
#define SS_P4    0x24
#define SS_P7    0x27
#define SS_P8    0x28
#define SS_CHZZZ 0x30 
#define SS_SYNC  0x40
#define SS_PP1   0x51
#define SS_PP2   0x52
#define SS_PP3   0x53
#define SS_PP4   0x54
#define SS_PP5   0x55
#define SS_PP6   0x56
#define SS_PP7   0x57
#define SS_PP8   0x58

uint8_t SensState=SS_WAIT;       
uint32_t ssec;     // Текущее время в миллисекундах

// Действие (символ D в схеме автомата состояния)
#define SD_WAITSISP   0
#define SD_GENSI      1
#define SD_INDEPEND   2
#define SD_TRANSMIT   3
volatile uint8_t SensD=SD_WAITSISP;

// Конфигуратор линии СЕНС
// 7-й бит - генерация синхроимпульсов устройством (устанавливается пользователем через ConfigSENS)
// 0-й бит=1 если передаются блоки состояний
#define IGENSYNC 0x80
#define EMPTYRX  0x40
#define NOPS     0x04     // Запрет передачи блока состояний
#define PSMODE   0x02     // Передается блок состояний
#define CANTRPS  0x01
volatile uint8_t SensConfig; 

#define SetCFGB(N) {SensConfig|=N;}
#define ResetCFGB(N) {SensConfig&=~N;}

// Константы времени, задаются из расчёта 1 тик = 0.025 мс
// Вариации на тему 0.25мс
#define TIME0_25MS (250/25)      // 0.25 миллисекунды
#define TIME_TOP3  8             // Время перед разрешением синхронизации
/// Вариации на тему 0.5мс
#define TIME0_5MS (500/25)       // 0.5 миллисекунды
#define TIME_WAITSYNC    28      // Время ожидания синхронизации
#define TIME_BITSYNC     16      // Время после фронта до получения сигнала
//
#define TIME1MS   (1000/25)      // 1 миллисекунда - для getssec
#define TIME1_5MS (1500/25)      // 1.5 миллисекунды
#define TIME2MS   (2000/25)      // 2 миллисекунды
#define TIME2_5MS (2500/25)      // 2 миллисекунды
#define TIME3MS   (3000/25)      // 3 миллисекунды
#define TIME4MS   (4000/25)      // 4 миллисекунды
#define TIME14MS  (15000/25)     // Пауза до генерации синхроимпульса
#define TIME20MS  (20000/25)     // Ожидание синхроимпульса или синхропаузы
#define TIME30MS  (30000/25)     // Пауза до начала генерации СИ
#define TIME50MS  (50000/25)     // Пауза до перехода в автономную работу

////////////////////////////////////////////////////////////////////////////
// Переменные процесса обмена по линии
volatile char Ni=0;          // Номер текущего синхроимпульса
volatile char NCK=0;         // Флаг состояния нового цикла линии
volatile uint8_t * BSPointer=0; // Указатель на буфер блока пакетов состояний (инициализируется в основной программе)
volatile uint8_t * curBSPointer;// Указатель внутри буфера блока пакетов состояний

#define BUFFERS_LENGTH 256

// Буфер передачи - формат аналогичен буферу состояний, но буфер кольцевой и пакеты после передачи выкидываются
uint8_t TrBuffer[BUFFERS_LENGTH];
volatile uint8_t * TrBufPointerTail=TrBuffer; // Указатель на не переданный пакет в буфере
volatile uint8_t * TrBufPointerHead=TrBuffer; // Указатель на начало свободного места в буфере
// Переменные передатчика
volatile uint8_t * curtrbuf; // Передаваемый пакет
volatile uint8_t curnum;     // Число передаваемых байт
volatile uint8_t trnum=0;    // Число пройденных попыток передать пакет
// Общие переменные приемопередатчика
volatile uint8_t crc;        // Контрольная сумма
volatile uint8_t curbyte;    // Текущий байт
volatile uint8_t curbit;     // Текущий бит
// Переменные приемной части
volatile uint8_t * currdbuf; // Указатель на приемный буфер при приеме
volatile uint8_t ZZZ;        // Битовые значения Z3, Z2, Z1 (младшие 3 бита, bit0=Z1)
volatile uint8_t bytecnt;    // Счетчик байтов в пакете
// Буфер приема 
// Первый байт - длина всего пакета, далее - пакет. И так - по кругу.
volatile uint8_t RdBuffer[BUFFERS_LENGTH];
volatile uint8_t * RdBufPointerTail=RdBuffer; // Указатель 
volatile uint8_t * RdBufPointerHead=RdBuffer; // Указатель 

// Получить последний пакет с линии
// Возвращает 0 если пакетов нет или длину пакета - если есть
// Копирует в указанный буфер пакет (до 64 байт)
uint8_t GetPkg(uint8_t * buf);
// Поставить пакет в очередь отправки
// buf - указатель на область памяти, содержащую пакет
// num - число байт в пакете
// iadr - эквивалентный адрес передачи (пакет считается запросом).
//        =0 если передавать немедленно, =255 если передавать от адреса пакета
uint8_t SendPkg(uint8_t * buf,uint8_t num,uint8_t iadr);
// 

// Прерывания по любому фронту
void EnableINT(void) { 
  EXTI_ClearFlag(EXTI_LINE);
  NVIC_EnableIRQ(NVIC_IRQn);
}     

// Прерывания отключены
void DisableINT(void) { 
  NVIC_DisableIRQ(NVIC_IRQn);
}     

// Возвращает состояние входа
char GetInput(void) {
  return (INPORT->IDR&INPIN)?1:0;
}

// Функции для обеспечения независимости от аппаратуры
// По тексту всё равно встречаются ссылки напрямую в таймер

// Запрет прерываний от таймера
void DisableTIM(void) {
  TIM_ITConfig(TIM3,TIM_IT_CC1,DISABLE);  
}

// Разрешение прерываний от таймера
void EnableTIM(void) {
  TIM_ClearFlag(TIM3,TIM_IT_CC1);  
  TIM_ITConfig(TIM3,TIM_IT_CC1,ENABLE);  
}

// Прибавляет константу к предыдущему порогу срабатывания
void AddTime(uint16_t adconst) {
  TIM3->CCR1+=adconst;
  TIM_ClearFlag(TIM3,TIM_IT_CC1);  
  TIM_ITConfig(TIM3,TIM_IT_CC1,ENABLE);  
}

// Устанавливает новый момент времени от текущего
void NewTime(uint16_t adconst) {
  TIM3->CCR1=TIM3->CNT+adconst;
  TIM_ClearFlag(TIM3,TIM_IT_CC1);  
  TIM_ITConfig(TIM3,TIM_IT_CC1,ENABLE);  
}

// Получает предыдущий момент времени
unsigned int GetLstTime(void) {
  return TIM3->CNT;
}

void SetOut(char n) {
  GPIO_WriteBit(OUTPORT,OUTPIN,n?Bit_SET:Bit_RESET);
}

////////////////////////////////////////////////////////////////////////////
//

void ConfigSENS(uint8_t cfgs) {
  RCC_ClocksTypeDef RCC_Clocks;
  TIM_OCInitTypeDef TIM_OCInitStruct;
  GPIO_InitTypeDef GPIOInitStruct;
#ifndef NO_USE_COMP
  COMP_InitTypeDef COMPInitStruct;
#endif
  TIM_TimeBaseInitTypeDef TIMInitStruct;
  EXTI_InitTypeDef EXTI_InitStruct;  // Прерывания
//

/*  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC,ENABLE);
  GPIOInitStruct.GPIO_Mode=GPIO_Mode_OUT;
  GPIOInitStruct.GPIO_OType=GPIO_OType_PP;
  GPIOInitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;
  GPIOInitStruct.GPIO_Speed=GPIO_Speed_Level_2;
  GPIOInitStruct.GPIO_Pin=GPIO_Pin_14;
  GPIO_Init(GPIOC,&GPIOInitStruct);
*/


  // ЭТА ФУНКЦИЯ НЕ ОБНУЛЯЕТ БИТЫ SetCFGB(cfgs);
  SensConfig=cfgs; // Это вместо неё

#if defined(USE_COMP1_051)
// Конфигурируем ножки компараторов
  GPIOInitStruct.GPIO_Mode=GPIO_Mode_AN;
  GPIOInitStruct.GPIO_OType=GPIO_OType_PP;
  GPIOInitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;
  GPIOInitStruct.GPIO_Speed=GPIO_Speed_Level_2;
  GPIOInitStruct.GPIO_Pin=GPIO_Pin_1|GPIO_Pin_0;
  GPIO_Init(GPIOA,&GPIOInitStruct);
// Конфигурируем компаратор входа линии СЕНС
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);     
  COMP_DeInit();
  COMPInitStruct.COMP_Hysteresis=COMP_Hysteresis_Low;
  COMPInitStruct.COMP_InvertingInput=COMP_InvertingInput_IO;
  COMPInitStruct.COMP_Mode=COMP_Mode_LowPower;
  COMPInitStruct.COMP_Output=COMP_Output_None;
  COMPInitStruct.COMP_OutputPol=COMP_OutputPol_Inverted;
  COMP_Init(COMP_Selection_COMP1,&COMPInitStruct);
  COMP_Cmd(COMP_Selection_COMP1,ENABLE);
// Конфигурируем линию PA6 как выход компаратора и вход прерывания
  GPIOInitStruct.GPIO_Mode=GPIO_Mode_AF;
  GPIOInitStruct.GPIO_OType=GPIO_OType_PP;
  GPIOInitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;
  GPIOInitStruct.GPIO_Speed=GPIO_Speed_Level_2;
  GPIOInitStruct.GPIO_Pin=GPIO_Pin_6;
  GPIO_Init(GPIOA,&GPIOInitStruct);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_7);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource6);        ////// ??????????????????
#elif defined(USE_COMP1_303)
// Конфигурируем ножки компараторов
  GPIOInitStruct.GPIO_Mode=GPIO_Mode_AN;
  GPIOInitStruct.GPIO_OType=GPIO_OType_PP;
  GPIOInitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;
  GPIOInitStruct.GPIO_Speed=GPIO_Speed_Level_2;
  GPIOInitStruct.GPIO_Pin=GPIO_Pin_1|GPIO_Pin_0;
  GPIO_Init(GPIOA,&GPIOInitStruct);
// Конфигурируем компаратор входа линии СЕНС
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);     
  COMP_DeInit(COMP_Selection_COMP1);
  COMPInitStruct.COMP_Hysteresis=COMP_Hysteresis_Low;
  COMPInitStruct.COMP_InvertingInput=COMP_InvertingInput_IO1;
  COMPInitStruct.COMP_Mode=COMP_Mode_LowPower;
  COMPInitStruct.COMP_Output=COMP_Output_None;
  COMPInitStruct.COMP_OutputPol=COMP_OutputPol_Inverted;
  COMPInitStruct.COMP_BlankingSrce=COMP_BlankingSrce_None;
  COMPInitStruct.COMP_NonInvertingInput=COMP_NonInvertingInput_IO1;
  COMP_Init(COMP_Selection_COMP1,&COMPInitStruct);
  COMP_Cmd(COMP_Selection_COMP1,ENABLE);
// Конфигурируем линию PA6 как выход компаратора и вход прерывания
  GPIOInitStruct.GPIO_Mode=GPIO_Mode_AF;
  GPIOInitStruct.GPIO_OType=GPIO_OType_PP;
  GPIOInitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;
  GPIOInitStruct.GPIO_Speed=GPIO_Speed_Level_2;
  GPIOInitStruct.GPIO_Pin=GPIO_Pin_6;
  GPIO_Init(INPORT,&GPIOInitStruct);
  GPIO_PinAFConfig(INPORT,EXTI_PINSOURCE,GPIO_AF_8);  // STM32F303!!!
#elif defined(NO_USE_COMP)
  // Тут конфигурируем только прерывания
  GPIOInitStruct.GPIO_Mode=GPIO_Mode_IN;
  GPIOInitStruct.GPIO_OType=GPIO_OType_PP;
  GPIOInitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;
  GPIOInitStruct.GPIO_Speed=PINSPEED;
  GPIOInitStruct.GPIO_Pin=GPIO_Pin_6;
  GPIO_Init(INPORT,&GPIOInitStruct);
#else
  #error "Отконфигурировать вход - ножка или компаратор"
#endif

// Конфигурируем прерывания от входа
  SYSCFG_EXTILineConfig(EXTI_SOURCE,EXTI_PINSOURCE);  
// Прерывания от входа
  EXTI_InitStruct.EXTI_Line=EXTI_LINE;
  EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;
  EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising_Falling;
  EXTI_InitStruct.EXTI_LineCmd=ENABLE;
  EXTI_Init(&EXTI_InitStruct);

// Конфигурируем таймер 3
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
  TIM_DeInit(TIM3);
  TIM_InternalClockConfig(TIM3);
// Нужно учитывать частоту ядра. Требуется частота прибавления 40КГц

	RCC_GetClocksFreq(&RCC_Clocks);
  TIM_TimeBaseStructInit(&TIMInitStruct);
#ifndef PMP201ARM
  TIMInitStruct.TIM_Prescaler=((uint32_t)RCC_Clocks.HCLK_Frequency)/(uint32_t)40000U - 1U; //CK_INT = HCLK = APB1 * 2 = 72MHz (для DKS) - не универсально!!!
#else
  TIMInitStruct.TIM_Prescaler=((uint32_t)RCC_Clocks.PCLK1_Frequency)/(uint32_t)40000U- 1U;
#endif

  TIMInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
  TIMInitStruct.TIM_Period=0xFFFF;
  TIMInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM3,&TIMInitStruct);
// Конфигурируем второй канал сравнения - частота 1КГц
  TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_Timing;
  TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Disable;
  TIM_OCInitStruct.TIM_Pulse=TIM_GetCounter(TIM3)+TIME1MS;
  TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High;
  TIM_OC2Init(TIM3, &TIM_OCInitStruct);
// Конфигурируем третий канал сравнения - против помех
  TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_Timing;
  TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Disable;
  TIM_OCInitStruct.TIM_Pulse=TIM_GetCounter(TIM3);
  TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High;
  TIM_OC3Init(TIM3, &TIM_OCInitStruct);
// Конфигурируем первый канал сравнения
  TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_Timing;
  TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Disable;
  TIM_OCInitStruct.TIM_Pulse=TIM_GetCounter(TIM3)+TIME20MS;
  TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High;
  TIM_OC1Init(TIM3, &TIM_OCInitStruct);
//
// Конфигурируем прерывания
// Таймер линии и системного времени
  TIM_ITConfig(TIM3,TIM_IT_CC1,ENABLE);
  TIM_ITConfig(TIM3,TIM_IT_CC2,ENABLE);
  TIM_ITConfig(TIM3,TIM_IT_CC3,DISABLE);
  NVIC_EnableIRQ(TIM3_IRQn);

// Конфигурируем выходную ножку
  GPIOInitStruct.GPIO_Mode=GPIO_Mode_OUT;
  GPIOInitStruct.GPIO_OType=GPIO_OType_PP;
  GPIOInitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;
  GPIOInitStruct.GPIO_Speed=PINSPEED;
  GPIOInitStruct.GPIO_Pin=OUTPIN;
  GPIO_Init(OUTPORT,&GPIOInitStruct);
  GPIO_WriteBit(OUTPORT,OUTPIN,Bit_RESET);

// Запускаем автомат в работу  
  TIM_Cmd(TIM3,ENABLE);
  NVIC_EnableIRQ(NVIC_IRQn);
}

// Перевод аппаратной части в режим простоя
void StopSENS(void) {
////////////////////////////////////////////////////////////////////// vvvv
// Добавлено для уменьшения потребления во сне
  EXTI_InitTypeDef EXTI_InitStruct;  // Прерывания
// Прерывания от входа
  EXTI_InitStruct.EXTI_Line=EXTI_LINE;
  EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;
  EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising_Falling;
  EXTI_InitStruct.EXTI_LineCmd=DISABLE;
  EXTI_Init(&EXTI_InitStruct);
////////////////////////////////////////////////////////////////////// ^^^^
// Отключаем алгоритм
  TIM_Cmd(TIM3,DISABLE);
  NVIC_DisableIRQ(NVIC_IRQn);
// Переводим выход в "ноль"
  GPIO_ResetBits(OUTPORT,OUTPIN);
// Пока всё - можно в дальнейшем поотключать периферию
}

//////////////////////////////////
// Функции работы со временем
uint32_t getssec() {
  uint32_t rez;
  IN_CRITICAL();
  rez=ssec;
  OUT_CRITICAL();
  return rez;
};      

uint32_t zgetssec() {
  uint32_t rez;
  IN_CRITICAL();
  rez=ssec;
  OUT_CRITICAL();
  if (rez==0) rez--;
  return rez;
};      

// Разница веремн
uint32_t tdlt(uint32_t time) {
  uint32_t t;
  IN_CRITICAL();
  t=ssec;
  OUT_CRITICAL();
  if (time>t) {
    return (~time)+t;
  } else return (t-time);
}

////////////////////////////////////////////////////////////////////////////////////////////
// Базовая часть - функции, соответствующие действиям автомата вне обработчиков прерываний

void SetWritePkg(volatile uint8_t * buf) {
  int tm;
  curtrbuf=buf+2;
  if (!(SensConfig&PSMODE)) {
    if (curtrbuf>=(TrBuffer+BUFFERS_LENGTH)) curtrbuf-=BUFFERS_LENGTH;
  }
  crc=0;
  curbit=0;
  curnum=(*curtrbuf)+1; // На единицу больше из-за CRC
  SensD=SD_TRANSMIT;
  buf++;
  if (!(SensConfig&PSMODE)) {
    if (buf>=(TrBuffer+BUFFERS_LENGTH)) buf-=BUFFERS_LENGTH;
  }
  tm=*buf;
  if (tm<=3) AddTime(TIME3MS); else AddTime(TIME1MS*tm);
}

// Выбирает действие (кружочек Выбор Действия)
// При входе в эту процедуру значение CCR2 должно всегда соответствовать моменту прихода последнего
// нарастающего фронта (это должно обеспечиваться внешними силами)
void SelectND(void) {
  EnableINT();
  if (Ni>=32) {
// Передавать нечего, выдерживаем паузу до синхропаузы
    AddTime(TIME20MS);
    SensState=SS_WAIT;
    SensD=SD_WAITSISP;
  } else {
    SensState=SS_WAIT;
// Смотрим, не передать ли пакет из буфера?
    if (TrBufPointerTail!=TrBufPointerHead) {
      if ((*(TrBufPointerTail)==Ni)||(*(TrBufPointerTail)==0)||(SensD==SD_INDEPEND)) {
        ResetCFGB(PSMODE);
        SetWritePkg(TrBufPointerTail);
        return;
      }
    }
// 
    if (curBSPointer&&(!(SensConfig&NOPS))) {
      if (Ni==*(curBSPointer)) {
// Пытаемся передать очередное состояние
        SetCFGB(PSMODE);
        SetWritePkg(curBSPointer);
        return;
      }
    }
// Остаётся генерация синхроимпульса, если разрешена
    if (SensConfig&IGENSYNC) {
      AddTime(TIME14MS);
      SensD=SD_GENSI;
    } else {
      AddTime(TIME20MS);
      SensD=SD_WAITSISP;
    }
  }
}


void RemovePkg(char flgerr) {
  uint8_t len;
  if (!(SensConfig&PSMODE)) {
    if (flgerr) {
// Ошибка передачи пакета
      trnum++;
      if (trnum<3) return;
    }
// Удаление пакета
    TrBufPointerTail+=2;
    if (TrBufPointerTail>=(TrBuffer+BUFFERS_LENGTH)) TrBufPointerTail-=BUFFERS_LENGTH;
    len=*TrBufPointerTail;
    TrBufPointerTail+=(len+1);
    if (TrBufPointerTail>=(TrBuffer+BUFFERS_LENGTH)) TrBufPointerTail-=BUFFERS_LENGTH;
  } else {
// Состояние - перематываем на следующее независимо от причины
    curBSPointer=curBSPointer+(*(curBSPointer+2))+3;
    ResetCFGB(PSMODE); 
  }
  trnum=0;
}

// Передача следующего бита
void NextBit(void) {
  if (curbit==0) {
    if (curnum) {
      if (curnum!=1) {
        curtrbuf++;
        if (!(SensConfig&PSMODE)) {
          if (curtrbuf>=(TrBuffer+BUFFERS_LENGTH)) curtrbuf-=BUFFERS_LENGTH;
        }
        curbyte=*curtrbuf;
        crc+=curbyte;
        curnum--;
        curbit=0x80;
      } else {
        curbyte=crc+(crc<<4)|0x08;            
        curbit=0x10;
        curnum--;
      }
    } else {
//// Всё-всё передали, и прошла 1 мс единицы
      RemovePkg(0);
      AddTime((uint16_t)(-TIME1MS));
      SelectND();
      return;
    }
  }
  if (curbyte&0x80) {
    SensState=SS_PP2;
    SetOut(1);
    ZZZ<<=1;
    AddTime(TIME1MS);
  } else {
    SensState=SS_PP5;
    SetOut(0);
    AddTime(TIME0_5MS);
  }
  curbyte<<=1;
  curbit>>=1;
}

void CheckZZZ(void) {
  if (ZZZ&4) {
// Прошло только 1500 мкс нуля
    EnableINT();  
    SensState=SS_P7;
    AddTime(TIME1MS);
  } else {
// Уже прошло 2500 мкс нуля
    SetOut(1);
    AddTime(TIME1_5MS);
    SensState=SS_P8;        
  }
}

////////////////////////////////////////////////////////////////////////////////////////////
// Базовая часть - обработчики прерываний. Реализуют всю схему переключения автомата

// Обработчик прерывания от таймера 3
void TIM3_IRQHandler(void) {
// Внутренние метки времени
  if (TIM_GetITStatus(TIM3,TIM_IT_CC2)) {
    ssec++;
    TIM_ClearFlag(TIM3,TIM_IT_CC2);
    TIM3->CCR2=TIM3->CCR2+TIME1MS;
  }
/////////////////// ЛИНИЯ СЕНС
// Помехоотсеивающий дополнительный таймер
  if (TIM3->DIER&TIM3->SR&0x0008) {
    TIM_ITConfig(TIM3,TIM_IT_CC3,DISABLE);
    if (!GetInput()) {
// Начинается приём нулика на входе
      SensState=SS_P1;
      TIM3->CCR1=TIM3->CCR3+TIME1MS;
      TIM_ClearFlag(TIM3,TIM_IT_CC1);  
      TIM_ITConfig(TIM3,TIM_IT_CC1,ENABLE);  
    } else TIM_ITConfig(TIM3,TIM_IT_CC1,ENABLE);
  }
// Основной таймер
  if (TIM3->DIER&TIM3->SR&0x0002) {
    TIM3->SR=0xFFFD;  // Было &=
// Отменяем прерывания от фронта
    DisableINT();    
// И от помехоотсеивающего таймера
    TIM_ITConfig(TIM3,TIM_IT_CC3,DISABLE);
// Обработчик прерываний по достижению метки времени
    switch (SensState) {
      case (SS_WAIT):
// Выбор действия
        switch (SensD) {
          case (SD_TRANSMIT):
// Передача
            SensState=SS_PP1;
            SetOut(1);
            AddTime(TIME1MS);            
            break;
          case (SD_WAITSISP):
// Подготовка данных к новому синхроциклу
            EnableINT();
            Ni=0;
            if (BSPointer) {
              curBSPointer=BSPointer;
            } else ResetCFGB(CANTRPS);
            if (NCK==0) NCK=4; else NCK=NCK&0x05;   
            if (SensConfig&IGENSYNC) {  // Генерирует синхроимпульсы
              NewTime(TIME30MS-TIME20MS);
              SensD=SD_GENSI;
            } else {                // Не генерирует синхроимпульсы
              NewTime(TIME50MS-TIME20MS);  
              SensD=SD_INDEPEND;
            }
            break;
          case (SD_GENSI):
            SensState=SS_P8;
            SetOut(1);
            AddTime(TIME4MS);
            break;
// Независимый режим
          case (SD_INDEPEND):
            Ni=0;
            SelectND();
            EnableINT();
            break;    
          default:            // Аварийная ветвь, сюда попадать не должно
            EnableINT();
            break;
        }
        break;
//
      case (SS_P1):
// Переходим в П3
          SensState=SS_P3;
          AddTime(TIME_TOP3);
// Стартовый бит
          currdbuf=RdBufPointerHead+1;           
          if (currdbuf>=(RdBuffer+BUFFERS_LENGTH)) {
// Да, нужно переместиться в начало буфера
            currdbuf-=BUFFERS_LENGTH;
          }
          if (currdbuf==RdBufPointerTail) {
            SensConfig|=EMPTYRX; 
          } else {
            bytecnt=0;
            SensConfig&=~EMPTYRX; 
          }
          crc=0;
          curbit=1;
          ZZZ=(GetInput()?1:0);
        break;
// 
      case (SS_P2):
        ZZZ<<=1;
        if (GetInput()) ZZZ|=1;
        SensState=SS_P3;
        AddTime(TIME_TOP3);
        break;
//
      case (SS_P3):
        SensState=SS_P4;
        EnableINT();
        AddTime(TIME_WAITSYNC);
        break;
//
      case (SS_P4):
        SensState=SS_CHZZZ;
        DisableINT();
        AddTime(TIME0_25MS);
        break;
//
      case (SS_P7):
        SetOut(1);
        AddTime(TIME1_5MS);
        DisableINT();
        SensState=SS_P8;
        break;
//
      case (SS_P8):
        EnableINT();
        SensState=SS_SYNC;
        SetOut(0);
        NewTime(TIME20MS);
        break;
//
      case (SS_CHZZZ):        // Обработка полученных полубайтов
        ZZZ<<=1;
        if (GetInput()) ZZZ|=1;
        switch (ZZZ&3) {
          case (0):           // Двойной ноль - ошибка или СИ
            CheckZZZ();
            break;
// Прием очередного бита
          case (1):
          case (2):
            curbyte<<=1;
            if (ZZZ&1) curbyte|=1;
            curbit<<=1;
            if (!curbit) {
              curbit=1;
              if (currdbuf==RdBufPointerTail) SensConfig|=EMPTYRX;
              if (!(SensConfig&EMPTYRX)) {
                *(currdbuf++)=curbyte;
                if (currdbuf==(RdBuffer+BUFFERS_LENGTH)) currdbuf=RdBuffer;
                crc+=curbyte;
                bytecnt++;
              }
            }
// Возврат в точку начала приема очередного бита
            AddTime(TIME1MS);
            SensState=SS_P2;
            break;
          case (3):           // Принята стоповая комбинация
            if ((curbit==0x20)&&(curbyte&1)&&!(SensConfig&EMPTYRX)) {
// Длина пакета разумная
              curbyte>>=1;
              crc=(crc+(crc>>4))&0x0F;
              if ((curbyte&0x0F)==crc) {
// Пакет верный
                *RdBufPointerHead=bytecnt;
                RdBufPointerHead=currdbuf;
              }
            }
// Возвращаемся к ожиданию действия
            AddTime((uint16_t)(-TIME2_5MS));
            SelectND();
            break;
        }
        break;
      case (SS_SYNC):
        Ni=0;
        SelectND();
        break;
/////////////////////////////////////////////////// Передающая часть
      case (SS_PP1):
        ZZZ=0x06;
      case (SS_PP4):
        NextBit();
        break;
      case (SS_PP2):
        SetOut(0);
        AddTime(TIME0_5MS);
        SensState=SS_PP3;
        break;   
      case (SS_PP3):
        if (GetInput()) {
          ZZZ<<=1;
          ZZZ|=0x01;
          SensState=SS_PP4;
          AddTime(TIME0_5MS);
        } else {
// Ошибочка
          RemovePkg(1);
          ZZZ<<=1;
          CheckZZZ();
        }     
        break;
      case (SS_PP5):
        if (GetInput()) {
          ZZZ<<=1;
          ZZZ|=0x01;
          SensState=SS_PP6;
          AddTime(TIME0_5MS);
        } else {
          ZZZ<<=1;
// Текущий квант д.б. 1 а он 0
// Это первый квант бита, поэтому предыдущие два кванта могут быть 01 или 10, но не 00
// Поэтому перехода в поддержку СИ быть не может
// Нужно только выбрать время T
          if (ZZZ&2) {
// Прошло 500 мс нуля
            AddTime(TIME2MS);
          } else {
// Прошло 1500 мс нуля
            AddTime(TIME1MS);
          }
          RemovePkg(1);
          SensState=SS_P7;
          EnableINT();          
          SetOut(0);
        }     
        break;
      case (SS_PP6):
        SetOut(1);
        ZZZ<<=1;
        SensState=SS_PP4;
        AddTime(TIME1MS);
        break;
    }
  }
};

// Обработчик прерывания от фронта
void INTERRUPT_HANDLER() {
  if (EXTI_GetFlagStatus(EXTI_LINE)) {
    EXTI_ClearFlag(EXTI_LINE);
    DisableINT();    
    switch (SensState) {
      case (SS_SYNC):
        TIM3->CCR1=TIM3->CNT;
        if (Ni<31) {
          Ni++; 
        } else if (Ni==31) {
          Ni++;
          NCK=1;
        } 
        SelectND();
        break;
// Пришел фронт сигнала
      case (SS_WAIT):
        TIM3->CCR3=TIM3->CNT+TIME0_5MS;
        TIM_ClearFlag(TIM3,TIM_IT_CC3);
        TIM_ITConfig(TIM3,TIM_IT_CC1,DISABLE);
        TIM_ITConfig(TIM3,TIM_IT_CC3,ENABLE);
        break;    
// Ошибочный запуск - импульс более 1.5 но менее 2.5 мс
      case (SS_P7):
        TIM3->CCR1=TIM3->CNT;
        SelectND();
        break;
// Внутрибитовая синхронизация
      case (SS_P4):
        NewTime(TIME_BITSYNC);
        SensState=SS_CHZZZ;
        break; 
    }
  }
}

/////////////////////////////////////////////////////////////////////////// Основные функции

 // Получение одного пакета, возвращает длину или 0 если пакета нет
uint8_t SensGetPkg(uint8_t * buf) {
  volatile uint8_t * Pnt;
  uint8_t len,n;
  IN_CRITICAL();
  if (RdBufPointerHead==RdBufPointerTail) {
    OUT_CRITICAL();
    return 0;
  }
  OUT_CRITICAL();
// Считываем пакет
  Pnt=RdBufPointerTail;
  len=*Pnt;
  *(buf++)=len;
  if (len<=64) {
    for (n=0;n<len;n++) {
      Pnt++;
      if (Pnt>=(RdBuffer+BUFFERS_LENGTH)) Pnt-=BUFFERS_LENGTH;
      *(buf++)=*(Pnt);
    }
  }
  IN_CRITICAL();
  RdBufPointerTail=RdBufPointerTail+len+1;
  if (RdBufPointerTail>=(RdBuffer+BUFFERS_LENGTH)) RdBufPointerTail-=BUFFERS_LENGTH;
  OUT_CRITICAL();
  return len;
};

// Ставит пакет в очередь отправки
// buf - указатель на область памяти, содержащую пакет
// iadr - эквивалентный адрес передачи. =0 если передавать немедленно, =255 если передавать от адреса пакета
void SensSendPkg(uint8_t * buf,uint8_t iadr) {
  uint8_t n,num;
  volatile uint8_t * pnt;
  volatile uint8_t * pnt2;
  volatile uint8_t * pntps;
  uint8_t ni,ps;
  pnt=TrBufPointerHead+2;
  if (pnt>=(TrBuffer+BUFFERS_LENGTH)) pnt-=BUFFERS_LENGTH;
  pnt2=buf;
  num=(*pnt2)+1;
  for (n=0;n<num;n++) {    
    *(pnt++)=*(pnt2++);
    if (pnt>=(TrBuffer+BUFFERS_LENGTH)) pnt-=BUFFERS_LENGTH;
    if (pnt==TrBufPointerTail) return;                        // ОТМЕНЯЕМ ПЕРЕДАЧУ
  }
// Пакет скопировали, рассчитываем адреса и паузы
  pntps=TrBufPointerHead+1;
  if (pntps>=(TrBuffer+BUFFERS_LENGTH)) pntps-=BUFFERS_LENGTH;  
  if (iadr==0xFF) iadr=buf[1];
  if (iadr==0) {
// Немедленная передача через 3 или 4 мс
    if (buf[2]&0x80) ps=3; else ps=4;
    ni=0;
  } else {
    ni=(iadr-1)%31+1;
    ps=(iadr-1)/31+3;
    if (!(buf[2]&0x80)) ps+=1;
  } 
  *TrBufPointerHead=ni;
  *pntps=ps;
  TrBufPointerHead=pnt;
}

/////////////////////////////////////////////////////////////////////////// Работа с пакетами состояний

// Блокирует передачу байтов состояния и разрешает изменение
void BlockST(void) {
  while (SensConfig&PSMODE);
  IN_CRITICAL();
  SensConfig|=NOPS;
  OUT_CRITICAL();
}

// Разблокирует передачу байтов состояния
void UnblockST(void) {
// Пересчитываем адреса и паузы
  volatile uint8_t * bsp;
  uint8_t adr,ni,ps;
  bsp=BSPointer;
  if (bsp) {
    for (;;) {
      adr=*(bsp+3);
      if (adr) {
        ni=(adr-1)%31+1;
        ps=(adr-1)/31+4;
        *bsp=ni;
        *(bsp+1)=ps;
      } else {
        *bsp=0;
        *(bsp+1)=0;
        break;
      }
      bsp+=(*(bsp+2)+3);
    }
  }
  IN_CRITICAL();
  SensConfig&=~NOPS;
  OUT_CRITICAL();
}

// Устанавливает указатель на буфер состояний
void SetBSBuf(void * bsbufpnt) {
  BSPointer=bsbufpnt;
}

// Считывает первый передаваемый пакет из буфера состояний. Возвращает длину пакета или ноль если пакета нет
uint8_t GetBSBuf(void * bsbufpnt) {
  uint8_t * dbf;
  uint8_t * sbf;
  uint8_t n,num;
  if (!BSPointer) return 0;
// Копируем один пакет  
  dbf=bsbufpnt;
  sbf=(uint8_t*)BSPointer;
  sbf+=2;
  num=(*sbf)+1;
  for (n=0;n<num;n++) *(dbf++)=*(sbf++);
  return num-1;
}

/////////////////////////////////////////////////////////////////////////// Состояние передачи байта состояний
uint8_t CheckBSTr(void) {
  if (SensConfig&PSMODE) return 1; else return 0;
}

/////////////////////////////////////////////////////////////////////////// Работа с синхроциклом

uint8_t GetNC(void) {
  uint8_t rez;
  IN_CRITICAL();
  if (NCK&5) {
    NCK=(NCK<<1)&2;
    rez=1;
  } else rez=0;
  OUT_CRITICAL();
  return rez;
}

