#ifndef __RTOS_HEADERS_H
#define __RTOS_HEADERS_H

#pragma anon_unions

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "croutine.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "portmacro.h"
#include "timers.h"

//----- Задачи FreeRTOS -----------------------------------------------------------------------------------------
//void coding_switch( void * );                  //обработка кодового переключателя
//void discrete_out( void * );                   //управление дискретным выходом
//void base_to_pc( void * );                     //обмен базы с ПК; только для базы
//void base_to_sens( void * );                   //обмен базы с датчиком; только для базы
//void sens_to_master( void * );                 //обмен датчика с мастером (ПК либо базой); только для датчика
//void mems( void * );                           //работа с МЭМС-датчиком
//void dev_ctrl( void * );                       //управление устройством
//void logger_ctrl( void * );                    //управление функцией логгера (манипуляция с данными для долговременного хранения во внешней Flash)
//void vLedCtrl( void * );
//void vPushButton( void * );

void lev_gauge_to_pc( void * );                //обмен уровнемера с ПК
//---------------------------------------------------------------------------------------------------------------

//----- Семафоры FreeRTOS ---------------------------------------------------------------------------------------
extern SemaphoreHandle_t I2C_RxSem;            //данные считаны по I2C
extern SemaphoreHandle_t I2C_TxSem;            //данные записаны по I2C
extern SemaphoreHandle_t CodeSwTmr_TrigSem;    //таймер CodeSwTmr сработал
extern SemaphoreHandle_t DoUncalibTmr_TrigSem; //таймер DoUncalibTmr сработал                                    
extern SemaphoreHandle_t RawDataMems_RdySem;   //данные датчика готовы (можно считывать). из прерывания от INT
//extern SemaphoreHandle_t FlashData_WrSem;      //данные для записи во Flash готовы
extern SemaphoreHandle_t SlaveRtoTrigSem;      //сработал флаг RTO для Slave-устройства в USART
extern SemaphoreHandle_t SlaveCommErrSem;      //есть ошибка/ошибки связи по USART для Slave-устройства
extern SemaphoreHandle_t MasterRtoTrigSem;     //сработал флаг RTO для Master-устройства в USART
extern SemaphoreHandle_t MasterCommErrSem;     //есть ошибка/ошибки связи по USART для Master-устройства
extern SemaphoreHandle_t PdSem;                //тестовый периодический опрос СЕНС ПД
//extern SemaphoreHandle_t FlashTmr_TrigSem;     //сработал таймер программирования элемента Flash
//extern SemaphoreHandle_t MemsWaitStopSem;      //сработал таймер на перезапуск mems-автомата
//extern SemaphoreHandle_t TxSPI1_RdySem;        //данные переданы через SPI1
//extern SemaphoreHandle_t RxSPI1_RdySem;        //данные приняты через SPI1
extern SemaphoreHandle_t WrFlash;              //команда на запись данных во Flash
extern SemaphoreHandle_t BtnTimer_TrigSem;     //
extern SemaphoreHandle_t LedTimer_TrigSem;     //

//семафоры обработки измерений МЭМС
//extern SemaphoreHandle_t MemsDevRdySem;        //МЭМС готов измерять
//extern SemaphoreHandle_t MemsDevDoneSem;       //МЭМС измерил
//extern SemaphoreHandle_t DevMemsStartSem;      //запуск измерений МЭМС на своей плате
//extern SemaphoreHandle_t DevSensStartSem;      //запуск измерений МЭМС на плате датчика
//extern SemaphoreHandle_t DevSensResSem;        //запрос результатов измерений МЭМС на плате датчика
//extern SemaphoreHandle_t SensDevOkSem;         //измерения МЭМС на плате датчика стартовали
//extern SemaphoreHandle_t SensDevResSem;        //результаты измерений МЭМС на плате датчика получены
//extern SemaphoreHandle_t SensWrAxisRotateSem;  //необходимо записать AxisRotate в датчик
//extern SemaphoreHandle_t SensAxisRotateOkSem;  //значение AxisRotate успешно записано в датчик

//семафоры калибровки акселерометра
//extern SemaphoreHandle_t AccCalib_X_UP_Sem;
//extern SemaphoreHandle_t AccCalib_X_DOWN_Sem;
//extern SemaphoreHandle_t AccCalib_Y_UP_Sem;
//extern SemaphoreHandle_t AccCalib_Y_DOWN_Sem;
//extern SemaphoreHandle_t AccCalib_Z_UP_Sem;
//extern SemaphoreHandle_t AccCalib_Z_DOWN_Sem;
     
//семафоры калибровки по углу
//extern SemaphoreHandle_t User_Zeroing_Out_Sem;
//extern SemaphoreHandle_t Factory_Zeroing_Out_Sem;

extern SemaphoreHandle_t Tmr_TrigSem;
extern SemaphoreHandle_t DiffExti_TrigSem;
//---------------------------------------------------------------------------------------------------------------

//----- Мьютексы FreeRTOS ---------------------------------------------------------------------------------------
extern SemaphoreHandle_t MainMut;              //мьютекс буфера, хранящего основные данные усройства
extern SemaphoreHandle_t SENSMut;              //мьютекс данных протокола СЕНС
//---------------------------------------------------------------------------------------------------------------

//----- Таймеры FreeRTOS ----------------------------------------------------------------------------------------
extern TimerHandle_t CodeSwTmr;                //таймер обработки кодового переключателя
void code_sw_tmr( TimerHandle_t );             //обработка таймера кодового переключателя
                                               
extern TimerHandle_t DoUncalibTmr;             //таймер формирования спец сигнала при некалиброванном акселерометре
void do_uncalib_tmr( TimerHandle_t );          //обработка таймера формирования спец сигнала при некалиброванном акселерометре
                                               
extern TimerHandle_t PdTmr;                    //тестовый таймер
void pd_tmr( TimerHandle_t );                  //тестовый таймер


extern TimerHandle_t BtnTimer;
void vTimerCallback(TimerHandle_t xTimer);

//extern TimerHandle_t MemsTmr;                  //таймер для нужд mems-датчика
//void mems_tmr( TimerHandle_t );                //обработка таймера для нужд mems-датчика
//---------------------------------------------------------------------------------------------------------------

#endif //__RTOS_HEADERS_H
