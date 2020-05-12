#ifndef CMDCODES_H
#define CMDCODES_H

///////////////////////////////////////////////////////////////////
//
// История изменения
// Версия_1
// 19.05.18 Добавлены параметры 0xB1, 0xB2, 0xB4
// 28.04.18 Добавлены параметры 0x16 0x17 0x18 для радарного уровнемера
// 19.12.17 Добавлен параметр идентификации протокола VERPROT
// 04.12.17 Добавлены определения команд 32-х битного обмена
// 15.08.17 Добавлена таблица LINTAB - линеаризация, отображается как float
// 13.08.15 Добавлено меню MENU_DENS
// 24.06.15 Добавлен параметр 0x15
// 08.04.15 Добавлены параметры 0x18???,  0x19???
// 25.11.14 Добавлен параметр 0x6A
// 24.09.14 Добавлены параметры 0x3F, 0x1A, 0x1B
// 02.05.14 Добавлено меню настройки адаптера 0x83, параметр "Адрес Omnicomm" 0x93
// 24.03.14 Добавлен параметр II
// 25.10.13 Добавлен параметр IU
// 16.11.13 Добавлено меню MENU_METR
// 15.10.13 Добавлена команда CMD_WAIT
// 11.10.13 Добавлены параметры 0x3C-0x3E - погрешности массы и градуировочной таблицы
// 17.04.13 Добавлены: VALUE1,VALUE2,GIST0, IBUTPRC
// 25.10.12 Добавлено меню: MENU_COUNT
// 07.08.12 Добавлены параметры 0x4D, 0x4E
// 01.07.12 Параметры 0x11, 0x12, 0x13, 0x5A, 0x5C, 0x5D убраны, параметры 0x10, 0x5B переименованы
// 25.06.12 Параметры 0x12, 0x5C переименованы, введены параметры 0x13, 0x5D, 0x5E
// 12.04.12 Добавлен: PMP201SET, LELPRC, DPRESS, SYNCPRESS
// 09.03.11 Добавлены: DMPTIME, GIST1..GISTF, PASWD0..2, MENU_GIST, MENU_PSWD, 
// 03.12.10 Добавлены: COUNTx, D9, EIZM0..F
// 21.04.09 Добавлены идентификаторы WTIME1-WTIME4
// 06.08.08 Введена команда CMD_TIMECMD с кодом 0x20

// Определены секции //# КОДЫ КОМАНД, //# КОДЫ ПАРАМЕТРОВ, //# КОДЫ ТАБЛИЦ //# КОДЫ МЕНЮ

//# КОДЫ КОМАНД
// Коды команд
// Код команды формируется из следующих полей:
// 7 бит - установлен в ответе, сброшен в запросе
// 6 бит - Режим эмуляции
// 5 бит - установлен, если не нужно менять главного
#define NOCHG_MOD   0x20  // Не менять главного
#define ANSWER_MOD  0x80  // Это-ответ
// Далее - код команды от 00h до 1fh
#define CMD_GETSTATE 0x00 // Запуск измерения с получением состояния
#define CMD_GETPI   0x01  // Получение всех измеряемых параметров
#define CMD_GETPS   0x02  // Получение всех установочных параметров
#define CMD_GETTN   0x03  // Получение кода программы и всех таблиц
#define CMD_MENULEV 0x08  // Получение уровня меню по его номеру
#define CMD_PRMLIST 0x09  // Получение списка параметров
#define CMD_GETTBL  0x0A  // Получение фрагмента таблицы
#define CMD_GETTBL32 0x0B // Получение таблиц с форматом Float32
#define CMD_GETPR32 0x0E  // Команда получения параметров во Float32
#define CMD_GETPR   0x0f  // Получение параметров
#define CMD_START   0x10  // Запуск измерения
#define CMD_SETPR   0x11  // Установка параметра
#define CMD_SETPR32 0x12  // Команда установки параметров во Float32
#define CMD_WAIT    0x13  // Оповещение о необходимости увеличения таймаута
#define CMD_SETTBL  0x1A  // Установка фрагмента таблицы  
#define CMD_SETTBL32 0x1B // Установка таблицы  в Float32
#define CMD_CAL32   0x1D  // Установка технологической точки с параметрами Float32
#define CMD_SETPOINT 0x1e // Установка технологической точки
#define CMD_TIMECMD 0x18  // Команды работы с временем
// Специальные команды
#define CMD_OFFSIREN		0x1f

//# КОДЫ ПАРАМЕТРОВ
// Коды параметров
// Измеряемые
#define LEVEL   0x01	/* Уровень основного поплавка */
#define CELS    0x02    /* Средняя температура в продукте */
#define PERCENT 0x03    /* Процентное заполнение по объему */
#define VALUE   0x04    /* Общий объем */
#define MASS    0x05    /* Масса */
#define PLOTN   0x06    /* Плотность */
#define VAL1    0x07    /* Объем основного продукта*/
#define LEV2    0x08    /* Уровень подтоварной воды */
#define PRESS   0x09	/* Давление */
#define CELSHI  0x0a    /* Средняя температура в паровой фазе */
#define MASSHI  0x0b    /* Масса паровой фазы */
#define MASSLO  0x0c    /* Масса жидкой фазы */
#define CHLEL   0x0d    /* Процент объемных долей по пропану */ 
#define DPRESS  0x0e    /* Дифференциальное давление */ 
#define CH3     0x10    /* Концентрация определяемого компонента */
// Специальные измеряемые
#define VAL1T   0x11    /* Объем продукта, приведенный к указанной температуре */
#define VAL115  0x12    /* Объем продукта, приведенный к 15 */
#define DENSPT  0x13    /* Плотность, приведенная к указанной температуре */
#define DENS15  0x14    /* Плотность, приведенная к 15 */
#define VAL2    0x15    /* Объем подтоварной жидкости */
//Radar parameters
#define LEVELPROD 0x16    /* Level to product  */
#define LEVELFREE 0x17    /* Level free */
#define CELSHEAD 0x18    /* Temperatura ustroistva  */
#define COPYDENSPT  0x1A  /* Временно - Плотность, приведенная к указанной температуре */
#define COPYDENS15  0x1B  /* Временно - Плотность, приведенная к 15 */
#define CLKTIME 0x1C    /* Время цикла линии */
#define STTIME1 0x1D	/* Время струны - целая часть */
#define STTIME2 0x1E	/* Время струны - дробная часть */
#define EPRM    0x1F	/* Последний параметр, вызвавший срабатывание */
// Хранимые
#define HLOW    0x20	/* Нижняя контрольная точка */
#define HHIGH   0x21    /* Верхняя контрольная точка */
#define DH      0x22    /* Погружение основного поплавка */
#define H0      0x23    /* Расстояние от дна до направляющей датчика */
#define GRAD    0x24    /* Тип градуировки */
#define HFULL   0x25    /* Полная высота резервуара */
#define VFULL   0x26    /* Полный объем резервуара */
#define TBLPNT  0x27    /* Количество точек в градуировочной таблице */
#define II      0x28    /* Ток */
#define PLTLOW  0x29    /* Минимальная плотность */
#define PLTHIGH 0x2a    /* Максимальная плотность */
#define VK      0x2b    /* Коэффициент об{емного расширения */
#define RO      0x2c    /* Установленная плотность вещества */
#define ST      0x2d    /* Базовая температура плотности */
#define DU      0x2e    /* Погружение поплавка подтоварной воды */
#define PRPPRC  0x2f    /* Процент пропана */
// Коммутатора
#define IMPCNT  0x30    /* Число импульсов */
#define BITS    0x31    /* Состояние выходов/битов */
// Дополнительные точки
#define D3      0x32    /* Нижний уровень отключения плотномера */
#define D6      0x33    /* Порог обнуления показаний уровня раздела сред */
#define D7      0x34 
#define D8      0x35    /* Разница высот основного поплавка и поплавка раздела сред */
#define PRESS_H	0x36    /* Верхняя базовая точка давления */
#define PRESS_L	0x37	/* Нижняя базовая точка давления */
#define SEGNUM  0x38    /* Количество сегментов */
#define BUTPRC  0x39    /* Процент н-бутана */
#define D9      0x3A    /* Верхнее ограничение поплавка плотности */
#define IBUTPRC 0x3B    /* Процент i-бутана */
#define DOMASS  0x3C    /* Относительная погрешность измерения массы */
#define DAMASS  0x3D    /* Абсолютная погрешность измерения массы */
#define DOTABL  0x3E    /* Относительная погрешность градуировочной таблицы */
#define CDCELS  0x3F    /* Температура приведения плотности */
// Параметры адаптера ModBus
#define RS232SPEED  0x40 /* Скорость передачи */
#define RS232PARITY 0x41/* Четность */
#define MODADDR 0x42    /* Адрес адаптера в ModBus */
#define TIME    0x43 	/* Время */
#define MODIFY  0x44    /* Модификатор */
// Параметры адаптера линия-модем
#define MDMBITS 0x45    /* Настроечные биты смс-ок */
#define ADPBITS 0x46    /* Настроечные биты адаптеров */
// 
#define SIRENBIT 0x47	  /* Настройка битов сирен */
// Параметры адаптера Лин-токовый выход
#define CRNTBIT 0x48  	/* Биты настройки токового адаптера */
#define SEEPARAM 0x49   /* Установленный на анализ параметр */
//
#define RESTIME 0x4A    /* Время переинициализации модема */
//
#define DMPTIME 0x4B    /* Постоянная времени демпфирования сигнала */
#define LELPRC  0x4C    /* НКПР */
#define PLTIZM  0x4D    /* Измеренная плотность */
#define PLTCELS 0x4E    /* Температура измерения плотности */
#define IU      0x4F    /* Напряжение */
//
#define PLC     0x50    /* Длина струны в миллиметрах (приблизительно) */
#define PDL     0x51    /* Процент отклонения времени струны */
// Счетчики изменений пунктов меню
#define COUNT1  0x52    
#define COUNT2  0x53
#define COUNT3  0x54
#define COUNT4  0x55
#define COUNT5  0x56

#define SYNCPRESS 0x57	/* Синхронное значене давления */
#define SN1     0x58    /* Первая половина серийного номера (HEX отображение) */
#define SN2     0x59    /* Вторая половина серийного номера (HEX отображение) */
#define VERPROT 0x5A    /* Идентификатор протокола Float32 */
#define RELSNS  0x5B    /* Относительная чувствительность */

#define SELCH   0x5E    /* Выбранный режим работы */

// Индикатора
#define OUTADDR 0x60    /* Выводимый адрес */
#define STRTADDR 0x61   /* Ведомый адрес */
#define NUMADDR  0x62   /* Размер окна */
#define OFFSADDR 0x63   /* Смещение адресов пакетов */
#define SBRIGHT  0x64   /* Яркость индикатора */
#define WTIME1   0x65   /* Время режима */
#define WTIME2   0x66   /* Время режима */
#define WTIME3   0x67   /* Время режима */
#define WTIME4   0x68   /* Время режима */

#define PMP201ST 0x69  /* Выбор алгоритма работы ПМП-201 */
#define DENSDLT  0x6A  /* Корректировка показаний плотномера */
#define LATITUDE 0x6b  /* Широта */
#define LONGITUDE 0x6c /* Долгота */

// Единицы измерения
#define EIZM0   0x70    /* Уровень отображаемый */
#define EIZM1   0x71    /* Температура */
#define EIZM2   0x72    /* Объем отображаемый */
#define EIZM3   0x73    /* Масса отображаемая */
#define EIZM4   0x74    /* Плотность отображаемая */
#define EIZM5   0x75    /* Давление отображаемое */
#define EIZM6   0x76    /* Резерв */ 
#define EIZM7   0x77    /* Резерв */ 
#define EIZM8   0x78    /* Уровень устанавливаемый, большие величины */
#define EIZM9   0x79    /* Уровень устанавливаемый, маленькие величины */
#define EIZMA   0x7A    /* Объем устанавливаемый */
#define EIZMB   0x7B    /* Плотность устанавливаемая */
#define EIZMC   0x7C    /* Давление устанавливаемое */
#define EIZMD   0x7D    /* Единицы измерения градуировочной таблицы давления */ 
#define EIZME   0x7E    /* Резерв */ 
#define EIZMF   0x7F    /* Резерв */ 

// Гистерезисы
#define GIST0   0x80    /* Гистерезис */
#define GIST1   0x81    /* Гистерезис по уровню */
#define GIST2   0x82    /* Гистерезис по температуре */
#define GIST3   0x83    /* Гистерезис по процентам */
#define GIST4   0x84    /* Гистерезис по объему */
#define GIST5   0x85    /* Гистерезис по массе */
#define GIST6   0x86    /* Гистерезис по плотности */
#define GIST7   0x87    /* Гистерезис по давлению */
#define GIST8   0x88    /* Гистерезис */
#define GIST9   0x89    /* Гистерезис */
#define GISTA   0x8A    /* Гистерезис */
#define GISTB   0x8B    /* Гистерезис */
#define GISTC   0x8C    /* Гистерезис */
#define GISTD   0x8D    /* Гистерезис процента по НКПР */
#define GISTE   0x8E    /* Гистерезис */
#define GISTF   0x8F    /* Гистерезис */

// Параметры общего назначения
#define VALUE1  0x91    /* Значение параметра 1 */
#define VALUE2  0x92    /* Значение параметра 2 */
#define OADDR   0x93    /* Адрес Omnicomm */

// Параметры после таблиц
#define CRCVAL1 0xB1    /* Младшая половина CRC32 */
#define CRCVAL2 0xB2    /* Старшая половина CRC32 */
#define IMPCNT2 0xB4    /* Число импульсов */

// Технологические точки
#define M_TPNT  0xe0  // Маркер наличия технологических точек
#define PNT1    0xe1    /**/
#define PNT2    0xe2    /**/
#define PNT3    0xe3    /**/
#define PNT4    0xe4    /**/
#define PNT5    0xe5    /**/
#define PNT6    0xe6    /**/
#define PNT7    0xe7    /**/
#define PNT8    0xe8    /**/
#define PNT9    0xe9    /**/
#define PNTA    0xea    /**/
#define PNTB    0xeb    /**/
#define PNTC    0xec    /**/
#define PNTD    0xed    /**/
#define PNTE    0xee    /**/
#define CELSNUM 0xef  // Количество датчиков температуры
// Технологические обязательные
#define ERRCODE 0xf0
#define MYADDR  0xf1
#define VERSION 0xf2
#define NEWVERSION 0xf3	/* Дубликат номера версии */
// Пароли
#define PASSWD0 0xF4	/* Пароль возврата к пользовательскому уровню и сохранения изменений */
#define PASSWD1 0xF5	/* Пароль перехода к администраторскому уровню */
#define PASSWD2 0xF6	/* Пароль перехода к технологическому уровню */
// Технологические специальные
#define IZMPRM  0xf9    /* Наличие измеряемых параметров в датчике */ 
#define PARAM   0xfa    /* Обозначение параметра */
#define ANYFLOAT 0xfb
#define INVSMOD 0xfc    // Для модификации пункта меню контролем инверсии наличия переменной при описании
#define FLAGMOD 0xfd    // Для задания флагов пункта меню при описании
#define SPECMOD 0xfe	// Для модификации пункта меню контролем наличия переменной при описании
#define MENUMOD 0xff	// Указатель на меню

//# КОДЫ ТАБЛИЦ

// Таблицы
#define LINTAB	  0x90 // Таблица линеаризации, не отображается МСК
#define TKTAB	    0x94 // Таблица термокомпенсации датчика давления
#define PGSTAB    0x95 // Таблица ПГС для СЕНС-ГС
#define STRTBL3   0x96 // Дополнительная таблица для адаптера
#define PATAB     0x97 // Таблица соответствия параметров и адресов УСЛ
#define SIRENTBL1 0x98 // Первая таблица сирены
#define SIRENTBL2 0x99 // Вторая таблица сирены
#define SIRENTBL3 0x9a // Третья таблица сирены
#define DTBT8TAB  0x9b // Фиксированная таблица датчиков и битов
#define STRTBL2   0x9c  // 
#define USRTBL    0x9d  // Таблица пользователей
#define HISTTBL   0x9e  // Таблица истории
#define STRTBL    0x9f  // Строковая таблица - одна строка
#define GRDTBL    0xa0  // Градуировочная
#define CLVTAB    0xa1  // Температурных датчиков
#define LEVTAB    0xa2  // Уровней сигнализации
#define GISTAB    0xa3  // Гистерезисов
#define DATBITTBL 0xa4 // Датчиков и битов регистрации
#define DENSTAB   0xa5  // Параметров вещества
#define VMTAB     0xa6  // Выводимых датчиков
#define CELSTAB   0xa7  // Значений температурных датчиков
#define A8TAB     0xa8  // Фиксированного количества адресов
#define DTBT2     0xa9  // Вторая таблица датчиков и битов
#define DTBT3     0xaa  // Третья таблица датчиков и битов
#define PRCTAB    0xab  // Соответствия входов процентам
#define VIEWTAB   0xac  // Состояния просматриваемых параметров
#define CONTAB    0xad  // Прямого подключения входов ПМП
#define STTAB	    0xae  // Таблица состояний битов реагирования
#define PLTTAB    0xaf  // Таблица плотностей плотнмеров


//# КОДЫ МЕНЮ

#define MENUTABL    0xb0    // Таблица структуры меню
// Коды менюшек
#define MENU_SETT 	0x02	// Меню установки параметров
#define MENU_TN		0x03	// Главное меню настройки
#define MENU_INFO	0x04	// Основные настроечные параметры
#define MENU_USER   0x05    // Меню быстрого доступа
#define MENU_INPRM	0x10	// Вводимые параметры
#define MENU_OUTPRM 0x11    // Выводимые параметры
#define MENU_CLB	  0x12  // Калибровочный пункт
#define MENU_DENS	  0x13  // Параметры вещества
#define VMENU_DENSTAB 0xF0  // Параметры вещества
// Новые меню
#define MENU_GIST	0x20	// Список гистерезисов
#define MENU_PSWD	0x80	// Пароли к режимам
#define MENU_COUNT	0x81	// Меню контрольных счетчиков
#define MENU_METR   0x82    // Метрологическое меню
#define MENU_SADA   0x83    // Меню настройки адаптера

#define PRM_ALLIZM     0x01    // Все измеряемые параметры
#define PRM_ALLROM     0x02    // Все устанавливаемые параметры
#define PRM_SETTABL    0x03    // Все устанавливаемые таблицы

#endif
