#ifndef __SYS_ERROR_CODES_H
#define __SYS_ERROR_CODES_H

/////// Управление ошибками

#define ERRORTIME         300000 // Время хранения сообщения об ошибке (5 минут?)

/* Коды ошибок
*/
//сбрасываемые
#define MAINERR1          0x0001
#define STIMEERR          0x0002
#define OTINITERR         0x0040
#define IZMTOERR          0x0080
#define PWRONERR          0x0008

//несбрасываемые
#define READCONST_ERROR   0x8000
#define READCONST_WARNING 0x4000
#define GRTBL_ERROR       0x2000 // Ошибка градуировочной таблицы

#endif
