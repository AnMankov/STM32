#ifndef SENSOR_H
#define SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

//#include "main.h"
#include <stdint.h>

#define ED_IZM_TBL_SIZE			8
#define MAX_IDX_ED_IZM_TBL	(ED_IZM_TBL_SIZE - 1)

#define MAXSWIM 5

#define MAXOPDT 800

// ��������� �������� � SensStep (������ � ���)
#define ADCOFF    0					// ������� ��������� ��������
#define START     1					// ���� ��� �������������� � �������� � ���������
#define START_A   2					// ���������� ��������
#define START_B   3					// ���������� ��������
#define START_C   4					// ������ ���������
#define GETDATA   5					// ����������� ��������� ������

// ���� ��� ��������� �����������	
#define TEMPIZM_A 40				// ������ � ������� IO
#define TEMPIZM_B 41				// ������ � ������� CONFIG
#define TEMPIZM_C 42 				// �������������� �������� ����
#define TEMPIZM_D 43				// ���������� ��������
#define TEMPIZM_E 44 				// ������ ������ ��������� �����������
#define TEMPIZM_F 45 				// ���������� ����������
#define PRESTART  46 				// ������ � ������� IO ��� ��������� ��������

// ��������� �������� � CalcProc (��������� ���������)
#define WAIT_NEWREZ   0			// �������� ����� ������
#define CALC_CURPRESS 1			// ������ �������� ��������
#define CALC_DAMPING  2			// ������������� ����������
#define CALC_OUTPARAM 3			// ������������ �������� ����������
#define CALC_CRITICAL 4			// ������������ ����������� �������

#define NUMKOEFF 5

// ���� ������
#define ERR_CODE_01  0x0001		// ������ ��������� ��������
#define ERR_CODE_02  0x0002		// ������ ��������� �����������
#define ERR_CODE_04  0x0004		// ������ ������ ���������
#define ERR_CODE_08  0x0008		// ������ ��������� ��������� ��������
//#define ERR_CODE_80  0x80		// ������ ��� ���������������� EEPROM

extern uint8_t gds;			// ��������� �������� � SensStep

///// ��������� ���������
// ��������� ���������:
// 0 - ������, � ������� ��������
// 1 - �������, � ������� ����������
// 3 - ����������, � ����������� ������� ������������ �������
extern volatile uint8_t itype;           // �������� ������� � ���������
extern volatile uint8_t invert;          // ���� ������������� �������
extern volatile uint32_t swimnum;         // ����� ��������� � �������
///// ��������� ���������
// ������� ��������� ���������, ������� ������
extern volatile uint32_t times[MAXSWIM];  // ������� ���������, ��
extern volatile uint32_t curbasetime;     // ���������� ������� �����, ��
extern volatile uint32_t dtime;           // ����� ������ ���������. 
#define IERROK        0   // ��������� ������� ���������
#define IERRSWIMNUM   2   // �������� �������� SWIMNUM (0 ��� ����� ���������)
#define IERRLOWIMPNUM 4   // ��������� ������� ���������
#define IERROPTIMEERR 8   // ������� ��������� ������� ������ ��� ������ ���������
#define IERRORTYPE    16  // ��������� ������� ���������
#define IERRLSTOPTIME 32  // ������� � ���������� ������� �������� ������� ������
#define IWAITSTART    65536  // �������� �����������
extern volatile uint32_t izmflag;         // ���� ��������� ���������
//
extern volatile float icels[8];  // ���������� �����������
//
extern volatile uint8_t CelsDatOk; // ���� ������� ������ � �����������

void ConfigIZM(void); // ������������ ���������� - ��������� �� __enable_irq
void InitTermo(void); // ������������ ������������� - ��������� ����� __enable_irq

// �������� ������. 
// ����� ��������� ����� ����� - ����������� �������� dtime
// ����� �������� izmflag ����������� � ��� �� 0 �� 32 � ����������� �� ����������
void SensProc(void);  
// �������� ��� ��������, ����� �� ������������ fasti2c � �������� ���������
// 0 - ������������ ������
// 1 - ������������ �����
// ������� � ���������� ���������� �������� � �������� ���������
char CanFastI2C(void); 


uint8_t SetBaseTime(void); // ��������� �������� �������

float GetHeadCels(void); // �������� ����������� ������ ������� � ��������

uint8_t GetReadOnly(void);   // ���������� 1 ���� ������ ���������


extern float temperatureC;

//////////////////////////////////////// ����������� �������� ����� HCLK � PCLK1
// fast=1 - ������� ���� �����������, �������� ������� �� PCLK1
// fast=0 - ������� ���� ����������, �������� ������� �� HCLK
void SetClockState(uint8_t fast);
uint8_t CurSpeedHi(void);         // ���������� 1 ���� ������� ���� �����������


// �������
#define TEMPADDR 104 // ������� ����������������
#define GISTADDR 440 // ������� ������������
#define LTADDR   35  // ������� ����������� �������

#define erddouble(x)  (*((float*)x))
#define erdint(x)			(*((uint16_t *)x))

// ������ � �������� ���������������� :
// ������ ���������� �������� P0 (4 �����)
#define P0_TempComp (erddouble(TEMPADDR))  
// ������ ���� �������� Pdlt (4 �����)
#define Pdlt_TempComp (erddouble(TEMPADDR+4))
// ������ ����������� ������ (� ����� ���) TC(i) (2 �����) � i ������
#define TC(i) ((uint16_t/*long*/)(erdint((TEMPADDR+8)+(i)*26)))
// ������ ����������� ������ (�� ���������� ����������) TCst(i) (2 ����� � i ������
#define TCst(i) ((uint16_t)(erdint((TEMPADDR+10)+(i)*26)))
// ������ n-��� �������� (n=0..11) ���� ��� AC(i) � i ������
//#define AC(i,n) ((/*unsigned int*/ long)(erdint((TEMPADDR+12)+(n)*2+(i)*26)))
//unsigned int AC(char i, char n);


uint8_t StartSensor(void);
void SensStep(void);
int CalcProc(void);

uint8_t GetEizm(float val);

int cal_01(float *val);
int cal_02(float *val);

void EIzmProc(void);

//char SensAnswerPkg(uint8_t * buf);       // ������� �������� �� ����� ����

#ifdef __cplusplus
}
#endif

#endif
