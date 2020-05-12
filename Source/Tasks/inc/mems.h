#ifndef __MEMS_H
#define __MEMS_H

#include "main.h"
#include "model.h"
#include "rtos_headers.h"
#include "angles_cnt.h"

constexpr uint16_t ACCEL_BUF_SIZE = 300U;
//constexpr uint16_t ACCEL_BUF_SIZE = 8U;
typedef TModel::TAccelData (TMemsAggregate)[ACCEL_BUF_SIZE];

class TMems
{
public:
  struct THystF
  {
    float Low;
    float High;
  };

  struct TAxesHyst
  {
    THystF X;
    THystF Y;
    THystF Z;
  };
  
public:
  TMems( MPU_9250::TAccGyroMagDriver_HL & );
  ~TMems();
  
  //----- процедура калибровки акселерометра ---------------------------
  void acc_calib();
  //--------------------------------------------------------------------
  
  //----- сбор и накопление данных -------------------------------------
  void acquire_data( TMemsAggregate & );
  //--------------------------------------------------------------------
  
  //----- проверка валидности накопленных данных с выборками -----------
  bool is_valid_data( const TMemsAggregate & );
  //--------------------------------------------------------------------
  
  //----- вычисление среднего значения из накопленных данных -----------
  void cnt_average( const TMemsAggregate &, TModel::TAccelData & );
  //--------------------------------------------------------------------
  
  //----- демпфирование ------------------------------------------------
  void cnt_damp(__packed float &damp_axis, float cur_axis, float coeff);
  //--------------------------------------------------------------------
	
	//----- установка состояния МЭМС -------------------------------------
	bool chk_mems_state();
	//--------------------------------------------------------------------
  
  MPU_9250::TAccGyroMagDriver_HL &Hw;
  
  constexpr static uint8_t ANSWER_WAIT_MS = 100U;
  

protected:
private:
  void wait_sem( const TModel::TSem *endC, const TModel::TSem *begC, const TModel::TSem **cEl );
  bool chk_sample(const TModel::TAccelData &Data, TModel::TMemsOrient _ORIENT); //проверка валидности одной выборки
  
  constexpr static float VALID_THR = 1.3f;
};

#endif //__MEMS_H
