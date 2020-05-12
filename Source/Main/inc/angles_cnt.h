#ifndef __ANGLES_CNT_H
#define __ANGLES_CNT_H

#include <stdint.h>
#include <math.h>

#include "model.h"

//диапазон углов, выдаваемых после пересчета данных с акселерометра в углы \
  по формулам из AN4509, раздел Tri-axis tilt sensing

namespace TPlaceType
{
  enum T : uint8_t
  {
    __UP   = 0,
    __DOWN = 1,
  };
}

namespace TMemsMount
{
  enum T : uint8_t
  {
    __UP   = 0, //на верхней стороне платы
    __DOWN = 1, //на нижней стороне платы
    
    __MAX  = __DOWN + 1,
  };
}

class TAngles
{
public:
  enum TQuadrant : uint8_t
  {
    __Q_ONE    = 0,
    __Q_TWO    = 1,
    __Q_THREE  = 2,
    __Q_FOUR   = 3,
    
    __MAX      = __Q_FOUR + 1,
  };
  
  enum TSign : bool
  {
    __POZITIVE = 0,
    __NEGATIVE = 1,
  };
  
//  enum TPlaceType : uint8_t
//  {
//    __UP   = 0,
//    __DOWN = 1,
//  };
  
  
  typedef int16_t (TAngles::*TDevCntFnct)( int8_t );
  
  struct TRange 
  {
    int16_t     Low;
    int16_t     High;
    TSign       ZSign;
    TDevCntFnct CntFnct;
  };

  typedef int16_t (TAngles::*TOpenCntFnct)( int16_t, int16_t );

  struct TOpenDetect
  {
    int16_t      Low;
    int16_t      High;
    TOpenCntFnct CntFnct;
  };

  struct TThrF
  {
    float Low;
    float High;
  };

  struct TThrI
  {
    int16_t Low;
    int16_t High;
  };
  
  struct THyst
  {
    TThrI Up;
    TThrI Down;
  };

public:
  TAngles();
  ~TAngles();

  void cnt_my_angle( TModel::TDevType, TModel::TAxisRotate, bool IsBiasNeed );
  void cnt_open_angle();
  
  //----- Вычисление угла блока -------------------------------
  int16_t q1_pos_cnt( int8_t );
  int16_t q2_pos_cnt( int8_t );
  int16_t q3_pos_cnt( int8_t );
  int16_t q4_pos_cnt( int8_t );

  int16_t q1_neg_cnt( int8_t );
  int16_t q2_neg_cnt( int8_t );
  int16_t q3_neg_cnt( int8_t );
  int16_t q4_neg_cnt( int8_t );
  //-----------------------------------------------------------
  
  //----- Вычисление угла открытия ----------------------------
  int16_t q1_pos_cnt( int16_t, int16_t );
  int16_t q2_pos_cnt( int16_t, int16_t );
  int16_t q3_pos_cnt( int16_t, int16_t );
  int16_t q4_pos_cnt( int16_t, int16_t );

  int16_t q1_neg_cnt( int16_t, int16_t );
  int16_t q2_neg_cnt( int16_t, int16_t );
  int16_t q3_neg_cnt( int16_t, int16_t );
  int16_t q4_neg_cnt( int16_t, int16_t );
  //-----------------------------------------------------------
  
  uint8_t DampCoeff;
protected:
  
private:    
  void    base( TModel::TAxisRotate, bool IsBiasNeed );
  void    sens( TModel::TAxisRotate, bool IsBiasNeed );
  void    cnt_process( TModel::TAxisRotate, bool IsBiasNeed, TMemsMount::T MemsMount );
  
  
  TSign   get_z_sign();
  
  void    place_up( int16_t, int16_t );   //база в первом квадранте
  void    place_down( int16_t, int16_t ); //база в четветом квадранте
  
  void cast_open_angle( int16_t * ); //приведение угла к формату: 0..260, -1..-80
  void cnt_damp( float &prev_angle, int16_t cur_angle, float coeff );
  
  constexpr static THyst BASE_THR = 
  {
    { 
     CONSTS::MAX_BASE_ANGLE - CONSTS::HYST_BASE_ANGLE, 
     CONSTS::MAX_BASE_ANGLE                   
    },
    { 
     CONSTS::MIN_BASE_ANGLE,
     CONSTS::MIN_BASE_ANGLE + CONSTS::HYST_BASE_ANGLE
    },
  };
  
  int16_t PrevBaseAngle; 
  
};

extern TAngles Angles;

#endif //__ANGLES_CNT_H
