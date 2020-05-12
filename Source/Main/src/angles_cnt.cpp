#include "angles_cnt.h"
#include "model.h"
#include "median_filter.h"

TMedianFilter BaseFilter( CONSTS::MIN_DEV_ANGLE );
TMedianFilter SensFilter( CONSTS::MIN_DEV_ANGLE );

constexpr TAngles::TRange ExtRange[ TMemsMount::__MAX ]
                                  [ TModel::TAxisRotate::__MAX_ROTATE ]
                                  [ TAngles::TQuadrant::__MAX ] 
=
{
  { //TMemsMount::__UP
    {
      {   0, 90, TAngles::TSign::__POZITIVE, &TAngles::q1_pos_cnt },
      {   0, 90, TAngles::TSign::__NEGATIVE, &TAngles::q2_pos_cnt },
      { -90,  0, TAngles::TSign::__NEGATIVE, &TAngles::q3_pos_cnt },
      { -90,  0, TAngles::TSign::__POZITIVE, &TAngles::q4_pos_cnt },
    },                       
    {                        
      { -90,  0, TAngles::TSign::__POZITIVE, &TAngles::q1_neg_cnt },
      { -90,  0, TAngles::TSign::__NEGATIVE, &TAngles::q2_neg_cnt },
      {   0, 90, TAngles::TSign::__NEGATIVE, &TAngles::q3_neg_cnt },
      {   0, 90, TAngles::TSign::__POZITIVE, &TAngles::q4_neg_cnt },
    },                       
    {                        
      { -90,  0, TAngles::TSign::__POZITIVE, &TAngles::q1_neg_cnt },
      { -90,  0, TAngles::TSign::__NEGATIVE, &TAngles::q2_neg_cnt },
      {   0, 90, TAngles::TSign::__NEGATIVE, &TAngles::q3_neg_cnt },
      {   0, 90, TAngles::TSign::__POZITIVE, &TAngles::q4_neg_cnt },
    },                       
    {                        
      {   0, 90, TAngles::TSign::__POZITIVE, &TAngles::q1_pos_cnt },
      {   0, 90, TAngles::TSign::__NEGATIVE, &TAngles::q2_pos_cnt },
      { -90,  0, TAngles::TSign::__NEGATIVE, &TAngles::q3_pos_cnt },
      { -90,  0, TAngles::TSign::__POZITIVE, &TAngles::q4_pos_cnt },
    },
  },
  { //TMemsMount::__DOWN
    {
      {   0, 90, TAngles::TSign::__NEGATIVE, &TAngles::q1_pos_cnt },
      {   0, 90, TAngles::TSign::__POZITIVE, &TAngles::q2_pos_cnt },
      { -90,  0, TAngles::TSign::__POZITIVE, &TAngles::q3_pos_cnt },
      { -90,  0, TAngles::TSign::__NEGATIVE, &TAngles::q4_pos_cnt },
    },                       
    {                        
      {   0, 90, TAngles::TSign::__NEGATIVE, &TAngles::q1_pos_cnt },
      {   0, 90, TAngles::TSign::__POZITIVE, &TAngles::q2_pos_cnt },
      { -90,  0, TAngles::TSign::__POZITIVE, &TAngles::q3_pos_cnt },
      { -90,  0, TAngles::TSign::__NEGATIVE, &TAngles::q4_pos_cnt },
    },                       
    {                        
      { -90,  0, TAngles::TSign::__NEGATIVE, &TAngles::q1_neg_cnt },
      { -90,  0, TAngles::TSign::__POZITIVE, &TAngles::q2_neg_cnt },
      {   0, 90, TAngles::TSign::__POZITIVE, &TAngles::q3_neg_cnt },
      {   0, 90, TAngles::TSign::__NEGATIVE, &TAngles::q4_neg_cnt },
    },                       
    {                        
      { -90,  0, TAngles::TSign::__NEGATIVE, &TAngles::q1_neg_cnt },
      { -90,  0, TAngles::TSign::__POZITIVE, &TAngles::q2_neg_cnt },
      {   0, 90, TAngles::TSign::__POZITIVE, &TAngles::q3_neg_cnt },
      {   0, 90, TAngles::TSign::__NEGATIVE, &TAngles::q4_neg_cnt },
    },  
  },  
};
  
TAngles Angles;

TAngles::TAngles()
:
DampCoeff( CONSTS::DAMP_COEFF ),
PrevBaseAngle( 0 )
{

}

TAngles::~TAngles()
{

}

typedef TAngles::TRange (TExtRange)[ TAngles::TQuadrant::__MAX ];

void TAngles::cnt_my_angle( TModel::TDevType Dev, TModel::TAxisRotate AxisRotate, bool IsBiasNeed )
{
  typedef void (TAngles::*TFnct)( TModel::TAxisRotate, bool );
  
  TFnct Fnct[] =
  {
    &TAngles::base,
    &TAngles::sens,    
  };
  
  (this->*Fnct[ Dev ])( AxisRotate, IsBiasNeed );
}

void TAngles::base( TModel::TAxisRotate AxisRotate, bool IsBiasNeed  )
{
  cnt_process( AxisRotate, IsBiasNeed, TMemsMount::__DOWN );
}

void TAngles::sens( TModel::TAxisRotate AxisRotate, bool IsBiasNeed  )
{
  cnt_process( AxisRotate, IsBiasNeed, TMemsMount::__UP );
}

void TAngles::cnt_process( TModel::TAxisRotate AxisRotate, bool IsBiasNeed, TMemsMount::T MemsMount )
{
  typedef int8_t (TModel::*TGetRawFnct)();

  struct THandler
  {
    TGetRawFnct get_raw_fnct;
    TGetRawFnct get_bias_fnct;
    int8_t      Coeff;
  };
  
  THandler Handler[] =
  {
    { &TModel::get_raw_roll,  &TModel::get_roll_bias_angle,   1, }, //TModel::TAxisRotate::__0_DEG
    { &TModel::get_raw_pitch, &TModel::get_pitch_bias_angle,  1, }, //TModel::TAxisRotate::__90_DEG
    { &TModel::get_raw_roll,  &TModel::get_roll_bias_angle,  -1, }, //TModel::TAxisRotate::__180_DEG
    { &TModel::get_raw_pitch, &TModel::get_pitch_bias_angle, -1, }, //TModel::TAxisRotate::__270_DEG 
  };  
  
//  TModel::TAxisRotate AxisRotate = static_cast<TModel::TAxisRotate>(Model.get_axis_rotate());
  int8_t Bias = 0;
  if ( IsBiasNeed == true )
  {
    Bias  = ( Model.*Handler[ AxisRotate ].get_bias_fnct )();
    Bias *= Handler[ AxisRotate ].Coeff;
  }
  
  for ( auto item : ExtRange[ MemsMount ][ AxisRotate ] )
  {
    int8_t Angle = ( Model.*Handler[ AxisRotate ].get_raw_fnct )();
    
    if ( 
        Angle >= item.Low
        &&
        Angle <= item.High
        &&
        get_z_sign() == item.ZSign
       )
    {
      Model.set_my_angle( (this->*item.CntFnct)( Angle ) - Bias );

      break;
    }
  }
}

void TAngles::cnt_open_angle()
{
  //определяем тип расположения(движения): в гору или с горы
  typedef void ( TAngles::*TPlaceFnct )( int16_t, int16_t );
  TPlaceFnct place_fnct[] =
  {
    &TAngles::place_up,
    &TAngles::place_down,
  };

  int16_t Base = Model.get_my_angle();
  int16_t Sens = Model.get_sens_angle();
  
  Base = BaseFilter.process( Base );
  Sens = SensFilter.process( Sens );  

  TPlaceType::T PlaceType = ( Base >= 0 ) ? TPlaceType::__UP
                                          : TPlaceType::__DOWN;
  ( this->*place_fnct[ PlaceType ] )( Base, Sens );
}

void TAngles::place_up( int16_t Base, int16_t Sens )
{
  if (
      Base <= BASE_THR.Up.Low
      ||
      (
       Base < BASE_THR.Up.High
       &&
       Model.get_base_pos_err() == TModel::TPosErr::_POS_OK
      )
     )
  {
    //база в первом квадранте
    Model.set_base_pos_err( TModel::TPosErr::_POS_OK );
        
    TOpenDetect OpenDetect[] =
    {
      {   0,  90, &TAngles::q1_pos_cnt }, //__Q_ONE  
      {  91, 180, &TAngles::q2_pos_cnt }, //__Q_TWO  
      { 181, 270, &TAngles::q3_pos_cnt }, //__Q_THREE
      { -90,  -1, &TAngles::q4_pos_cnt }, //__Q_FOUR 
    };
    
    for ( auto item : OpenDetect )
    {
      if (                  //определение квадранта датчика
          Sens >= item.Low
          &&
          Sens <= item.High
         )
      {
        int16_t CurOpenAngle = ( this->*item.CntFnct )( Base, Sens );
        cast_open_angle( &CurOpenAngle );                              //приведение угла открытия к формату: 0..260, -1..-80
        CurOpenAngle -= Model.get_bias();                              //с учетом смещения из настроек

        float PrevOpenAngle = Model.get_open_angle();

        cnt_damp( PrevOpenAngle, CurOpenAngle, DampCoeff );   //демпфирование угла
        Model.set_open_angle( PrevOpenAngle );
      }
    }
  }
  else
  {
    //положение базы недопустимо
    Model.set_base_pos_err( TModel::TPosErr::_POS_ERR );
    Model.set_open_angle( 0 );
    //угол открытия остается в предыдущем состоянии
  }
}

void TAngles::place_down( int16_t Base, int16_t Sens )
{
  if (
      Base >= BASE_THR.Down.High
      ||
      (
       Base > BASE_THR.Down.Low
       &&
       Model.get_base_pos_err() == TModel::TPosErr::_POS_OK
      )      
     )
  {
    //база в четвертом квадранте
    Model.set_base_pos_err( TModel::TPosErr::_POS_OK );

    TOpenDetect OpenDetect[] =
    {
      {   0,  90, &TAngles::q1_neg_cnt }, //__Q_ONE  
      {  91, 180, &TAngles::q2_neg_cnt }, //__Q_TWO  
      { 181, 270, &TAngles::q3_neg_cnt }, //__Q_THREE
      { -90,  -1, &TAngles::q4_neg_cnt }, //__Q_FOUR 
    };
    
    for ( auto item : OpenDetect )
    {
      if (
          Sens >= item.Low  //определение квадранта датчика
          &&
          Sens <= item.High
         )
      {
        int16_t CurOpenAngle = ( this->*item.CntFnct )( Base, Sens );
        cast_open_angle( &CurOpenAngle );                              //приведение угла открытия к формату: 0..260, -1..-80
        CurOpenAngle -= Model.get_bias();                              //с учетом смещения из настроек
        
        float PrevOpenAngle = Model.get_open_angle();

        cnt_damp( PrevOpenAngle, CurOpenAngle, DampCoeff );   //демпфирование угла
        Model.set_open_angle( PrevOpenAngle );
      }
    }
  }
  else
  {
    //положение базы недопустимо
    Model.set_base_pos_err( TModel::TPosErr::_POS_ERR );
    Model.set_open_angle( 0 );
    //угол открытия остается в предыдущем состоянии
  }
}

void TAngles::cnt_damp( float &prev_angle, int16_t cur_angle, float coeff )
{
  prev_angle = prev_angle + ( static_cast<float>(cur_angle) - prev_angle ) / coeff;
}

void TAngles::cast_open_angle( int16_t *Angle )
{
  if ( *Angle > CONSTS::MAX_DEV_ANGLE )
  {
    *Angle = CONSTS::MAX_DEV_ANGLE;
    
    return;
  }
  if ( *Angle < CONSTS::MIN_DEV_ANGLE )
  {
    *Angle = CONSTS::MIN_DEV_ANGLE;
  }
}

TAngles::TSign TAngles::get_z_sign()
{
  return ( Model.get_accel_z() >= 0 ) ? __POZITIVE
                                      : __NEGATIVE;
}
//-----------------------------------------------------------

//----- Вычисление угла блока -------------------------------
int16_t TAngles::q1_pos_cnt( int8_t Angle )
{
  return Angle;
}

int16_t TAngles::q2_pos_cnt( int8_t Angle )
{
  return 180 - Angle;
}

int16_t TAngles::q3_pos_cnt( int8_t Angle )
{
  int16_t Res = 180 - Angle;
  
  if ( Res > CONSTS::MAX_DEV_ANGLE )
  {
    Res = CONSTS::MAX_DEV_ANGLE;
  }
  
  return Res;
}

int16_t TAngles::q4_pos_cnt( int8_t Angle )
{
  return ( Angle >= CONSTS::MIN_DEV_ANGLE ) ? Angle
                                            : CONSTS::MIN_DEV_ANGLE;
}

int16_t TAngles::q1_neg_cnt( int8_t Angle )
{
  return -Angle;
}

int16_t TAngles::q2_neg_cnt( int8_t Angle )
{
  return 180 + Angle;
}

int16_t TAngles::q3_neg_cnt( int8_t Angle )
{
  int16_t Res = 180 + Angle;
  
  if ( Res > CONSTS::MAX_DEV_ANGLE )
  {
    Res = CONSTS::MAX_DEV_ANGLE;
  }
  
  return Res;
}

int16_t TAngles::q4_neg_cnt( int8_t Angle )
{
  return ( -Angle >= CONSTS::MIN_DEV_ANGLE ) ? -Angle
                                             : CONSTS::MIN_DEV_ANGLE;
}
//-----------------------------------------------------------
  
//----- Вычисление угла открытия ----------------------------
int16_t TAngles::q1_pos_cnt( int16_t Base, int16_t Sens )
{
  return Sens - Base;
}

int16_t TAngles::q2_pos_cnt( int16_t Base, int16_t Sens )
{
  return Sens - Base;
}

int16_t TAngles::q3_pos_cnt( int16_t Base, int16_t Sens )
{
  return Sens - Base;
}

int16_t TAngles::q4_pos_cnt( int16_t Base, int16_t Sens )
{
  return ( (Sens - Base) >= CONSTS::MIN_DEV_ANGLE ) ? Sens - Base
                                                    : 360 + Sens - Base;
}


int16_t TAngles::q1_neg_cnt( int16_t Base, int16_t Sens )
{
  return Sens - Base;
}

int16_t TAngles::q2_neg_cnt( int16_t Base, int16_t Sens )
{
  return Sens - Base;
}

int16_t TAngles::q3_neg_cnt( int16_t Base, int16_t Sens )
{
  return Sens - Base;
}

int16_t TAngles::q4_neg_cnt( int16_t Base, int16_t Sens )
{
  return Sens - Base;
}
//-----------------------------------------------------------
