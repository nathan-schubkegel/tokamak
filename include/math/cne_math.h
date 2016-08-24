#ifndef CNE_MATH_H
#define CNE_MATH_H

#include <math.h>
#include <float.h>
#include "cne_type.h"
#include "cne_debug.h"
#include "cne_smath.h"

/****************************************************************************
*
*  cneV3
*
****************************************************************************/ 

//s32 cneNextDim1[] = {1, 2, 0};
//s32 cneNextDim2[] = {2, 0, 1};

TOKAMAK_STRUCT(cneV3)
{
  f32 v[4];
};
void cneV3_Init(cneV3 * obj);
//NEINLINE cneV3 & SetZero (void );
//NEINLINE cneV3 & SetOne(void);
//NEINLINE cneV3 & SetHalf(void);
//NEINLINE cneV3 & Set(f32 value);
void cneV3_Set(cneV3 * obj, f32 x, f32 y, f32 z);
//NEINLINE cneV3 & Set  (const cneV3& V);
//NEINLINE cneV3 & Set  (const cneQ& Q);
//NEINLINE void  Set  (f32 val[3]);
//NEINLINE void  Get  (f32 val[3]);
//NEINLINE f32&  operator   []   (s32 I);
//NEINLINE f32  operator   []  (s32 I) const;
//NEINLINE f32 X() const;
//NEINLINE f32 Y() const;
//NEINLINE f32 Z() const;
//NEINLINE f32 W() const;
//NEINLINE void   Normalize  (void);
//NEINLINE f32    Length      (void) const;
//NEINLINE f32    Dot         (const cneV3& V) const;
//NEINLINE cneV3   Cross       (const cneV3& V) const;
//NEINLINE void  RotateX      (cneRadian Rx);
//NEINLINE void  RotateY      (cneRadian Ry);
//NEINLINE void  RotateZ      (cneRadian Rz);
//NEINLINE cneRadian GetPitch      (void) const;
//NEINLINE cneRadian GetYaw        (void) const;
//NEINLINE void  SetBoxTensor  (f32 width, f32 height, f32 depth, f32 mass);
//NEINLINE void  SetAbs      (const cneV3 & v);
//NEINLINE f32  GetDistanceFromLine(const cneV3 & point1, const cneV3 & point2);
//NEINLINE f32  GetDistanceFromLine2(cneV3 & project, const cneV3 & pointA, const cneV3 & pointB);
//NEINLINE f32  GetDistanceFromLineAndProject(cneV3 & result, const cneV3 & startPoint, const cneV3 & dir);
//NEINLINE cneBool  GetIntersectPlane(cneV3 & normal, cneV3 & pointOnPlane, cneV3 & point1, cneV3 & point2);
//NEINLINE void  SetMin      (const cneV3& V1, const cneV3& V2);
//NEINLINE void  SetMax      (const cneV3& V1, const cneV3& V2);
//NEINLINE void  RemoveComponent  (const cneV3& V);
//NEINLINE bool  IsConsiderZero  () const; 
//NEINLINE bool  IsFinite    () const;
//NEINLINE cneV3  Project      (const cneV3 & v) const;
//NEINLINE cneV3& operator /= (f32 S);
//NEINLINE cneV3& operator *= (f32 S);
//NEINLINE cneV3& operator += (const neV3& V);
//NEINLINE cneV3& operator -= (const neV3& V);
//NEINLINE cneV3 friend operator + (const cneV3& V1, const cneV3& V2);
//NEINLINE cneV3 friend operator - (const cneV3& V1, const cneV3& V2);
//NEINLINE cneV3 friend operator / (const cneV3& V,  f32 S);
//NEINLINE cneV3 friend operator * (const cneV3& V,  f32 S);
//NEINLINE cneV3 friend operator * (const cneV3& V1, const cneV3& V2);
//NEINLINE cneV3 friend operator * (const cneV3& V,  const cneM3& M);
//NEINLINE cneM3 friend operator ^ (const cneV3 & V, const cneM3 & M); //cross product operator
//NEINLINE friend cneV3 operator - (const cneV3& V );
//NEINLINE friend cneV3 operator * (f32 S,  const cneV3& V  );


/****************************************************************************
*
*  cneV4
*
****************************************************************************/ 

TOKAMAK_STRUCT(cneV4)
{
    f32 X, Y, Z, W;

    // functions
    //NEINLINE neV4 ( void );
    //NEINLINE neV4 ( f32 x, f32 y, f32 z, f32 w );
    //NEINLINE neV4 ( const neV3& V, f32 w );
    //NEINLINE neV4 ( const neV4& V );
    //NEINLINE void SetZero ( void );
    //NEINLINE void Set ( f32 x, f32 y, f32 z, f32 w );

    //NEINLINE f32&    operator   []   ( s32 I );
    //NEINLINE neV4&   operator   /=   ( f32 S );
    //NEINLINE neV4&   operator   *=   ( f32 S );
    //NEINLINE neV4&   operator   +=   ( const neV4& V );
    //NEINLINE neV4&   operator   -=   ( const neV4& V );
    //NEINLINE neV4&   Normalize       ( void );
    //NEINLINE f32     Length          ( void ) const;
    //NEINLINE f32     Dot             ( const neV4& V ) const;

    //NEINLINE friend neV4 operator -  ( const neV4& V );
    //NEINLINE friend neV4 operator *  ( f32             S,  const neV4& V  );
    //NEINLINE friend neV4 operator /  ( const neV4& V,  f32             S  );
    //NEINLINE friend neV4 operator *  ( const neV4& V,  f32             S  );
    //NEINLINE friend neV4 operator +  ( const neV4& V1, const neV4& V2 );
    //NEINLINE friend neV4 operator -  ( const neV4& V1, const neV4& V2 );
};

/****************************************************************************
*
*  cneM3
*
****************************************************************************/ 

TOKAMAK_STRUCT(cneM3)
{
  cneV3 M[3];

  //  NEINLINE neV3&  operator   [] ( s32 I );
  //NEINLINE neV3  operator   [] ( s32 I ) const;

  //  NEINLINE void SetZero ( void );
  //  NEINLINE void SetIdentity ( void );
  //NEINLINE neBool SetInvert(const neM3 & rhs);
  //NEINLINE neM3 & SetTranspose ( neM3 & M );
  //NEINLINE void GetColumns( neV3& V1, neV3& V2, neV3& V3 ) const;
  //NEINLINE void SetColumns( const neV3& V1, const neV3& V2, const neV3& V3 );
  //NEINLINE neV3 GetDiagonal();
  //NEINLINE neV3 TransposeMulV3(const neV3 & V);
  //NEINLINE void RotateXYZ(const neV3 & rotate);
  //NEINLINE neM3 & SkewSymmetric(const neV3 & V);

  //NEINLINE neBool IsIdentity() const;
  //NEINLINE neBool IsOrthogonalNormal() const;

  //NEINLINE neBool IsFinite() const;
  //
  //NEINLINE neM3 &operator += ( const neM3& add);
  //NEINLINE neM3 operator ^ (const neV3 & vec) const; //cross product
  //NEINLINE neM3 operator * (f32 scalar) const;

  //NEINLINE friend neM3 operator +    ( const neM3& add1, const neM3& add2);
  //NEINLINE friend neM3 operator -    ( const neM3& sub1, const neM3& sub2);
  //NEINLINE friend neM3 operator *     ( const neM3& M1, const neM3& M2 );
  //NEINLINE friend neV3 operator *     ( const neM3& M1, const neV3& V  );
  //NEINLINE friend neM3 operator *     ( f32 scalar, const neM3& M );
  //NEINLINE friend neM3& operator *=   ( neM3& M1, const f32 f  );
  //NEINLINE friend neM3& operator /=   ( neM3& M1, const f32 f  );
};

/****************************************************************************
*
*  cneM4
*
****************************************************************************/ 

TOKAMAK_STRUCT(cneM4)
{
    f32 M[4][4];

  //  // functions
  //NEINLINE void Set(float row00, float row01, float row02, float row03
  //          , float row10, float row11, float row12, float row13
  //          , float row20, float row21, float row22, float row23
  //          , float row30, float row31, float row32, float row33);

  //NEINLINE void Set(int row, int col, f32 val){M[col][row] = val;}
  //  NEINLINE void SetZero        ( void );
  //  NEINLINE void SetIdentity    ( void );
  //                                          
  //  NEINLINE neV3  GetScale       ( void ) const;
  //  NEINLINE neV3  GetTranslation ( void ) const;

  //  NEINLINE void  SetTranslation ( const neV3& V ); 
  //  NEINLINE void  SetScale       ( const neV3& V ); 

  //  NEINLINE f32&  operator []    ( s32 I );
  //  NEINLINE neM4& operator *=    ( const neM4& M );
  //NEINLINE neM4& operator =     ( const neM3& M );
  //  NEINLINE neV3  TransformAs3x3 ( const neV3& V ) const;

  //  NEINLINE void  GetRows        ( neV3& V1, neV3& V2, neV3& V3 ) const;
  //  NEINLINE void  SetRows        ( const neV3& V1, const neV3& V2, const neV3& V3 );
  //  NEINLINE void  GetColumns     ( neV3& V1, neV3& V2, neV3& V3 ) const;
  //  NEINLINE void  SetColumns     ( const neV3& V1, const neV3& V2, const neV3& V3 );
  //NEINLINE void  GetColumn    ( neV3& V1, u32 col) const;

  //NEINLINE void SetTranspose(const neM4 & M);
  //NEINLINE void SetFastInvert  ( const neM4& Src );
  //
  //NEINLINE friend neM4 operator *             ( const neM4& M1, const neM4& M2 );
  //  NEINLINE friend neV3 operator *             ( const neM4& M1, const neV3& V  );
};

/****************************************************************************
*
*  cneQ
*
****************************************************************************/ 

TOKAMAK_STRUCT(cneQ)
{
    f32 X, Y, Z, W;

    // functions
  //  NEINLINE neQ         ( void );
  //NEINLINE neQ         ( f32 X, f32 Y, f32 Z, f32 W );
  //NEINLINE neQ         ( const neM4& M );

  //  NEINLINE void    Zero                ( void );
  //  NEINLINE void       Identity            ( void );
  //  NEINLINE void       SetupFromMatrix     ( const neM4& Matrix );
  //NEINLINE void       SetupFromMatrix3    ( const neM3& Matrix );
  //  NEINLINE void       GetAxisAngle        ( neV3& Axis, neRadian& Angle ) const;

  //  NEINLINE neM4       BuildMatrix         ( void ) const;
  //NEINLINE neM3       BuildMatrix3         ( void ) const;
  //  NEINLINE neQ&       Normalize           ( void );
  //  NEINLINE f32        Dot                 ( const neQ& Q ) const;
  //  NEINLINE neQ&       Invert              ( void );
  //NEINLINE neBool    IsFinite      ();

  //  NEINLINE neQ&       operator *=         ( const neQ& Q );
  //  NEINLINE neQ&       operator *=         ( f32 S );
  //  NEINLINE neQ&       operator +=         ( const neQ& Q );
  //  NEINLINE neQ&       operator -=         ( const neQ& Q );

  //NEINLINE neQ&    Set          (f32 X, f32 Y, f32 Z, f32 W);
  //NEINLINE neQ&    Set          (const neV3 & V, f32 W);
  //NEINLINE neQ&    Set          (f32 angle, const neV3 & axis);

  //  NEINLINE friend neQ  operator -          ( const neQ& V );
  //  NEINLINE friend neQ  operator *          ( const neQ& Qa, const neQ& Qb );
  //  NEINLINE friend neV3 operator *          ( const neQ& Q,  const neV3& V );
  //  NEINLINE friend neQ  operator *          ( const neQ& Q, f32 S );
  //  NEINLINE friend neQ  operator *          ( f32 S, const neQ& Q );
  //  NEINLINE friend neQ  operator +          ( const neQ& Qa, const neQ& Qb );
  //  NEINLINE friend neQ  operator -          ( const neQ& Qa, const neQ& Qb );
};

/****************************************************************************
*
*  cneT3
*
****************************************************************************/ 

TOKAMAK_STRUCT(cneT3)
{
  cneM3 rot;
  cneV3 pos;
};
void cneT3_Init(cneT3 * obj);
//NEINLINE neT3 FastInverse();
//NEINLINE neT3 operator * (const neT3 & t);
//NEINLINE neV3 operator * (const neV3 & v);
//NEINLINE neBool IsFinite();
//NEINLINE void MakeD3DCompatibleMatrix()
//{
//  rot[0].v[3] = 0.0f;
//  rot[1].v[3] = 0.0f;
//  rot[2].v[3] = 0.0f;
//  pos.v[3] = 1.0f;
//}
//  NEINLINE void SetIdentity()
//  {
//    rot.SetIdentity();
//    pos.SetZero();
///*
// *    additional code to make this binary compatible with rendering matrix
// */
//    MakeD3DCompatibleMatrix();
//  }

///////////////////////////////////////////////////////////////////////////
// INCLUDE INLINE HEADERS
///////////////////////////////////////////////////////////////////////////

#include "cne_math_misc_inline.h"
#include "cne_math_v3_inline.h"
#include "cne_math_v4_inline.h"
#include "cne_math_m4_inline.h"
#include "cne_math_m3_inline.h"
#include "cne_math_q_inline.h"
#include "cne_math_t3_inline.h"

///////////////////////////////////////////////////////////////////////////
// END
///////////////////////////////////////////////////////////////////////////
#endif //CNE_MATH_H
