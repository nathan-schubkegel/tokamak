#ifndef CNE_SMATH_H
#define CNE_SMATH_H

#include <math.h>
#include "cne_type.h"

#define CNE_PI  (3.141592653589793238462643f)

#define CNE_RAD_TO_DEG(A) ((f32)(((A) * (180.0f / CNE_PI))))
#define CNE_DEG_TO_RAD(A) ((f32)(((A) * (NE_PI / 180.0f))))
#define CNE_RI        CNE_DEG_TO_RAD(1)   
#define CNE_ZERO (1.0e-6f)

typedef f32 cneRadian;

///////////////////////////////////////////////////////////////////////////
//
// GENERAL
//
///////////////////////////////////////////////////////////////////////////

f32      cneFRand      ( f32 Min, f32 Max );
f32      cneSin        ( cneRadian S );
cneRadian    cneASin       ( f32 S );
f32         cneCos        ( cneRadian C );
cneRadian    cneACos       ( f32 C );
f32         cneTan        ( cneRadian T );
cneRadian    cneATan       ( f32 T );
cneRadian    cneATan2      ( f32 y, f32 x );
cneBool    cneRealsEqual  ( f32 s1, f32 s2);
cneBool    cneIsConsiderZero(f32 f);
cneBool    cneIsFinite  (f32);

//template< class ta >                     NEINLINE ta      neAbs     ( const ta&  A )                               { return ( A < 0 ) ? -A : A;   }

#if 0

NEINLINE f32 cneAbs(f32 f)
{
  return (f32)fabs(f);
}

template< class ta, class tb, class tc > NEINLINE neBool  neInRange ( const ta&  X, const tb& Min, const tc& Max ) { return (Min <= X) && (X <= Max);}
template< class ta, class tb, class tc > NEINLINE ta      neRange   ( const ta&  X, const tb& Min, const tc& Max ) { if( X < Min ) return Min; return(X > Max) ? Max : X; }
template< class ta>             NEINLINE void    neSwap    ( ta &  X, ta & Y) { ta tmp = X; X = Y; Y = tmp; }
template< class ta >                     NEINLINE ta      neSqr     ( const ta&  A )                               { return A * A; }
template< class ta >                     NEINLINE ta      neMin     ( const ta&  A, const ta& B )                  { return ( A < B ) ?  A : B;   }
template< class ta >                     NEINLINE ta      neMax     ( const ta&  A, const ta& B )                  { return ( A > B ) ?  A : B;   }
NEINLINE f32     neMin     ( const s32& A, const f32& B )                 { return ( A < B ) ?  A : B;   }
NEINLINE f32     neMax     ( const s32& A, const f32& B )                 { return ( A > B ) ?  A : B;   }
NEINLINE f32     neMin     ( const f32& A, const s32& B )                 { return ( A < B ) ?  A : B;   }
NEINLINE f32     neMax     ( const f32& A, const s32& B )                 { return ( A > B ) ?  A : B;   }
NEINLINE neBool     neIsFinite  (f32 n) {return neFinite((double)n);} 
#endif

#endif
