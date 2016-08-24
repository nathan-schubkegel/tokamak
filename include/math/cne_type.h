#ifndef CNE_TYPE_H
#define CNE_TYPE_H

//#include <stdarg.h>
//#include <tchar.h>
//#include <strsafe.h>
///////////////////////////////////////////////////////////////////////////
// DEFINES
///////////////////////////////////////////////////////////////////////////

//#ifdef NULL
//#undef NULL
//#endif
//
//#ifdef TRUE
//#undef TRUE
//#endif
//
//#ifdef FALSE
//#undef FALSE
//#endif

///////////////////////////////////////////////////////////////////////////

//#define FALSE       0                   // make sure that we know what false is
//#define TRUE        1                   // Make sure that we know what true is
//#define NULL        0                   // Make sure that null does have a type

#define TOKAMAK_STRUCT(t) struct t; typedef struct t t; struct t
#define TOKAMAK_STRUCT_DECL(t) struct t; typedef struct t t;
#define TOKAMAK_STRUCT_IMPL(t) struct t

///////////////////////////////////////////////////////////////////////////
// BASIC TYPES
///////////////////////////////////////////////////////////////////////////

typedef unsigned char       u8;
typedef unsigned short      u16;
typedef unsigned int        u32;
typedef signed   char       s8;
typedef signed   short      s16;
typedef signed   int        s32;
typedef float        f32;
typedef double              f64;
typedef u8                  cneByte;
typedef s32                 cneErr;
typedef s32                 cneBool;

  typedef signed   __int64    s64;
  typedef unsigned __int64    u64;
  #define cneFinite _finite
  //#define inline   __forceinline       // Make sure that the compiler inlines when we tell him
  //#define CNEINLINE __forceinline
  //const char PATH_SEP = '\\';

#endif //NE_TYPE_H
