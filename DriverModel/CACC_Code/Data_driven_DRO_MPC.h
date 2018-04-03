//
// MATLAB Compiler: 6.4 (R2017a)
// Date: Mon Feb 12 16:13:27 2018
// Arguments:
// "-B""macro_default""-W""cpplib:Data_driven_DRO_MPC""-T""link:lib""-d""X:\Pape
// r\CACC\Code\Solver\DRO_MPC\DRO_MPC_DLL\Data_driven_DRO_MPC\for_testing""-v""X
// :\Paper\CACC\Code\Solver\DRO_MPC\Data_driven_DRO_MPC.m"
//

#ifndef __Data_driven_DRO_MPC_h
#define __Data_driven_DRO_MPC_h 1

#if defined(__cplusplus) && !defined(mclmcrrt_h) && defined(__linux__)
#  pragma implementation "mclmcrrt.h"
#endif
#include "mclmcrrt.h"
#include "mclcppclass.h"
#ifdef __cplusplus
extern "C" {
#endif

#if defined(__SUNPRO_CC)
/* Solaris shared libraries use __global, rather than mapfiles
 * to define the API exported from a shared library. __global is
 * only necessary when building the library -- files including
 * this header file to use the library do not need the __global
 * declaration; hence the EXPORTING_<library> logic.
 */

#ifdef EXPORTING_Data_driven_DRO_MPC
#define PUBLIC_Data_driven_DRO_MPC_C_API __global
#else
#define PUBLIC_Data_driven_DRO_MPC_C_API /* No import statement needed. */
#endif

#define LIB_Data_driven_DRO_MPC_C_API PUBLIC_Data_driven_DRO_MPC_C_API

#elif defined(_HPUX_SOURCE)

#ifdef EXPORTING_Data_driven_DRO_MPC
#define PUBLIC_Data_driven_DRO_MPC_C_API __declspec(dllexport)
#else
#define PUBLIC_Data_driven_DRO_MPC_C_API __declspec(dllimport)
#endif

#define LIB_Data_driven_DRO_MPC_C_API PUBLIC_Data_driven_DRO_MPC_C_API


#else

#define LIB_Data_driven_DRO_MPC_C_API

#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_Data_driven_DRO_MPC_C_API 
#define LIB_Data_driven_DRO_MPC_C_API /* No special import/export declaration */
#endif

extern LIB_Data_driven_DRO_MPC_C_API 
bool MW_CALL_CONV Data_driven_DRO_MPCInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_Data_driven_DRO_MPC_C_API 
bool MW_CALL_CONV Data_driven_DRO_MPCInitialize(void);

extern LIB_Data_driven_DRO_MPC_C_API 
void MW_CALL_CONV Data_driven_DRO_MPCTerminate(void);



extern LIB_Data_driven_DRO_MPC_C_API 
void MW_CALL_CONV Data_driven_DRO_MPCPrintStackTrace(void);

extern LIB_Data_driven_DRO_MPC_C_API 
bool MW_CALL_CONV mlxData_driven_DRO_MPC(int nlhs, mxArray *plhs[], int nrhs, mxArray 
                                         *prhs[]);


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__BORLANDC__)

#ifdef EXPORTING_Data_driven_DRO_MPC
#define PUBLIC_Data_driven_DRO_MPC_CPP_API __declspec(dllexport)
#else
#define PUBLIC_Data_driven_DRO_MPC_CPP_API __declspec(dllimport)
#endif

#define LIB_Data_driven_DRO_MPC_CPP_API PUBLIC_Data_driven_DRO_MPC_CPP_API

#else

#if !defined(LIB_Data_driven_DRO_MPC_CPP_API)
#if defined(LIB_Data_driven_DRO_MPC_C_API)
#define LIB_Data_driven_DRO_MPC_CPP_API LIB_Data_driven_DRO_MPC_C_API
#else
#define LIB_Data_driven_DRO_MPC_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_Data_driven_DRO_MPC_CPP_API void MW_CALL_CONV Data_driven_DRO_MPC(int nargout, mwArray& res, const mwArray& x_init, const mwArray& u_init, const mwArray& nveh_loc, const mwArray& dt, const mwArray& head, const mwArray& vlength, const mwArray& Obs);

#endif
#endif
