#ifndef __c2_nonlinear_model_h__
#define __c2_nonlinear_model_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc2_nonlinear_modelInstanceStruct
#define typedef_SFc2_nonlinear_modelInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c2_sfEvent;
  boolean_T c2_isStable;
  boolean_T c2_doneDoubleBufferReInit;
  uint8_T c2_is_active_c2_nonlinear_model;
  real_T *c2_time;
  real_T (*c2_d_states)[5];
  real_T (*c2_states)[5];
  real_T (*c2_ctrls)[2];
  real_T *c2_cD0;
  real_T *c2_cDa;
  real_T *c2_cDa2;
  real_T *c2_cL0;
  real_T *c2_cLa;
  real_T *c2_cLa2;
  real_T *c2_cLa3;
  real_T *c2_cLq;
  real_T *c2_cLde;
  real_T *c2_cm0;
  real_T *c2_cma;
  real_T *c2_cmq;
  real_T *c2_cmde;
  real_T *c2_cT0;
  real_T *c2_cT1;
  real_T *c2_cT2;
  real_T *c2_tauT;
  real_T *c2_clb;
  real_T *c2_clp;
  real_T *c2_clr;
  real_T *c2_clda;
  real_T *c2_cYb;
  real_T *c2_cnb;
  real_T *c2_cnp;
  real_T *c2_cnr;
  real_T *c2_cndr;
  real_T *c2_wn;
  real_T *c2_we;
  real_T *c2_wd;
  real_T (*c2_output)[5];
} SFc2_nonlinear_modelInstanceStruct;

#endif                                 /*typedef_SFc2_nonlinear_modelInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c2_nonlinear_model_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c2_nonlinear_model_get_check_sum(mxArray *plhs[]);
extern void c2_nonlinear_model_method_dispatcher(SimStruct *S, int_T method,
  void *data);

#endif
