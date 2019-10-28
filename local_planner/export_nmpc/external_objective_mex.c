#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "mex.h"
#include "acado_common.h"
#include "lsq_objective.c"

/* Define number of outputs */
#define NOO 5
#define N_AUX 15

enum aux {
    AUX_E_LAT = 0,
    AUX_E_LON,
    AUX_H_TERR,
    AUX_R_OCC_FWD,
    AUX_R_OCC_LEFT,
    AUX_R_OCC_RIGHT,
    AUX_OCC_DETECT_FWD,
    AUX_OCC_DETECT_LEFT,
    AUX_OCC_DETECT_RIGHT,
    AUX_P_OCC_FWD,
    AUX_N_OCC_FWD = 12
};

/** Instance of the user data structure. */
ACADOvariables acadoVariables;
/** Instance of the private workspace structure. */
ACADOworkspace acadoWorkspace;

/** A bit more advanced printing function. */
void mexErrMsgTxtAdv(	char* string,
						...
						)
{
	static char buffer[ 128 ];
	
	va_list printArgs;
	va_start(printArgs, string);
	
	vsprintf(buffer, string, printArgs);
	va_end( printArgs );

	mexErrMsgTxt( buffer );
}

/** A simple helper function. */
void printMatrix(	const char* name,
					real_t* mat,
					unsigned nRows,
					unsigned nCols
					)
{
    unsigned r, c;
    mexPrintf("%s: \n", name);
    for (r = 0; r < nRows; ++r)
    {
        for(c = 0; c < nCols; ++c)
            mexPrintf("\t%f", mat[r * nCols + c]);
        mexPrintf("\n");
    }
}

/** A function for copying data from MATLAB to C array. */
int getArray(	const unsigned mandatory,
				const mxArray* source,
				const int index,
				const char* name,
				real_t* destination,
				const unsigned nRows,
				const unsigned nCols
				)
{
	mxArray* mxPtr = mxGetField(source, index, name);
	unsigned i, j;
	double* dPtr;
	
	if (mxPtr == NULL)
	{
		if ( !mandatory )
			return -1;
		else
			mexErrMsgTxtAdv("Field %s not found.", name);
	}

    if ( !mxIsDouble( mxPtr ) )
		mexErrMsgTxtAdv("Field %s must be an array of doubles.", name);

    if (mxGetM( mxPtr ) != nRows || mxGetN( mxPtr ) != nCols )
		mexErrMsgTxtAdv("Field %s must be of size: %d x %d.", name, nRows, nCols);

	dPtr = mxGetPr( mxPtr );
	
	if (destination == NULL)
		destination = (real_t*)mxCalloc(nRows * nCols, sizeof( real_t ));

	if (nRows == 1 && nCols == 1)
		*destination = (real_t)*dPtr;
	else
		for (i = 0; i < nRows; ++i)
			for (j = 0; j < nCols; ++j)
				destination[i * nCols + j] = (real_t)dPtr[j * nRows + i];
			
	return 0;
}

void setArray( 	mxArray* destination,
				const int index,
				const char* name,
				real_t* source,
				const unsigned nRows,
				const unsigned nCols
				)
{
	mxArray* mxPtr = mxCreateDoubleMatrix(nRows, nCols, mxREAL);
	double* dPtr = mxGetPr( mxPtr );
	unsigned i, j;
	
	if (nRows == 1 && nCols == 1)
		*dPtr = (double)*source;
	else
		for (i = 0; i < nRows; ++i)
			for(j = 0; j < nCols; ++j)
				dPtr[j * nRows + i] = (double)source[i * nCols + j];

	mxSetField(destination, index, name, mxPtr);
}

/** The MEX interface function. */
void mexFunction(	int nlhs,
					mxArray *plhs[],
					int nrhs,
					const mxArray *prhs[]
					)
{
	const mxArray* src = prhs[ 0 ];
	
	real_t tmp[ 1 ];
	
	const char *outNames[ NOO ];
    outNames[ 0 ] = "y";
	outNames[ 1 ] = "yN";
    outNames[ 2 ] = "od";
    outNames[ 3 ] = "priorities";
    outNames[ 4 ] = "aux";
    
	if (nrhs != 1)
		mexErrMsgTxt(
			"This function requires exactly one input: a structure with parameters.");
			
	if (nlhs != 1)
		mexErrMsgTxt(
			"This function returns one output.");
			
	if( !mxIsStruct( src ) )
		mexErrMsgTxt("The function argument must be a structure.");
		
	/* Copy MATLAB arrays to C arrays. */
	getArray(1, src, 0, "x", acadoVariables.x, ACADO_N + 1, ACADO_NX);
	getArray(1, src, 0, "y", acadoVariables.y, ACADO_N, ACADO_NY);
    getArray(1, src, 0, "yN", acadoVariables.yN, 1, ACADO_NYN);
	getArray(1, src, 0, "od", acadoVariables.od, ACADO_N + 1, ACADO_NOD);
    
    double aoa_params[5];
    double terr_params[13];
    double terr_map[LEN_IDX_N*LEN_IDX_E];
    double path_reference[5];
    double guidance_params[4];
    getArray(1, src, 0, "aoa_params", aoa_params, 1, 5);
    getArray(1, src, 0, "terr_params", terr_params, 1, 13);
    getArray(1, src, 0, "terr_map", terr_map, 1, LEN_IDX_N*LEN_IDX_E);
    getArray(1, src, 0, "path_reference", path_reference, 1, 5);
    getArray(1, src, 0, "guidance_params", guidance_params, 1, 4);
    
    /* init */
    double sig_aoa;
    double jac_sig_aoa[2];
    double prio_aoa;
    double sig_h;
    double jac_sig_h[4];
    double prio_h;
    double sig_r_fwd;
    double sig_r_left;
    double sig_r_right;
    double jac_sig_r[6];
    double jac_sig_r_fwd[6];
    double jac_sig_r_left[6];
    double jac_sig_r_right[6];
    double prio_r;
    double priorities[(ACADO_N+1)*3];
    double aux_output[(ACADO_N+1)*N_AUX];
    int occ_detected_fwd = 0;
    int occ_detected_left = 0;
    int occ_detected_right = 0;
    double r_occ_fwd;
    double r_occ_left;
    double r_occ_right;
    double sig_input_r_fwd;
    double sig_input_r_left;
    double sig_input_r_right;
    double p_occ_fwd[3];
    double n_occ_fwd[3];
    
    /* evaluate external objectives */
    
    int runObj;
    for (runObj = 0; runObj < ACADO_N +1; ++runObj)
    {
        /* soft aoa constraint */
        calculate_aoa_objective(&sig_aoa, jac_sig_aoa, &prio_aoa, acadoVariables.x + (runObj * ACADO_NX), aoa_params);
                
        acadoVariables.od[runObj * ACADO_NOD + 8] = sig_aoa;
        acadoVariables.od[runObj * ACADO_NOD + 9] = jac_sig_aoa[0];
        acadoVariables.od[runObj * ACADO_NOD + 10] = jac_sig_aoa[1];
                
        priorities[runObj * 3 + 0] = prio_aoa * prio_aoa;
                
        /* soft height constraint */
        double h_terr;
        calculate_height_objective(&sig_h, jac_sig_h, &prio_h, &h_terr, acadoVariables.x + (runObj * ACADO_NX), terr_params, terr_map);
                
        acadoVariables.od[runObj * ACADO_NOD + 11] = sig_h;
        acadoVariables.od[runObj * ACADO_NOD + 12] = jac_sig_h[0];
        acadoVariables.od[runObj * ACADO_NOD + 13] = jac_sig_h[1];
        acadoVariables.od[runObj * ACADO_NOD + 14] = jac_sig_h[2];
        acadoVariables.od[runObj * ACADO_NOD + 15] = jac_sig_h[3];
                
        priorities[runObj * 3 + 1] = prio_h * prio_h;
        
        aux_output[runObj * N_AUX + AUX_H_TERR] = h_terr;
                
        /* calculate speed states */
        const double v = acadoVariables.x[runObj * ACADO_NX + 3];
        const double gamma = acadoVariables.x[runObj * ACADO_NX + 4];
        const double xi = acadoVariables.x[runObj * ACADO_NX + 5];
        const double w_n = acadoVariables.od[runObj * ACADO_NOD + 1];
        const double w_e = acadoVariables.od[runObj * ACADO_NOD + 2];
        const double w_d = acadoVariables.od[runObj * ACADO_NOD + 3];
        double speed_states[12];
        calculate_speed_states(speed_states, v, gamma, xi, w_n, w_e, w_d);
                
        /* soft radial constraint */
        
        /* forward ray */
        double v_ray[3] = {speed_states[10], speed_states[9], -speed_states[11]};
        calculate_radial_objective(&sig_r_fwd, jac_sig_r_fwd, &r_occ_fwd, p_occ_fwd, n_occ_fwd, &sig_input_r_fwd, &occ_detected_fwd, v_ray, acadoVariables.x + (runObj * ACADO_NX), speed_states, terr_params, terr_map);

        aux_output[runObj * N_AUX + AUX_P_OCC_FWD] = p_occ_fwd[0];
        aux_output[runObj * N_AUX + AUX_P_OCC_FWD+1] = p_occ_fwd[1];
        aux_output[runObj * N_AUX + AUX_P_OCC_FWD+2] = p_occ_fwd[2];
        aux_output[runObj * N_AUX + AUX_N_OCC_FWD] = n_occ_fwd[0];
        aux_output[runObj * N_AUX + AUX_N_OCC_FWD+1] = n_occ_fwd[1];
        aux_output[runObj * N_AUX + AUX_N_OCC_FWD+2] = n_occ_fwd[2];
        
        /* left ray */
        v_ray[0] = -speed_states[9];
        v_ray[1] = speed_states[10];
        v_ray[2] = 0.0;
        calculate_radial_objective(&sig_r_left, jac_sig_r_left, &r_occ_left, p_occ_fwd, n_occ_fwd, &sig_input_r_left, &occ_detected_left, v_ray, acadoVariables.x + (runObj * ACADO_NX), speed_states, terr_params, terr_map);
       
        /* right ray */
        v_ray[0] = speed_states[9];
        v_ray[1] = -speed_states[10];
        v_ray[2] = 0.0;
        calculate_radial_objective(&sig_r_right, jac_sig_r_right, &r_occ_right, p_occ_fwd, n_occ_fwd, &sig_input_r_right, &occ_detected_right, v_ray, acadoVariables.x + (runObj * ACADO_NX), speed_states, terr_params, terr_map);
        
        jac_sig_r[0] = jac_sig_r_fwd[0] + jac_sig_r_left[0] + jac_sig_r_right[0];
        jac_sig_r[1] = jac_sig_r_fwd[1] + jac_sig_r_left[1] + jac_sig_r_right[1];
        jac_sig_r[2] = jac_sig_r_fwd[2] + jac_sig_r_left[2] + jac_sig_r_right[2];
        jac_sig_r[3] = jac_sig_r_fwd[3] + jac_sig_r_left[3] + jac_sig_r_right[3];
        jac_sig_r[4] = jac_sig_r_fwd[4] + jac_sig_r_left[4] + jac_sig_r_right[4];
        jac_sig_r[5] = jac_sig_r_fwd[5] + jac_sig_r_left[5] + jac_sig_r_right[5];
        
        const double sig_r = sig_r_fwd + sig_r_left + sig_r_right;
        
        acadoVariables.od[runObj * ACADO_NOD + 16] = sig_r;
        acadoVariables.od[runObj * ACADO_NOD + 17] = jac_sig_r[0];
        acadoVariables.od[runObj * ACADO_NOD + 18] = jac_sig_r[1];
        acadoVariables.od[runObj * ACADO_NOD + 19] = jac_sig_r[2];
        acadoVariables.od[runObj * ACADO_NOD + 20] = jac_sig_r[3];
        acadoVariables.od[runObj * ACADO_NOD + 21] = jac_sig_r[4];
        acadoVariables.od[runObj * ACADO_NOD + 22] = jac_sig_r[5];
                
        /* prioritization */
        const double one_over_sqrt_w_r = terr_params[9];
        if (!(one_over_sqrt_w_r<0.0) && (occ_detected_fwd + occ_detected_left + occ_detected_right>0)) {
            double sig_input = (sig_input_r_fwd < sig_input_r_left) ? sig_input_r_fwd : sig_input_r_left;
            sig_input = (sig_input_r_left < sig_input_r_right) ? sig_input_r_left : sig_input_r_right;
            prio_r = constrain_double(sig_input, 0.0, 1.0);
        }
        else {
            prio_r = 1.0;
        }
        priorities[runObj * 3 + 2] = prio_r;
        
        aux_output[runObj * N_AUX + AUX_R_OCC_FWD] = r_occ_fwd;
        aux_output[runObj * N_AUX + AUX_R_OCC_LEFT] = r_occ_left;
        aux_output[runObj * N_AUX + AUX_R_OCC_RIGHT] = r_occ_right;
        aux_output[runObj * N_AUX + AUX_OCC_DETECT_FWD] = occ_detected_fwd;
        aux_output[runObj * N_AUX + AUX_OCC_DETECT_LEFT] = occ_detected_left;
        aux_output[runObj * N_AUX + AUX_OCC_DETECT_RIGHT] = occ_detected_right;
                
        /* velocity reference */
        double v_ref[3];
        double e_lat, e_lon;
        calculate_velocity_reference(v_ref, &e_lat, &e_lon, acadoVariables.x + (runObj * ACADO_NX), path_reference, guidance_params,
            speed_states, jac_sig_r, prio_r);
                
        if (runObj < ACADO_N) {
            acadoVariables.y[runObj * ACADO_NY + 0] = v_ref[0];
            acadoVariables.y[runObj * ACADO_NY + 1] = v_ref[1];
            acadoVariables.y[runObj * ACADO_NY + 2] = v_ref[2];
        }
        else {
            acadoVariables.yN[0] = v_ref[0];
            acadoVariables.yN[1] = v_ref[1];
            acadoVariables.yN[2] = v_ref[2];
        }
        
        aux_output[runObj * N_AUX + AUX_E_LAT] = e_lat;
        aux_output[runObj * N_AUX + AUX_E_LON] = e_lon;
    }
   
    /* Prepare return argument */
	
	plhs[ 0 ] = mxCreateStructMatrix(1, 1, NOO, outNames);
	
    setArray(plhs[ 0 ], 0, "y", acadoVariables.y, ACADO_N, ACADO_NY);
    setArray(plhs[ 0 ], 0, "yN", acadoVariables.yN, 1, ACADO_NYN);
    setArray(plhs[ 0 ], 0, "od", acadoVariables.od, ACADO_N + 1, ACADO_NOD);
    setArray(plhs[ 0 ], 0, "priorities", priorities, ACADO_N + 1, 3);
    setArray(plhs[ 0 ], 0, "aux", aux_output, ACADO_N + 1, N_AUX);
}