#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "mex.h"
#include "acado_common.h"
#include "lsq_objective.c"

/* Define number of outputs */
#define NOO 6
#define N_AUX 12
#define N_OCC 7

enum aux {
    AUX_E_LAT = 0,
    AUX_E_LON,
    AUX_H_TERR,
    AUX_R_OCC_FWD,
    AUX_R_OCC_LEFT,
    AUX_R_OCC_RIGHT,
    AUX_OCC_DETECT_FWD, /* XXX: this is a repeat atm */
    AUX_OCC_DETECT_LEFT,
    AUX_OCC_DETECT_RIGHT,
    AUX_PRIO_R_FWD
};

enum occ {
    OCC_DETECT_FWD = 0,
    P_OCC_FWD = 1,
    N_OCC_FWD = 4
};

#define IDX_LEN_SW 10
#define IDX_OD_OBJ 9

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
    outNames[ 5 ] = "occ_slw";
    
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
    
    /* soft angle of attack parameters */
    double aoa_params[5];
    getArray(1, src, 0, "aoa_params", aoa_params, 1, 5);
    
    /* soft terrain parameters */
    double terr_params[14];
    getArray(1, src, 0, "terr_params", terr_params, 1, 11);
    const double log_sqrt_w_over_sig1_r = terr_params[8]; /* XXX: be careful if this list changes order... */
    const double one_over_sqrt_w_r = terr_params[9];
    
    /* terrain map parameters */
    double map_dimension[2];
    double map_params[3];
    getArray(1, src, 0, "map_dimension", map_dimension, 1, 2);
    getArray(1, src, 0, "map_params", map_params, 1, 3);
    const int map_height = round(map_dimension[0]);
    const int map_width = round(map_dimension[1]);
    const double terr_local_origin_n = map_params[0];
    const double terr_local_origin_e = map_params[1]; 
    const double map_resolution = map_params[2];
    
    /* terrain map */
    double terr_map[map_height*map_width];
    getArray(1, src, 0, "terr_map", terr_map, 1, map_height*map_width);
    
    /* path/guidance parameters */
    double path_reference[5];
    double guidance_params[4];
    getArray(1, src, 0, "path_reference", path_reference, 1, 5);
    getArray(1, src, 0, "guidance_params", guidance_params, 1, 4);
    
    /* sliding window array */
    int len_sliding_window = constrain_int(round(terr_params[IDX_LEN_SW]), 1, ACADO_N);
    double occ_slw[(len_sliding_window-1+ACADO_N+1) * N_OCC];
    getArray(1, src, 0, "occ_slw", occ_slw, len_sliding_window-1+ACADO_N+1, N_OCC);
    
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
    double prio_r_fwd;
    double prio_r_left;
    double prio_r_right;
    double p_occ_fwd[3];
    double n_occ_fwd[3];
    
    /* shift detection window */
    int jHorizon;
    for (jHorizon = 0; jHorizon < len_sliding_window; ++jHorizon) {
        occ_slw[jHorizon * N_OCC + OCC_DETECT_FWD] = occ_slw[(jHorizon+1) * N_OCC + OCC_DETECT_FWD];
        occ_slw[jHorizon * N_OCC + P_OCC_FWD] = occ_slw[(jHorizon+1) * N_OCC + P_OCC_FWD];
        occ_slw[jHorizon * N_OCC + P_OCC_FWD+1] = occ_slw[(jHorizon+1) * N_OCC + P_OCC_FWD+1];
        occ_slw[jHorizon * N_OCC + P_OCC_FWD+2] = occ_slw[(jHorizon+1) * N_OCC + P_OCC_FWD+2];
        occ_slw[jHorizon * N_OCC + N_OCC_FWD] = occ_slw[(jHorizon+1) * N_OCC + N_OCC_FWD];
        occ_slw[jHorizon * N_OCC + N_OCC_FWD+1] = occ_slw[(jHorizon+1) * N_OCC + N_OCC_FWD+1];
        occ_slw[jHorizon * N_OCC + N_OCC_FWD+2] = occ_slw[(jHorizon+1) * N_OCC + N_OCC_FWD+2];
    }
    
    /* cast rays along ground speed vector at every node */
    int kHorizon;
    for (kHorizon = 0; kHorizon < ACADO_N+1; ++kHorizon)
    {
        /* calculate speed states */
        const double v = acadoVariables.x[kHorizon * ACADO_NX + 3];
        const double gamma = acadoVariables.x[kHorizon * ACADO_NX + 4];
        const double xi = acadoVariables.x[kHorizon * ACADO_NX + 5];
        const double w_n = acadoVariables.od[kHorizon * ACADO_NOD + 1];
        const double w_e = acadoVariables.od[kHorizon * ACADO_NOD + 2];
        const double w_d = acadoVariables.od[kHorizon * ACADO_NOD + 3];
        double speed_states[12];
        calculate_speed_states(speed_states, v, gamma, xi, w_n, w_e, w_d);
        
        /* forward ray */
        get_occ_along_gsp_vec(p_occ_fwd, n_occ_fwd, &r_occ_fwd, &occ_detected_fwd,
                acadoVariables.x + (kHorizon * ACADO_NX), speed_states, terr_params,
                terr_local_origin_n, terr_local_origin_e, map_height, map_width, map_resolution, terr_map);
        
        /* log detections */
        aux_output[kHorizon * N_AUX + AUX_R_OCC_FWD] = r_occ_fwd;
        aux_output[kHorizon * N_AUX + AUX_OCC_DETECT_FWD] = occ_detected_fwd;
        occ_slw[(len_sliding_window-1+kHorizon) * N_OCC + OCC_DETECT_FWD] = occ_detected_fwd;
        occ_slw[(len_sliding_window-1+kHorizon) * N_OCC + P_OCC_FWD] = p_occ_fwd[0];
        occ_slw[(len_sliding_window-1+kHorizon) * N_OCC + P_OCC_FWD+1] = p_occ_fwd[1];
        occ_slw[(len_sliding_window-1+kHorizon) * N_OCC + P_OCC_FWD+2] = p_occ_fwd[2];
        occ_slw[(len_sliding_window-1+kHorizon) * N_OCC + N_OCC_FWD] = n_occ_fwd[0];
        occ_slw[(len_sliding_window-1+kHorizon) * N_OCC + N_OCC_FWD+1] = n_occ_fwd[1];
        occ_slw[(len_sliding_window-1+kHorizon) * N_OCC + N_OCC_FWD+2] = n_occ_fwd[2];
    }

    /* sum detections on sliding horizon window for each node */
    for (kHorizon = 0; kHorizon < ACADO_N+1; ++kHorizon)
    {
        /* init */
        double jac_sig_r_fwd[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
        double jac_r_unit[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
        double r_unit_min;
        bool f_min;
        int occ_count = 0;
        
        /* calculate speed states */
        const double v = acadoVariables.x[kHorizon * ACADO_NX + 3];
        const double gamma = acadoVariables.x[kHorizon * ACADO_NX + 4];
        const double xi = acadoVariables.x[kHorizon * ACADO_NX + 5];
        const double w_n = acadoVariables.od[kHorizon * ACADO_NOD + 1];
        const double w_e = acadoVariables.od[kHorizon * ACADO_NOD + 2];
        const double w_d = acadoVariables.od[kHorizon * ACADO_NOD + 3];
        double speed_states[12];
        calculate_speed_states(speed_states, v, gamma, xi, w_n, w_e, w_d);
        
        /* sum detections in sliding window */
        for (jHorizon = kHorizon; jHorizon < (kHorizon + len_sliding_window); ++jHorizon)
        {
            if (occ_slw[jHorizon * N_OCC + OCC_DETECT_FWD]>0) {
                add_unit_radial_distance_and_gradient(jac_r_unit, &r_unit_min, &f_min, &occ_count,
                        occ_slw + (jHorizon * N_OCC + P_OCC_FWD), occ_slw + (jHorizon * N_OCC + N_OCC_FWD),
                        acadoVariables.x + (kHorizon * ACADO_NX), speed_states, terr_params);
            }   
        }
        
        /* calculate objective/jacobian */
        prio_r_fwd = 1.0;
        if (occ_count>0) {
            if (f_min) {
                sig_r_fwd = 1.0 - log_sqrt_w_over_sig1_r * r_unit_min;
                jac_sig_r_fwd[0] = -log_sqrt_w_over_sig1_r * jac_r_unit[0] / occ_count;
                jac_sig_r_fwd[1] = -log_sqrt_w_over_sig1_r * jac_r_unit[1] / occ_count;
                jac_sig_r_fwd[2] = -log_sqrt_w_over_sig1_r * jac_r_unit[2] / occ_count;
                jac_sig_r_fwd[3] = -log_sqrt_w_over_sig1_r * jac_r_unit[3] / occ_count;
                jac_sig_r_fwd[4] = -log_sqrt_w_over_sig1_r * jac_r_unit[4] / occ_count;
                jac_sig_r_fwd[5] = -log_sqrt_w_over_sig1_r * jac_r_unit[5] / occ_count;
            }
            else {
                sig_r_fwd = exp(-r_unit_min * log_sqrt_w_over_sig1_r);
                jac_sig_r_fwd[0] = -log_sqrt_w_over_sig1_r * sig_r_fwd * jac_r_unit[0] / occ_count;
                jac_sig_r_fwd[1] = -log_sqrt_w_over_sig1_r * sig_r_fwd * jac_r_unit[1] / occ_count;
                jac_sig_r_fwd[2] = -log_sqrt_w_over_sig1_r * sig_r_fwd * jac_r_unit[2] / occ_count;
                jac_sig_r_fwd[3] = -log_sqrt_w_over_sig1_r * sig_r_fwd * jac_r_unit[3] / occ_count;
                jac_sig_r_fwd[4] = -log_sqrt_w_over_sig1_r * sig_r_fwd * jac_r_unit[4] / occ_count;
                jac_sig_r_fwd[5] = -log_sqrt_w_over_sig1_r * sig_r_fwd * jac_r_unit[5] / occ_count;
            }
            jac_sig_r_fwd[3] = 0.0; /* discourage mpc from using airspeed to combat costs */
            prio_r_fwd = constrain_double(r_unit_min, 0.0, 1.0);
        }
        aux_output[kHorizon * N_AUX + AUX_PRIO_R_FWD] = prio_r_fwd;
        
        /* log cost and jacobian */
        acadoVariables.od[kHorizon * ACADO_NOD + IDX_OD_OBJ+8] = sig_r_fwd;
        acadoVariables.od[kHorizon * ACADO_NOD + IDX_OD_OBJ+9] = jac_sig_r_fwd[0];
        acadoVariables.od[kHorizon * ACADO_NOD + IDX_OD_OBJ+10] = jac_sig_r_fwd[1];
        acadoVariables.od[kHorizon * ACADO_NOD + IDX_OD_OBJ+11] = jac_sig_r_fwd[2];
        acadoVariables.od[kHorizon * ACADO_NOD + IDX_OD_OBJ+12] = jac_sig_r_fwd[3];
        acadoVariables.od[kHorizon * ACADO_NOD + IDX_OD_OBJ+13] = jac_sig_r_fwd[4];
        acadoVariables.od[kHorizon * ACADO_NOD + IDX_OD_OBJ+14] = jac_sig_r_fwd[5];
    }
    
    /* evaluate external objectives */
    double v_ray[3];
    for (kHorizon = 0; kHorizon < ACADO_N +1; ++kHorizon)
    {
        /* soft aoa constraint */
        calculate_aoa_objective(&sig_aoa, jac_sig_aoa, &prio_aoa, acadoVariables.x + (kHorizon * ACADO_NX), aoa_params);
                
        acadoVariables.od[kHorizon * ACADO_NOD + IDX_OD_OBJ] = sig_aoa;
        acadoVariables.od[kHorizon * ACADO_NOD + IDX_OD_OBJ+1] = jac_sig_aoa[0];
        acadoVariables.od[kHorizon * ACADO_NOD + IDX_OD_OBJ+2] = jac_sig_aoa[1];
                
        priorities[kHorizon * 3 + 0] = prio_aoa;
                
        /* soft height constraint */
        double h_terr;
        calculate_height_objective(&sig_h, jac_sig_h, &prio_h, &h_terr, acadoVariables.x + (kHorizon * ACADO_NX), terr_params,
                terr_local_origin_n, terr_local_origin_e, map_height, map_width, map_resolution, terr_map);
                
        acadoVariables.od[kHorizon * ACADO_NOD + IDX_OD_OBJ+3] = sig_h;
        acadoVariables.od[kHorizon * ACADO_NOD + IDX_OD_OBJ+4] = jac_sig_h[0];
        acadoVariables.od[kHorizon * ACADO_NOD + IDX_OD_OBJ+5] = jac_sig_h[1];
        acadoVariables.od[kHorizon * ACADO_NOD + IDX_OD_OBJ+6] = jac_sig_h[2];
        acadoVariables.od[kHorizon * ACADO_NOD + IDX_OD_OBJ+7] = jac_sig_h[3];
                
        priorities[kHorizon * 3 + 1] = prio_h;
        
        aux_output[kHorizon * N_AUX + AUX_H_TERR] = h_terr;
                
        /* calculate speed states */
        const double v = acadoVariables.x[kHorizon * ACADO_NX + 3];
        const double gamma = acadoVariables.x[kHorizon * ACADO_NX + 4];
        const double xi = acadoVariables.x[kHorizon * ACADO_NX + 5];
        const double w_n = acadoVariables.od[kHorizon * ACADO_NOD + 1];
        const double w_e = acadoVariables.od[kHorizon * ACADO_NOD + 2];
        const double w_d = acadoVariables.od[kHorizon * ACADO_NOD + 3];
        double speed_states[12];
        calculate_speed_states(speed_states, v, gamma, xi, w_n, w_e, w_d);
                
        /* soft radial constraint */

        /* left ray */
        v_ray[0] = -speed_states[9];
        v_ray[1] = speed_states[10];
        v_ray[2] = 0.0;
        calculate_radial_objective(&sig_r_left, jac_sig_r_left, &r_occ_left, p_occ_fwd, n_occ_fwd, &prio_r_left, &occ_detected_left,
                v_ray, acadoVariables.x + (kHorizon * ACADO_NX), speed_states, terr_params, terr_local_origin_n, terr_local_origin_e, 
                map_height, map_width, map_resolution, terr_map);
        
        /* right ray */
        v_ray[0] = speed_states[9];
        v_ray[1] = -speed_states[10];
        v_ray[2] = 0.0;
        calculate_radial_objective(&sig_r_right, jac_sig_r_right, &r_occ_right, p_occ_fwd, n_occ_fwd, &prio_r_right, &occ_detected_right,
                v_ray, acadoVariables.x + (kHorizon * ACADO_NX), speed_states, terr_params, terr_local_origin_n, terr_local_origin_e, 
                map_height, map_width, map_resolution, terr_map);
        
        jac_sig_r[0] = jac_sig_r_left[0] + jac_sig_r_right[0];
        jac_sig_r[1] = jac_sig_r_left[1] + jac_sig_r_right[1];
        jac_sig_r[2] = jac_sig_r_left[2] + jac_sig_r_right[2];
        jac_sig_r[3] = jac_sig_r_left[3] + jac_sig_r_right[3];
        jac_sig_r[4] = jac_sig_r_left[4] + jac_sig_r_right[4];
        jac_sig_r[5] = jac_sig_r_left[5] + jac_sig_r_right[5];
        
        const double sig_r = sig_r_left + sig_r_right;
        
        /* add to objective cost / jacobian */
        acadoVariables.od[kHorizon * ACADO_NOD + IDX_OD_OBJ+8] += sig_r;
        acadoVariables.od[kHorizon * ACADO_NOD + IDX_OD_OBJ+9] += jac_sig_r[0];
        acadoVariables.od[kHorizon * ACADO_NOD + IDX_OD_OBJ+10] += jac_sig_r[1];
        acadoVariables.od[kHorizon * ACADO_NOD + IDX_OD_OBJ+11] += jac_sig_r[2];
        acadoVariables.od[kHorizon * ACADO_NOD + IDX_OD_OBJ+12] += jac_sig_r[3];
        acadoVariables.od[kHorizon * ACADO_NOD + IDX_OD_OBJ+13] += jac_sig_r[4];
        acadoVariables.od[kHorizon * ACADO_NOD + IDX_OD_OBJ+14] += jac_sig_r[5];
        
        aux_output[kHorizon * N_AUX + AUX_PRIO_R_FWD+1] = prio_r_left;
        aux_output[kHorizon * N_AUX + AUX_PRIO_R_FWD+2] = prio_r_right;
                
        /* prioritization */
        occ_detected_fwd = occ_slw[(len_sliding_window-1+kHorizon) * N_OCC + OCC_DETECT_FWD];
        if (!(one_over_sqrt_w_r<0.0) && (occ_detected_fwd + occ_detected_left + occ_detected_right>0)) {
            
            /* take minimum unit radial distance */
            prio_r_fwd = aux_output[kHorizon * N_AUX + AUX_PRIO_R_FWD];
            prio_r = (prio_r_fwd < prio_r_left) ? prio_r_fwd : prio_r_left;
            prio_r = (prio_r < prio_r_right) ? prio_r : prio_r_right;
        }
        else {
            prio_r = 1.0;
        }
        priorities[kHorizon * 3 + 2] = prio_r;
        
        aux_output[kHorizon * N_AUX + AUX_R_OCC_LEFT] = r_occ_left;
        aux_output[kHorizon * N_AUX + AUX_R_OCC_RIGHT] = r_occ_right;
        aux_output[kHorizon * N_AUX + AUX_OCC_DETECT_LEFT] = occ_detected_left;
        aux_output[kHorizon * N_AUX + AUX_OCC_DETECT_RIGHT] = occ_detected_right;
                
        /* velocity reference */
        double v_ref[3];
        double e_lat, e_lon;
        calculate_velocity_reference(v_ref, &e_lat, &e_lon, acadoVariables.x + (kHorizon * ACADO_NX), path_reference, guidance_params,
            speed_states, jac_sig_r, prio_r);
                
        if (kHorizon < ACADO_N) {
            acadoVariables.y[kHorizon * ACADO_NY + 0] = v_ref[0];
            acadoVariables.y[kHorizon * ACADO_NY + 1] = v_ref[1];
            acadoVariables.y[kHorizon * ACADO_NY + 2] = v_ref[2];
        }
        else {
            acadoVariables.yN[0] = v_ref[0];
            acadoVariables.yN[1] = v_ref[1];
            acadoVariables.yN[2] = v_ref[2];
        }
        
        aux_output[kHorizon * N_AUX + AUX_E_LAT] = e_lat;
        aux_output[kHorizon * N_AUX + AUX_E_LON] = e_lon;
    }
   
    /* Prepare return argument */
	
	plhs[ 0 ] = mxCreateStructMatrix(1, 1, NOO, outNames);
	
    setArray(plhs[ 0 ], 0, "y", acadoVariables.y, ACADO_N, ACADO_NY);
    setArray(plhs[ 0 ], 0, "yN", acadoVariables.yN, 1, ACADO_NYN);
    setArray(plhs[ 0 ], 0, "od", acadoVariables.od, ACADO_N + 1, ACADO_NOD);
    setArray(plhs[ 0 ], 0, "priorities", priorities, ACADO_N + 1, 3);
    setArray(plhs[ 0 ], 0, "aux", aux_output, ACADO_N + 1, N_AUX);
    setArray(plhs[ 0 ], 0, "occ_slw", occ_slw, len_sliding_window-1+ACADO_N+1, N_OCC);
}