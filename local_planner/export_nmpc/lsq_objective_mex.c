#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "mex.h"
#include "acado_common.h"
#include "lsq_objective.c"

/* Define number of outputs */
#define NOO 5

typedef struct ACADOoutputs_
{
    real_t y[ ACADO_NY*ACADO_N ];
        
    real_t dydx[ ACADO_NY*ACADO_NX*ACADO_N ];
    
    real_t dydu[ ACADO_NY*ACADO_NU*ACADO_N ];
    
    real_t yN[ ACADO_NYN ];
    
    real_t dyNdx[ ACADO_NYN*ACADO_NX ];

} ACADOoutputs;

/** Instance of the user data structure. */
ACADOvariables acadoVariables;
/** Instance of the private workspace structure. */
ACADOworkspace acadoWorkspace;
/** Instance of the output data structure. */
ACADOoutputs acadoOutputs;


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
	outNames[ 1 ] = "dydx";
    outNames[ 2 ] = "dydu";
    outNames[ 3 ] = "yN";
    outNames[ 4 ] = "dyNdx";
    
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
	getArray(1, src, 0, "u", acadoVariables.u, ACADO_N, ACADO_NU);

#if ACADO_NOD
	getArray(1, src, 0, "od", acadoVariables.od, ACADO_N + 1, ACADO_NOD);
#endif
    
    /* evaluate objectives */
    
    int lx;
    int lu;
    int lod;
    int ly;
    int lyx;
    int lyu;
    int runObj;
    for (runObj = 0; runObj < ACADO_N; ++runObj)
    {
        /* set objective input */
        for (lx = 0; lx < ACADO_NX; ++lx) {
            acadoWorkspace.objValueIn[lx] = acadoVariables.x[runObj * ACADO_NX + lx];
        }
        for (lu = 0; lu < ACADO_NU; ++lu) {
            acadoWorkspace.objValueIn[ACADO_NX+lu] = acadoVariables.u[runObj * ACADO_NU + lu];
        }
        for (lod = 0; lod < ACADO_NOD; ++lod) {
            acadoWorkspace.objValueIn[lod + ACADO_NX+ACADO_NU] = acadoVariables.od[(runObj * ACADO_NOD) + (lod)];
        }
        
        /* evaluate lsq */
        acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
        
        /* populate output structs */
        for (ly = 0; ly < ACADO_NY; ++ly) {
            acadoOutputs.y[runObj * ACADO_NY + ly] = acadoWorkspace.objValueOut[ly];
        }
        for (lyx = 0; lyx < ACADO_NY*ACADO_NX; ++lyx) {
            acadoOutputs.dydx[runObj * ACADO_NY*ACADO_NX + lyx] = acadoWorkspace.objValueOut[ACADO_NY + lyx];
        }
        for (lyu = 0; lyu < ACADO_NY*ACADO_NU; ++lyu) {
            acadoOutputs.dydu[runObj * ACADO_NY*ACADO_NU + lyu] = acadoWorkspace.objValueOut[ACADO_NY + ACADO_NY*ACADO_NX + lyu];
        }
    }
    
    /* set objective input */
    for (lx = 0; lx < ACADO_NX; ++lx) {
        acadoWorkspace.objValueIn[lx] = acadoVariables.x[ACADO_N * ACADO_NX + lx];
    }
    for (lod = 0; lod < ACADO_NOD; ++lod) {
        acadoWorkspace.objValueIn[lod + ACADO_NX] = acadoVariables.od[(ACADO_N * ACADO_NOD) + (lod)];
    }
    
    /* evaluate lsq end term */
    acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
    
    /* populate output structs */
    for (ly = 0; ly < ACADO_NYN; ++ly) {
        acadoOutputs.yN[ly] = acadoWorkspace.objValueOut[ly];
    }
    for (lyx = 0; lyx < ACADO_NYN*ACADO_NX; ++lyx) {
        acadoOutputs.dyNdx[lyx] = acadoWorkspace.objValueOut[ACADO_NYN + lyx];
    }
    
    /* Prepare return argument */
	
	plhs[ 0 ] = mxCreateStructMatrix(1, 1, NOO, outNames);
	
    setArray(plhs[ 0 ], 0, "y", acadoOutputs.y, ACADO_N, ACADO_NY);
    setArray(plhs[ 0 ], 0, "dydx", acadoOutputs.dydx, ACADO_N, ACADO_NY*ACADO_NX);
    setArray(plhs[ 0 ], 0, "dydu", acadoOutputs.dydu, ACADO_N, ACADO_NY*ACADO_NU);
    setArray(plhs[ 0 ], 0, "yN", acadoOutputs.yN, 1, ACADO_NYN);
    setArray(plhs[ 0 ], 0, "dyNdx", acadoOutputs.dyNdx, 1, ACADO_NYN*ACADO_NX);
	
}