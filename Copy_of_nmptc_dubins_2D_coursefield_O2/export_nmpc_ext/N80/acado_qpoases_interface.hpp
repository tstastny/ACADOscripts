/*
 *    This file was auto-generated by ACADO Code Generation Tool.
 *    
 *    ACADO Code Generation tool is a sub-package of ACADO toolkit --
 *    A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
 *    
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *    
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *    
 */


#ifndef QPOASES_HEADER
#define QPOASES_HEADER

#ifdef PC_DEBUG
#include <stdio.h>
#endif /* PC_DEBUG */

#include <math.h>

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

/*
 * A set of options for qpOASES
 */

/** Maximum number of optimization variables. */
#define QPOASES_NVMAX      80
/** Maximum number of constraints. */
#define QPOASES_NCMAX      0
/** Maximum number of working set recalculations. */
#define QPOASES_NWSRMAX    240
/** Print level for qpOASES. */
#define QPOASES_PRINTLEVEL PL_NONE
/** The value of EPS */
#define QPOASES_EPS        2.221e-16
/** Internally used floating point type */
typedef double real_t;

/*
 * Forward function declarations
 */

/** A function that calls the QP solver */
EXTERNC int solve( void );

/** Get the number of active set changes */
EXTERNC int getNWSR( void );

/** Get the error string. */
const char* getErrorString(int error);

#endif /* QPOASES_HEADER */
