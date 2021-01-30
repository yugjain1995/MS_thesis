#define S_FUNCTION_NAME ACC_MPC_CVXGEN_sol
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#define EXECTRA

#ifdef EXECTRA
#include "solver.h"
#endif

#ifndef MATLAB_MEX_FILE
//#include <Tic1401.h>
#else
#include <time.h>
#endif


#ifdef EXECTRA
Vars vars;
Params params;
Workspace work;
Settings settings;
#endif

static void mdlInitializeSizes(SimStruct *S)
{
    /* Initialize S-function parameters */
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch reported by the Simulink engine*/
    }
    
    
    /* Initialize input port */
    if (!ssSetNumInputPorts(S, 9)) return;
    
    int idx;
    for(idx = 0; idx <= 9; idx++)
    {
        ssSetInputPortDirectFeedThrough(S, idx, 1);
    }
    if(ssSetInputPortMatrixDimensions(S, 0, 2, 2) == 0){return;}
    if(ssSetInputPortVectorDimension(S, 1, 2) == 0){return;}
    if(ssSetInputPortVectorDimension(S, 2, 2) == 0){return;}
    if(ssSetInputPortVectorDimension(S, 3, 2) == 0){return;}
    if(ssSetInputPortVectorDimension(S, 4, 1) == 0){return;}
    if(ssSetInputPortVectorDimension(S, 5, 20) == 0){return;}
    if(ssSetInputPortVectorDimension(S, 6, 4) == 0){return;}
    if(ssSetInputPortMatrixDimensions(S, 7, 4, 3) == 0){return;}
    if(ssSetInputPortVectorDimension(S, 8, 1) == 0){return;}
    
    
    /* Initialize output port */
    if (!ssSetNumOutputPorts(S,3)) return;
    for(idx = 0; idx <= 2; idx++)
    {
        ssSetOutputPortDataType(S, idx, DYNAMICALLY_TYPED);
    }
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortWidth(S, 1, 1);
    ssSetOutputPortWidth(S, 2, 1);
    /*
     * ssSetOutputPortWidth(S, 2, 20);
     * ssSetOutputPortWidth(S, 3, 20);
     * ssSetOutputPortWidth(S, 4, 20);
     */
    
    /* Initialize sample times */
    ssSetNumSampleTimes(S, 1);
    
    /* Take care when specifying exception free code - see sfuntmpl.doc */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}


static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, 0.1);
    ssSetOffsetTime(S, 0, 0.0);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
#ifndef MATLAB_MEX_FILE
    // ds1401_tic_start();
#else
    clock_t begin = clock();
#endif
    
    /*CVXGEN MPC parameters - Some are inputs while some are parameters
     * for S-function*/
    
    /* S-functions inputs */
    
#ifdef EXECTRA
    
    set_defaults();
    setup_indexing();
    settings.verbose = 1;
    settings.max_iters = 30;
    /* Input port 1 */
    {
        real_T **inputPtr = ssGetInputPortRealSignalPtrs(S, 0);
        params.A[0] = *(*inputPtr);
        params.A[1] = *(*inputPtr + 2 + 0);
        params.A[2] = *(*inputPtr + 2 + 1);
    }
    
    /* Input port 2 */
    {
        real_T **inputPtr = ssGetInputPortRealSignalPtrs(S, 1);
        params.Bd[0] = *(*inputPtr);
        params.Bd[1] = *(*inputPtr + 1);
    }
    
    /* Input port 3 */
    {
        real_T **inputPtr = ssGetInputPortRealSignalPtrs(S, 2);
        params.Bd_bar[0] = *(*inputPtr);
    }
    
    /* Input port 4 */
    {
        real_T **inputPtr = ssGetInputPortRealSignalPtrs(S, 3);
        params.x_0[0] = *(*inputPtr);
        params.x_0[1] = *(*inputPtr+1);
    }
    
    /* Input port  5 */
    {
        real_T **inputPtr = ssGetInputPortRealSignalPtrs(S, 4);
        params.u0[0] = *(*inputPtr);
    }
    
    /* Input port 6 */
    {
        real_T **inputPtr = ssGetInputPortRealSignalPtrs(S, 5);
        params.w_0[0] = *(*inputPtr);
        params.w_1[0] = *(*inputPtr + 1);
        params.w_2[0] = *(*inputPtr + 2);
        params.w_3[0] = *(*inputPtr + 3);
        params.w_4[0] = *(*inputPtr + 4);
        params.w_5[0] = *(*inputPtr + 5);
        params.w_6[0] = *(*inputPtr + 6);
        params.w_7[0] = *(*inputPtr + 7);
        params.w_8[0] = *(*inputPtr + 8);
        params.w_9[0] = *(*inputPtr + 9);
        params.w_10[0] = *(*inputPtr + 10);
        params.w_11[0] = *(*inputPtr + 11);
        params.w_12[0] = *(*inputPtr + 12);
        params.w_13[0] = *(*inputPtr + 13);
        params.w_14[0] = *(*inputPtr + 14);
        params.w_15[0] = *(*inputPtr + 15);
        params.w_16[0] = *(*inputPtr + 16);
        params.w_17[0] = *(*inputPtr + 17);
        params.w_18[0] = *(*inputPtr + 18);
        params.w_19[0] = *(*inputPtr + 19);
    }
    
    /* Input port 7 */
    {
        real_T **inputPtr = ssGetInputPortRealSignalPtrs(S, 6);
        params.m1[0] = *(*inputPtr);
        params.c1[0] = *(*inputPtr + 1);
        params.m2[0] = *(*inputPtr + 2);
        params.c2[0] = *(*inputPtr + 3);
    }
    
    /* Input port  8 */
    {
        real_T **inputPtr = ssGetInputPortRealSignalPtrs(S, 7);
        int idx;
        for(idx = 0; idx <= 3; idx++)
        {
            params.Cv[idx] = *(*inputPtr + idx);
            params.Ca[idx] = *(*inputPtr + 4 + idx);
            params.b[idx] = *(*inputPtr + (2*4) + idx);
        }
    }
    
    /* Input port  9 */
    {
        real_T **inputPtr = ssGetInputPortRealSignalPtrs(S, 8);
        params.v_set[0] = *(*inputPtr);
    }
    
    
    /* S-functions parameters */
    params.a_max[0] = 2.0;
    params.a_min[0] = -3.5;
    params.del_a_max[0] = 0.25;
    params.del_a_min[0] = -0.25;
    params.del_d_max[0] = 10;
    params.del_d_min[0] = 0;
    params.A1[0] = 1;
    params.A2[0] = 1;
    
    
    /* Solve MPC problem */
    solve();
#endif
    
#ifndef MATLAB_MEX_FILE
    //double solverTime = ds1401_tic_read();
#else
    clock_t end = clock();
    double solverTime = (double)(end - begin) / CLOCKS_PER_SEC;
#endif
    
    
    /* Update output ports*/
    real_T *y1 = ssGetOutputPortRealSignal(S, 0);
    real_T *y2 = ssGetOutputPortRealSignal(S, 1);
    real_T *y6 = ssGetOutputPortRealSignal(S, 2);
    /*
     * real_T *y3 = ssGetOutputPortRealSignal(S, 2);
     * real_T *y4 = ssGetOutputPortRealSignal(S, 3);
     * real_T *y5 = ssGetOutputPortRealSignal(S, 4);
     */
    #ifdef EXECTRA
    *y2 = work.converged; /* Solution feasibility status */
    
    if (*y2 != 0)
    {
        *y1 = *vars.u_0; /* Control output */
    }
    *y6 = solverTime; /* Solver time */
    #endif
    /* del_d_min violation
     *(y3) = *vars.gama2_1;
     *(y3+1) = *vars.gama2_2;
     *(y3+2) = *vars.gama2_3;
     *(y3+3) = *vars.gama2_4;
     *(y3+4) = *vars.gama2_5;
     *(y3+5) = *vars.gama2_6;
     *(y3+6) = *vars.gama2_7;
     *(y3+7) = *vars.gama2_8;
     *(y3+8) = *vars.gama2_9;
     *(y3+9) = *vars.gama2_10;
     *(y3+10) = *vars.gama2_11;
     *(y3+11) = *vars.gama2_12;
     *(y3+12) = *vars.gama2_13;
     *(y3+13) = *vars.gama2_14;
     *(y3+14) = *vars.gama2_15;
     *(y3+15) = *vars.gama2_16;
     *(y3+16) = *vars.gama2_17;
     *(y3+17) = *vars.gama2_18;
     *(y3+18) = *vars.gama2_19;
     *(y3+19) = *vars.gama2_20;*/
    
    /* del_d_max violation
     *(y4) = *vars.gama1_1;
     *(y4+1) = *vars.gama1_2;
     *(y4+2) = *vars.gama1_3;
     *(y4+3) = *vars.gama1_4;
     *(y4+4) = *vars.gama1_5;
     *(y4+5) = *vars.gama1_6;
     *(y4+6) = *vars.gama1_7;
     *(y4+7) = *vars.gama1_8;
     *(y4+8) = *vars.gama1_9;
     *(y4+9) = *vars.gama1_10;
     *(y4+10) = *vars.gama1_11;
     *(y4+11) = *vars.gama1_12;
     *(y4+12) = *vars.gama1_13;
     *(y4+13) = *vars.gama1_14;
     *(y4+14) = *vars.gama1_15;
     *(y4+15) = *vars.gama1_16;
     *(y4+16) = *vars.gama1_17;
     *(y4+17) = *vars.gama1_18;
     *(y4+18) = *vars.gama1_19;
     *(y4+19) = *vars.gama1_20;*/
    
    /* Estimated fuel rate
     *(y5) = *vars.q_0;
     *(y5+1) = *vars.q_1;
     *(y5+2) = *vars.q_2;
     *(y5+3) = *vars.q_3;
     *(y5+4) = *vars.q_4;
     *(y5+5) = *vars.q_5;
     *(y5+6) = *vars.q_6;
     *(y5+7) = *vars.q_7;
     *(y5+8) = *vars.q_8;
     *(y5+9) = *vars.q_9;
     *(y5+10) = *vars.q_10;
     *(y5+11) = *vars.q_11;
     *(y5+12) = *vars.q_12;
     *(y5+13) = *vars.q_13;
     *(y5+14) = *vars.q_14;
     *(y5+15) = *vars.q_15;
     *(y5+16) = *vars.q_16;
     *(y5+17) = *vars.q_17;
     *(y5+18) = *vars.q_18;
     *(y5+19) = *vars.q_19;*/
}

static void mdlTerminate(SimStruct *S){}


#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif

