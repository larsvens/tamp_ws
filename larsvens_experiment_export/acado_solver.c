/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 6];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 6 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 6 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 6 + 3];
acadoWorkspace.state[4] = acadoVariables.x[lRun1 * 6 + 4];
acadoWorkspace.state[5] = acadoVariables.x[lRun1 * 6 + 5];
acadoWorkspace.state[6] = acadoVariables.z[lRun1];

acadoWorkspace.state[56] = acadoVariables.u[lRun1];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 6] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 6 + 6];
acadoWorkspace.d[lRun1 * 6 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 6 + 7];
acadoWorkspace.d[lRun1 * 6 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 6 + 8];
acadoWorkspace.d[lRun1 * 6 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 6 + 9];
acadoWorkspace.d[lRun1 * 6 + 4] = acadoWorkspace.state[4] - acadoVariables.x[lRun1 * 6 + 10];
acadoWorkspace.d[lRun1 * 6 + 5] = acadoWorkspace.state[5] - acadoVariables.x[lRun1 * 6 + 11];

acadoVariables.z[lRun1] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 36] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 36 + 1] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 36 + 2] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 36 + 3] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 36 + 4] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 36 + 5] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 36 + 6] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 36 + 7] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 36 + 8] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 36 + 9] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 36 + 10] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 36 + 11] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 36 + 12] = acadoWorkspace.state[19];
acadoWorkspace.evGx[lRun1 * 36 + 13] = acadoWorkspace.state[20];
acadoWorkspace.evGx[lRun1 * 36 + 14] = acadoWorkspace.state[21];
acadoWorkspace.evGx[lRun1 * 36 + 15] = acadoWorkspace.state[22];
acadoWorkspace.evGx[lRun1 * 36 + 16] = acadoWorkspace.state[23];
acadoWorkspace.evGx[lRun1 * 36 + 17] = acadoWorkspace.state[24];
acadoWorkspace.evGx[lRun1 * 36 + 18] = acadoWorkspace.state[25];
acadoWorkspace.evGx[lRun1 * 36 + 19] = acadoWorkspace.state[26];
acadoWorkspace.evGx[lRun1 * 36 + 20] = acadoWorkspace.state[27];
acadoWorkspace.evGx[lRun1 * 36 + 21] = acadoWorkspace.state[28];
acadoWorkspace.evGx[lRun1 * 36 + 22] = acadoWorkspace.state[29];
acadoWorkspace.evGx[lRun1 * 36 + 23] = acadoWorkspace.state[30];
acadoWorkspace.evGx[lRun1 * 36 + 24] = acadoWorkspace.state[31];
acadoWorkspace.evGx[lRun1 * 36 + 25] = acadoWorkspace.state[32];
acadoWorkspace.evGx[lRun1 * 36 + 26] = acadoWorkspace.state[33];
acadoWorkspace.evGx[lRun1 * 36 + 27] = acadoWorkspace.state[34];
acadoWorkspace.evGx[lRun1 * 36 + 28] = acadoWorkspace.state[35];
acadoWorkspace.evGx[lRun1 * 36 + 29] = acadoWorkspace.state[36];
acadoWorkspace.evGx[lRun1 * 36 + 30] = acadoWorkspace.state[37];
acadoWorkspace.evGx[lRun1 * 36 + 31] = acadoWorkspace.state[38];
acadoWorkspace.evGx[lRun1 * 36 + 32] = acadoWorkspace.state[39];
acadoWorkspace.evGx[lRun1 * 36 + 33] = acadoWorkspace.state[40];
acadoWorkspace.evGx[lRun1 * 36 + 34] = acadoWorkspace.state[41];
acadoWorkspace.evGx[lRun1 * 36 + 35] = acadoWorkspace.state[42];

acadoWorkspace.evGu[lRun1 * 6] = acadoWorkspace.state[49];
acadoWorkspace.evGu[lRun1 * 6 + 1] = acadoWorkspace.state[50];
acadoWorkspace.evGu[lRun1 * 6 + 2] = acadoWorkspace.state[51];
acadoWorkspace.evGu[lRun1 * 6 + 3] = acadoWorkspace.state[52];
acadoWorkspace.evGu[lRun1 * 6 + 4] = acadoWorkspace.state[53];
acadoWorkspace.evGu[lRun1 * 6 + 5] = acadoWorkspace.state[54];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 6;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = u[0];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 10; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 6];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 6 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 6 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 6 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 6 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 6 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.u[runObj];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 7] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 7 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 7 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 7 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 7 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 7 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 7 + 6] = acadoWorkspace.objValueOut[6];

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[60];
acadoWorkspace.objValueIn[1] = acadoVariables.x[61];
acadoWorkspace.objValueIn[2] = acadoVariables.x[62];
acadoWorkspace.objValueIn[3] = acadoVariables.x[63];
acadoWorkspace.objValueIn[4] = acadoVariables.x[64];
acadoWorkspace.objValueIn[5] = acadoVariables.x[65];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5];

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5];
dNew[1] += + Gx1[6]*dOld[0] + Gx1[7]*dOld[1] + Gx1[8]*dOld[2] + Gx1[9]*dOld[3] + Gx1[10]*dOld[4] + Gx1[11]*dOld[5];
dNew[2] += + Gx1[12]*dOld[0] + Gx1[13]*dOld[1] + Gx1[14]*dOld[2] + Gx1[15]*dOld[3] + Gx1[16]*dOld[4] + Gx1[17]*dOld[5];
dNew[3] += + Gx1[18]*dOld[0] + Gx1[19]*dOld[1] + Gx1[20]*dOld[2] + Gx1[21]*dOld[3] + Gx1[22]*dOld[4] + Gx1[23]*dOld[5];
dNew[4] += + Gx1[24]*dOld[0] + Gx1[25]*dOld[1] + Gx1[26]*dOld[2] + Gx1[27]*dOld[3] + Gx1[28]*dOld[4] + Gx1[29]*dOld[5];
dNew[5] += + Gx1[30]*dOld[0] + Gx1[31]*dOld[1] + Gx1[32]*dOld[2] + Gx1[33]*dOld[3] + Gx1[34]*dOld[4] + Gx1[35]*dOld[5];
}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
Gx2[9] = Gx1[9];
Gx2[10] = Gx1[10];
Gx2[11] = Gx1[11];
Gx2[12] = Gx1[12];
Gx2[13] = Gx1[13];
Gx2[14] = Gx1[14];
Gx2[15] = Gx1[15];
Gx2[16] = Gx1[16];
Gx2[17] = Gx1[17];
Gx2[18] = Gx1[18];
Gx2[19] = Gx1[19];
Gx2[20] = Gx1[20];
Gx2[21] = Gx1[21];
Gx2[22] = Gx1[22];
Gx2[23] = Gx1[23];
Gx2[24] = Gx1[24];
Gx2[25] = Gx1[25];
Gx2[26] = Gx1[26];
Gx2[27] = Gx1[27];
Gx2[28] = Gx1[28];
Gx2[29] = Gx1[29];
Gx2[30] = Gx1[30];
Gx2[31] = Gx1[31];
Gx2[32] = Gx1[32];
Gx2[33] = Gx1[33];
Gx2[34] = Gx1[34];
Gx2[35] = Gx1[35];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[6] + Gx1[2]*Gx2[12] + Gx1[3]*Gx2[18] + Gx1[4]*Gx2[24] + Gx1[5]*Gx2[30];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[7] + Gx1[2]*Gx2[13] + Gx1[3]*Gx2[19] + Gx1[4]*Gx2[25] + Gx1[5]*Gx2[31];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[8] + Gx1[2]*Gx2[14] + Gx1[3]*Gx2[20] + Gx1[4]*Gx2[26] + Gx1[5]*Gx2[32];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[9] + Gx1[2]*Gx2[15] + Gx1[3]*Gx2[21] + Gx1[4]*Gx2[27] + Gx1[5]*Gx2[33];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[10] + Gx1[2]*Gx2[16] + Gx1[3]*Gx2[22] + Gx1[4]*Gx2[28] + Gx1[5]*Gx2[34];
Gx3[5] = + Gx1[0]*Gx2[5] + Gx1[1]*Gx2[11] + Gx1[2]*Gx2[17] + Gx1[3]*Gx2[23] + Gx1[4]*Gx2[29] + Gx1[5]*Gx2[35];
Gx3[6] = + Gx1[6]*Gx2[0] + Gx1[7]*Gx2[6] + Gx1[8]*Gx2[12] + Gx1[9]*Gx2[18] + Gx1[10]*Gx2[24] + Gx1[11]*Gx2[30];
Gx3[7] = + Gx1[6]*Gx2[1] + Gx1[7]*Gx2[7] + Gx1[8]*Gx2[13] + Gx1[9]*Gx2[19] + Gx1[10]*Gx2[25] + Gx1[11]*Gx2[31];
Gx3[8] = + Gx1[6]*Gx2[2] + Gx1[7]*Gx2[8] + Gx1[8]*Gx2[14] + Gx1[9]*Gx2[20] + Gx1[10]*Gx2[26] + Gx1[11]*Gx2[32];
Gx3[9] = + Gx1[6]*Gx2[3] + Gx1[7]*Gx2[9] + Gx1[8]*Gx2[15] + Gx1[9]*Gx2[21] + Gx1[10]*Gx2[27] + Gx1[11]*Gx2[33];
Gx3[10] = + Gx1[6]*Gx2[4] + Gx1[7]*Gx2[10] + Gx1[8]*Gx2[16] + Gx1[9]*Gx2[22] + Gx1[10]*Gx2[28] + Gx1[11]*Gx2[34];
Gx3[11] = + Gx1[6]*Gx2[5] + Gx1[7]*Gx2[11] + Gx1[8]*Gx2[17] + Gx1[9]*Gx2[23] + Gx1[10]*Gx2[29] + Gx1[11]*Gx2[35];
Gx3[12] = + Gx1[12]*Gx2[0] + Gx1[13]*Gx2[6] + Gx1[14]*Gx2[12] + Gx1[15]*Gx2[18] + Gx1[16]*Gx2[24] + Gx1[17]*Gx2[30];
Gx3[13] = + Gx1[12]*Gx2[1] + Gx1[13]*Gx2[7] + Gx1[14]*Gx2[13] + Gx1[15]*Gx2[19] + Gx1[16]*Gx2[25] + Gx1[17]*Gx2[31];
Gx3[14] = + Gx1[12]*Gx2[2] + Gx1[13]*Gx2[8] + Gx1[14]*Gx2[14] + Gx1[15]*Gx2[20] + Gx1[16]*Gx2[26] + Gx1[17]*Gx2[32];
Gx3[15] = + Gx1[12]*Gx2[3] + Gx1[13]*Gx2[9] + Gx1[14]*Gx2[15] + Gx1[15]*Gx2[21] + Gx1[16]*Gx2[27] + Gx1[17]*Gx2[33];
Gx3[16] = + Gx1[12]*Gx2[4] + Gx1[13]*Gx2[10] + Gx1[14]*Gx2[16] + Gx1[15]*Gx2[22] + Gx1[16]*Gx2[28] + Gx1[17]*Gx2[34];
Gx3[17] = + Gx1[12]*Gx2[5] + Gx1[13]*Gx2[11] + Gx1[14]*Gx2[17] + Gx1[15]*Gx2[23] + Gx1[16]*Gx2[29] + Gx1[17]*Gx2[35];
Gx3[18] = + Gx1[18]*Gx2[0] + Gx1[19]*Gx2[6] + Gx1[20]*Gx2[12] + Gx1[21]*Gx2[18] + Gx1[22]*Gx2[24] + Gx1[23]*Gx2[30];
Gx3[19] = + Gx1[18]*Gx2[1] + Gx1[19]*Gx2[7] + Gx1[20]*Gx2[13] + Gx1[21]*Gx2[19] + Gx1[22]*Gx2[25] + Gx1[23]*Gx2[31];
Gx3[20] = + Gx1[18]*Gx2[2] + Gx1[19]*Gx2[8] + Gx1[20]*Gx2[14] + Gx1[21]*Gx2[20] + Gx1[22]*Gx2[26] + Gx1[23]*Gx2[32];
Gx3[21] = + Gx1[18]*Gx2[3] + Gx1[19]*Gx2[9] + Gx1[20]*Gx2[15] + Gx1[21]*Gx2[21] + Gx1[22]*Gx2[27] + Gx1[23]*Gx2[33];
Gx3[22] = + Gx1[18]*Gx2[4] + Gx1[19]*Gx2[10] + Gx1[20]*Gx2[16] + Gx1[21]*Gx2[22] + Gx1[22]*Gx2[28] + Gx1[23]*Gx2[34];
Gx3[23] = + Gx1[18]*Gx2[5] + Gx1[19]*Gx2[11] + Gx1[20]*Gx2[17] + Gx1[21]*Gx2[23] + Gx1[22]*Gx2[29] + Gx1[23]*Gx2[35];
Gx3[24] = + Gx1[24]*Gx2[0] + Gx1[25]*Gx2[6] + Gx1[26]*Gx2[12] + Gx1[27]*Gx2[18] + Gx1[28]*Gx2[24] + Gx1[29]*Gx2[30];
Gx3[25] = + Gx1[24]*Gx2[1] + Gx1[25]*Gx2[7] + Gx1[26]*Gx2[13] + Gx1[27]*Gx2[19] + Gx1[28]*Gx2[25] + Gx1[29]*Gx2[31];
Gx3[26] = + Gx1[24]*Gx2[2] + Gx1[25]*Gx2[8] + Gx1[26]*Gx2[14] + Gx1[27]*Gx2[20] + Gx1[28]*Gx2[26] + Gx1[29]*Gx2[32];
Gx3[27] = + Gx1[24]*Gx2[3] + Gx1[25]*Gx2[9] + Gx1[26]*Gx2[15] + Gx1[27]*Gx2[21] + Gx1[28]*Gx2[27] + Gx1[29]*Gx2[33];
Gx3[28] = + Gx1[24]*Gx2[4] + Gx1[25]*Gx2[10] + Gx1[26]*Gx2[16] + Gx1[27]*Gx2[22] + Gx1[28]*Gx2[28] + Gx1[29]*Gx2[34];
Gx3[29] = + Gx1[24]*Gx2[5] + Gx1[25]*Gx2[11] + Gx1[26]*Gx2[17] + Gx1[27]*Gx2[23] + Gx1[28]*Gx2[29] + Gx1[29]*Gx2[35];
Gx3[30] = + Gx1[30]*Gx2[0] + Gx1[31]*Gx2[6] + Gx1[32]*Gx2[12] + Gx1[33]*Gx2[18] + Gx1[34]*Gx2[24] + Gx1[35]*Gx2[30];
Gx3[31] = + Gx1[30]*Gx2[1] + Gx1[31]*Gx2[7] + Gx1[32]*Gx2[13] + Gx1[33]*Gx2[19] + Gx1[34]*Gx2[25] + Gx1[35]*Gx2[31];
Gx3[32] = + Gx1[30]*Gx2[2] + Gx1[31]*Gx2[8] + Gx1[32]*Gx2[14] + Gx1[33]*Gx2[20] + Gx1[34]*Gx2[26] + Gx1[35]*Gx2[32];
Gx3[33] = + Gx1[30]*Gx2[3] + Gx1[31]*Gx2[9] + Gx1[32]*Gx2[15] + Gx1[33]*Gx2[21] + Gx1[34]*Gx2[27] + Gx1[35]*Gx2[33];
Gx3[34] = + Gx1[30]*Gx2[4] + Gx1[31]*Gx2[10] + Gx1[32]*Gx2[16] + Gx1[33]*Gx2[22] + Gx1[34]*Gx2[28] + Gx1[35]*Gx2[34];
Gx3[35] = + Gx1[30]*Gx2[5] + Gx1[31]*Gx2[11] + Gx1[32]*Gx2[17] + Gx1[33]*Gx2[23] + Gx1[34]*Gx2[29] + Gx1[35]*Gx2[35];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[1] + Gx1[2]*Gu1[2] + Gx1[3]*Gu1[3] + Gx1[4]*Gu1[4] + Gx1[5]*Gu1[5];
Gu2[1] = + Gx1[6]*Gu1[0] + Gx1[7]*Gu1[1] + Gx1[8]*Gu1[2] + Gx1[9]*Gu1[3] + Gx1[10]*Gu1[4] + Gx1[11]*Gu1[5];
Gu2[2] = + Gx1[12]*Gu1[0] + Gx1[13]*Gu1[1] + Gx1[14]*Gu1[2] + Gx1[15]*Gu1[3] + Gx1[16]*Gu1[4] + Gx1[17]*Gu1[5];
Gu2[3] = + Gx1[18]*Gu1[0] + Gx1[19]*Gu1[1] + Gx1[20]*Gu1[2] + Gx1[21]*Gu1[3] + Gx1[22]*Gu1[4] + Gx1[23]*Gu1[5];
Gu2[4] = + Gx1[24]*Gu1[0] + Gx1[25]*Gu1[1] + Gx1[26]*Gu1[2] + Gx1[27]*Gu1[3] + Gx1[28]*Gu1[4] + Gx1[29]*Gu1[5];
Gu2[5] = + Gx1[30]*Gu1[0] + Gx1[31]*Gu1[1] + Gx1[32]*Gu1[2] + Gx1[33]*Gu1[3] + Gx1[34]*Gu1[4] + Gx1[35]*Gu1[5];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 10) + (iCol)] += + Gu1[0]*Gu2[0] + Gu1[1]*Gu2[1] + Gu1[2]*Gu2[2] + Gu1[3]*Gu2[3] + Gu1[4]*Gu2[4] + Gu1[5]*Gu2[5];
}

void acado_setBlockH11_R1( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 10) + (iCol)] = (real_t)1.0000000000000000e+00;
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 10) + (iCol)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 10) + (iCol)] = acadoWorkspace.H[(iCol * 10) + (iRow)];
}

void acado_multQ1d( real_t* const dOld, real_t* const dNew )
{
dNew[0] = +dOld[0];
dNew[1] = +dOld[1];
dNew[2] = +dOld[2];
dNew[3] = +dOld[3];
dNew[4] = +dOld[4];
dNew[5] = +dOld[5];
}

void acado_multQN1d( real_t* const dOld, real_t* const dNew )
{
dNew[0] = + (real_t)1.0000000000000000e+01*dOld[0];
dNew[1] = + (real_t)1.0000000000000000e+01*dOld[1];
dNew[2] = + (real_t)1.0000000000000000e+01*dOld[2];
dNew[3] = + (real_t)1.0000000000000000e+01*dOld[3];
dNew[4] = + (real_t)1.0000000000000000e+01*dOld[4];
dNew[5] = + (real_t)1.0000000000000000e+01*dOld[5];
}

void acado_multRDy( real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = +Dy1[6];
}

void acado_multQDy( real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = +Dy1[0];
QDy1[1] = +Dy1[1];
QDy1[2] = +Dy1[2];
QDy1[3] = +Dy1[3];
QDy1[4] = +Dy1[4];
QDy1[5] = +Dy1[5];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[1]*QDy1[1] + E1[2]*QDy1[2] + E1[3]*QDy1[3] + E1[4]*QDy1[4] + E1[5]*QDy1[5];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[1]*Gx1[6] + E1[2]*Gx1[12] + E1[3]*Gx1[18] + E1[4]*Gx1[24] + E1[5]*Gx1[30];
H101[1] += + E1[0]*Gx1[1] + E1[1]*Gx1[7] + E1[2]*Gx1[13] + E1[3]*Gx1[19] + E1[4]*Gx1[25] + E1[5]*Gx1[31];
H101[2] += + E1[0]*Gx1[2] + E1[1]*Gx1[8] + E1[2]*Gx1[14] + E1[3]*Gx1[20] + E1[4]*Gx1[26] + E1[5]*Gx1[32];
H101[3] += + E1[0]*Gx1[3] + E1[1]*Gx1[9] + E1[2]*Gx1[15] + E1[3]*Gx1[21] + E1[4]*Gx1[27] + E1[5]*Gx1[33];
H101[4] += + E1[0]*Gx1[4] + E1[1]*Gx1[10] + E1[2]*Gx1[16] + E1[3]*Gx1[22] + E1[4]*Gx1[28] + E1[5]*Gx1[34];
H101[5] += + E1[0]*Gx1[5] + E1[1]*Gx1[11] + E1[2]*Gx1[17] + E1[3]*Gx1[23] + E1[4]*Gx1[29] + E1[5]*Gx1[35];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 6; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0];
dNew[1] += + E1[1]*U1[0];
dNew[2] += + E1[2]*U1[0];
dNew[3] += + E1[3]*U1[0];
dNew[4] += + E1[4]*U1[0];
dNew[5] += + E1[5]*U1[0];
}

void acado_multQ1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = +Gx1[0];
Gx2[1] = +Gx1[1];
Gx2[2] = +Gx1[2];
Gx2[3] = +Gx1[3];
Gx2[4] = +Gx1[4];
Gx2[5] = +Gx1[5];
Gx2[6] = +Gx1[6];
Gx2[7] = +Gx1[7];
Gx2[8] = +Gx1[8];
Gx2[9] = +Gx1[9];
Gx2[10] = +Gx1[10];
Gx2[11] = +Gx1[11];
Gx2[12] = +Gx1[12];
Gx2[13] = +Gx1[13];
Gx2[14] = +Gx1[14];
Gx2[15] = +Gx1[15];
Gx2[16] = +Gx1[16];
Gx2[17] = +Gx1[17];
Gx2[18] = +Gx1[18];
Gx2[19] = +Gx1[19];
Gx2[20] = +Gx1[20];
Gx2[21] = +Gx1[21];
Gx2[22] = +Gx1[22];
Gx2[23] = +Gx1[23];
Gx2[24] = +Gx1[24];
Gx2[25] = +Gx1[25];
Gx2[26] = +Gx1[26];
Gx2[27] = +Gx1[27];
Gx2[28] = +Gx1[28];
Gx2[29] = +Gx1[29];
Gx2[30] = +Gx1[30];
Gx2[31] = +Gx1[31];
Gx2[32] = +Gx1[32];
Gx2[33] = +Gx1[33];
Gx2[34] = +Gx1[34];
Gx2[35] = +Gx1[35];
}

void acado_multQN1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = + (real_t)1.0000000000000000e+01*Gx1[0];
Gx2[1] = + (real_t)1.0000000000000000e+01*Gx1[1];
Gx2[2] = + (real_t)1.0000000000000000e+01*Gx1[2];
Gx2[3] = + (real_t)1.0000000000000000e+01*Gx1[3];
Gx2[4] = + (real_t)1.0000000000000000e+01*Gx1[4];
Gx2[5] = + (real_t)1.0000000000000000e+01*Gx1[5];
Gx2[6] = + (real_t)1.0000000000000000e+01*Gx1[6];
Gx2[7] = + (real_t)1.0000000000000000e+01*Gx1[7];
Gx2[8] = + (real_t)1.0000000000000000e+01*Gx1[8];
Gx2[9] = + (real_t)1.0000000000000000e+01*Gx1[9];
Gx2[10] = + (real_t)1.0000000000000000e+01*Gx1[10];
Gx2[11] = + (real_t)1.0000000000000000e+01*Gx1[11];
Gx2[12] = + (real_t)1.0000000000000000e+01*Gx1[12];
Gx2[13] = + (real_t)1.0000000000000000e+01*Gx1[13];
Gx2[14] = + (real_t)1.0000000000000000e+01*Gx1[14];
Gx2[15] = + (real_t)1.0000000000000000e+01*Gx1[15];
Gx2[16] = + (real_t)1.0000000000000000e+01*Gx1[16];
Gx2[17] = + (real_t)1.0000000000000000e+01*Gx1[17];
Gx2[18] = + (real_t)1.0000000000000000e+01*Gx1[18];
Gx2[19] = + (real_t)1.0000000000000000e+01*Gx1[19];
Gx2[20] = + (real_t)1.0000000000000000e+01*Gx1[20];
Gx2[21] = + (real_t)1.0000000000000000e+01*Gx1[21];
Gx2[22] = + (real_t)1.0000000000000000e+01*Gx1[22];
Gx2[23] = + (real_t)1.0000000000000000e+01*Gx1[23];
Gx2[24] = + (real_t)1.0000000000000000e+01*Gx1[24];
Gx2[25] = + (real_t)1.0000000000000000e+01*Gx1[25];
Gx2[26] = + (real_t)1.0000000000000000e+01*Gx1[26];
Gx2[27] = + (real_t)1.0000000000000000e+01*Gx1[27];
Gx2[28] = + (real_t)1.0000000000000000e+01*Gx1[28];
Gx2[29] = + (real_t)1.0000000000000000e+01*Gx1[29];
Gx2[30] = + (real_t)1.0000000000000000e+01*Gx1[30];
Gx2[31] = + (real_t)1.0000000000000000e+01*Gx1[31];
Gx2[32] = + (real_t)1.0000000000000000e+01*Gx1[32];
Gx2[33] = + (real_t)1.0000000000000000e+01*Gx1[33];
Gx2[34] = + (real_t)1.0000000000000000e+01*Gx1[34];
Gx2[35] = + (real_t)1.0000000000000000e+01*Gx1[35];
}

void acado_multQ1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = +Gu1[0];
Gu2[1] = +Gu1[1];
Gu2[2] = +Gu1[2];
Gu2[3] = +Gu1[3];
Gu2[4] = +Gu1[4];
Gu2[5] = +Gu1[5];
}

void acado_multQN1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + (real_t)1.0000000000000000e+01*Gu1[0];
Gu2[1] = + (real_t)1.0000000000000000e+01*Gu1[1];
Gu2[2] = + (real_t)1.0000000000000000e+01*Gu1[2];
Gu2[3] = + (real_t)1.0000000000000000e+01*Gu1[3];
Gu2[4] = + (real_t)1.0000000000000000e+01*Gu1[4];
Gu2[5] = + (real_t)1.0000000000000000e+01*Gu1[5];
}

void acado_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
}

void acado_condensePrep(  )
{
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
acado_moveGxT( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.T );
acado_multGxd( acadoWorkspace.d, &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.d[ 6 ]) );
acado_multGxGx( acadoWorkspace.T, acadoWorkspace.evGx, &(acadoWorkspace.evGx[ 36 ]) );

acado_multGxGu( acadoWorkspace.T, acadoWorkspace.E, &(acadoWorkspace.E[ 6 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 6 ]), &(acadoWorkspace.E[ 12 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 72 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 6 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.d[ 12 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.evGx[ 72 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.E[ 18 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.E[ 24 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 12 ]), &(acadoWorkspace.E[ 30 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 108 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.d[ 18 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.evGx[ 108 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.E[ 36 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.E[ 42 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.E[ 48 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 18 ]), &(acadoWorkspace.E[ 54 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 18 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.d[ 24 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.evGx[ 144 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.E[ 60 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.E[ 66 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.E[ 72 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.E[ 78 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 24 ]), &(acadoWorkspace.E[ 84 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 180 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.d[ 30 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.evGx[ 180 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.E[ 90 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.E[ 96 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.E[ 102 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.E[ 108 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.E[ 114 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 30 ]), &(acadoWorkspace.E[ 120 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 216 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 30 ]), &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.d[ 36 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.evGx[ 216 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.E[ 126 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.E[ 132 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.E[ 138 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.E[ 144 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.E[ 150 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.E[ 156 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 36 ]), &(acadoWorkspace.E[ 162 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 252 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.d[ 42 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.evGx[ 252 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.E[ 168 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.E[ 174 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.E[ 180 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.E[ 186 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.E[ 192 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.E[ 198 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 162 ]), &(acadoWorkspace.E[ 204 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 42 ]), &(acadoWorkspace.E[ 210 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 42 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.d[ 48 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.evGx[ 288 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.E[ 216 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.E[ 222 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.E[ 228 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 186 ]), &(acadoWorkspace.E[ 234 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.E[ 240 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.E[ 246 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.E[ 252 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.E[ 258 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.E[ 264 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 324 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.d[ 54 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.evGx[ 324 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.E[ 270 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.E[ 276 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.E[ 282 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.E[ 288 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.E[ 294 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.E[ 300 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.E[ 306 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 258 ]), &(acadoWorkspace.E[ 312 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.E[ 318 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 54 ]), &(acadoWorkspace.E[ 324 ]) );

acado_multQ1Gu( acadoWorkspace.E, acadoWorkspace.QE );
acado_multQ1Gu( &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.QE[ 6 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 12 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.QE[ 18 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 30 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.QE[ 42 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 66 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 90 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.QE[ 102 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.QE[ 114 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QE[ 126 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 138 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 162 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QE[ 174 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 186 ]), &(acadoWorkspace.QE[ 186 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.QE[ 198 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.QE[ 222 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 234 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.QE[ 246 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 258 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_multQ1Gu( &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 276 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 282 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.QE[ 294 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 306 ]), &(acadoWorkspace.QE[ 306 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.QE[ 318 ]) );
acado_multQN1Gu( &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 324 ]) );

acado_zeroBlockH10( acadoWorkspace.H10 );
acado_multQETGx( acadoWorkspace.QE, acadoWorkspace.evGx, acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 6 ]), &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 18 ]), &(acadoWorkspace.evGx[ 72 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 36 ]), &(acadoWorkspace.evGx[ 108 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 60 ]), &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 90 ]), &(acadoWorkspace.evGx[ 180 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 126 ]), &(acadoWorkspace.evGx[ 216 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 168 ]), &(acadoWorkspace.evGx[ 252 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 216 ]), &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 270 ]), &(acadoWorkspace.evGx[ 324 ]), acadoWorkspace.H10 );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 12 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 24 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 42 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 66 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 96 ]), &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 132 ]), &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 174 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 222 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 276 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 30 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 48 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 72 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 102 ]), &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 138 ]), &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 180 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 228 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 282 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 54 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 78 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 108 ]), &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 144 ]), &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 186 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 234 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 288 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 84 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 114 ]), &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 150 ]), &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 192 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 240 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 294 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 120 ]), &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 156 ]), &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 198 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 246 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 300 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 162 ]), &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 204 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 252 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 306 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 42 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 210 ]), &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.H10[ 42 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 258 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 42 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 312 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 42 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 264 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 318 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 54 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 324 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 54 ]) );

acado_setBlockH11_R1( 0, 0 );
acado_setBlockH11( 0, 0, acadoWorkspace.E, acadoWorkspace.QE );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.QE[ 6 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.QE[ 18 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 90 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QE[ 126 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 270 ]) );

acado_zeroBlockH11( 0, 1 );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.QE[ 12 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 42 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 66 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 174 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 222 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 276 ]) );

acado_zeroBlockH11( 0, 2 );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.QE[ 30 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 102 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QE[ 138 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 282 ]) );

acado_zeroBlockH11( 0, 3 );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 186 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 234 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 288 ]) );

acado_zeroBlockH11( 0, 4 );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 114 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 294 ]) );

acado_zeroBlockH11( 0, 5 );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 198 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 246 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 300 ]) );

acado_zeroBlockH11( 0, 6 );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 306 ]) );

acado_zeroBlockH11( 0, 7 );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 312 ]) );

acado_zeroBlockH11( 0, 8 );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 318 ]) );

acado_zeroBlockH11( 0, 9 );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 324 ]) );

acado_setBlockH11_R1( 1, 1 );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 12 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.QE[ 42 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 66 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QE[ 174 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.QE[ 222 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 276 ]) );

acado_zeroBlockH11( 1, 2 );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 30 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 102 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 138 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 282 ]) );

acado_zeroBlockH11( 1, 3 );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QE[ 186 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.QE[ 234 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 288 ]) );

acado_zeroBlockH11( 1, 4 );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 114 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 294 ]) );

acado_zeroBlockH11( 1, 5 );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QE[ 198 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.QE[ 246 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 300 ]) );

acado_zeroBlockH11( 1, 6 );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 306 ]) );

acado_zeroBlockH11( 1, 7 );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 312 ]) );

acado_zeroBlockH11( 1, 8 );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 318 ]) );

acado_zeroBlockH11( 1, 9 );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 324 ]) );

acado_setBlockH11_R1( 2, 2 );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 30 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.QE[ 102 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 138 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 282 ]) );

acado_zeroBlockH11( 2, 3 );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 186 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 234 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 288 ]) );

acado_zeroBlockH11( 2, 4 );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.QE[ 114 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 294 ]) );

acado_zeroBlockH11( 2, 5 );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 198 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 246 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 300 ]) );

acado_zeroBlockH11( 2, 6 );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 306 ]) );

acado_zeroBlockH11( 2, 7 );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 312 ]) );

acado_zeroBlockH11( 2, 8 );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 318 ]) );

acado_zeroBlockH11( 2, 9 );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 324 ]) );

acado_setBlockH11_R1( 3, 3 );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 186 ]), &(acadoWorkspace.QE[ 186 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 234 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 288 ]) );

acado_zeroBlockH11( 3, 4 );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 114 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 186 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 294 ]) );

acado_zeroBlockH11( 3, 5 );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 186 ]), &(acadoWorkspace.QE[ 198 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 246 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 300 ]) );

acado_zeroBlockH11( 3, 6 );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 186 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 306 ]) );

acado_zeroBlockH11( 3, 7 );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 186 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 312 ]) );

acado_zeroBlockH11( 3, 8 );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 318 ]) );

acado_zeroBlockH11( 3, 9 );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 324 ]) );

acado_setBlockH11_R1( 4, 4 );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.QE[ 114 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.QE[ 294 ]) );

acado_zeroBlockH11( 4, 5 );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 198 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 246 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.QE[ 300 ]) );

acado_zeroBlockH11( 4, 6 );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.QE[ 306 ]) );

acado_zeroBlockH11( 4, 7 );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.QE[ 312 ]) );

acado_zeroBlockH11( 4, 8 );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.QE[ 318 ]) );

acado_zeroBlockH11( 4, 9 );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.QE[ 324 ]) );

acado_setBlockH11_R1( 5, 5 );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.QE[ 198 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.QE[ 246 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 300 ]) );

acado_zeroBlockH11( 5, 6 );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 306 ]) );

acado_zeroBlockH11( 5, 7 );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 312 ]) );

acado_zeroBlockH11( 5, 8 );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 318 ]) );

acado_zeroBlockH11( 5, 9 );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 324 ]) );

acado_setBlockH11_R1( 6, 6 );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 162 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 306 ]), &(acadoWorkspace.QE[ 306 ]) );

acado_zeroBlockH11( 6, 7 );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 306 ]), &(acadoWorkspace.QE[ 312 ]) );

acado_zeroBlockH11( 6, 8 );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 306 ]), &(acadoWorkspace.QE[ 318 ]) );

acado_zeroBlockH11( 6, 9 );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 306 ]), &(acadoWorkspace.QE[ 324 ]) );

acado_setBlockH11_R1( 7, 7 );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 258 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 312 ]) );

acado_zeroBlockH11( 7, 8 );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 258 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 318 ]) );

acado_zeroBlockH11( 7, 9 );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 324 ]) );

acado_setBlockH11_R1( 8, 8 );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.QE[ 318 ]) );

acado_zeroBlockH11( 8, 9 );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.QE[ 324 ]) );

acado_setBlockH11_R1( 9, 9 );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 324 ]) );


acado_copyHTH( 1, 0 );
acado_copyHTH( 2, 0 );
acado_copyHTH( 2, 1 );
acado_copyHTH( 3, 0 );
acado_copyHTH( 3, 1 );
acado_copyHTH( 3, 2 );
acado_copyHTH( 4, 0 );
acado_copyHTH( 4, 1 );
acado_copyHTH( 4, 2 );
acado_copyHTH( 4, 3 );
acado_copyHTH( 5, 0 );
acado_copyHTH( 5, 1 );
acado_copyHTH( 5, 2 );
acado_copyHTH( 5, 3 );
acado_copyHTH( 5, 4 );
acado_copyHTH( 6, 0 );
acado_copyHTH( 6, 1 );
acado_copyHTH( 6, 2 );
acado_copyHTH( 6, 3 );
acado_copyHTH( 6, 4 );
acado_copyHTH( 6, 5 );
acado_copyHTH( 7, 0 );
acado_copyHTH( 7, 1 );
acado_copyHTH( 7, 2 );
acado_copyHTH( 7, 3 );
acado_copyHTH( 7, 4 );
acado_copyHTH( 7, 5 );
acado_copyHTH( 7, 6 );
acado_copyHTH( 8, 0 );
acado_copyHTH( 8, 1 );
acado_copyHTH( 8, 2 );
acado_copyHTH( 8, 3 );
acado_copyHTH( 8, 4 );
acado_copyHTH( 8, 5 );
acado_copyHTH( 8, 6 );
acado_copyHTH( 8, 7 );
acado_copyHTH( 9, 0 );
acado_copyHTH( 9, 1 );
acado_copyHTH( 9, 2 );
acado_copyHTH( 9, 3 );
acado_copyHTH( 9, 4 );
acado_copyHTH( 9, 5 );
acado_copyHTH( 9, 6 );
acado_copyHTH( 9, 7 );
acado_copyHTH( 9, 8 );

acado_multQ1d( acadoWorkspace.d, acadoWorkspace.Qd );
acado_multQ1d( &(acadoWorkspace.d[ 6 ]), &(acadoWorkspace.Qd[ 6 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.Qd[ 12 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 18 ]), &(acadoWorkspace.Qd[ 18 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.Qd[ 24 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 30 ]), &(acadoWorkspace.Qd[ 30 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.Qd[ 36 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 42 ]), &(acadoWorkspace.Qd[ 42 ]) );
acado_multQ1d( &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.Qd[ 48 ]) );
acado_multQN1d( &(acadoWorkspace.d[ 54 ]), &(acadoWorkspace.Qd[ 54 ]) );

acado_macETSlu( acadoWorkspace.QE, acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 6 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 18 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 36 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 60 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 90 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 126 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 168 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 216 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 270 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 12 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 24 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 42 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 66 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 96 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 132 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 174 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 222 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 276 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 30 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 48 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 72 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 102 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 138 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 180 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 228 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 282 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 54 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 78 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 108 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 144 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 186 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 234 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 288 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 84 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 114 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 150 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 192 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 240 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 294 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 120 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 156 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 198 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 246 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 300 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 162 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 204 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 252 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 306 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 210 ]), &(acadoWorkspace.g[ 7 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 258 ]), &(acadoWorkspace.g[ 7 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 312 ]), &(acadoWorkspace.g[ 7 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 264 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 318 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 324 ]), &(acadoWorkspace.g[ 9 ]) );
}

void acado_condenseFdb(  )
{
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];

acadoWorkspace.Dy[0] -= acadoVariables.y[0];
acadoWorkspace.Dy[1] -= acadoVariables.y[1];
acadoWorkspace.Dy[2] -= acadoVariables.y[2];
acadoWorkspace.Dy[3] -= acadoVariables.y[3];
acadoWorkspace.Dy[4] -= acadoVariables.y[4];
acadoWorkspace.Dy[5] -= acadoVariables.y[5];
acadoWorkspace.Dy[6] -= acadoVariables.y[6];
acadoWorkspace.Dy[7] -= acadoVariables.y[7];
acadoWorkspace.Dy[8] -= acadoVariables.y[8];
acadoWorkspace.Dy[9] -= acadoVariables.y[9];
acadoWorkspace.Dy[10] -= acadoVariables.y[10];
acadoWorkspace.Dy[11] -= acadoVariables.y[11];
acadoWorkspace.Dy[12] -= acadoVariables.y[12];
acadoWorkspace.Dy[13] -= acadoVariables.y[13];
acadoWorkspace.Dy[14] -= acadoVariables.y[14];
acadoWorkspace.Dy[15] -= acadoVariables.y[15];
acadoWorkspace.Dy[16] -= acadoVariables.y[16];
acadoWorkspace.Dy[17] -= acadoVariables.y[17];
acadoWorkspace.Dy[18] -= acadoVariables.y[18];
acadoWorkspace.Dy[19] -= acadoVariables.y[19];
acadoWorkspace.Dy[20] -= acadoVariables.y[20];
acadoWorkspace.Dy[21] -= acadoVariables.y[21];
acadoWorkspace.Dy[22] -= acadoVariables.y[22];
acadoWorkspace.Dy[23] -= acadoVariables.y[23];
acadoWorkspace.Dy[24] -= acadoVariables.y[24];
acadoWorkspace.Dy[25] -= acadoVariables.y[25];
acadoWorkspace.Dy[26] -= acadoVariables.y[26];
acadoWorkspace.Dy[27] -= acadoVariables.y[27];
acadoWorkspace.Dy[28] -= acadoVariables.y[28];
acadoWorkspace.Dy[29] -= acadoVariables.y[29];
acadoWorkspace.Dy[30] -= acadoVariables.y[30];
acadoWorkspace.Dy[31] -= acadoVariables.y[31];
acadoWorkspace.Dy[32] -= acadoVariables.y[32];
acadoWorkspace.Dy[33] -= acadoVariables.y[33];
acadoWorkspace.Dy[34] -= acadoVariables.y[34];
acadoWorkspace.Dy[35] -= acadoVariables.y[35];
acadoWorkspace.Dy[36] -= acadoVariables.y[36];
acadoWorkspace.Dy[37] -= acadoVariables.y[37];
acadoWorkspace.Dy[38] -= acadoVariables.y[38];
acadoWorkspace.Dy[39] -= acadoVariables.y[39];
acadoWorkspace.Dy[40] -= acadoVariables.y[40];
acadoWorkspace.Dy[41] -= acadoVariables.y[41];
acadoWorkspace.Dy[42] -= acadoVariables.y[42];
acadoWorkspace.Dy[43] -= acadoVariables.y[43];
acadoWorkspace.Dy[44] -= acadoVariables.y[44];
acadoWorkspace.Dy[45] -= acadoVariables.y[45];
acadoWorkspace.Dy[46] -= acadoVariables.y[46];
acadoWorkspace.Dy[47] -= acadoVariables.y[47];
acadoWorkspace.Dy[48] -= acadoVariables.y[48];
acadoWorkspace.Dy[49] -= acadoVariables.y[49];
acadoWorkspace.Dy[50] -= acadoVariables.y[50];
acadoWorkspace.Dy[51] -= acadoVariables.y[51];
acadoWorkspace.Dy[52] -= acadoVariables.y[52];
acadoWorkspace.Dy[53] -= acadoVariables.y[53];
acadoWorkspace.Dy[54] -= acadoVariables.y[54];
acadoWorkspace.Dy[55] -= acadoVariables.y[55];
acadoWorkspace.Dy[56] -= acadoVariables.y[56];
acadoWorkspace.Dy[57] -= acadoVariables.y[57];
acadoWorkspace.Dy[58] -= acadoVariables.y[58];
acadoWorkspace.Dy[59] -= acadoVariables.y[59];
acadoWorkspace.Dy[60] -= acadoVariables.y[60];
acadoWorkspace.Dy[61] -= acadoVariables.y[61];
acadoWorkspace.Dy[62] -= acadoVariables.y[62];
acadoWorkspace.Dy[63] -= acadoVariables.y[63];
acadoWorkspace.Dy[64] -= acadoVariables.y[64];
acadoWorkspace.Dy[65] -= acadoVariables.y[65];
acadoWorkspace.Dy[66] -= acadoVariables.y[66];
acadoWorkspace.Dy[67] -= acadoVariables.y[67];
acadoWorkspace.Dy[68] -= acadoVariables.y[68];
acadoWorkspace.Dy[69] -= acadoVariables.y[69];
acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];
acadoWorkspace.DyN[4] -= acadoVariables.yN[4];
acadoWorkspace.DyN[5] -= acadoVariables.yN[5];

acado_multRDy( acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.Dy[ 7 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 14 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 21 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 28 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 49 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 63 ]), &(acadoWorkspace.g[ 9 ]) );

acado_multQDy( acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Dy[ 7 ]), &(acadoWorkspace.QDy[ 6 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 14 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 21 ]), &(acadoWorkspace.QDy[ 18 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 28 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.QDy[ 30 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 49 ]), &(acadoWorkspace.QDy[ 42 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 63 ]), &(acadoWorkspace.QDy[ 54 ]) );

acadoWorkspace.QDy[60] = + (real_t)1.0000000000000000e+01*acadoWorkspace.DyN[0];
acadoWorkspace.QDy[61] = + (real_t)1.0000000000000000e+01*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[62] = + (real_t)1.0000000000000000e+01*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[63] = + (real_t)1.0000000000000000e+01*acadoWorkspace.DyN[3];
acadoWorkspace.QDy[64] = + (real_t)1.0000000000000000e+01*acadoWorkspace.DyN[4];
acadoWorkspace.QDy[65] = + (real_t)1.0000000000000000e+01*acadoWorkspace.DyN[5];

acadoWorkspace.QDy[6] += acadoWorkspace.Qd[0];
acadoWorkspace.QDy[7] += acadoWorkspace.Qd[1];
acadoWorkspace.QDy[8] += acadoWorkspace.Qd[2];
acadoWorkspace.QDy[9] += acadoWorkspace.Qd[3];
acadoWorkspace.QDy[10] += acadoWorkspace.Qd[4];
acadoWorkspace.QDy[11] += acadoWorkspace.Qd[5];
acadoWorkspace.QDy[12] += acadoWorkspace.Qd[6];
acadoWorkspace.QDy[13] += acadoWorkspace.Qd[7];
acadoWorkspace.QDy[14] += acadoWorkspace.Qd[8];
acadoWorkspace.QDy[15] += acadoWorkspace.Qd[9];
acadoWorkspace.QDy[16] += acadoWorkspace.Qd[10];
acadoWorkspace.QDy[17] += acadoWorkspace.Qd[11];
acadoWorkspace.QDy[18] += acadoWorkspace.Qd[12];
acadoWorkspace.QDy[19] += acadoWorkspace.Qd[13];
acadoWorkspace.QDy[20] += acadoWorkspace.Qd[14];
acadoWorkspace.QDy[21] += acadoWorkspace.Qd[15];
acadoWorkspace.QDy[22] += acadoWorkspace.Qd[16];
acadoWorkspace.QDy[23] += acadoWorkspace.Qd[17];
acadoWorkspace.QDy[24] += acadoWorkspace.Qd[18];
acadoWorkspace.QDy[25] += acadoWorkspace.Qd[19];
acadoWorkspace.QDy[26] += acadoWorkspace.Qd[20];
acadoWorkspace.QDy[27] += acadoWorkspace.Qd[21];
acadoWorkspace.QDy[28] += acadoWorkspace.Qd[22];
acadoWorkspace.QDy[29] += acadoWorkspace.Qd[23];
acadoWorkspace.QDy[30] += acadoWorkspace.Qd[24];
acadoWorkspace.QDy[31] += acadoWorkspace.Qd[25];
acadoWorkspace.QDy[32] += acadoWorkspace.Qd[26];
acadoWorkspace.QDy[33] += acadoWorkspace.Qd[27];
acadoWorkspace.QDy[34] += acadoWorkspace.Qd[28];
acadoWorkspace.QDy[35] += acadoWorkspace.Qd[29];
acadoWorkspace.QDy[36] += acadoWorkspace.Qd[30];
acadoWorkspace.QDy[37] += acadoWorkspace.Qd[31];
acadoWorkspace.QDy[38] += acadoWorkspace.Qd[32];
acadoWorkspace.QDy[39] += acadoWorkspace.Qd[33];
acadoWorkspace.QDy[40] += acadoWorkspace.Qd[34];
acadoWorkspace.QDy[41] += acadoWorkspace.Qd[35];
acadoWorkspace.QDy[42] += acadoWorkspace.Qd[36];
acadoWorkspace.QDy[43] += acadoWorkspace.Qd[37];
acadoWorkspace.QDy[44] += acadoWorkspace.Qd[38];
acadoWorkspace.QDy[45] += acadoWorkspace.Qd[39];
acadoWorkspace.QDy[46] += acadoWorkspace.Qd[40];
acadoWorkspace.QDy[47] += acadoWorkspace.Qd[41];
acadoWorkspace.QDy[48] += acadoWorkspace.Qd[42];
acadoWorkspace.QDy[49] += acadoWorkspace.Qd[43];
acadoWorkspace.QDy[50] += acadoWorkspace.Qd[44];
acadoWorkspace.QDy[51] += acadoWorkspace.Qd[45];
acadoWorkspace.QDy[52] += acadoWorkspace.Qd[46];
acadoWorkspace.QDy[53] += acadoWorkspace.Qd[47];
acadoWorkspace.QDy[54] += acadoWorkspace.Qd[48];
acadoWorkspace.QDy[55] += acadoWorkspace.Qd[49];
acadoWorkspace.QDy[56] += acadoWorkspace.Qd[50];
acadoWorkspace.QDy[57] += acadoWorkspace.Qd[51];
acadoWorkspace.QDy[58] += acadoWorkspace.Qd[52];
acadoWorkspace.QDy[59] += acadoWorkspace.Qd[53];
acadoWorkspace.QDy[60] += acadoWorkspace.Qd[54];
acadoWorkspace.QDy[61] += acadoWorkspace.Qd[55];
acadoWorkspace.QDy[62] += acadoWorkspace.Qd[56];
acadoWorkspace.QDy[63] += acadoWorkspace.Qd[57];
acadoWorkspace.QDy[64] += acadoWorkspace.Qd[58];
acadoWorkspace.QDy[65] += acadoWorkspace.Qd[59];

acado_multEQDy( acadoWorkspace.E, &(acadoWorkspace.QDy[ 6 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QDy[ 30 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QDy[ 42 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QDy[ 48 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QDy[ 54 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QDy[ 60 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 186 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 162 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 306 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 258 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 9 ]) );

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[1] += + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[2] += + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[3] += + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[20]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[21]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[22]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[23]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[4] += + acadoWorkspace.H10[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[27]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[28]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[29]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[5] += + acadoWorkspace.H10[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[32]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[33]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[34]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[35]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[6] += + acadoWorkspace.H10[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[39]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[40]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[41]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[7] += + acadoWorkspace.H10[42]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[43]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[44]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[45]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[46]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[47]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[8] += + acadoWorkspace.H10[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[51]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[52]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[53]*acadoWorkspace.Dx0[5];
acadoWorkspace.g[9] += + acadoWorkspace.H10[54]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[55]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[56]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[57]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[58]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[59]*acadoWorkspace.Dx0[5];

acadoWorkspace.lb[0] = acadoVariables.lbValues[0] - acadoVariables.u[0];
acadoWorkspace.lb[1] = acadoVariables.lbValues[1] - acadoVariables.u[1];
acadoWorkspace.lb[2] = acadoVariables.lbValues[2] - acadoVariables.u[2];
acadoWorkspace.lb[3] = acadoVariables.lbValues[3] - acadoVariables.u[3];
acadoWorkspace.lb[4] = acadoVariables.lbValues[4] - acadoVariables.u[4];
acadoWorkspace.lb[5] = acadoVariables.lbValues[5] - acadoVariables.u[5];
acadoWorkspace.lb[6] = acadoVariables.lbValues[6] - acadoVariables.u[6];
acadoWorkspace.lb[7] = acadoVariables.lbValues[7] - acadoVariables.u[7];
acadoWorkspace.lb[8] = acadoVariables.lbValues[8] - acadoVariables.u[8];
acadoWorkspace.lb[9] = acadoVariables.lbValues[9] - acadoVariables.u[9];
acadoWorkspace.ub[0] = acadoVariables.ubValues[0] - acadoVariables.u[0];
acadoWorkspace.ub[1] = acadoVariables.ubValues[1] - acadoVariables.u[1];
acadoWorkspace.ub[2] = acadoVariables.ubValues[2] - acadoVariables.u[2];
acadoWorkspace.ub[3] = acadoVariables.ubValues[3] - acadoVariables.u[3];
acadoWorkspace.ub[4] = acadoVariables.ubValues[4] - acadoVariables.u[4];
acadoWorkspace.ub[5] = acadoVariables.ubValues[5] - acadoVariables.u[5];
acadoWorkspace.ub[6] = acadoVariables.ubValues[6] - acadoVariables.u[6];
acadoWorkspace.ub[7] = acadoVariables.ubValues[7] - acadoVariables.u[7];
acadoWorkspace.ub[8] = acadoVariables.ubValues[8] - acadoVariables.u[8];
acadoWorkspace.ub[9] = acadoVariables.ubValues[9] - acadoVariables.u[9];

}

void acado_expand(  )
{
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];

acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];
acadoVariables.x[2] += acadoWorkspace.Dx0[2];
acadoVariables.x[3] += acadoWorkspace.Dx0[3];
acadoVariables.x[4] += acadoWorkspace.Dx0[4];
acadoVariables.x[5] += acadoWorkspace.Dx0[5];

acadoVariables.x[6] += + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[0];
acadoVariables.x[7] += + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[1];
acadoVariables.x[8] += + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[2];
acadoVariables.x[9] += + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[3];
acadoVariables.x[10] += + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[4];
acadoVariables.x[11] += + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[5];
acadoVariables.x[12] += + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[40]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[41]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[6];
acadoVariables.x[13] += + acadoWorkspace.evGx[42]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[43]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[7];
acadoVariables.x[14] += + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[51]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[52]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[53]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[8];
acadoVariables.x[15] += + acadoWorkspace.evGx[54]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[55]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[56]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[57]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[58]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[59]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[9];
acadoVariables.x[16] += + acadoWorkspace.evGx[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[64]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[10];
acadoVariables.x[17] += + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[11];
acadoVariables.x[18] += + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[75]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[76]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[77]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[12];
acadoVariables.x[19] += + acadoWorkspace.evGx[78]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[79]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[80]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[81]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[82]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[83]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[13];
acadoVariables.x[20] += + acadoWorkspace.evGx[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[87]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[88]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[89]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[14];
acadoVariables.x[21] += + acadoWorkspace.evGx[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[94]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[15];
acadoVariables.x[22] += + acadoWorkspace.evGx[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[99]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[100]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[101]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[16];
acadoVariables.x[23] += + acadoWorkspace.evGx[102]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[103]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[104]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[105]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[106]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[107]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[17];
acadoVariables.x[24] += + acadoWorkspace.evGx[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[111]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[112]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[113]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[18];
acadoVariables.x[25] += + acadoWorkspace.evGx[114]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[115]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[116]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[117]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[118]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[119]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[19];
acadoVariables.x[26] += + acadoWorkspace.evGx[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[124]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[125]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[20];
acadoVariables.x[27] += + acadoWorkspace.evGx[126]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[127]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[128]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[129]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[130]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[131]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[21];
acadoVariables.x[28] += + acadoWorkspace.evGx[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[134]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[135]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[136]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[137]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[22];
acadoVariables.x[29] += + acadoWorkspace.evGx[138]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[139]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[140]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[141]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[142]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[143]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[23];
acadoVariables.x[30] += + acadoWorkspace.evGx[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[147]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[148]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[149]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[24];
acadoVariables.x[31] += + acadoWorkspace.evGx[150]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[151]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[152]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[153]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[154]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[155]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[25];
acadoVariables.x[32] += + acadoWorkspace.evGx[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[158]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[159]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[160]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[161]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[26];
acadoVariables.x[33] += + acadoWorkspace.evGx[162]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[163]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[164]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[165]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[166]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[167]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[27];
acadoVariables.x[34] += + acadoWorkspace.evGx[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[171]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[172]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[173]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[28];
acadoVariables.x[35] += + acadoWorkspace.evGx[174]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[175]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[176]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[177]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[178]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[179]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[29];
acadoVariables.x[36] += + acadoWorkspace.evGx[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[184]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[185]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[30];
acadoVariables.x[37] += + acadoWorkspace.evGx[186]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[187]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[188]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[189]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[190]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[191]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[31];
acadoVariables.x[38] += + acadoWorkspace.evGx[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[194]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[195]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[196]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[197]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[32];
acadoVariables.x[39] += + acadoWorkspace.evGx[198]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[199]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[200]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[201]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[202]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[203]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[33];
acadoVariables.x[40] += + acadoWorkspace.evGx[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[206]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[207]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[208]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[209]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[34];
acadoVariables.x[41] += + acadoWorkspace.evGx[210]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[211]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[212]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[213]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[214]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[215]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[35];
acadoVariables.x[42] += + acadoWorkspace.evGx[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[219]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[220]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[221]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[36];
acadoVariables.x[43] += + acadoWorkspace.evGx[222]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[223]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[224]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[225]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[226]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[227]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[37];
acadoVariables.x[44] += + acadoWorkspace.evGx[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[230]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[231]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[232]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[233]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[38];
acadoVariables.x[45] += + acadoWorkspace.evGx[234]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[235]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[236]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[237]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[238]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[239]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[39];
acadoVariables.x[46] += + acadoWorkspace.evGx[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[244]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[245]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[40];
acadoVariables.x[47] += + acadoWorkspace.evGx[246]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[247]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[248]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[249]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[250]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[251]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[41];
acadoVariables.x[48] += + acadoWorkspace.evGx[252]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[253]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[254]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[255]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[256]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[257]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[42];
acadoVariables.x[49] += + acadoWorkspace.evGx[258]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[259]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[260]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[261]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[262]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[263]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[43];
acadoVariables.x[50] += + acadoWorkspace.evGx[264]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[265]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[266]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[267]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[268]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[269]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[44];
acadoVariables.x[51] += + acadoWorkspace.evGx[270]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[271]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[272]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[273]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[274]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[275]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[45];
acadoVariables.x[52] += + acadoWorkspace.evGx[276]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[277]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[278]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[279]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[280]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[281]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[46];
acadoVariables.x[53] += + acadoWorkspace.evGx[282]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[283]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[284]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[285]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[286]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[287]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[47];
acadoVariables.x[54] += + acadoWorkspace.evGx[288]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[289]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[290]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[291]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[292]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[293]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[48];
acadoVariables.x[55] += + acadoWorkspace.evGx[294]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[295]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[296]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[297]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[298]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[299]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[49];
acadoVariables.x[56] += + acadoWorkspace.evGx[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[303]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[304]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[305]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[50];
acadoVariables.x[57] += + acadoWorkspace.evGx[306]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[307]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[308]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[309]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[310]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[311]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[51];
acadoVariables.x[58] += + acadoWorkspace.evGx[312]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[313]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[314]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[315]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[316]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[317]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[52];
acadoVariables.x[59] += + acadoWorkspace.evGx[318]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[319]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[320]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[321]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[322]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[323]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[53];
acadoVariables.x[60] += + acadoWorkspace.evGx[324]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[325]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[326]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[327]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[328]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[329]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[54];
acadoVariables.x[61] += + acadoWorkspace.evGx[330]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[331]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[332]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[333]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[334]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[335]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[55];
acadoVariables.x[62] += + acadoWorkspace.evGx[336]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[337]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[338]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[339]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[340]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[341]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[56];
acadoVariables.x[63] += + acadoWorkspace.evGx[342]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[343]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[344]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[345]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[346]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[347]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[57];
acadoVariables.x[64] += + acadoWorkspace.evGx[348]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[349]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[350]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[351]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[352]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[353]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[58];
acadoVariables.x[65] += + acadoWorkspace.evGx[354]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[355]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[356]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[357]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[358]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[359]*acadoWorkspace.Dx0[5] + acadoWorkspace.d[59];

acado_multEDu( acadoWorkspace.E, acadoWorkspace.x, &(acadoVariables.x[ 6 ]) );
acado_multEDu( &(acadoWorkspace.E[ 6 ]), acadoWorkspace.x, &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 18 ]), acadoWorkspace.x, &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 36 ]), acadoWorkspace.x, &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 60 ]), acadoWorkspace.x, &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 90 ]), acadoWorkspace.x, &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 126 ]), acadoWorkspace.x, &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 162 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 168 ]), acadoWorkspace.x, &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 186 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.x[ 7 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 216 ]), acadoWorkspace.x, &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 258 ]), &(acadoWorkspace.x[ 7 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 270 ]), acadoWorkspace.x, &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 306 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.x[ 7 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 60 ]) );
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
acadoVariables.lbValues[0] = -2.0000000000000000e+01;
acadoVariables.lbValues[1] = -2.0000000000000000e+01;
acadoVariables.lbValues[2] = -2.0000000000000000e+01;
acadoVariables.lbValues[3] = -2.0000000000000000e+01;
acadoVariables.lbValues[4] = -2.0000000000000000e+01;
acadoVariables.lbValues[5] = -2.0000000000000000e+01;
acadoVariables.lbValues[6] = -2.0000000000000000e+01;
acadoVariables.lbValues[7] = -2.0000000000000000e+01;
acadoVariables.lbValues[8] = -2.0000000000000000e+01;
acadoVariables.lbValues[9] = -2.0000000000000000e+01;
acadoVariables.ubValues[0] = 2.0000000000000000e+01;
acadoVariables.ubValues[1] = 2.0000000000000000e+01;
acadoVariables.ubValues[2] = 2.0000000000000000e+01;
acadoVariables.ubValues[3] = 2.0000000000000000e+01;
acadoVariables.ubValues[4] = 2.0000000000000000e+01;
acadoVariables.ubValues[5] = 2.0000000000000000e+01;
acadoVariables.ubValues[6] = 2.0000000000000000e+01;
acadoVariables.ubValues[7] = 2.0000000000000000e+01;
acadoVariables.ubValues[8] = 2.0000000000000000e+01;
acadoVariables.ubValues[9] = 2.0000000000000000e+01;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 6];
acadoWorkspace.state[1] = acadoVariables.x[index * 6 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 6 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 6 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 6 + 4];
acadoWorkspace.state[5] = acadoVariables.x[index * 6 + 5];
if (index > 0){acadoWorkspace.state[6] = acadoVariables.z[index-1];
}
acadoWorkspace.state[56] = acadoVariables.u[index];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 6 + 6] = acadoWorkspace.state[0];
acadoVariables.x[index * 6 + 7] = acadoWorkspace.state[1];
acadoVariables.x[index * 6 + 8] = acadoWorkspace.state[2];
acadoVariables.x[index * 6 + 9] = acadoWorkspace.state[3];
acadoVariables.x[index * 6 + 10] = acadoWorkspace.state[4];
acadoVariables.x[index * 6 + 11] = acadoWorkspace.state[5];
acadoVariables.z[index] = acadoWorkspace.state[6];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoVariables.x[index * 6] = acadoVariables.x[index * 6 + 6];
acadoVariables.x[index * 6 + 1] = acadoVariables.x[index * 6 + 7];
acadoVariables.x[index * 6 + 2] = acadoVariables.x[index * 6 + 8];
acadoVariables.x[index * 6 + 3] = acadoVariables.x[index * 6 + 9];
acadoVariables.x[index * 6 + 4] = acadoVariables.x[index * 6 + 10];
acadoVariables.x[index * 6 + 5] = acadoVariables.x[index * 6 + 11];
}
for (index = 0; index < 9; ++index)
{
acadoVariables.z[index] = acadoVariables.z[index + 1];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[60] = xEnd[0];
acadoVariables.x[61] = xEnd[1];
acadoVariables.x[62] = xEnd[2];
acadoVariables.x[63] = xEnd[3];
acadoVariables.x[64] = xEnd[4];
acadoVariables.x[65] = xEnd[5];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[60];
acadoWorkspace.state[1] = acadoVariables.x[61];
acadoWorkspace.state[2] = acadoVariables.x[62];
acadoWorkspace.state[3] = acadoVariables.x[63];
acadoWorkspace.state[4] = acadoVariables.x[64];
acadoWorkspace.state[5] = acadoVariables.x[65];
acadoWorkspace.state[6] = acadoVariables.z[9];
if (uEnd != 0)
{
acadoWorkspace.state[56] = uEnd[0];
}
else
{
acadoWorkspace.state[56] = acadoVariables.u[9];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[60] = acadoWorkspace.state[0];
acadoVariables.x[61] = acadoWorkspace.state[1];
acadoVariables.x[62] = acadoWorkspace.state[2];
acadoVariables.x[63] = acadoWorkspace.state[3];
acadoVariables.x[64] = acadoWorkspace.state[4];
acadoVariables.x[65] = acadoWorkspace.state[5];

acadoVariables.z[9] = acadoWorkspace.state[6];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 9; ++index)
{
acadoVariables.u[index] = acadoVariables.u[index + 1];
}

if (uEnd != 0)
{
acadoVariables.u[9] = uEnd[0];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9];
kkt = fabs( kkt );
for (index = 0; index < 10; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 7 */
real_t tmpDy[ 7 ];

/** Row vector of size: 6 */
real_t tmpDyN[ 6 ];

for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 6];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 6 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 6 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 6 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 6 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 6 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.u[lRun1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 7] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 7];
acadoWorkspace.Dy[lRun1 * 7 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 7 + 1];
acadoWorkspace.Dy[lRun1 * 7 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 7 + 2];
acadoWorkspace.Dy[lRun1 * 7 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 7 + 3];
acadoWorkspace.Dy[lRun1 * 7 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 7 + 4];
acadoWorkspace.Dy[lRun1 * 7 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 7 + 5];
acadoWorkspace.Dy[lRun1 * 7 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 7 + 6];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[60];
acadoWorkspace.objValueIn[1] = acadoVariables.x[61];
acadoWorkspace.objValueIn[2] = acadoVariables.x[62];
acadoWorkspace.objValueIn[3] = acadoVariables.x[63];
acadoWorkspace.objValueIn[4] = acadoVariables.x[64];
acadoWorkspace.objValueIn[5] = acadoVariables.x[65];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4] - acadoVariables.yN[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5] - acadoVariables.yN[5];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 7];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 7 + 1];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 7 + 2];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 7 + 3];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 7 + 4];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 7 + 5];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 7 + 6];
objVal += + acadoWorkspace.Dy[lRun1 * 7]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 7 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 7 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 7 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 7 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 7 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 7 + 6]*tmpDy[6];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*(real_t)1.0000000000000000e+01;
tmpDyN[1] = + acadoWorkspace.DyN[1]*(real_t)1.0000000000000000e+01;
tmpDyN[2] = + acadoWorkspace.DyN[2]*(real_t)1.0000000000000000e+01;
tmpDyN[3] = + acadoWorkspace.DyN[3]*(real_t)1.0000000000000000e+01;
tmpDyN[4] = + acadoWorkspace.DyN[4]*(real_t)1.0000000000000000e+01;
tmpDyN[5] = + acadoWorkspace.DyN[5]*(real_t)1.0000000000000000e+01;
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3] + acadoWorkspace.DyN[4]*tmpDyN[4] + acadoWorkspace.DyN[5]*tmpDyN[5];

objVal *= 0.5;
return objVal;
}

