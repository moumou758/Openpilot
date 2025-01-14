#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2081385545714313315) {
   out_2081385545714313315[0] = delta_x[0] + nom_x[0];
   out_2081385545714313315[1] = delta_x[1] + nom_x[1];
   out_2081385545714313315[2] = delta_x[2] + nom_x[2];
   out_2081385545714313315[3] = delta_x[3] + nom_x[3];
   out_2081385545714313315[4] = delta_x[4] + nom_x[4];
   out_2081385545714313315[5] = delta_x[5] + nom_x[5];
   out_2081385545714313315[6] = delta_x[6] + nom_x[6];
   out_2081385545714313315[7] = delta_x[7] + nom_x[7];
   out_2081385545714313315[8] = delta_x[8] + nom_x[8];
   out_2081385545714313315[9] = delta_x[9] + nom_x[9];
   out_2081385545714313315[10] = delta_x[10] + nom_x[10];
   out_2081385545714313315[11] = delta_x[11] + nom_x[11];
   out_2081385545714313315[12] = delta_x[12] + nom_x[12];
   out_2081385545714313315[13] = delta_x[13] + nom_x[13];
   out_2081385545714313315[14] = delta_x[14] + nom_x[14];
   out_2081385545714313315[15] = delta_x[15] + nom_x[15];
   out_2081385545714313315[16] = delta_x[16] + nom_x[16];
   out_2081385545714313315[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1086297426282480324) {
   out_1086297426282480324[0] = -nom_x[0] + true_x[0];
   out_1086297426282480324[1] = -nom_x[1] + true_x[1];
   out_1086297426282480324[2] = -nom_x[2] + true_x[2];
   out_1086297426282480324[3] = -nom_x[3] + true_x[3];
   out_1086297426282480324[4] = -nom_x[4] + true_x[4];
   out_1086297426282480324[5] = -nom_x[5] + true_x[5];
   out_1086297426282480324[6] = -nom_x[6] + true_x[6];
   out_1086297426282480324[7] = -nom_x[7] + true_x[7];
   out_1086297426282480324[8] = -nom_x[8] + true_x[8];
   out_1086297426282480324[9] = -nom_x[9] + true_x[9];
   out_1086297426282480324[10] = -nom_x[10] + true_x[10];
   out_1086297426282480324[11] = -nom_x[11] + true_x[11];
   out_1086297426282480324[12] = -nom_x[12] + true_x[12];
   out_1086297426282480324[13] = -nom_x[13] + true_x[13];
   out_1086297426282480324[14] = -nom_x[14] + true_x[14];
   out_1086297426282480324[15] = -nom_x[15] + true_x[15];
   out_1086297426282480324[16] = -nom_x[16] + true_x[16];
   out_1086297426282480324[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_8714192956250408026) {
   out_8714192956250408026[0] = 1.0;
   out_8714192956250408026[1] = 0.0;
   out_8714192956250408026[2] = 0.0;
   out_8714192956250408026[3] = 0.0;
   out_8714192956250408026[4] = 0.0;
   out_8714192956250408026[5] = 0.0;
   out_8714192956250408026[6] = 0.0;
   out_8714192956250408026[7] = 0.0;
   out_8714192956250408026[8] = 0.0;
   out_8714192956250408026[9] = 0.0;
   out_8714192956250408026[10] = 0.0;
   out_8714192956250408026[11] = 0.0;
   out_8714192956250408026[12] = 0.0;
   out_8714192956250408026[13] = 0.0;
   out_8714192956250408026[14] = 0.0;
   out_8714192956250408026[15] = 0.0;
   out_8714192956250408026[16] = 0.0;
   out_8714192956250408026[17] = 0.0;
   out_8714192956250408026[18] = 0.0;
   out_8714192956250408026[19] = 1.0;
   out_8714192956250408026[20] = 0.0;
   out_8714192956250408026[21] = 0.0;
   out_8714192956250408026[22] = 0.0;
   out_8714192956250408026[23] = 0.0;
   out_8714192956250408026[24] = 0.0;
   out_8714192956250408026[25] = 0.0;
   out_8714192956250408026[26] = 0.0;
   out_8714192956250408026[27] = 0.0;
   out_8714192956250408026[28] = 0.0;
   out_8714192956250408026[29] = 0.0;
   out_8714192956250408026[30] = 0.0;
   out_8714192956250408026[31] = 0.0;
   out_8714192956250408026[32] = 0.0;
   out_8714192956250408026[33] = 0.0;
   out_8714192956250408026[34] = 0.0;
   out_8714192956250408026[35] = 0.0;
   out_8714192956250408026[36] = 0.0;
   out_8714192956250408026[37] = 0.0;
   out_8714192956250408026[38] = 1.0;
   out_8714192956250408026[39] = 0.0;
   out_8714192956250408026[40] = 0.0;
   out_8714192956250408026[41] = 0.0;
   out_8714192956250408026[42] = 0.0;
   out_8714192956250408026[43] = 0.0;
   out_8714192956250408026[44] = 0.0;
   out_8714192956250408026[45] = 0.0;
   out_8714192956250408026[46] = 0.0;
   out_8714192956250408026[47] = 0.0;
   out_8714192956250408026[48] = 0.0;
   out_8714192956250408026[49] = 0.0;
   out_8714192956250408026[50] = 0.0;
   out_8714192956250408026[51] = 0.0;
   out_8714192956250408026[52] = 0.0;
   out_8714192956250408026[53] = 0.0;
   out_8714192956250408026[54] = 0.0;
   out_8714192956250408026[55] = 0.0;
   out_8714192956250408026[56] = 0.0;
   out_8714192956250408026[57] = 1.0;
   out_8714192956250408026[58] = 0.0;
   out_8714192956250408026[59] = 0.0;
   out_8714192956250408026[60] = 0.0;
   out_8714192956250408026[61] = 0.0;
   out_8714192956250408026[62] = 0.0;
   out_8714192956250408026[63] = 0.0;
   out_8714192956250408026[64] = 0.0;
   out_8714192956250408026[65] = 0.0;
   out_8714192956250408026[66] = 0.0;
   out_8714192956250408026[67] = 0.0;
   out_8714192956250408026[68] = 0.0;
   out_8714192956250408026[69] = 0.0;
   out_8714192956250408026[70] = 0.0;
   out_8714192956250408026[71] = 0.0;
   out_8714192956250408026[72] = 0.0;
   out_8714192956250408026[73] = 0.0;
   out_8714192956250408026[74] = 0.0;
   out_8714192956250408026[75] = 0.0;
   out_8714192956250408026[76] = 1.0;
   out_8714192956250408026[77] = 0.0;
   out_8714192956250408026[78] = 0.0;
   out_8714192956250408026[79] = 0.0;
   out_8714192956250408026[80] = 0.0;
   out_8714192956250408026[81] = 0.0;
   out_8714192956250408026[82] = 0.0;
   out_8714192956250408026[83] = 0.0;
   out_8714192956250408026[84] = 0.0;
   out_8714192956250408026[85] = 0.0;
   out_8714192956250408026[86] = 0.0;
   out_8714192956250408026[87] = 0.0;
   out_8714192956250408026[88] = 0.0;
   out_8714192956250408026[89] = 0.0;
   out_8714192956250408026[90] = 0.0;
   out_8714192956250408026[91] = 0.0;
   out_8714192956250408026[92] = 0.0;
   out_8714192956250408026[93] = 0.0;
   out_8714192956250408026[94] = 0.0;
   out_8714192956250408026[95] = 1.0;
   out_8714192956250408026[96] = 0.0;
   out_8714192956250408026[97] = 0.0;
   out_8714192956250408026[98] = 0.0;
   out_8714192956250408026[99] = 0.0;
   out_8714192956250408026[100] = 0.0;
   out_8714192956250408026[101] = 0.0;
   out_8714192956250408026[102] = 0.0;
   out_8714192956250408026[103] = 0.0;
   out_8714192956250408026[104] = 0.0;
   out_8714192956250408026[105] = 0.0;
   out_8714192956250408026[106] = 0.0;
   out_8714192956250408026[107] = 0.0;
   out_8714192956250408026[108] = 0.0;
   out_8714192956250408026[109] = 0.0;
   out_8714192956250408026[110] = 0.0;
   out_8714192956250408026[111] = 0.0;
   out_8714192956250408026[112] = 0.0;
   out_8714192956250408026[113] = 0.0;
   out_8714192956250408026[114] = 1.0;
   out_8714192956250408026[115] = 0.0;
   out_8714192956250408026[116] = 0.0;
   out_8714192956250408026[117] = 0.0;
   out_8714192956250408026[118] = 0.0;
   out_8714192956250408026[119] = 0.0;
   out_8714192956250408026[120] = 0.0;
   out_8714192956250408026[121] = 0.0;
   out_8714192956250408026[122] = 0.0;
   out_8714192956250408026[123] = 0.0;
   out_8714192956250408026[124] = 0.0;
   out_8714192956250408026[125] = 0.0;
   out_8714192956250408026[126] = 0.0;
   out_8714192956250408026[127] = 0.0;
   out_8714192956250408026[128] = 0.0;
   out_8714192956250408026[129] = 0.0;
   out_8714192956250408026[130] = 0.0;
   out_8714192956250408026[131] = 0.0;
   out_8714192956250408026[132] = 0.0;
   out_8714192956250408026[133] = 1.0;
   out_8714192956250408026[134] = 0.0;
   out_8714192956250408026[135] = 0.0;
   out_8714192956250408026[136] = 0.0;
   out_8714192956250408026[137] = 0.0;
   out_8714192956250408026[138] = 0.0;
   out_8714192956250408026[139] = 0.0;
   out_8714192956250408026[140] = 0.0;
   out_8714192956250408026[141] = 0.0;
   out_8714192956250408026[142] = 0.0;
   out_8714192956250408026[143] = 0.0;
   out_8714192956250408026[144] = 0.0;
   out_8714192956250408026[145] = 0.0;
   out_8714192956250408026[146] = 0.0;
   out_8714192956250408026[147] = 0.0;
   out_8714192956250408026[148] = 0.0;
   out_8714192956250408026[149] = 0.0;
   out_8714192956250408026[150] = 0.0;
   out_8714192956250408026[151] = 0.0;
   out_8714192956250408026[152] = 1.0;
   out_8714192956250408026[153] = 0.0;
   out_8714192956250408026[154] = 0.0;
   out_8714192956250408026[155] = 0.0;
   out_8714192956250408026[156] = 0.0;
   out_8714192956250408026[157] = 0.0;
   out_8714192956250408026[158] = 0.0;
   out_8714192956250408026[159] = 0.0;
   out_8714192956250408026[160] = 0.0;
   out_8714192956250408026[161] = 0.0;
   out_8714192956250408026[162] = 0.0;
   out_8714192956250408026[163] = 0.0;
   out_8714192956250408026[164] = 0.0;
   out_8714192956250408026[165] = 0.0;
   out_8714192956250408026[166] = 0.0;
   out_8714192956250408026[167] = 0.0;
   out_8714192956250408026[168] = 0.0;
   out_8714192956250408026[169] = 0.0;
   out_8714192956250408026[170] = 0.0;
   out_8714192956250408026[171] = 1.0;
   out_8714192956250408026[172] = 0.0;
   out_8714192956250408026[173] = 0.0;
   out_8714192956250408026[174] = 0.0;
   out_8714192956250408026[175] = 0.0;
   out_8714192956250408026[176] = 0.0;
   out_8714192956250408026[177] = 0.0;
   out_8714192956250408026[178] = 0.0;
   out_8714192956250408026[179] = 0.0;
   out_8714192956250408026[180] = 0.0;
   out_8714192956250408026[181] = 0.0;
   out_8714192956250408026[182] = 0.0;
   out_8714192956250408026[183] = 0.0;
   out_8714192956250408026[184] = 0.0;
   out_8714192956250408026[185] = 0.0;
   out_8714192956250408026[186] = 0.0;
   out_8714192956250408026[187] = 0.0;
   out_8714192956250408026[188] = 0.0;
   out_8714192956250408026[189] = 0.0;
   out_8714192956250408026[190] = 1.0;
   out_8714192956250408026[191] = 0.0;
   out_8714192956250408026[192] = 0.0;
   out_8714192956250408026[193] = 0.0;
   out_8714192956250408026[194] = 0.0;
   out_8714192956250408026[195] = 0.0;
   out_8714192956250408026[196] = 0.0;
   out_8714192956250408026[197] = 0.0;
   out_8714192956250408026[198] = 0.0;
   out_8714192956250408026[199] = 0.0;
   out_8714192956250408026[200] = 0.0;
   out_8714192956250408026[201] = 0.0;
   out_8714192956250408026[202] = 0.0;
   out_8714192956250408026[203] = 0.0;
   out_8714192956250408026[204] = 0.0;
   out_8714192956250408026[205] = 0.0;
   out_8714192956250408026[206] = 0.0;
   out_8714192956250408026[207] = 0.0;
   out_8714192956250408026[208] = 0.0;
   out_8714192956250408026[209] = 1.0;
   out_8714192956250408026[210] = 0.0;
   out_8714192956250408026[211] = 0.0;
   out_8714192956250408026[212] = 0.0;
   out_8714192956250408026[213] = 0.0;
   out_8714192956250408026[214] = 0.0;
   out_8714192956250408026[215] = 0.0;
   out_8714192956250408026[216] = 0.0;
   out_8714192956250408026[217] = 0.0;
   out_8714192956250408026[218] = 0.0;
   out_8714192956250408026[219] = 0.0;
   out_8714192956250408026[220] = 0.0;
   out_8714192956250408026[221] = 0.0;
   out_8714192956250408026[222] = 0.0;
   out_8714192956250408026[223] = 0.0;
   out_8714192956250408026[224] = 0.0;
   out_8714192956250408026[225] = 0.0;
   out_8714192956250408026[226] = 0.0;
   out_8714192956250408026[227] = 0.0;
   out_8714192956250408026[228] = 1.0;
   out_8714192956250408026[229] = 0.0;
   out_8714192956250408026[230] = 0.0;
   out_8714192956250408026[231] = 0.0;
   out_8714192956250408026[232] = 0.0;
   out_8714192956250408026[233] = 0.0;
   out_8714192956250408026[234] = 0.0;
   out_8714192956250408026[235] = 0.0;
   out_8714192956250408026[236] = 0.0;
   out_8714192956250408026[237] = 0.0;
   out_8714192956250408026[238] = 0.0;
   out_8714192956250408026[239] = 0.0;
   out_8714192956250408026[240] = 0.0;
   out_8714192956250408026[241] = 0.0;
   out_8714192956250408026[242] = 0.0;
   out_8714192956250408026[243] = 0.0;
   out_8714192956250408026[244] = 0.0;
   out_8714192956250408026[245] = 0.0;
   out_8714192956250408026[246] = 0.0;
   out_8714192956250408026[247] = 1.0;
   out_8714192956250408026[248] = 0.0;
   out_8714192956250408026[249] = 0.0;
   out_8714192956250408026[250] = 0.0;
   out_8714192956250408026[251] = 0.0;
   out_8714192956250408026[252] = 0.0;
   out_8714192956250408026[253] = 0.0;
   out_8714192956250408026[254] = 0.0;
   out_8714192956250408026[255] = 0.0;
   out_8714192956250408026[256] = 0.0;
   out_8714192956250408026[257] = 0.0;
   out_8714192956250408026[258] = 0.0;
   out_8714192956250408026[259] = 0.0;
   out_8714192956250408026[260] = 0.0;
   out_8714192956250408026[261] = 0.0;
   out_8714192956250408026[262] = 0.0;
   out_8714192956250408026[263] = 0.0;
   out_8714192956250408026[264] = 0.0;
   out_8714192956250408026[265] = 0.0;
   out_8714192956250408026[266] = 1.0;
   out_8714192956250408026[267] = 0.0;
   out_8714192956250408026[268] = 0.0;
   out_8714192956250408026[269] = 0.0;
   out_8714192956250408026[270] = 0.0;
   out_8714192956250408026[271] = 0.0;
   out_8714192956250408026[272] = 0.0;
   out_8714192956250408026[273] = 0.0;
   out_8714192956250408026[274] = 0.0;
   out_8714192956250408026[275] = 0.0;
   out_8714192956250408026[276] = 0.0;
   out_8714192956250408026[277] = 0.0;
   out_8714192956250408026[278] = 0.0;
   out_8714192956250408026[279] = 0.0;
   out_8714192956250408026[280] = 0.0;
   out_8714192956250408026[281] = 0.0;
   out_8714192956250408026[282] = 0.0;
   out_8714192956250408026[283] = 0.0;
   out_8714192956250408026[284] = 0.0;
   out_8714192956250408026[285] = 1.0;
   out_8714192956250408026[286] = 0.0;
   out_8714192956250408026[287] = 0.0;
   out_8714192956250408026[288] = 0.0;
   out_8714192956250408026[289] = 0.0;
   out_8714192956250408026[290] = 0.0;
   out_8714192956250408026[291] = 0.0;
   out_8714192956250408026[292] = 0.0;
   out_8714192956250408026[293] = 0.0;
   out_8714192956250408026[294] = 0.0;
   out_8714192956250408026[295] = 0.0;
   out_8714192956250408026[296] = 0.0;
   out_8714192956250408026[297] = 0.0;
   out_8714192956250408026[298] = 0.0;
   out_8714192956250408026[299] = 0.0;
   out_8714192956250408026[300] = 0.0;
   out_8714192956250408026[301] = 0.0;
   out_8714192956250408026[302] = 0.0;
   out_8714192956250408026[303] = 0.0;
   out_8714192956250408026[304] = 1.0;
   out_8714192956250408026[305] = 0.0;
   out_8714192956250408026[306] = 0.0;
   out_8714192956250408026[307] = 0.0;
   out_8714192956250408026[308] = 0.0;
   out_8714192956250408026[309] = 0.0;
   out_8714192956250408026[310] = 0.0;
   out_8714192956250408026[311] = 0.0;
   out_8714192956250408026[312] = 0.0;
   out_8714192956250408026[313] = 0.0;
   out_8714192956250408026[314] = 0.0;
   out_8714192956250408026[315] = 0.0;
   out_8714192956250408026[316] = 0.0;
   out_8714192956250408026[317] = 0.0;
   out_8714192956250408026[318] = 0.0;
   out_8714192956250408026[319] = 0.0;
   out_8714192956250408026[320] = 0.0;
   out_8714192956250408026[321] = 0.0;
   out_8714192956250408026[322] = 0.0;
   out_8714192956250408026[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_5560781299359714526) {
   out_5560781299359714526[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_5560781299359714526[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_5560781299359714526[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_5560781299359714526[3] = dt*state[12] + state[3];
   out_5560781299359714526[4] = dt*state[13] + state[4];
   out_5560781299359714526[5] = dt*state[14] + state[5];
   out_5560781299359714526[6] = state[6];
   out_5560781299359714526[7] = state[7];
   out_5560781299359714526[8] = state[8];
   out_5560781299359714526[9] = state[9];
   out_5560781299359714526[10] = state[10];
   out_5560781299359714526[11] = state[11];
   out_5560781299359714526[12] = state[12];
   out_5560781299359714526[13] = state[13];
   out_5560781299359714526[14] = state[14];
   out_5560781299359714526[15] = state[15];
   out_5560781299359714526[16] = state[16];
   out_5560781299359714526[17] = state[17];
}
void F_fun(double *state, double dt, double *out_2772014030390626314) {
   out_2772014030390626314[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2772014030390626314[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2772014030390626314[2] = 0;
   out_2772014030390626314[3] = 0;
   out_2772014030390626314[4] = 0;
   out_2772014030390626314[5] = 0;
   out_2772014030390626314[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2772014030390626314[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2772014030390626314[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_2772014030390626314[9] = 0;
   out_2772014030390626314[10] = 0;
   out_2772014030390626314[11] = 0;
   out_2772014030390626314[12] = 0;
   out_2772014030390626314[13] = 0;
   out_2772014030390626314[14] = 0;
   out_2772014030390626314[15] = 0;
   out_2772014030390626314[16] = 0;
   out_2772014030390626314[17] = 0;
   out_2772014030390626314[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2772014030390626314[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2772014030390626314[20] = 0;
   out_2772014030390626314[21] = 0;
   out_2772014030390626314[22] = 0;
   out_2772014030390626314[23] = 0;
   out_2772014030390626314[24] = 0;
   out_2772014030390626314[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2772014030390626314[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_2772014030390626314[27] = 0;
   out_2772014030390626314[28] = 0;
   out_2772014030390626314[29] = 0;
   out_2772014030390626314[30] = 0;
   out_2772014030390626314[31] = 0;
   out_2772014030390626314[32] = 0;
   out_2772014030390626314[33] = 0;
   out_2772014030390626314[34] = 0;
   out_2772014030390626314[35] = 0;
   out_2772014030390626314[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2772014030390626314[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2772014030390626314[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2772014030390626314[39] = 0;
   out_2772014030390626314[40] = 0;
   out_2772014030390626314[41] = 0;
   out_2772014030390626314[42] = 0;
   out_2772014030390626314[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2772014030390626314[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_2772014030390626314[45] = 0;
   out_2772014030390626314[46] = 0;
   out_2772014030390626314[47] = 0;
   out_2772014030390626314[48] = 0;
   out_2772014030390626314[49] = 0;
   out_2772014030390626314[50] = 0;
   out_2772014030390626314[51] = 0;
   out_2772014030390626314[52] = 0;
   out_2772014030390626314[53] = 0;
   out_2772014030390626314[54] = 0;
   out_2772014030390626314[55] = 0;
   out_2772014030390626314[56] = 0;
   out_2772014030390626314[57] = 1;
   out_2772014030390626314[58] = 0;
   out_2772014030390626314[59] = 0;
   out_2772014030390626314[60] = 0;
   out_2772014030390626314[61] = 0;
   out_2772014030390626314[62] = 0;
   out_2772014030390626314[63] = 0;
   out_2772014030390626314[64] = 0;
   out_2772014030390626314[65] = 0;
   out_2772014030390626314[66] = dt;
   out_2772014030390626314[67] = 0;
   out_2772014030390626314[68] = 0;
   out_2772014030390626314[69] = 0;
   out_2772014030390626314[70] = 0;
   out_2772014030390626314[71] = 0;
   out_2772014030390626314[72] = 0;
   out_2772014030390626314[73] = 0;
   out_2772014030390626314[74] = 0;
   out_2772014030390626314[75] = 0;
   out_2772014030390626314[76] = 1;
   out_2772014030390626314[77] = 0;
   out_2772014030390626314[78] = 0;
   out_2772014030390626314[79] = 0;
   out_2772014030390626314[80] = 0;
   out_2772014030390626314[81] = 0;
   out_2772014030390626314[82] = 0;
   out_2772014030390626314[83] = 0;
   out_2772014030390626314[84] = 0;
   out_2772014030390626314[85] = dt;
   out_2772014030390626314[86] = 0;
   out_2772014030390626314[87] = 0;
   out_2772014030390626314[88] = 0;
   out_2772014030390626314[89] = 0;
   out_2772014030390626314[90] = 0;
   out_2772014030390626314[91] = 0;
   out_2772014030390626314[92] = 0;
   out_2772014030390626314[93] = 0;
   out_2772014030390626314[94] = 0;
   out_2772014030390626314[95] = 1;
   out_2772014030390626314[96] = 0;
   out_2772014030390626314[97] = 0;
   out_2772014030390626314[98] = 0;
   out_2772014030390626314[99] = 0;
   out_2772014030390626314[100] = 0;
   out_2772014030390626314[101] = 0;
   out_2772014030390626314[102] = 0;
   out_2772014030390626314[103] = 0;
   out_2772014030390626314[104] = dt;
   out_2772014030390626314[105] = 0;
   out_2772014030390626314[106] = 0;
   out_2772014030390626314[107] = 0;
   out_2772014030390626314[108] = 0;
   out_2772014030390626314[109] = 0;
   out_2772014030390626314[110] = 0;
   out_2772014030390626314[111] = 0;
   out_2772014030390626314[112] = 0;
   out_2772014030390626314[113] = 0;
   out_2772014030390626314[114] = 1;
   out_2772014030390626314[115] = 0;
   out_2772014030390626314[116] = 0;
   out_2772014030390626314[117] = 0;
   out_2772014030390626314[118] = 0;
   out_2772014030390626314[119] = 0;
   out_2772014030390626314[120] = 0;
   out_2772014030390626314[121] = 0;
   out_2772014030390626314[122] = 0;
   out_2772014030390626314[123] = 0;
   out_2772014030390626314[124] = 0;
   out_2772014030390626314[125] = 0;
   out_2772014030390626314[126] = 0;
   out_2772014030390626314[127] = 0;
   out_2772014030390626314[128] = 0;
   out_2772014030390626314[129] = 0;
   out_2772014030390626314[130] = 0;
   out_2772014030390626314[131] = 0;
   out_2772014030390626314[132] = 0;
   out_2772014030390626314[133] = 1;
   out_2772014030390626314[134] = 0;
   out_2772014030390626314[135] = 0;
   out_2772014030390626314[136] = 0;
   out_2772014030390626314[137] = 0;
   out_2772014030390626314[138] = 0;
   out_2772014030390626314[139] = 0;
   out_2772014030390626314[140] = 0;
   out_2772014030390626314[141] = 0;
   out_2772014030390626314[142] = 0;
   out_2772014030390626314[143] = 0;
   out_2772014030390626314[144] = 0;
   out_2772014030390626314[145] = 0;
   out_2772014030390626314[146] = 0;
   out_2772014030390626314[147] = 0;
   out_2772014030390626314[148] = 0;
   out_2772014030390626314[149] = 0;
   out_2772014030390626314[150] = 0;
   out_2772014030390626314[151] = 0;
   out_2772014030390626314[152] = 1;
   out_2772014030390626314[153] = 0;
   out_2772014030390626314[154] = 0;
   out_2772014030390626314[155] = 0;
   out_2772014030390626314[156] = 0;
   out_2772014030390626314[157] = 0;
   out_2772014030390626314[158] = 0;
   out_2772014030390626314[159] = 0;
   out_2772014030390626314[160] = 0;
   out_2772014030390626314[161] = 0;
   out_2772014030390626314[162] = 0;
   out_2772014030390626314[163] = 0;
   out_2772014030390626314[164] = 0;
   out_2772014030390626314[165] = 0;
   out_2772014030390626314[166] = 0;
   out_2772014030390626314[167] = 0;
   out_2772014030390626314[168] = 0;
   out_2772014030390626314[169] = 0;
   out_2772014030390626314[170] = 0;
   out_2772014030390626314[171] = 1;
   out_2772014030390626314[172] = 0;
   out_2772014030390626314[173] = 0;
   out_2772014030390626314[174] = 0;
   out_2772014030390626314[175] = 0;
   out_2772014030390626314[176] = 0;
   out_2772014030390626314[177] = 0;
   out_2772014030390626314[178] = 0;
   out_2772014030390626314[179] = 0;
   out_2772014030390626314[180] = 0;
   out_2772014030390626314[181] = 0;
   out_2772014030390626314[182] = 0;
   out_2772014030390626314[183] = 0;
   out_2772014030390626314[184] = 0;
   out_2772014030390626314[185] = 0;
   out_2772014030390626314[186] = 0;
   out_2772014030390626314[187] = 0;
   out_2772014030390626314[188] = 0;
   out_2772014030390626314[189] = 0;
   out_2772014030390626314[190] = 1;
   out_2772014030390626314[191] = 0;
   out_2772014030390626314[192] = 0;
   out_2772014030390626314[193] = 0;
   out_2772014030390626314[194] = 0;
   out_2772014030390626314[195] = 0;
   out_2772014030390626314[196] = 0;
   out_2772014030390626314[197] = 0;
   out_2772014030390626314[198] = 0;
   out_2772014030390626314[199] = 0;
   out_2772014030390626314[200] = 0;
   out_2772014030390626314[201] = 0;
   out_2772014030390626314[202] = 0;
   out_2772014030390626314[203] = 0;
   out_2772014030390626314[204] = 0;
   out_2772014030390626314[205] = 0;
   out_2772014030390626314[206] = 0;
   out_2772014030390626314[207] = 0;
   out_2772014030390626314[208] = 0;
   out_2772014030390626314[209] = 1;
   out_2772014030390626314[210] = 0;
   out_2772014030390626314[211] = 0;
   out_2772014030390626314[212] = 0;
   out_2772014030390626314[213] = 0;
   out_2772014030390626314[214] = 0;
   out_2772014030390626314[215] = 0;
   out_2772014030390626314[216] = 0;
   out_2772014030390626314[217] = 0;
   out_2772014030390626314[218] = 0;
   out_2772014030390626314[219] = 0;
   out_2772014030390626314[220] = 0;
   out_2772014030390626314[221] = 0;
   out_2772014030390626314[222] = 0;
   out_2772014030390626314[223] = 0;
   out_2772014030390626314[224] = 0;
   out_2772014030390626314[225] = 0;
   out_2772014030390626314[226] = 0;
   out_2772014030390626314[227] = 0;
   out_2772014030390626314[228] = 1;
   out_2772014030390626314[229] = 0;
   out_2772014030390626314[230] = 0;
   out_2772014030390626314[231] = 0;
   out_2772014030390626314[232] = 0;
   out_2772014030390626314[233] = 0;
   out_2772014030390626314[234] = 0;
   out_2772014030390626314[235] = 0;
   out_2772014030390626314[236] = 0;
   out_2772014030390626314[237] = 0;
   out_2772014030390626314[238] = 0;
   out_2772014030390626314[239] = 0;
   out_2772014030390626314[240] = 0;
   out_2772014030390626314[241] = 0;
   out_2772014030390626314[242] = 0;
   out_2772014030390626314[243] = 0;
   out_2772014030390626314[244] = 0;
   out_2772014030390626314[245] = 0;
   out_2772014030390626314[246] = 0;
   out_2772014030390626314[247] = 1;
   out_2772014030390626314[248] = 0;
   out_2772014030390626314[249] = 0;
   out_2772014030390626314[250] = 0;
   out_2772014030390626314[251] = 0;
   out_2772014030390626314[252] = 0;
   out_2772014030390626314[253] = 0;
   out_2772014030390626314[254] = 0;
   out_2772014030390626314[255] = 0;
   out_2772014030390626314[256] = 0;
   out_2772014030390626314[257] = 0;
   out_2772014030390626314[258] = 0;
   out_2772014030390626314[259] = 0;
   out_2772014030390626314[260] = 0;
   out_2772014030390626314[261] = 0;
   out_2772014030390626314[262] = 0;
   out_2772014030390626314[263] = 0;
   out_2772014030390626314[264] = 0;
   out_2772014030390626314[265] = 0;
   out_2772014030390626314[266] = 1;
   out_2772014030390626314[267] = 0;
   out_2772014030390626314[268] = 0;
   out_2772014030390626314[269] = 0;
   out_2772014030390626314[270] = 0;
   out_2772014030390626314[271] = 0;
   out_2772014030390626314[272] = 0;
   out_2772014030390626314[273] = 0;
   out_2772014030390626314[274] = 0;
   out_2772014030390626314[275] = 0;
   out_2772014030390626314[276] = 0;
   out_2772014030390626314[277] = 0;
   out_2772014030390626314[278] = 0;
   out_2772014030390626314[279] = 0;
   out_2772014030390626314[280] = 0;
   out_2772014030390626314[281] = 0;
   out_2772014030390626314[282] = 0;
   out_2772014030390626314[283] = 0;
   out_2772014030390626314[284] = 0;
   out_2772014030390626314[285] = 1;
   out_2772014030390626314[286] = 0;
   out_2772014030390626314[287] = 0;
   out_2772014030390626314[288] = 0;
   out_2772014030390626314[289] = 0;
   out_2772014030390626314[290] = 0;
   out_2772014030390626314[291] = 0;
   out_2772014030390626314[292] = 0;
   out_2772014030390626314[293] = 0;
   out_2772014030390626314[294] = 0;
   out_2772014030390626314[295] = 0;
   out_2772014030390626314[296] = 0;
   out_2772014030390626314[297] = 0;
   out_2772014030390626314[298] = 0;
   out_2772014030390626314[299] = 0;
   out_2772014030390626314[300] = 0;
   out_2772014030390626314[301] = 0;
   out_2772014030390626314[302] = 0;
   out_2772014030390626314[303] = 0;
   out_2772014030390626314[304] = 1;
   out_2772014030390626314[305] = 0;
   out_2772014030390626314[306] = 0;
   out_2772014030390626314[307] = 0;
   out_2772014030390626314[308] = 0;
   out_2772014030390626314[309] = 0;
   out_2772014030390626314[310] = 0;
   out_2772014030390626314[311] = 0;
   out_2772014030390626314[312] = 0;
   out_2772014030390626314[313] = 0;
   out_2772014030390626314[314] = 0;
   out_2772014030390626314[315] = 0;
   out_2772014030390626314[316] = 0;
   out_2772014030390626314[317] = 0;
   out_2772014030390626314[318] = 0;
   out_2772014030390626314[319] = 0;
   out_2772014030390626314[320] = 0;
   out_2772014030390626314[321] = 0;
   out_2772014030390626314[322] = 0;
   out_2772014030390626314[323] = 1;
}
void h_4(double *state, double *unused, double *out_6530784052392321189) {
   out_6530784052392321189[0] = state[6] + state[9];
   out_6530784052392321189[1] = state[7] + state[10];
   out_6530784052392321189[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_8978390115404430983) {
   out_8978390115404430983[0] = 0;
   out_8978390115404430983[1] = 0;
   out_8978390115404430983[2] = 0;
   out_8978390115404430983[3] = 0;
   out_8978390115404430983[4] = 0;
   out_8978390115404430983[5] = 0;
   out_8978390115404430983[6] = 1;
   out_8978390115404430983[7] = 0;
   out_8978390115404430983[8] = 0;
   out_8978390115404430983[9] = 1;
   out_8978390115404430983[10] = 0;
   out_8978390115404430983[11] = 0;
   out_8978390115404430983[12] = 0;
   out_8978390115404430983[13] = 0;
   out_8978390115404430983[14] = 0;
   out_8978390115404430983[15] = 0;
   out_8978390115404430983[16] = 0;
   out_8978390115404430983[17] = 0;
   out_8978390115404430983[18] = 0;
   out_8978390115404430983[19] = 0;
   out_8978390115404430983[20] = 0;
   out_8978390115404430983[21] = 0;
   out_8978390115404430983[22] = 0;
   out_8978390115404430983[23] = 0;
   out_8978390115404430983[24] = 0;
   out_8978390115404430983[25] = 1;
   out_8978390115404430983[26] = 0;
   out_8978390115404430983[27] = 0;
   out_8978390115404430983[28] = 1;
   out_8978390115404430983[29] = 0;
   out_8978390115404430983[30] = 0;
   out_8978390115404430983[31] = 0;
   out_8978390115404430983[32] = 0;
   out_8978390115404430983[33] = 0;
   out_8978390115404430983[34] = 0;
   out_8978390115404430983[35] = 0;
   out_8978390115404430983[36] = 0;
   out_8978390115404430983[37] = 0;
   out_8978390115404430983[38] = 0;
   out_8978390115404430983[39] = 0;
   out_8978390115404430983[40] = 0;
   out_8978390115404430983[41] = 0;
   out_8978390115404430983[42] = 0;
   out_8978390115404430983[43] = 0;
   out_8978390115404430983[44] = 1;
   out_8978390115404430983[45] = 0;
   out_8978390115404430983[46] = 0;
   out_8978390115404430983[47] = 1;
   out_8978390115404430983[48] = 0;
   out_8978390115404430983[49] = 0;
   out_8978390115404430983[50] = 0;
   out_8978390115404430983[51] = 0;
   out_8978390115404430983[52] = 0;
   out_8978390115404430983[53] = 0;
}
void h_10(double *state, double *unused, double *out_6173900798648768843) {
   out_6173900798648768843[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_6173900798648768843[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_6173900798648768843[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_5102189638329563187) {
   out_5102189638329563187[0] = 0;
   out_5102189638329563187[1] = 9.8100000000000005*cos(state[1]);
   out_5102189638329563187[2] = 0;
   out_5102189638329563187[3] = 0;
   out_5102189638329563187[4] = -state[8];
   out_5102189638329563187[5] = state[7];
   out_5102189638329563187[6] = 0;
   out_5102189638329563187[7] = state[5];
   out_5102189638329563187[8] = -state[4];
   out_5102189638329563187[9] = 0;
   out_5102189638329563187[10] = 0;
   out_5102189638329563187[11] = 0;
   out_5102189638329563187[12] = 1;
   out_5102189638329563187[13] = 0;
   out_5102189638329563187[14] = 0;
   out_5102189638329563187[15] = 1;
   out_5102189638329563187[16] = 0;
   out_5102189638329563187[17] = 0;
   out_5102189638329563187[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_5102189638329563187[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_5102189638329563187[20] = 0;
   out_5102189638329563187[21] = state[8];
   out_5102189638329563187[22] = 0;
   out_5102189638329563187[23] = -state[6];
   out_5102189638329563187[24] = -state[5];
   out_5102189638329563187[25] = 0;
   out_5102189638329563187[26] = state[3];
   out_5102189638329563187[27] = 0;
   out_5102189638329563187[28] = 0;
   out_5102189638329563187[29] = 0;
   out_5102189638329563187[30] = 0;
   out_5102189638329563187[31] = 1;
   out_5102189638329563187[32] = 0;
   out_5102189638329563187[33] = 0;
   out_5102189638329563187[34] = 1;
   out_5102189638329563187[35] = 0;
   out_5102189638329563187[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_5102189638329563187[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_5102189638329563187[38] = 0;
   out_5102189638329563187[39] = -state[7];
   out_5102189638329563187[40] = state[6];
   out_5102189638329563187[41] = 0;
   out_5102189638329563187[42] = state[4];
   out_5102189638329563187[43] = -state[3];
   out_5102189638329563187[44] = 0;
   out_5102189638329563187[45] = 0;
   out_5102189638329563187[46] = 0;
   out_5102189638329563187[47] = 0;
   out_5102189638329563187[48] = 0;
   out_5102189638329563187[49] = 0;
   out_5102189638329563187[50] = 1;
   out_5102189638329563187[51] = 0;
   out_5102189638329563187[52] = 0;
   out_5102189638329563187[53] = 1;
}
void h_13(double *state, double *unused, double *out_7476437276712303054) {
   out_7476437276712303054[0] = state[3];
   out_7476437276712303054[1] = state[4];
   out_7476437276712303054[2] = state[5];
}
void H_13(double *state, double *unused, double *out_5766116290072098182) {
   out_5766116290072098182[0] = 0;
   out_5766116290072098182[1] = 0;
   out_5766116290072098182[2] = 0;
   out_5766116290072098182[3] = 1;
   out_5766116290072098182[4] = 0;
   out_5766116290072098182[5] = 0;
   out_5766116290072098182[6] = 0;
   out_5766116290072098182[7] = 0;
   out_5766116290072098182[8] = 0;
   out_5766116290072098182[9] = 0;
   out_5766116290072098182[10] = 0;
   out_5766116290072098182[11] = 0;
   out_5766116290072098182[12] = 0;
   out_5766116290072098182[13] = 0;
   out_5766116290072098182[14] = 0;
   out_5766116290072098182[15] = 0;
   out_5766116290072098182[16] = 0;
   out_5766116290072098182[17] = 0;
   out_5766116290072098182[18] = 0;
   out_5766116290072098182[19] = 0;
   out_5766116290072098182[20] = 0;
   out_5766116290072098182[21] = 0;
   out_5766116290072098182[22] = 1;
   out_5766116290072098182[23] = 0;
   out_5766116290072098182[24] = 0;
   out_5766116290072098182[25] = 0;
   out_5766116290072098182[26] = 0;
   out_5766116290072098182[27] = 0;
   out_5766116290072098182[28] = 0;
   out_5766116290072098182[29] = 0;
   out_5766116290072098182[30] = 0;
   out_5766116290072098182[31] = 0;
   out_5766116290072098182[32] = 0;
   out_5766116290072098182[33] = 0;
   out_5766116290072098182[34] = 0;
   out_5766116290072098182[35] = 0;
   out_5766116290072098182[36] = 0;
   out_5766116290072098182[37] = 0;
   out_5766116290072098182[38] = 0;
   out_5766116290072098182[39] = 0;
   out_5766116290072098182[40] = 0;
   out_5766116290072098182[41] = 1;
   out_5766116290072098182[42] = 0;
   out_5766116290072098182[43] = 0;
   out_5766116290072098182[44] = 0;
   out_5766116290072098182[45] = 0;
   out_5766116290072098182[46] = 0;
   out_5766116290072098182[47] = 0;
   out_5766116290072098182[48] = 0;
   out_5766116290072098182[49] = 0;
   out_5766116290072098182[50] = 0;
   out_5766116290072098182[51] = 0;
   out_5766116290072098182[52] = 0;
   out_5766116290072098182[53] = 0;
}
void h_14(double *state, double *unused, double *out_8287580119295854294) {
   out_8287580119295854294[0] = state[6];
   out_8287580119295854294[1] = state[7];
   out_8287580119295854294[2] = state[8];
}
void H_14(double *state, double *unused, double *out_5015149259064946454) {
   out_5015149259064946454[0] = 0;
   out_5015149259064946454[1] = 0;
   out_5015149259064946454[2] = 0;
   out_5015149259064946454[3] = 0;
   out_5015149259064946454[4] = 0;
   out_5015149259064946454[5] = 0;
   out_5015149259064946454[6] = 1;
   out_5015149259064946454[7] = 0;
   out_5015149259064946454[8] = 0;
   out_5015149259064946454[9] = 0;
   out_5015149259064946454[10] = 0;
   out_5015149259064946454[11] = 0;
   out_5015149259064946454[12] = 0;
   out_5015149259064946454[13] = 0;
   out_5015149259064946454[14] = 0;
   out_5015149259064946454[15] = 0;
   out_5015149259064946454[16] = 0;
   out_5015149259064946454[17] = 0;
   out_5015149259064946454[18] = 0;
   out_5015149259064946454[19] = 0;
   out_5015149259064946454[20] = 0;
   out_5015149259064946454[21] = 0;
   out_5015149259064946454[22] = 0;
   out_5015149259064946454[23] = 0;
   out_5015149259064946454[24] = 0;
   out_5015149259064946454[25] = 1;
   out_5015149259064946454[26] = 0;
   out_5015149259064946454[27] = 0;
   out_5015149259064946454[28] = 0;
   out_5015149259064946454[29] = 0;
   out_5015149259064946454[30] = 0;
   out_5015149259064946454[31] = 0;
   out_5015149259064946454[32] = 0;
   out_5015149259064946454[33] = 0;
   out_5015149259064946454[34] = 0;
   out_5015149259064946454[35] = 0;
   out_5015149259064946454[36] = 0;
   out_5015149259064946454[37] = 0;
   out_5015149259064946454[38] = 0;
   out_5015149259064946454[39] = 0;
   out_5015149259064946454[40] = 0;
   out_5015149259064946454[41] = 0;
   out_5015149259064946454[42] = 0;
   out_5015149259064946454[43] = 0;
   out_5015149259064946454[44] = 1;
   out_5015149259064946454[45] = 0;
   out_5015149259064946454[46] = 0;
   out_5015149259064946454[47] = 0;
   out_5015149259064946454[48] = 0;
   out_5015149259064946454[49] = 0;
   out_5015149259064946454[50] = 0;
   out_5015149259064946454[51] = 0;
   out_5015149259064946454[52] = 0;
   out_5015149259064946454[53] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_2081385545714313315) {
  err_fun(nom_x, delta_x, out_2081385545714313315);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_1086297426282480324) {
  inv_err_fun(nom_x, true_x, out_1086297426282480324);
}
void pose_H_mod_fun(double *state, double *out_8714192956250408026) {
  H_mod_fun(state, out_8714192956250408026);
}
void pose_f_fun(double *state, double dt, double *out_5560781299359714526) {
  f_fun(state,  dt, out_5560781299359714526);
}
void pose_F_fun(double *state, double dt, double *out_2772014030390626314) {
  F_fun(state,  dt, out_2772014030390626314);
}
void pose_h_4(double *state, double *unused, double *out_6530784052392321189) {
  h_4(state, unused, out_6530784052392321189);
}
void pose_H_4(double *state, double *unused, double *out_8978390115404430983) {
  H_4(state, unused, out_8978390115404430983);
}
void pose_h_10(double *state, double *unused, double *out_6173900798648768843) {
  h_10(state, unused, out_6173900798648768843);
}
void pose_H_10(double *state, double *unused, double *out_5102189638329563187) {
  H_10(state, unused, out_5102189638329563187);
}
void pose_h_13(double *state, double *unused, double *out_7476437276712303054) {
  h_13(state, unused, out_7476437276712303054);
}
void pose_H_13(double *state, double *unused, double *out_5766116290072098182) {
  H_13(state, unused, out_5766116290072098182);
}
void pose_h_14(double *state, double *unused, double *out_8287580119295854294) {
  h_14(state, unused, out_8287580119295854294);
}
void pose_H_14(double *state, double *unused, double *out_5015149259064946454) {
  H_14(state, unused, out_5015149259064946454);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
