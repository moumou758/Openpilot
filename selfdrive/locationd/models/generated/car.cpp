#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7282555611597409756) {
   out_7282555611597409756[0] = delta_x[0] + nom_x[0];
   out_7282555611597409756[1] = delta_x[1] + nom_x[1];
   out_7282555611597409756[2] = delta_x[2] + nom_x[2];
   out_7282555611597409756[3] = delta_x[3] + nom_x[3];
   out_7282555611597409756[4] = delta_x[4] + nom_x[4];
   out_7282555611597409756[5] = delta_x[5] + nom_x[5];
   out_7282555611597409756[6] = delta_x[6] + nom_x[6];
   out_7282555611597409756[7] = delta_x[7] + nom_x[7];
   out_7282555611597409756[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7016179484475136775) {
   out_7016179484475136775[0] = -nom_x[0] + true_x[0];
   out_7016179484475136775[1] = -nom_x[1] + true_x[1];
   out_7016179484475136775[2] = -nom_x[2] + true_x[2];
   out_7016179484475136775[3] = -nom_x[3] + true_x[3];
   out_7016179484475136775[4] = -nom_x[4] + true_x[4];
   out_7016179484475136775[5] = -nom_x[5] + true_x[5];
   out_7016179484475136775[6] = -nom_x[6] + true_x[6];
   out_7016179484475136775[7] = -nom_x[7] + true_x[7];
   out_7016179484475136775[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5240788052860617638) {
   out_5240788052860617638[0] = 1.0;
   out_5240788052860617638[1] = 0;
   out_5240788052860617638[2] = 0;
   out_5240788052860617638[3] = 0;
   out_5240788052860617638[4] = 0;
   out_5240788052860617638[5] = 0;
   out_5240788052860617638[6] = 0;
   out_5240788052860617638[7] = 0;
   out_5240788052860617638[8] = 0;
   out_5240788052860617638[9] = 0;
   out_5240788052860617638[10] = 1.0;
   out_5240788052860617638[11] = 0;
   out_5240788052860617638[12] = 0;
   out_5240788052860617638[13] = 0;
   out_5240788052860617638[14] = 0;
   out_5240788052860617638[15] = 0;
   out_5240788052860617638[16] = 0;
   out_5240788052860617638[17] = 0;
   out_5240788052860617638[18] = 0;
   out_5240788052860617638[19] = 0;
   out_5240788052860617638[20] = 1.0;
   out_5240788052860617638[21] = 0;
   out_5240788052860617638[22] = 0;
   out_5240788052860617638[23] = 0;
   out_5240788052860617638[24] = 0;
   out_5240788052860617638[25] = 0;
   out_5240788052860617638[26] = 0;
   out_5240788052860617638[27] = 0;
   out_5240788052860617638[28] = 0;
   out_5240788052860617638[29] = 0;
   out_5240788052860617638[30] = 1.0;
   out_5240788052860617638[31] = 0;
   out_5240788052860617638[32] = 0;
   out_5240788052860617638[33] = 0;
   out_5240788052860617638[34] = 0;
   out_5240788052860617638[35] = 0;
   out_5240788052860617638[36] = 0;
   out_5240788052860617638[37] = 0;
   out_5240788052860617638[38] = 0;
   out_5240788052860617638[39] = 0;
   out_5240788052860617638[40] = 1.0;
   out_5240788052860617638[41] = 0;
   out_5240788052860617638[42] = 0;
   out_5240788052860617638[43] = 0;
   out_5240788052860617638[44] = 0;
   out_5240788052860617638[45] = 0;
   out_5240788052860617638[46] = 0;
   out_5240788052860617638[47] = 0;
   out_5240788052860617638[48] = 0;
   out_5240788052860617638[49] = 0;
   out_5240788052860617638[50] = 1.0;
   out_5240788052860617638[51] = 0;
   out_5240788052860617638[52] = 0;
   out_5240788052860617638[53] = 0;
   out_5240788052860617638[54] = 0;
   out_5240788052860617638[55] = 0;
   out_5240788052860617638[56] = 0;
   out_5240788052860617638[57] = 0;
   out_5240788052860617638[58] = 0;
   out_5240788052860617638[59] = 0;
   out_5240788052860617638[60] = 1.0;
   out_5240788052860617638[61] = 0;
   out_5240788052860617638[62] = 0;
   out_5240788052860617638[63] = 0;
   out_5240788052860617638[64] = 0;
   out_5240788052860617638[65] = 0;
   out_5240788052860617638[66] = 0;
   out_5240788052860617638[67] = 0;
   out_5240788052860617638[68] = 0;
   out_5240788052860617638[69] = 0;
   out_5240788052860617638[70] = 1.0;
   out_5240788052860617638[71] = 0;
   out_5240788052860617638[72] = 0;
   out_5240788052860617638[73] = 0;
   out_5240788052860617638[74] = 0;
   out_5240788052860617638[75] = 0;
   out_5240788052860617638[76] = 0;
   out_5240788052860617638[77] = 0;
   out_5240788052860617638[78] = 0;
   out_5240788052860617638[79] = 0;
   out_5240788052860617638[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1785653652839417707) {
   out_1785653652839417707[0] = state[0];
   out_1785653652839417707[1] = state[1];
   out_1785653652839417707[2] = state[2];
   out_1785653652839417707[3] = state[3];
   out_1785653652839417707[4] = state[4];
   out_1785653652839417707[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1785653652839417707[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1785653652839417707[7] = state[7];
   out_1785653652839417707[8] = state[8];
}
void F_fun(double *state, double dt, double *out_7961890063836476806) {
   out_7961890063836476806[0] = 1;
   out_7961890063836476806[1] = 0;
   out_7961890063836476806[2] = 0;
   out_7961890063836476806[3] = 0;
   out_7961890063836476806[4] = 0;
   out_7961890063836476806[5] = 0;
   out_7961890063836476806[6] = 0;
   out_7961890063836476806[7] = 0;
   out_7961890063836476806[8] = 0;
   out_7961890063836476806[9] = 0;
   out_7961890063836476806[10] = 1;
   out_7961890063836476806[11] = 0;
   out_7961890063836476806[12] = 0;
   out_7961890063836476806[13] = 0;
   out_7961890063836476806[14] = 0;
   out_7961890063836476806[15] = 0;
   out_7961890063836476806[16] = 0;
   out_7961890063836476806[17] = 0;
   out_7961890063836476806[18] = 0;
   out_7961890063836476806[19] = 0;
   out_7961890063836476806[20] = 1;
   out_7961890063836476806[21] = 0;
   out_7961890063836476806[22] = 0;
   out_7961890063836476806[23] = 0;
   out_7961890063836476806[24] = 0;
   out_7961890063836476806[25] = 0;
   out_7961890063836476806[26] = 0;
   out_7961890063836476806[27] = 0;
   out_7961890063836476806[28] = 0;
   out_7961890063836476806[29] = 0;
   out_7961890063836476806[30] = 1;
   out_7961890063836476806[31] = 0;
   out_7961890063836476806[32] = 0;
   out_7961890063836476806[33] = 0;
   out_7961890063836476806[34] = 0;
   out_7961890063836476806[35] = 0;
   out_7961890063836476806[36] = 0;
   out_7961890063836476806[37] = 0;
   out_7961890063836476806[38] = 0;
   out_7961890063836476806[39] = 0;
   out_7961890063836476806[40] = 1;
   out_7961890063836476806[41] = 0;
   out_7961890063836476806[42] = 0;
   out_7961890063836476806[43] = 0;
   out_7961890063836476806[44] = 0;
   out_7961890063836476806[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7961890063836476806[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7961890063836476806[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7961890063836476806[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7961890063836476806[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7961890063836476806[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7961890063836476806[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7961890063836476806[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7961890063836476806[53] = -9.8000000000000007*dt;
   out_7961890063836476806[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7961890063836476806[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7961890063836476806[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7961890063836476806[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7961890063836476806[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7961890063836476806[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7961890063836476806[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7961890063836476806[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7961890063836476806[62] = 0;
   out_7961890063836476806[63] = 0;
   out_7961890063836476806[64] = 0;
   out_7961890063836476806[65] = 0;
   out_7961890063836476806[66] = 0;
   out_7961890063836476806[67] = 0;
   out_7961890063836476806[68] = 0;
   out_7961890063836476806[69] = 0;
   out_7961890063836476806[70] = 1;
   out_7961890063836476806[71] = 0;
   out_7961890063836476806[72] = 0;
   out_7961890063836476806[73] = 0;
   out_7961890063836476806[74] = 0;
   out_7961890063836476806[75] = 0;
   out_7961890063836476806[76] = 0;
   out_7961890063836476806[77] = 0;
   out_7961890063836476806[78] = 0;
   out_7961890063836476806[79] = 0;
   out_7961890063836476806[80] = 1;
}
void h_25(double *state, double *unused, double *out_1017751437672724041) {
   out_1017751437672724041[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1261756931429642725) {
   out_1261756931429642725[0] = 0;
   out_1261756931429642725[1] = 0;
   out_1261756931429642725[2] = 0;
   out_1261756931429642725[3] = 0;
   out_1261756931429642725[4] = 0;
   out_1261756931429642725[5] = 0;
   out_1261756931429642725[6] = 1;
   out_1261756931429642725[7] = 0;
   out_1261756931429642725[8] = 0;
}
void h_24(double *state, double *unused, double *out_8695509325379892490) {
   out_8695509325379892490[0] = state[4];
   out_8695509325379892490[1] = state[5];
}
void H_24(double *state, double *unused, double *out_6156634702247284168) {
   out_6156634702247284168[0] = 0;
   out_6156634702247284168[1] = 0;
   out_6156634702247284168[2] = 0;
   out_6156634702247284168[3] = 0;
   out_6156634702247284168[4] = 1;
   out_6156634702247284168[5] = 0;
   out_6156634702247284168[6] = 0;
   out_6156634702247284168[7] = 0;
   out_6156634702247284168[8] = 0;
   out_6156634702247284168[9] = 0;
   out_6156634702247284168[10] = 0;
   out_6156634702247284168[11] = 0;
   out_6156634702247284168[12] = 0;
   out_6156634702247284168[13] = 0;
   out_6156634702247284168[14] = 1;
   out_6156634702247284168[15] = 0;
   out_6156634702247284168[16] = 0;
   out_6156634702247284168[17] = 0;
}
void h_30(double *state, double *unused, double *out_3080551860606647823) {
   out_3080551860606647823[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1391095878572882795) {
   out_1391095878572882795[0] = 0;
   out_1391095878572882795[1] = 0;
   out_1391095878572882795[2] = 0;
   out_1391095878572882795[3] = 0;
   out_1391095878572882795[4] = 1;
   out_1391095878572882795[5] = 0;
   out_1391095878572882795[6] = 0;
   out_1391095878572882795[7] = 0;
   out_1391095878572882795[8] = 0;
}
void h_26(double *state, double *unused, double *out_1223084521631968697) {
   out_1223084521631968697[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5003260250303698949) {
   out_5003260250303698949[0] = 0;
   out_5003260250303698949[1] = 0;
   out_5003260250303698949[2] = 0;
   out_5003260250303698949[3] = 0;
   out_5003260250303698949[4] = 0;
   out_5003260250303698949[5] = 0;
   out_5003260250303698949[6] = 0;
   out_5003260250303698949[7] = 1;
   out_5003260250303698949[8] = 0;
}
void h_27(double *state, double *unused, double *out_1999835531008022153) {
   out_1999835531008022153[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3565859190373307706) {
   out_3565859190373307706[0] = 0;
   out_3565859190373307706[1] = 0;
   out_3565859190373307706[2] = 0;
   out_3565859190373307706[3] = 1;
   out_3565859190373307706[4] = 0;
   out_3565859190373307706[5] = 0;
   out_3565859190373307706[6] = 0;
   out_3565859190373307706[7] = 0;
   out_3565859190373307706[8] = 0;
}
void h_29(double *state, double *unused, double *out_8565635891947480278) {
   out_8565635891947480278[0] = state[1];
}
void H_29(double *state, double *unused, double *out_880864534258490611) {
   out_880864534258490611[0] = 0;
   out_880864534258490611[1] = 1;
   out_880864534258490611[2] = 0;
   out_880864534258490611[3] = 0;
   out_880864534258490611[4] = 0;
   out_880864534258490611[5] = 0;
   out_880864534258490611[6] = 0;
   out_880864534258490611[7] = 0;
   out_880864534258490611[8] = 0;
}
void h_28(double *state, double *unused, double *out_3847929205483827493) {
   out_3847929205483827493[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3315591645677532488) {
   out_3315591645677532488[0] = 1;
   out_3315591645677532488[1] = 0;
   out_3315591645677532488[2] = 0;
   out_3315591645677532488[3] = 0;
   out_3315591645677532488[4] = 0;
   out_3315591645677532488[5] = 0;
   out_3315591645677532488[6] = 0;
   out_3315591645677532488[7] = 0;
   out_3315591645677532488[8] = 0;
}
void h_31(double *state, double *unused, double *out_742557375388218152) {
   out_742557375388218152[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1231110969552682297) {
   out_1231110969552682297[0] = 0;
   out_1231110969552682297[1] = 0;
   out_1231110969552682297[2] = 0;
   out_1231110969552682297[3] = 0;
   out_1231110969552682297[4] = 0;
   out_1231110969552682297[5] = 0;
   out_1231110969552682297[6] = 0;
   out_1231110969552682297[7] = 0;
   out_1231110969552682297[8] = 1;
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

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_7282555611597409756) {
  err_fun(nom_x, delta_x, out_7282555611597409756);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7016179484475136775) {
  inv_err_fun(nom_x, true_x, out_7016179484475136775);
}
void car_H_mod_fun(double *state, double *out_5240788052860617638) {
  H_mod_fun(state, out_5240788052860617638);
}
void car_f_fun(double *state, double dt, double *out_1785653652839417707) {
  f_fun(state,  dt, out_1785653652839417707);
}
void car_F_fun(double *state, double dt, double *out_7961890063836476806) {
  F_fun(state,  dt, out_7961890063836476806);
}
void car_h_25(double *state, double *unused, double *out_1017751437672724041) {
  h_25(state, unused, out_1017751437672724041);
}
void car_H_25(double *state, double *unused, double *out_1261756931429642725) {
  H_25(state, unused, out_1261756931429642725);
}
void car_h_24(double *state, double *unused, double *out_8695509325379892490) {
  h_24(state, unused, out_8695509325379892490);
}
void car_H_24(double *state, double *unused, double *out_6156634702247284168) {
  H_24(state, unused, out_6156634702247284168);
}
void car_h_30(double *state, double *unused, double *out_3080551860606647823) {
  h_30(state, unused, out_3080551860606647823);
}
void car_H_30(double *state, double *unused, double *out_1391095878572882795) {
  H_30(state, unused, out_1391095878572882795);
}
void car_h_26(double *state, double *unused, double *out_1223084521631968697) {
  h_26(state, unused, out_1223084521631968697);
}
void car_H_26(double *state, double *unused, double *out_5003260250303698949) {
  H_26(state, unused, out_5003260250303698949);
}
void car_h_27(double *state, double *unused, double *out_1999835531008022153) {
  h_27(state, unused, out_1999835531008022153);
}
void car_H_27(double *state, double *unused, double *out_3565859190373307706) {
  H_27(state, unused, out_3565859190373307706);
}
void car_h_29(double *state, double *unused, double *out_8565635891947480278) {
  h_29(state, unused, out_8565635891947480278);
}
void car_H_29(double *state, double *unused, double *out_880864534258490611) {
  H_29(state, unused, out_880864534258490611);
}
void car_h_28(double *state, double *unused, double *out_3847929205483827493) {
  h_28(state, unused, out_3847929205483827493);
}
void car_H_28(double *state, double *unused, double *out_3315591645677532488) {
  H_28(state, unused, out_3315591645677532488);
}
void car_h_31(double *state, double *unused, double *out_742557375388218152) {
  h_31(state, unused, out_742557375388218152);
}
void car_H_31(double *state, double *unused, double *out_1231110969552682297) {
  H_31(state, unused, out_1231110969552682297);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
