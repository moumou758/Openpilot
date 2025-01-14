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
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5237420678320443973) {
   out_5237420678320443973[0] = delta_x[0] + nom_x[0];
   out_5237420678320443973[1] = delta_x[1] + nom_x[1];
   out_5237420678320443973[2] = delta_x[2] + nom_x[2];
   out_5237420678320443973[3] = delta_x[3] + nom_x[3];
   out_5237420678320443973[4] = delta_x[4] + nom_x[4];
   out_5237420678320443973[5] = delta_x[5] + nom_x[5];
   out_5237420678320443973[6] = delta_x[6] + nom_x[6];
   out_5237420678320443973[7] = delta_x[7] + nom_x[7];
   out_5237420678320443973[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7188805063255433361) {
   out_7188805063255433361[0] = -nom_x[0] + true_x[0];
   out_7188805063255433361[1] = -nom_x[1] + true_x[1];
   out_7188805063255433361[2] = -nom_x[2] + true_x[2];
   out_7188805063255433361[3] = -nom_x[3] + true_x[3];
   out_7188805063255433361[4] = -nom_x[4] + true_x[4];
   out_7188805063255433361[5] = -nom_x[5] + true_x[5];
   out_7188805063255433361[6] = -nom_x[6] + true_x[6];
   out_7188805063255433361[7] = -nom_x[7] + true_x[7];
   out_7188805063255433361[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4304115201110115560) {
   out_4304115201110115560[0] = 1.0;
   out_4304115201110115560[1] = 0.0;
   out_4304115201110115560[2] = 0.0;
   out_4304115201110115560[3] = 0.0;
   out_4304115201110115560[4] = 0.0;
   out_4304115201110115560[5] = 0.0;
   out_4304115201110115560[6] = 0.0;
   out_4304115201110115560[7] = 0.0;
   out_4304115201110115560[8] = 0.0;
   out_4304115201110115560[9] = 0.0;
   out_4304115201110115560[10] = 1.0;
   out_4304115201110115560[11] = 0.0;
   out_4304115201110115560[12] = 0.0;
   out_4304115201110115560[13] = 0.0;
   out_4304115201110115560[14] = 0.0;
   out_4304115201110115560[15] = 0.0;
   out_4304115201110115560[16] = 0.0;
   out_4304115201110115560[17] = 0.0;
   out_4304115201110115560[18] = 0.0;
   out_4304115201110115560[19] = 0.0;
   out_4304115201110115560[20] = 1.0;
   out_4304115201110115560[21] = 0.0;
   out_4304115201110115560[22] = 0.0;
   out_4304115201110115560[23] = 0.0;
   out_4304115201110115560[24] = 0.0;
   out_4304115201110115560[25] = 0.0;
   out_4304115201110115560[26] = 0.0;
   out_4304115201110115560[27] = 0.0;
   out_4304115201110115560[28] = 0.0;
   out_4304115201110115560[29] = 0.0;
   out_4304115201110115560[30] = 1.0;
   out_4304115201110115560[31] = 0.0;
   out_4304115201110115560[32] = 0.0;
   out_4304115201110115560[33] = 0.0;
   out_4304115201110115560[34] = 0.0;
   out_4304115201110115560[35] = 0.0;
   out_4304115201110115560[36] = 0.0;
   out_4304115201110115560[37] = 0.0;
   out_4304115201110115560[38] = 0.0;
   out_4304115201110115560[39] = 0.0;
   out_4304115201110115560[40] = 1.0;
   out_4304115201110115560[41] = 0.0;
   out_4304115201110115560[42] = 0.0;
   out_4304115201110115560[43] = 0.0;
   out_4304115201110115560[44] = 0.0;
   out_4304115201110115560[45] = 0.0;
   out_4304115201110115560[46] = 0.0;
   out_4304115201110115560[47] = 0.0;
   out_4304115201110115560[48] = 0.0;
   out_4304115201110115560[49] = 0.0;
   out_4304115201110115560[50] = 1.0;
   out_4304115201110115560[51] = 0.0;
   out_4304115201110115560[52] = 0.0;
   out_4304115201110115560[53] = 0.0;
   out_4304115201110115560[54] = 0.0;
   out_4304115201110115560[55] = 0.0;
   out_4304115201110115560[56] = 0.0;
   out_4304115201110115560[57] = 0.0;
   out_4304115201110115560[58] = 0.0;
   out_4304115201110115560[59] = 0.0;
   out_4304115201110115560[60] = 1.0;
   out_4304115201110115560[61] = 0.0;
   out_4304115201110115560[62] = 0.0;
   out_4304115201110115560[63] = 0.0;
   out_4304115201110115560[64] = 0.0;
   out_4304115201110115560[65] = 0.0;
   out_4304115201110115560[66] = 0.0;
   out_4304115201110115560[67] = 0.0;
   out_4304115201110115560[68] = 0.0;
   out_4304115201110115560[69] = 0.0;
   out_4304115201110115560[70] = 1.0;
   out_4304115201110115560[71] = 0.0;
   out_4304115201110115560[72] = 0.0;
   out_4304115201110115560[73] = 0.0;
   out_4304115201110115560[74] = 0.0;
   out_4304115201110115560[75] = 0.0;
   out_4304115201110115560[76] = 0.0;
   out_4304115201110115560[77] = 0.0;
   out_4304115201110115560[78] = 0.0;
   out_4304115201110115560[79] = 0.0;
   out_4304115201110115560[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3691590642056691443) {
   out_3691590642056691443[0] = state[0];
   out_3691590642056691443[1] = state[1];
   out_3691590642056691443[2] = state[2];
   out_3691590642056691443[3] = state[3];
   out_3691590642056691443[4] = state[4];
   out_3691590642056691443[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3691590642056691443[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3691590642056691443[7] = state[7];
   out_3691590642056691443[8] = state[8];
}
void F_fun(double *state, double dt, double *out_7930732072963963441) {
   out_7930732072963963441[0] = 1;
   out_7930732072963963441[1] = 0;
   out_7930732072963963441[2] = 0;
   out_7930732072963963441[3] = 0;
   out_7930732072963963441[4] = 0;
   out_7930732072963963441[5] = 0;
   out_7930732072963963441[6] = 0;
   out_7930732072963963441[7] = 0;
   out_7930732072963963441[8] = 0;
   out_7930732072963963441[9] = 0;
   out_7930732072963963441[10] = 1;
   out_7930732072963963441[11] = 0;
   out_7930732072963963441[12] = 0;
   out_7930732072963963441[13] = 0;
   out_7930732072963963441[14] = 0;
   out_7930732072963963441[15] = 0;
   out_7930732072963963441[16] = 0;
   out_7930732072963963441[17] = 0;
   out_7930732072963963441[18] = 0;
   out_7930732072963963441[19] = 0;
   out_7930732072963963441[20] = 1;
   out_7930732072963963441[21] = 0;
   out_7930732072963963441[22] = 0;
   out_7930732072963963441[23] = 0;
   out_7930732072963963441[24] = 0;
   out_7930732072963963441[25] = 0;
   out_7930732072963963441[26] = 0;
   out_7930732072963963441[27] = 0;
   out_7930732072963963441[28] = 0;
   out_7930732072963963441[29] = 0;
   out_7930732072963963441[30] = 1;
   out_7930732072963963441[31] = 0;
   out_7930732072963963441[32] = 0;
   out_7930732072963963441[33] = 0;
   out_7930732072963963441[34] = 0;
   out_7930732072963963441[35] = 0;
   out_7930732072963963441[36] = 0;
   out_7930732072963963441[37] = 0;
   out_7930732072963963441[38] = 0;
   out_7930732072963963441[39] = 0;
   out_7930732072963963441[40] = 1;
   out_7930732072963963441[41] = 0;
   out_7930732072963963441[42] = 0;
   out_7930732072963963441[43] = 0;
   out_7930732072963963441[44] = 0;
   out_7930732072963963441[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7930732072963963441[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7930732072963963441[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7930732072963963441[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7930732072963963441[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7930732072963963441[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7930732072963963441[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7930732072963963441[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7930732072963963441[53] = -9.8000000000000007*dt;
   out_7930732072963963441[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7930732072963963441[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7930732072963963441[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7930732072963963441[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7930732072963963441[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7930732072963963441[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7930732072963963441[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7930732072963963441[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7930732072963963441[62] = 0;
   out_7930732072963963441[63] = 0;
   out_7930732072963963441[64] = 0;
   out_7930732072963963441[65] = 0;
   out_7930732072963963441[66] = 0;
   out_7930732072963963441[67] = 0;
   out_7930732072963963441[68] = 0;
   out_7930732072963963441[69] = 0;
   out_7930732072963963441[70] = 1;
   out_7930732072963963441[71] = 0;
   out_7930732072963963441[72] = 0;
   out_7930732072963963441[73] = 0;
   out_7930732072963963441[74] = 0;
   out_7930732072963963441[75] = 0;
   out_7930732072963963441[76] = 0;
   out_7930732072963963441[77] = 0;
   out_7930732072963963441[78] = 0;
   out_7930732072963963441[79] = 0;
   out_7930732072963963441[80] = 1;
}
void h_25(double *state, double *unused, double *out_2749738423920445478) {
   out_2749738423920445478[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7202900678907652648) {
   out_7202900678907652648[0] = 0;
   out_7202900678907652648[1] = 0;
   out_7202900678907652648[2] = 0;
   out_7202900678907652648[3] = 0;
   out_7202900678907652648[4] = 0;
   out_7202900678907652648[5] = 0;
   out_7202900678907652648[6] = 1;
   out_7202900678907652648[7] = 0;
   out_7202900678907652648[8] = 0;
}
void h_24(double *state, double *unused, double *out_2763472720582336807) {
   out_2763472720582336807[0] = state[4];
   out_2763472720582336807[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7685917795999575832) {
   out_7685917795999575832[0] = 0;
   out_7685917795999575832[1] = 0;
   out_7685917795999575832[2] = 0;
   out_7685917795999575832[3] = 0;
   out_7685917795999575832[4] = 1;
   out_7685917795999575832[5] = 0;
   out_7685917795999575832[6] = 0;
   out_7685917795999575832[7] = 0;
   out_7685917795999575832[8] = 0;
   out_7685917795999575832[9] = 0;
   out_7685917795999575832[10] = 0;
   out_7685917795999575832[11] = 0;
   out_7685917795999575832[12] = 0;
   out_7685917795999575832[13] = 0;
   out_7685917795999575832[14] = 1;
   out_7685917795999575832[15] = 0;
   out_7685917795999575832[16] = 0;
   out_7685917795999575832[17] = 0;
}
void h_30(double *state, double *unused, double *out_887711104333576872) {
   out_887711104333576872[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2675204348780044450) {
   out_2675204348780044450[0] = 0;
   out_2675204348780044450[1] = 0;
   out_2675204348780044450[2] = 0;
   out_2675204348780044450[3] = 0;
   out_2675204348780044450[4] = 1;
   out_2675204348780044450[5] = 0;
   out_2675204348780044450[6] = 0;
   out_2675204348780044450[7] = 0;
   out_2675204348780044450[8] = 0;
}
void h_26(double *state, double *unused, double *out_4113962163327268773) {
   out_4113962163327268773[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3461397360033596424) {
   out_3461397360033596424[0] = 0;
   out_3461397360033596424[1] = 0;
   out_3461397360033596424[2] = 0;
   out_3461397360033596424[3] = 0;
   out_3461397360033596424[4] = 0;
   out_3461397360033596424[5] = 0;
   out_3461397360033596424[6] = 0;
   out_3461397360033596424[7] = 1;
   out_3461397360033596424[8] = 0;
}
void h_27(double *state, double *unused, double *out_4872566989291761472) {
   out_4872566989291761472[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4898798419963987667) {
   out_4898798419963987667[0] = 0;
   out_4898798419963987667[1] = 0;
   out_4898798419963987667[2] = 0;
   out_4898798419963987667[3] = 1;
   out_4898798419963987667[4] = 0;
   out_4898798419963987667[5] = 0;
   out_4898798419963987667[6] = 0;
   out_4898798419963987667[7] = 0;
   out_4898798419963987667[8] = 0;
}
void h_29(double *state, double *unused, double *out_6113618320168901819) {
   out_6113618320168901819[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3185435693094436634) {
   out_3185435693094436634[0] = 0;
   out_3185435693094436634[1] = 1;
   out_3185435693094436634[2] = 0;
   out_3185435693094436634[3] = 0;
   out_3185435693094436634[4] = 0;
   out_3185435693094436634[5] = 0;
   out_3185435693094436634[6] = 0;
   out_3185435693094436634[7] = 0;
   out_3185435693094436634[8] = 0;
}
void h_28(double *state, double *unused, double *out_6730034328266440598) {
   out_6730034328266440598[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1896963323975093940) {
   out_1896963323975093940[0] = 1;
   out_1896963323975093940[1] = 0;
   out_1896963323975093940[2] = 0;
   out_1896963323975093940[3] = 0;
   out_1896963323975093940[4] = 0;
   out_1896963323975093940[5] = 0;
   out_1896963323975093940[6] = 0;
   out_1896963323975093940[7] = 0;
   out_1896963323975093940[8] = 0;
}
void h_31(double *state, double *unused, double *out_5318696205617109844) {
   out_5318696205617109844[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2835189257800244948) {
   out_2835189257800244948[0] = 0;
   out_2835189257800244948[1] = 0;
   out_2835189257800244948[2] = 0;
   out_2835189257800244948[3] = 0;
   out_2835189257800244948[4] = 0;
   out_2835189257800244948[5] = 0;
   out_2835189257800244948[6] = 0;
   out_2835189257800244948[7] = 0;
   out_2835189257800244948[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5237420678320443973) {
  err_fun(nom_x, delta_x, out_5237420678320443973);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7188805063255433361) {
  inv_err_fun(nom_x, true_x, out_7188805063255433361);
}
void car_H_mod_fun(double *state, double *out_4304115201110115560) {
  H_mod_fun(state, out_4304115201110115560);
}
void car_f_fun(double *state, double dt, double *out_3691590642056691443) {
  f_fun(state,  dt, out_3691590642056691443);
}
void car_F_fun(double *state, double dt, double *out_7930732072963963441) {
  F_fun(state,  dt, out_7930732072963963441);
}
void car_h_25(double *state, double *unused, double *out_2749738423920445478) {
  h_25(state, unused, out_2749738423920445478);
}
void car_H_25(double *state, double *unused, double *out_7202900678907652648) {
  H_25(state, unused, out_7202900678907652648);
}
void car_h_24(double *state, double *unused, double *out_2763472720582336807) {
  h_24(state, unused, out_2763472720582336807);
}
void car_H_24(double *state, double *unused, double *out_7685917795999575832) {
  H_24(state, unused, out_7685917795999575832);
}
void car_h_30(double *state, double *unused, double *out_887711104333576872) {
  h_30(state, unused, out_887711104333576872);
}
void car_H_30(double *state, double *unused, double *out_2675204348780044450) {
  H_30(state, unused, out_2675204348780044450);
}
void car_h_26(double *state, double *unused, double *out_4113962163327268773) {
  h_26(state, unused, out_4113962163327268773);
}
void car_H_26(double *state, double *unused, double *out_3461397360033596424) {
  H_26(state, unused, out_3461397360033596424);
}
void car_h_27(double *state, double *unused, double *out_4872566989291761472) {
  h_27(state, unused, out_4872566989291761472);
}
void car_H_27(double *state, double *unused, double *out_4898798419963987667) {
  H_27(state, unused, out_4898798419963987667);
}
void car_h_29(double *state, double *unused, double *out_6113618320168901819) {
  h_29(state, unused, out_6113618320168901819);
}
void car_H_29(double *state, double *unused, double *out_3185435693094436634) {
  H_29(state, unused, out_3185435693094436634);
}
void car_h_28(double *state, double *unused, double *out_6730034328266440598) {
  h_28(state, unused, out_6730034328266440598);
}
void car_H_28(double *state, double *unused, double *out_1896963323975093940) {
  H_28(state, unused, out_1896963323975093940);
}
void car_h_31(double *state, double *unused, double *out_5318696205617109844) {
  h_31(state, unused, out_5318696205617109844);
}
void car_H_31(double *state, double *unused, double *out_2835189257800244948) {
  H_31(state, unused, out_2835189257800244948);
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
