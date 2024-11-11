#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_7282555611597409756);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7016179484475136775);
void car_H_mod_fun(double *state, double *out_5240788052860617638);
void car_f_fun(double *state, double dt, double *out_1785653652839417707);
void car_F_fun(double *state, double dt, double *out_7961890063836476806);
void car_h_25(double *state, double *unused, double *out_1017751437672724041);
void car_H_25(double *state, double *unused, double *out_1261756931429642725);
void car_h_24(double *state, double *unused, double *out_8695509325379892490);
void car_H_24(double *state, double *unused, double *out_6156634702247284168);
void car_h_30(double *state, double *unused, double *out_3080551860606647823);
void car_H_30(double *state, double *unused, double *out_1391095878572882795);
void car_h_26(double *state, double *unused, double *out_1223084521631968697);
void car_H_26(double *state, double *unused, double *out_5003260250303698949);
void car_h_27(double *state, double *unused, double *out_1999835531008022153);
void car_H_27(double *state, double *unused, double *out_3565859190373307706);
void car_h_29(double *state, double *unused, double *out_8565635891947480278);
void car_H_29(double *state, double *unused, double *out_880864534258490611);
void car_h_28(double *state, double *unused, double *out_3847929205483827493);
void car_H_28(double *state, double *unused, double *out_3315591645677532488);
void car_h_31(double *state, double *unused, double *out_742557375388218152);
void car_H_31(double *state, double *unused, double *out_1231110969552682297);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}