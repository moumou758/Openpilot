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
void car_err_fun(double *nom_x, double *delta_x, double *out_5237420678320443973);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7188805063255433361);
void car_H_mod_fun(double *state, double *out_4304115201110115560);
void car_f_fun(double *state, double dt, double *out_3691590642056691443);
void car_F_fun(double *state, double dt, double *out_7930732072963963441);
void car_h_25(double *state, double *unused, double *out_2749738423920445478);
void car_H_25(double *state, double *unused, double *out_7202900678907652648);
void car_h_24(double *state, double *unused, double *out_2763472720582336807);
void car_H_24(double *state, double *unused, double *out_7685917795999575832);
void car_h_30(double *state, double *unused, double *out_887711104333576872);
void car_H_30(double *state, double *unused, double *out_2675204348780044450);
void car_h_26(double *state, double *unused, double *out_4113962163327268773);
void car_H_26(double *state, double *unused, double *out_3461397360033596424);
void car_h_27(double *state, double *unused, double *out_4872566989291761472);
void car_H_27(double *state, double *unused, double *out_4898798419963987667);
void car_h_29(double *state, double *unused, double *out_6113618320168901819);
void car_H_29(double *state, double *unused, double *out_3185435693094436634);
void car_h_28(double *state, double *unused, double *out_6730034328266440598);
void car_H_28(double *state, double *unused, double *out_1896963323975093940);
void car_h_31(double *state, double *unused, double *out_5318696205617109844);
void car_H_31(double *state, double *unused, double *out_2835189257800244948);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}