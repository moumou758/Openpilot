#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_5544698516120850768);
void live_err_fun(double *nom_x, double *delta_x, double *out_5502510773169834387);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8753644477647806127);
void live_H_mod_fun(double *state, double *out_3788869937802374328);
void live_f_fun(double *state, double dt, double *out_8994811028469595336);
void live_F_fun(double *state, double dt, double *out_2893045864013244147);
void live_h_4(double *state, double *unused, double *out_7730454398728015739);
void live_H_4(double *state, double *unused, double *out_2984549033000731083);
void live_h_9(double *state, double *unused, double *out_1970478802870864100);
void live_H_9(double *state, double *unused, double *out_2743359386371140438);
void live_h_10(double *state, double *unused, double *out_413333148631038553);
void live_H_10(double *state, double *unused, double *out_219400141982964404);
void live_h_12(double *state, double *unused, double *out_3707020674496306971);
void live_H_12(double *state, double *unused, double *out_2363450007953137416);
void live_h_35(double *state, double *unused, double *out_7722688764641606055);
void live_H_35(double *state, double *unused, double *out_382113024371876293);
void live_h_32(double *state, double *unused, double *out_3557545687819092972);
void live_H_32(double *state, double *unused, double *out_513277809312616762);
void live_h_13(double *state, double *unused, double *out_2452969601872273316);
void live_H_13(double *state, double *unused, double *out_229850031329282141);
void live_h_14(double *state, double *unused, double *out_1970478802870864100);
void live_H_14(double *state, double *unused, double *out_2743359386371140438);
void live_h_33(double *state, double *unused, double *out_8583424363407538881);
void live_H_33(double *state, double *unused, double *out_3532670029010733897);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}