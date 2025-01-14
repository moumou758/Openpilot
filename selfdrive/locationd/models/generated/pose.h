#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_2081385545714313315);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_1086297426282480324);
void pose_H_mod_fun(double *state, double *out_8714192956250408026);
void pose_f_fun(double *state, double dt, double *out_5560781299359714526);
void pose_F_fun(double *state, double dt, double *out_2772014030390626314);
void pose_h_4(double *state, double *unused, double *out_6530784052392321189);
void pose_H_4(double *state, double *unused, double *out_8978390115404430983);
void pose_h_10(double *state, double *unused, double *out_6173900798648768843);
void pose_H_10(double *state, double *unused, double *out_5102189638329563187);
void pose_h_13(double *state, double *unused, double *out_7476437276712303054);
void pose_H_13(double *state, double *unused, double *out_5766116290072098182);
void pose_h_14(double *state, double *unused, double *out_8287580119295854294);
void pose_H_14(double *state, double *unused, double *out_5015149259064946454);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}