/******************************************************************************
 *                       Code generated with sympy 1.4                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5980832391639204543);
void inv_err_fun(double *nom_x, double *true_x, double *out_6826989552479107489);
void H_mod_fun(double *state, double *out_8110683897812433211);
void f_fun(double *state, double dt, double *out_4069854451852979138);
void F_fun(double *state, double dt, double *out_7623770766617305840);
void h_3(double *state, double *unused, double *out_2427214045126927708);
void H_3(double *state, double *unused, double *out_779360453928196549);
void h_4(double *state, double *unused, double *out_4292142519979909079);
void H_4(double *state, double *unused, double *out_3857823087446464656);
void h_9(double *state, double *unused, double *out_8207058366409708979);
void H_9(double *state, double *unused, double *out_7010613847979517473);
void h_10(double *state, double *unused, double *out_3072321918441379687);
void H_10(double *state, double *unused, double *out_6452358144786808675);
void h_12(double *state, double *unused, double *out_1319262723789628631);
void H_12(double *state, double *unused, double *out_8187049150470539825);
void h_31(double *state, double *unused, double *out_2620944055442535346);
void H_31(double *state, double *unused, double *out_7915579043777887705);
void h_32(double *state, double *unused, double *out_6237228692279939913);
void H_32(double *state, double *unused, double *out_4347107018828618641);
void h_13(double *state, double *unused, double *out_3855238713273724679);
void H_13(double *state, double *unused, double *out_2935411932104408784);
void h_14(double *state, double *unused, double *out_8207058366409708979);
void H_14(double *state, double *unused, double *out_7010613847979517473);
void h_19(double *state, double *unused, double *out_3175236293408975299);
void H_19(double *state, double *unused, double *out_6583693685032589605);
#define DIM 23
#define EDIM 22
#define MEDIM 22
typedef void (*Hfun)(double *, double *, double *);

void predict(double *x, double *P, double *Q, double dt);
const static double MAHA_THRESH_3 = 3.841459;
void update_3(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814728;
void update_4(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_9 = 7.814728;
void update_9(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_10 = 7.814728;
void update_10(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_12 = 7.814728;
void update_12(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_31 = 7.814728;
void update_31(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_32 = 9.487729;
void update_32(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_13 = 7.814728;
void update_13(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_14 = 7.814728;
void update_14(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_19 = 7.814728;
void update_19(double *, double *, double *, double *, double *);