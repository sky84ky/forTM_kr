/******************************************************************************
 *                       Code generated with sympy 1.4                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_1381508745752257481);
void inv_err_fun(double *nom_x, double *true_x, double *out_3389848132089429673);
void H_mod_fun(double *state, double *out_5396539298439943387);
void f_fun(double *state, double dt, double *out_8652010582931564448);
void F_fun(double *state, double dt, double *out_5469067056279615360);
void h_3(double *state, double *unused, double *out_1763609194938360627);
void H_3(double *state, double *unused, double *out_7922774325543294151);
void h_4(double *state, double *unused, double *out_8209518604547368870);
void H_4(double *state, double *unused, double *out_3250312354647466145);
void h_9(double *state, double *unused, double *out_1934357434744742345);
void H_9(double *state, double *unused, double *out_7500974837772131769);
void h_10(double *state, double *unused, double *out_3181309449588727870);
void H_10(double *state, double *unused, double *out_2642400929753597256);
void h_12(double *state, double *unused, double *out_6980028907550595787);
void H_12(double *state, double *unused, double *out_7863996319371325431);
void h_13(double *state, double *unused, double *out_4219741788363980831);
void H_13(double *state, double *unused, double *out_5978383697775324986);
void h_14(double *state, double *unused, double *out_1934357434744742345);
void H_14(double *state, double *unused, double *out_7500974837772131769);
void h_19(double *state, double *unused, double *out_7110371401971763279);
void H_19(double *state, double *unused, double *out_4126521885742614707);
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
const static double MAHA_THRESH_13 = 7.814728;
void update_13(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_14 = 7.814728;
void update_14(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_19 = 7.814728;
void update_19(double *, double *, double *, double *, double *);