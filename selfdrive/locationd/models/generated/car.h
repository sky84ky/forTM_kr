/******************************************************************************
 *                      Code generated with sympy 1.6.1                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_500885191720028699);
void inv_err_fun(double *nom_x, double *true_x, double *out_5921937009728406874);
void H_mod_fun(double *state, double *out_7991025343011061386);
void f_fun(double *state, double dt, double *out_5997976432719015641);
void F_fun(double *state, double dt, double *out_4946059235232519695);
void h_25(double *state, double *unused, double *out_4532039487739016626);
void H_25(double *state, double *unused, double *out_7548107073685581365);
void h_24(double *state, double *unused, double *out_801354343157696891);
void H_24(double *state, double *unused, double *out_1270247387681230463);
void h_30(double *state, double *unused, double *out_7759257390129874861);
void H_30(double *state, double *unused, double *out_2065791837263601915);
void h_26(double *state, double *unused, double *out_2001272476136136905);
void H_26(double *state, double *unused, double *out_8854342845640422510);
void h_27(double *state, double *unused, double *out_5326541904165693582);
void H_27(double *state, double *unused, double *out_3353373825100227227);
void h_29(double *state, double *unused, double *out_5051347841881187693);
void H_29(double *state, double *unused, double *out_5889590606392433868);
void h_28(double *state, double *unused, double *out_6653069047710447711);
void H_28(double *state, double *unused, double *out_8288079865917725329);
#define DIM 8
#define EDIM 8
#define MEDIM 8
typedef void (*Hfun)(double *, double *, double *);

void predict(double *x, double *P, double *Q, double dt);
const static double MAHA_THRESH_25 = 3.841459;
void update_25(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_24 = 5.991465;
void update_24(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_30 = 3.841459;
void update_30(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_26 = 3.841459;
void update_26(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_27 = 3.841459;
void update_27(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_29 = 3.841459;
void update_29(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_28 = 5.991465;
void update_28(double *, double *, double *, double *, double *);
void set_mass(double x);

void set_rotational_inertia(double x);

void set_center_to_front(double x);

void set_center_to_rear(double x);

void set_stiffness_front(double x);

void set_stiffness_rear(double x);
