/******************************************************************************
 *                      Code generated with sympy 1.6.1                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6335456881470860121);
void inv_err_fun(double *nom_x, double *true_x, double *out_6354175412188589980);
void H_mod_fun(double *state, double *out_1591702516712346505);
void f_fun(double *state, double dt, double *out_3808020937023290417);
void F_fun(double *state, double dt, double *out_5619221698054243802);
void h_25(double *state, double *unused, double *out_6838715796604332757);
void H_25(double *state, double *unused, double *out_3915001296409223570);
void h_24(double *state, double *unused, double *out_4290608182807608693);
void H_24(double *state, double *unused, double *out_1207853802661120319);
void h_30(double *state, double *unused, double *out_6681529128253203612);
void H_30(double *state, double *unused, double *out_4917843866351144766);
void h_26(double *state, double *unused, double *out_6078213243576726095);
void H_26(double *state, double *unused, double *out_1789591858529985703);
void h_27(double *state, double *unused, double *out_7182667361806987326);
void H_27(double *state, double *unused, double *out_3630261878514519454);
void h_29(double *state, double *unused, double *out_4077123807444505961);
void H_29(double *state, double *unused, double *out_1094045097222312813);
void h_28(double *state, double *unused, double *out_3954764283378591393);
void H_28(double *state, double *unused, double *out_3175028504177079606);
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
