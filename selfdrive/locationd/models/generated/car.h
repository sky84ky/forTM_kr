/******************************************************************************
 *                       Code generated with sympy 1.4                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5398496376153355038);
void inv_err_fun(double *nom_x, double *true_x, double *out_603376721115460562);
void H_mod_fun(double *state, double *out_6317191626641793047);
void f_fun(double *state, double dt, double *out_2165550978238609978);
void F_fun(double *state, double dt, double *out_340044939014920214);
void h_25(double *state, double *unused, double *out_4999861906478216823);
void H_25(double *state, double *unused, double *out_7402585258441748385);
void h_24(double *state, double *unused, double *out_6488777785824412091);
void H_24(double *state, double *unused, double *out_1186921814037369331);
void h_30(double *state, double *unused, double *out_1947467846785327627);
void H_30(double *state, double *unused, double *out_8761833006428412341);
void h_26(double *state, double *unused, double *out_6247319590493932674);
void H_26(double *state, double *unused, double *out_3988533402666019031);
void h_27(double *state, double *unused, double *out_4375786520578969210);
void H_27(double *state, double *unused, double *out_4533734753923385989);
void h_29(double *state, double *unused, double *out_3745192795341832248);
void H_29(double *state, double *unused, double *out_1485838960018223057);
void h_28(double *state, double *unused, double *out_5312863564652293855);
void H_28(double *state, double *unused, double *out_8490632408783129801);
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
