/******************************************************************************
 *                      Code generated with sympy 1.6.1                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6825792617286569444);
void inv_err_fun(double *nom_x, double *true_x, double *out_6046387138583496308);
void H_mod_fun(double *state, double *out_9012640645338480329);
void f_fun(double *state, double dt, double *out_340157077377519544);
void F_fun(double *state, double dt, double *out_2398373229322174867);
void h_3(double *state, double *unused, double *out_4631671786377733337);
void H_3(double *state, double *unused, double *out_4387089402796251251);
void h_4(double *state, double *unused, double *out_1353166767537692330);
void H_4(double *state, double *unused, double *out_6612196665102172646);
void h_9(double *state, double *unused, double *out_595907118187607038);
void H_9(double *state, double *unused, double *out_7154695550387908277);
void h_10(double *state, double *unused, double *out_7774089303492200348);
void H_10(double *state, double *unused, double *out_7887157285401602435);
void h_12(double *state, double *unused, double *out_7237926975451160610);
void H_12(double *state, double *unused, double *out_6938745518256261876);
void h_31(double *state, double *unused, double *out_8935840877696597898);
void H_31(double *state, double *unused, double *out_4263655743112842055);
void h_32(double *state, double *unused, double *out_8553248422074692426);
void H_32(double *state, double *unused, double *out_1548967845896349272);
void h_13(double *state, double *unused, double *out_7840220119734319805);
void H_13(double *state, double *unused, double *out_1846478076261370112);
void h_14(double *state, double *unused, double *out_595907118187607038);
void H_14(double *state, double *unused, double *out_7154695550387908277);
void h_19(double *state, double *unused, double *out_1541548401697518227);
void H_19(double *state, double *unused, double *out_6859368672219212158);
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