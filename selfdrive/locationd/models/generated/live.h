/******************************************************************************
 *                      Code generated with sympy 1.6.1                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7404045535746273759);
void inv_err_fun(double *nom_x, double *true_x, double *out_529781785376239034);
void H_mod_fun(double *state, double *out_2464389214082132468);
void f_fun(double *state, double dt, double *out_8891146255040024923);
void F_fun(double *state, double dt, double *out_2574950744534855347);
void h_3(double *state, double *unused, double *out_7928145996990413027);
void H_3(double *state, double *unused, double *out_4492840592602009207);
void h_4(double *state, double *unused, double *out_2199518396757208591);
void H_4(double *state, double *unused, double *out_1518525225084737752);
void h_9(double *state, double *unused, double *out_3710774005327754548);
void H_9(double *state, double *unused, double *out_7559684016119101069);
void h_10(double *state, double *unused, double *out_2187632850582918429);
void H_10(double *state, double *unused, double *out_8370455280255123860);
void h_12(double *state, double *unused, double *out_2220448223072341040);
void H_12(double *state, double *unused, double *out_3625080736823947321);
void h_31(double *state, double *unused, double *out_7810930335785774518);
void H_31(double *state, double *unused, double *out_949990961680527500);
void h_32(double *state, double *unused, double *out_3293372641969964715);
void H_32(double *state, double *unused, double *out_9141798387973495299);
void h_13(double *state, double *unused, double *out_2642217957885711596);
void H_13(double *state, double *unused, double *out_6310234836257906015);
void h_14(double *state, double *unused, double *out_3710774005327754548);
void H_14(double *state, double *unused, double *out_7559684016119101069);
void h_19(double *state, double *unused, double *out_112114975523549058);
void H_19(double *state, double *unused, double *out_3127004165016669888);
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