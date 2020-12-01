
extern "C"{

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}

}
extern "C" {
#include <math.h>
/******************************************************************************
 *                      Code generated with sympy 1.6.1                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_500885191720028699) {
   out_500885191720028699[0] = delta_x[0] + nom_x[0];
   out_500885191720028699[1] = delta_x[1] + nom_x[1];
   out_500885191720028699[2] = delta_x[2] + nom_x[2];
   out_500885191720028699[3] = delta_x[3] + nom_x[3];
   out_500885191720028699[4] = delta_x[4] + nom_x[4];
   out_500885191720028699[5] = delta_x[5] + nom_x[5];
   out_500885191720028699[6] = delta_x[6] + nom_x[6];
   out_500885191720028699[7] = delta_x[7] + nom_x[7];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5921937009728406874) {
   out_5921937009728406874[0] = -nom_x[0] + true_x[0];
   out_5921937009728406874[1] = -nom_x[1] + true_x[1];
   out_5921937009728406874[2] = -nom_x[2] + true_x[2];
   out_5921937009728406874[3] = -nom_x[3] + true_x[3];
   out_5921937009728406874[4] = -nom_x[4] + true_x[4];
   out_5921937009728406874[5] = -nom_x[5] + true_x[5];
   out_5921937009728406874[6] = -nom_x[6] + true_x[6];
   out_5921937009728406874[7] = -nom_x[7] + true_x[7];
}
void H_mod_fun(double *state, double *out_7991025343011061386) {
   out_7991025343011061386[0] = 1.0;
   out_7991025343011061386[1] = 0.0;
   out_7991025343011061386[2] = 0.0;
   out_7991025343011061386[3] = 0.0;
   out_7991025343011061386[4] = 0.0;
   out_7991025343011061386[5] = 0.0;
   out_7991025343011061386[6] = 0.0;
   out_7991025343011061386[7] = 0.0;
   out_7991025343011061386[8] = 0.0;
   out_7991025343011061386[9] = 1.0;
   out_7991025343011061386[10] = 0.0;
   out_7991025343011061386[11] = 0.0;
   out_7991025343011061386[12] = 0.0;
   out_7991025343011061386[13] = 0.0;
   out_7991025343011061386[14] = 0.0;
   out_7991025343011061386[15] = 0.0;
   out_7991025343011061386[16] = 0.0;
   out_7991025343011061386[17] = 0.0;
   out_7991025343011061386[18] = 1.0;
   out_7991025343011061386[19] = 0.0;
   out_7991025343011061386[20] = 0.0;
   out_7991025343011061386[21] = 0.0;
   out_7991025343011061386[22] = 0.0;
   out_7991025343011061386[23] = 0.0;
   out_7991025343011061386[24] = 0.0;
   out_7991025343011061386[25] = 0.0;
   out_7991025343011061386[26] = 0.0;
   out_7991025343011061386[27] = 1.0;
   out_7991025343011061386[28] = 0.0;
   out_7991025343011061386[29] = 0.0;
   out_7991025343011061386[30] = 0.0;
   out_7991025343011061386[31] = 0.0;
   out_7991025343011061386[32] = 0.0;
   out_7991025343011061386[33] = 0.0;
   out_7991025343011061386[34] = 0.0;
   out_7991025343011061386[35] = 0.0;
   out_7991025343011061386[36] = 1.0;
   out_7991025343011061386[37] = 0.0;
   out_7991025343011061386[38] = 0.0;
   out_7991025343011061386[39] = 0.0;
   out_7991025343011061386[40] = 0.0;
   out_7991025343011061386[41] = 0.0;
   out_7991025343011061386[42] = 0.0;
   out_7991025343011061386[43] = 0.0;
   out_7991025343011061386[44] = 0.0;
   out_7991025343011061386[45] = 1.0;
   out_7991025343011061386[46] = 0.0;
   out_7991025343011061386[47] = 0.0;
   out_7991025343011061386[48] = 0.0;
   out_7991025343011061386[49] = 0.0;
   out_7991025343011061386[50] = 0.0;
   out_7991025343011061386[51] = 0.0;
   out_7991025343011061386[52] = 0.0;
   out_7991025343011061386[53] = 0.0;
   out_7991025343011061386[54] = 1.0;
   out_7991025343011061386[55] = 0.0;
   out_7991025343011061386[56] = 0.0;
   out_7991025343011061386[57] = 0.0;
   out_7991025343011061386[58] = 0.0;
   out_7991025343011061386[59] = 0.0;
   out_7991025343011061386[60] = 0.0;
   out_7991025343011061386[61] = 0.0;
   out_7991025343011061386[62] = 0.0;
   out_7991025343011061386[63] = 1.0;
}
void f_fun(double *state, double dt, double *out_5997976432719015641) {
   out_5997976432719015641[0] = state[0];
   out_5997976432719015641[1] = state[1];
   out_5997976432719015641[2] = state[2];
   out_5997976432719015641[3] = state[3];
   out_5997976432719015641[4] = state[4];
   out_5997976432719015641[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5997976432719015641[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5997976432719015641[7] = state[7];
}
void F_fun(double *state, double dt, double *out_4946059235232519695) {
   out_4946059235232519695[0] = 1;
   out_4946059235232519695[1] = 0;
   out_4946059235232519695[2] = 0;
   out_4946059235232519695[3] = 0;
   out_4946059235232519695[4] = 0;
   out_4946059235232519695[5] = 0;
   out_4946059235232519695[6] = 0;
   out_4946059235232519695[7] = 0;
   out_4946059235232519695[8] = 0;
   out_4946059235232519695[9] = 1;
   out_4946059235232519695[10] = 0;
   out_4946059235232519695[11] = 0;
   out_4946059235232519695[12] = 0;
   out_4946059235232519695[13] = 0;
   out_4946059235232519695[14] = 0;
   out_4946059235232519695[15] = 0;
   out_4946059235232519695[16] = 0;
   out_4946059235232519695[17] = 0;
   out_4946059235232519695[18] = 1;
   out_4946059235232519695[19] = 0;
   out_4946059235232519695[20] = 0;
   out_4946059235232519695[21] = 0;
   out_4946059235232519695[22] = 0;
   out_4946059235232519695[23] = 0;
   out_4946059235232519695[24] = 0;
   out_4946059235232519695[25] = 0;
   out_4946059235232519695[26] = 0;
   out_4946059235232519695[27] = 1;
   out_4946059235232519695[28] = 0;
   out_4946059235232519695[29] = 0;
   out_4946059235232519695[30] = 0;
   out_4946059235232519695[31] = 0;
   out_4946059235232519695[32] = 0;
   out_4946059235232519695[33] = 0;
   out_4946059235232519695[34] = 0;
   out_4946059235232519695[35] = 0;
   out_4946059235232519695[36] = 1;
   out_4946059235232519695[37] = 0;
   out_4946059235232519695[38] = 0;
   out_4946059235232519695[39] = 0;
   out_4946059235232519695[40] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_4946059235232519695[41] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_4946059235232519695[42] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4946059235232519695[43] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4946059235232519695[44] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_4946059235232519695[45] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_4946059235232519695[46] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_4946059235232519695[47] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_4946059235232519695[48] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_4946059235232519695[49] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_4946059235232519695[50] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4946059235232519695[51] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4946059235232519695[52] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_4946059235232519695[53] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_4946059235232519695[54] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_4946059235232519695[55] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4946059235232519695[56] = 0;
   out_4946059235232519695[57] = 0;
   out_4946059235232519695[58] = 0;
   out_4946059235232519695[59] = 0;
   out_4946059235232519695[60] = 0;
   out_4946059235232519695[61] = 0;
   out_4946059235232519695[62] = 0;
   out_4946059235232519695[63] = 1;
}
void h_25(double *state, double *unused, double *out_4532039487739016626) {
   out_4532039487739016626[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7548107073685581365) {
   out_7548107073685581365[0] = 0;
   out_7548107073685581365[1] = 0;
   out_7548107073685581365[2] = 0;
   out_7548107073685581365[3] = 0;
   out_7548107073685581365[4] = 0;
   out_7548107073685581365[5] = 0;
   out_7548107073685581365[6] = 1;
   out_7548107073685581365[7] = 0;
}
void h_24(double *state, double *unused, double *out_801354343157696891) {
   out_801354343157696891[0] = state[4];
   out_801354343157696891[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1270247387681230463) {
   out_1270247387681230463[0] = 0;
   out_1270247387681230463[1] = 0;
   out_1270247387681230463[2] = 0;
   out_1270247387681230463[3] = 0;
   out_1270247387681230463[4] = 1;
   out_1270247387681230463[5] = 0;
   out_1270247387681230463[6] = 0;
   out_1270247387681230463[7] = 0;
   out_1270247387681230463[8] = 0;
   out_1270247387681230463[9] = 0;
   out_1270247387681230463[10] = 0;
   out_1270247387681230463[11] = 0;
   out_1270247387681230463[12] = 0;
   out_1270247387681230463[13] = 1;
   out_1270247387681230463[14] = 0;
   out_1270247387681230463[15] = 0;
}
void h_30(double *state, double *unused, double *out_7759257390129874861) {
   out_7759257390129874861[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2065791837263601915) {
   out_2065791837263601915[0] = 0;
   out_2065791837263601915[1] = 0;
   out_2065791837263601915[2] = 0;
   out_2065791837263601915[3] = 0;
   out_2065791837263601915[4] = 1;
   out_2065791837263601915[5] = 0;
   out_2065791837263601915[6] = 0;
   out_2065791837263601915[7] = 0;
}
void h_26(double *state, double *unused, double *out_2001272476136136905) {
   out_2001272476136136905[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8854342845640422510) {
   out_8854342845640422510[0] = 0;
   out_8854342845640422510[1] = 0;
   out_8854342845640422510[2] = 0;
   out_8854342845640422510[3] = 0;
   out_8854342845640422510[4] = 0;
   out_8854342845640422510[5] = 0;
   out_8854342845640422510[6] = 0;
   out_8854342845640422510[7] = 1;
}
void h_27(double *state, double *unused, double *out_5326541904165693582) {
   out_5326541904165693582[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3353373825100227227) {
   out_3353373825100227227[0] = 0;
   out_3353373825100227227[1] = 0;
   out_3353373825100227227[2] = 0;
   out_3353373825100227227[3] = 1;
   out_3353373825100227227[4] = 0;
   out_3353373825100227227[5] = 0;
   out_3353373825100227227[6] = 0;
   out_3353373825100227227[7] = 0;
}
void h_29(double *state, double *unused, double *out_5051347841881187693) {
   out_5051347841881187693[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5889590606392433868) {
   out_5889590606392433868[0] = 0;
   out_5889590606392433868[1] = 1;
   out_5889590606392433868[2] = 0;
   out_5889590606392433868[3] = 0;
   out_5889590606392433868[4] = 0;
   out_5889590606392433868[5] = 0;
   out_5889590606392433868[6] = 0;
   out_5889590606392433868[7] = 0;
}
void h_28(double *state, double *unused, double *out_6653069047710447711) {
   out_6653069047710447711[0] = state[5];
   out_6653069047710447711[1] = state[6];
}
void H_28(double *state, double *unused, double *out_8288079865917725329) {
   out_8288079865917725329[0] = 0;
   out_8288079865917725329[1] = 0;
   out_8288079865917725329[2] = 0;
   out_8288079865917725329[3] = 0;
   out_8288079865917725329[4] = 0;
   out_8288079865917725329[5] = 1;
   out_8288079865917725329[6] = 0;
   out_8288079865917725329[7] = 0;
   out_8288079865917725329[8] = 0;
   out_8288079865917725329[9] = 0;
   out_8288079865917725329[10] = 0;
   out_8288079865917725329[11] = 0;
   out_8288079865917725329[12] = 0;
   out_8288079865917725329[13] = 0;
   out_8288079865917725329[14] = 1;
   out_8288079865917725329[15] = 0;
}
}

extern "C"{
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
}

#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}



extern "C"{

      void update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
      }
    
      void update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<2,3,0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
      }
    
      void update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
      }
    
      void update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
      }
    
      void update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
      }
    
      void update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
      }
    
      void update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<2,3,0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
      }
    
}
