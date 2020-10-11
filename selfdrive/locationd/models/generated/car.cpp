
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
 *                       Code generated with sympy 1.4                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5398496376153355038) {
   out_5398496376153355038[0] = delta_x[0] + nom_x[0];
   out_5398496376153355038[1] = delta_x[1] + nom_x[1];
   out_5398496376153355038[2] = delta_x[2] + nom_x[2];
   out_5398496376153355038[3] = delta_x[3] + nom_x[3];
   out_5398496376153355038[4] = delta_x[4] + nom_x[4];
   out_5398496376153355038[5] = delta_x[5] + nom_x[5];
   out_5398496376153355038[6] = delta_x[6] + nom_x[6];
   out_5398496376153355038[7] = delta_x[7] + nom_x[7];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_603376721115460562) {
   out_603376721115460562[0] = -nom_x[0] + true_x[0];
   out_603376721115460562[1] = -nom_x[1] + true_x[1];
   out_603376721115460562[2] = -nom_x[2] + true_x[2];
   out_603376721115460562[3] = -nom_x[3] + true_x[3];
   out_603376721115460562[4] = -nom_x[4] + true_x[4];
   out_603376721115460562[5] = -nom_x[5] + true_x[5];
   out_603376721115460562[6] = -nom_x[6] + true_x[6];
   out_603376721115460562[7] = -nom_x[7] + true_x[7];
}
void H_mod_fun(double *state, double *out_6317191626641793047) {
   out_6317191626641793047[0] = 1.0;
   out_6317191626641793047[1] = 0.0;
   out_6317191626641793047[2] = 0.0;
   out_6317191626641793047[3] = 0.0;
   out_6317191626641793047[4] = 0.0;
   out_6317191626641793047[5] = 0.0;
   out_6317191626641793047[6] = 0.0;
   out_6317191626641793047[7] = 0.0;
   out_6317191626641793047[8] = 0.0;
   out_6317191626641793047[9] = 1.0;
   out_6317191626641793047[10] = 0.0;
   out_6317191626641793047[11] = 0.0;
   out_6317191626641793047[12] = 0.0;
   out_6317191626641793047[13] = 0.0;
   out_6317191626641793047[14] = 0.0;
   out_6317191626641793047[15] = 0.0;
   out_6317191626641793047[16] = 0.0;
   out_6317191626641793047[17] = 0.0;
   out_6317191626641793047[18] = 1.0;
   out_6317191626641793047[19] = 0.0;
   out_6317191626641793047[20] = 0.0;
   out_6317191626641793047[21] = 0.0;
   out_6317191626641793047[22] = 0.0;
   out_6317191626641793047[23] = 0.0;
   out_6317191626641793047[24] = 0.0;
   out_6317191626641793047[25] = 0.0;
   out_6317191626641793047[26] = 0.0;
   out_6317191626641793047[27] = 1.0;
   out_6317191626641793047[28] = 0.0;
   out_6317191626641793047[29] = 0.0;
   out_6317191626641793047[30] = 0.0;
   out_6317191626641793047[31] = 0.0;
   out_6317191626641793047[32] = 0.0;
   out_6317191626641793047[33] = 0.0;
   out_6317191626641793047[34] = 0.0;
   out_6317191626641793047[35] = 0.0;
   out_6317191626641793047[36] = 1.0;
   out_6317191626641793047[37] = 0.0;
   out_6317191626641793047[38] = 0.0;
   out_6317191626641793047[39] = 0.0;
   out_6317191626641793047[40] = 0.0;
   out_6317191626641793047[41] = 0.0;
   out_6317191626641793047[42] = 0.0;
   out_6317191626641793047[43] = 0.0;
   out_6317191626641793047[44] = 0.0;
   out_6317191626641793047[45] = 1.0;
   out_6317191626641793047[46] = 0.0;
   out_6317191626641793047[47] = 0.0;
   out_6317191626641793047[48] = 0.0;
   out_6317191626641793047[49] = 0.0;
   out_6317191626641793047[50] = 0.0;
   out_6317191626641793047[51] = 0.0;
   out_6317191626641793047[52] = 0.0;
   out_6317191626641793047[53] = 0.0;
   out_6317191626641793047[54] = 1.0;
   out_6317191626641793047[55] = 0.0;
   out_6317191626641793047[56] = 0.0;
   out_6317191626641793047[57] = 0.0;
   out_6317191626641793047[58] = 0.0;
   out_6317191626641793047[59] = 0.0;
   out_6317191626641793047[60] = 0.0;
   out_6317191626641793047[61] = 0.0;
   out_6317191626641793047[62] = 0.0;
   out_6317191626641793047[63] = 1.0;
}
void f_fun(double *state, double dt, double *out_2165550978238609978) {
   out_2165550978238609978[0] = state[0];
   out_2165550978238609978[1] = state[1];
   out_2165550978238609978[2] = state[2];
   out_2165550978238609978[3] = state[3];
   out_2165550978238609978[4] = state[4];
   out_2165550978238609978[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2165550978238609978[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2165550978238609978[7] = state[7];
}
void F_fun(double *state, double dt, double *out_340044939014920214) {
   out_340044939014920214[0] = 1;
   out_340044939014920214[1] = 0;
   out_340044939014920214[2] = 0;
   out_340044939014920214[3] = 0;
   out_340044939014920214[4] = 0;
   out_340044939014920214[5] = 0;
   out_340044939014920214[6] = 0;
   out_340044939014920214[7] = 0;
   out_340044939014920214[8] = 0;
   out_340044939014920214[9] = 1;
   out_340044939014920214[10] = 0;
   out_340044939014920214[11] = 0;
   out_340044939014920214[12] = 0;
   out_340044939014920214[13] = 0;
   out_340044939014920214[14] = 0;
   out_340044939014920214[15] = 0;
   out_340044939014920214[16] = 0;
   out_340044939014920214[17] = 0;
   out_340044939014920214[18] = 1;
   out_340044939014920214[19] = 0;
   out_340044939014920214[20] = 0;
   out_340044939014920214[21] = 0;
   out_340044939014920214[22] = 0;
   out_340044939014920214[23] = 0;
   out_340044939014920214[24] = 0;
   out_340044939014920214[25] = 0;
   out_340044939014920214[26] = 0;
   out_340044939014920214[27] = 1;
   out_340044939014920214[28] = 0;
   out_340044939014920214[29] = 0;
   out_340044939014920214[30] = 0;
   out_340044939014920214[31] = 0;
   out_340044939014920214[32] = 0;
   out_340044939014920214[33] = 0;
   out_340044939014920214[34] = 0;
   out_340044939014920214[35] = 0;
   out_340044939014920214[36] = 1;
   out_340044939014920214[37] = 0;
   out_340044939014920214[38] = 0;
   out_340044939014920214[39] = 0;
   out_340044939014920214[40] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_340044939014920214[41] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_340044939014920214[42] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_340044939014920214[43] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_340044939014920214[44] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_340044939014920214[45] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_340044939014920214[46] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_340044939014920214[47] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_340044939014920214[48] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_340044939014920214[49] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_340044939014920214[50] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_340044939014920214[51] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_340044939014920214[52] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_340044939014920214[53] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_340044939014920214[54] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_340044939014920214[55] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_340044939014920214[56] = 0;
   out_340044939014920214[57] = 0;
   out_340044939014920214[58] = 0;
   out_340044939014920214[59] = 0;
   out_340044939014920214[60] = 0;
   out_340044939014920214[61] = 0;
   out_340044939014920214[62] = 0;
   out_340044939014920214[63] = 1;
}
void h_25(double *state, double *unused, double *out_4999861906478216823) {
   out_4999861906478216823[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7402585258441748385) {
   out_7402585258441748385[0] = 0;
   out_7402585258441748385[1] = 0;
   out_7402585258441748385[2] = 0;
   out_7402585258441748385[3] = 0;
   out_7402585258441748385[4] = 0;
   out_7402585258441748385[5] = 0;
   out_7402585258441748385[6] = 1;
   out_7402585258441748385[7] = 0;
}
void h_24(double *state, double *unused, double *out_6488777785824412091) {
   out_6488777785824412091[0] = state[4];
   out_6488777785824412091[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1186921814037369331) {
   out_1186921814037369331[0] = 0;
   out_1186921814037369331[1] = 0;
   out_1186921814037369331[2] = 0;
   out_1186921814037369331[3] = 0;
   out_1186921814037369331[4] = 1;
   out_1186921814037369331[5] = 0;
   out_1186921814037369331[6] = 0;
   out_1186921814037369331[7] = 0;
   out_1186921814037369331[8] = 0;
   out_1186921814037369331[9] = 0;
   out_1186921814037369331[10] = 0;
   out_1186921814037369331[11] = 0;
   out_1186921814037369331[12] = 0;
   out_1186921814037369331[13] = 1;
   out_1186921814037369331[14] = 0;
   out_1186921814037369331[15] = 0;
}
void h_30(double *state, double *unused, double *out_1947467846785327627) {
   out_1947467846785327627[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8761833006428412341) {
   out_8761833006428412341[0] = 0;
   out_8761833006428412341[1] = 0;
   out_8761833006428412341[2] = 0;
   out_8761833006428412341[3] = 0;
   out_8761833006428412341[4] = 1;
   out_8761833006428412341[5] = 0;
   out_8761833006428412341[6] = 0;
   out_8761833006428412341[7] = 0;
}
void h_26(double *state, double *unused, double *out_6247319590493932674) {
   out_6247319590493932674[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3988533402666019031) {
   out_3988533402666019031[0] = 0;
   out_3988533402666019031[1] = 0;
   out_3988533402666019031[2] = 0;
   out_3988533402666019031[3] = 0;
   out_3988533402666019031[4] = 0;
   out_3988533402666019031[5] = 0;
   out_3988533402666019031[6] = 0;
   out_3988533402666019031[7] = 1;
}
void h_27(double *state, double *unused, double *out_4375786520578969210) {
   out_4375786520578969210[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4533734753923385989) {
   out_4533734753923385989[0] = 0;
   out_4533734753923385989[1] = 0;
   out_4533734753923385989[2] = 0;
   out_4533734753923385989[3] = 1;
   out_4533734753923385989[4] = 0;
   out_4533734753923385989[5] = 0;
   out_4533734753923385989[6] = 0;
   out_4533734753923385989[7] = 0;
}
void h_29(double *state, double *unused, double *out_3745192795341832248) {
   out_3745192795341832248[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1485838960018223057) {
   out_1485838960018223057[0] = 0;
   out_1485838960018223057[1] = 1;
   out_1485838960018223057[2] = 0;
   out_1485838960018223057[3] = 0;
   out_1485838960018223057[4] = 0;
   out_1485838960018223057[5] = 0;
   out_1485838960018223057[6] = 0;
   out_1485838960018223057[7] = 0;
}
void h_28(double *state, double *unused, double *out_5312863564652293855) {
   out_5312863564652293855[0] = state[5];
   out_5312863564652293855[1] = state[6];
}
void H_28(double *state, double *unused, double *out_8490632408783129801) {
   out_8490632408783129801[0] = 0;
   out_8490632408783129801[1] = 0;
   out_8490632408783129801[2] = 0;
   out_8490632408783129801[3] = 0;
   out_8490632408783129801[4] = 0;
   out_8490632408783129801[5] = 1;
   out_8490632408783129801[6] = 0;
   out_8490632408783129801[7] = 0;
   out_8490632408783129801[8] = 0;
   out_8490632408783129801[9] = 0;
   out_8490632408783129801[10] = 0;
   out_8490632408783129801[11] = 0;
   out_8490632408783129801[12] = 0;
   out_8490632408783129801[13] = 0;
   out_8490632408783129801[14] = 1;
   out_8490632408783129801[15] = 0;
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
