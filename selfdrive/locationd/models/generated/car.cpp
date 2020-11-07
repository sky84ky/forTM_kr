
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
void err_fun(double *nom_x, double *delta_x, double *out_6335456881470860121) {
   out_6335456881470860121[0] = delta_x[0] + nom_x[0];
   out_6335456881470860121[1] = delta_x[1] + nom_x[1];
   out_6335456881470860121[2] = delta_x[2] + nom_x[2];
   out_6335456881470860121[3] = delta_x[3] + nom_x[3];
   out_6335456881470860121[4] = delta_x[4] + nom_x[4];
   out_6335456881470860121[5] = delta_x[5] + nom_x[5];
   out_6335456881470860121[6] = delta_x[6] + nom_x[6];
   out_6335456881470860121[7] = delta_x[7] + nom_x[7];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6354175412188589980) {
   out_6354175412188589980[0] = -nom_x[0] + true_x[0];
   out_6354175412188589980[1] = -nom_x[1] + true_x[1];
   out_6354175412188589980[2] = -nom_x[2] + true_x[2];
   out_6354175412188589980[3] = -nom_x[3] + true_x[3];
   out_6354175412188589980[4] = -nom_x[4] + true_x[4];
   out_6354175412188589980[5] = -nom_x[5] + true_x[5];
   out_6354175412188589980[6] = -nom_x[6] + true_x[6];
   out_6354175412188589980[7] = -nom_x[7] + true_x[7];
}
void H_mod_fun(double *state, double *out_1591702516712346505) {
   out_1591702516712346505[0] = 1.0;
   out_1591702516712346505[1] = 0.0;
   out_1591702516712346505[2] = 0.0;
   out_1591702516712346505[3] = 0.0;
   out_1591702516712346505[4] = 0.0;
   out_1591702516712346505[5] = 0.0;
   out_1591702516712346505[6] = 0.0;
   out_1591702516712346505[7] = 0.0;
   out_1591702516712346505[8] = 0.0;
   out_1591702516712346505[9] = 1.0;
   out_1591702516712346505[10] = 0.0;
   out_1591702516712346505[11] = 0.0;
   out_1591702516712346505[12] = 0.0;
   out_1591702516712346505[13] = 0.0;
   out_1591702516712346505[14] = 0.0;
   out_1591702516712346505[15] = 0.0;
   out_1591702516712346505[16] = 0.0;
   out_1591702516712346505[17] = 0.0;
   out_1591702516712346505[18] = 1.0;
   out_1591702516712346505[19] = 0.0;
   out_1591702516712346505[20] = 0.0;
   out_1591702516712346505[21] = 0.0;
   out_1591702516712346505[22] = 0.0;
   out_1591702516712346505[23] = 0.0;
   out_1591702516712346505[24] = 0.0;
   out_1591702516712346505[25] = 0.0;
   out_1591702516712346505[26] = 0.0;
   out_1591702516712346505[27] = 1.0;
   out_1591702516712346505[28] = 0.0;
   out_1591702516712346505[29] = 0.0;
   out_1591702516712346505[30] = 0.0;
   out_1591702516712346505[31] = 0.0;
   out_1591702516712346505[32] = 0.0;
   out_1591702516712346505[33] = 0.0;
   out_1591702516712346505[34] = 0.0;
   out_1591702516712346505[35] = 0.0;
   out_1591702516712346505[36] = 1.0;
   out_1591702516712346505[37] = 0.0;
   out_1591702516712346505[38] = 0.0;
   out_1591702516712346505[39] = 0.0;
   out_1591702516712346505[40] = 0.0;
   out_1591702516712346505[41] = 0.0;
   out_1591702516712346505[42] = 0.0;
   out_1591702516712346505[43] = 0.0;
   out_1591702516712346505[44] = 0.0;
   out_1591702516712346505[45] = 1.0;
   out_1591702516712346505[46] = 0.0;
   out_1591702516712346505[47] = 0.0;
   out_1591702516712346505[48] = 0.0;
   out_1591702516712346505[49] = 0.0;
   out_1591702516712346505[50] = 0.0;
   out_1591702516712346505[51] = 0.0;
   out_1591702516712346505[52] = 0.0;
   out_1591702516712346505[53] = 0.0;
   out_1591702516712346505[54] = 1.0;
   out_1591702516712346505[55] = 0.0;
   out_1591702516712346505[56] = 0.0;
   out_1591702516712346505[57] = 0.0;
   out_1591702516712346505[58] = 0.0;
   out_1591702516712346505[59] = 0.0;
   out_1591702516712346505[60] = 0.0;
   out_1591702516712346505[61] = 0.0;
   out_1591702516712346505[62] = 0.0;
   out_1591702516712346505[63] = 1.0;
}
void f_fun(double *state, double dt, double *out_3808020937023290417) {
   out_3808020937023290417[0] = state[0];
   out_3808020937023290417[1] = state[1];
   out_3808020937023290417[2] = state[2];
   out_3808020937023290417[3] = state[3];
   out_3808020937023290417[4] = state[4];
   out_3808020937023290417[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3808020937023290417[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3808020937023290417[7] = state[7];
}
void F_fun(double *state, double dt, double *out_5619221698054243802) {
   out_5619221698054243802[0] = 1;
   out_5619221698054243802[1] = 0;
   out_5619221698054243802[2] = 0;
   out_5619221698054243802[3] = 0;
   out_5619221698054243802[4] = 0;
   out_5619221698054243802[5] = 0;
   out_5619221698054243802[6] = 0;
   out_5619221698054243802[7] = 0;
   out_5619221698054243802[8] = 0;
   out_5619221698054243802[9] = 1;
   out_5619221698054243802[10] = 0;
   out_5619221698054243802[11] = 0;
   out_5619221698054243802[12] = 0;
   out_5619221698054243802[13] = 0;
   out_5619221698054243802[14] = 0;
   out_5619221698054243802[15] = 0;
   out_5619221698054243802[16] = 0;
   out_5619221698054243802[17] = 0;
   out_5619221698054243802[18] = 1;
   out_5619221698054243802[19] = 0;
   out_5619221698054243802[20] = 0;
   out_5619221698054243802[21] = 0;
   out_5619221698054243802[22] = 0;
   out_5619221698054243802[23] = 0;
   out_5619221698054243802[24] = 0;
   out_5619221698054243802[25] = 0;
   out_5619221698054243802[26] = 0;
   out_5619221698054243802[27] = 1;
   out_5619221698054243802[28] = 0;
   out_5619221698054243802[29] = 0;
   out_5619221698054243802[30] = 0;
   out_5619221698054243802[31] = 0;
   out_5619221698054243802[32] = 0;
   out_5619221698054243802[33] = 0;
   out_5619221698054243802[34] = 0;
   out_5619221698054243802[35] = 0;
   out_5619221698054243802[36] = 1;
   out_5619221698054243802[37] = 0;
   out_5619221698054243802[38] = 0;
   out_5619221698054243802[39] = 0;
   out_5619221698054243802[40] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5619221698054243802[41] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5619221698054243802[42] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5619221698054243802[43] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5619221698054243802[44] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5619221698054243802[45] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5619221698054243802[46] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5619221698054243802[47] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5619221698054243802[48] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5619221698054243802[49] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5619221698054243802[50] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5619221698054243802[51] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5619221698054243802[52] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5619221698054243802[53] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5619221698054243802[54] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5619221698054243802[55] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5619221698054243802[56] = 0;
   out_5619221698054243802[57] = 0;
   out_5619221698054243802[58] = 0;
   out_5619221698054243802[59] = 0;
   out_5619221698054243802[60] = 0;
   out_5619221698054243802[61] = 0;
   out_5619221698054243802[62] = 0;
   out_5619221698054243802[63] = 1;
}
void h_25(double *state, double *unused, double *out_6838715796604332757) {
   out_6838715796604332757[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3915001296409223570) {
   out_3915001296409223570[0] = 0;
   out_3915001296409223570[1] = 0;
   out_3915001296409223570[2] = 0;
   out_3915001296409223570[3] = 0;
   out_3915001296409223570[4] = 0;
   out_3915001296409223570[5] = 0;
   out_3915001296409223570[6] = 1;
   out_3915001296409223570[7] = 0;
}
void h_24(double *state, double *unused, double *out_4290608182807608693) {
   out_4290608182807608693[0] = state[4];
   out_4290608182807608693[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1207853802661120319) {
   out_1207853802661120319[0] = 0;
   out_1207853802661120319[1] = 0;
   out_1207853802661120319[2] = 0;
   out_1207853802661120319[3] = 0;
   out_1207853802661120319[4] = 1;
   out_1207853802661120319[5] = 0;
   out_1207853802661120319[6] = 0;
   out_1207853802661120319[7] = 0;
   out_1207853802661120319[8] = 0;
   out_1207853802661120319[9] = 0;
   out_1207853802661120319[10] = 0;
   out_1207853802661120319[11] = 0;
   out_1207853802661120319[12] = 0;
   out_1207853802661120319[13] = 1;
   out_1207853802661120319[14] = 0;
   out_1207853802661120319[15] = 0;
}
void h_30(double *state, double *unused, double *out_6681529128253203612) {
   out_6681529128253203612[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4917843866351144766) {
   out_4917843866351144766[0] = 0;
   out_4917843866351144766[1] = 0;
   out_4917843866351144766[2] = 0;
   out_4917843866351144766[3] = 0;
   out_4917843866351144766[4] = 1;
   out_4917843866351144766[5] = 0;
   out_4917843866351144766[6] = 0;
   out_4917843866351144766[7] = 0;
}
void h_26(double *state, double *unused, double *out_6078213243576726095) {
   out_6078213243576726095[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1789591858529985703) {
   out_1789591858529985703[0] = 0;
   out_1789591858529985703[1] = 0;
   out_1789591858529985703[2] = 0;
   out_1789591858529985703[3] = 0;
   out_1789591858529985703[4] = 0;
   out_1789591858529985703[5] = 0;
   out_1789591858529985703[6] = 0;
   out_1789591858529985703[7] = 1;
}
void h_27(double *state, double *unused, double *out_7182667361806987326) {
   out_7182667361806987326[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3630261878514519454) {
   out_3630261878514519454[0] = 0;
   out_3630261878514519454[1] = 0;
   out_3630261878514519454[2] = 0;
   out_3630261878514519454[3] = 1;
   out_3630261878514519454[4] = 0;
   out_3630261878514519454[5] = 0;
   out_3630261878514519454[6] = 0;
   out_3630261878514519454[7] = 0;
}
void h_29(double *state, double *unused, double *out_4077123807444505961) {
   out_4077123807444505961[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1094045097222312813) {
   out_1094045097222312813[0] = 0;
   out_1094045097222312813[1] = 1;
   out_1094045097222312813[2] = 0;
   out_1094045097222312813[3] = 0;
   out_1094045097222312813[4] = 0;
   out_1094045097222312813[5] = 0;
   out_1094045097222312813[6] = 0;
   out_1094045097222312813[7] = 0;
}
void h_28(double *state, double *unused, double *out_3954764283378591393) {
   out_3954764283378591393[0] = state[5];
   out_3954764283378591393[1] = state[6];
}
void H_28(double *state, double *unused, double *out_3175028504177079606) {
   out_3175028504177079606[0] = 0;
   out_3175028504177079606[1] = 0;
   out_3175028504177079606[2] = 0;
   out_3175028504177079606[3] = 0;
   out_3175028504177079606[4] = 0;
   out_3175028504177079606[5] = 1;
   out_3175028504177079606[6] = 0;
   out_3175028504177079606[7] = 0;
   out_3175028504177079606[8] = 0;
   out_3175028504177079606[9] = 0;
   out_3175028504177079606[10] = 0;
   out_3175028504177079606[11] = 0;
   out_3175028504177079606[12] = 0;
   out_3175028504177079606[13] = 0;
   out_3175028504177079606[14] = 1;
   out_3175028504177079606[15] = 0;
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
