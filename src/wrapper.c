#include "M_func.h"
#include "C_func.h"
#include "forward_dynamics.h"
#include "forward_dynamics_deriv.h"
#include "inverse_dynamics.h"
#include "inverse_dynamics_deriv.h"
#include "velocity_kinematics.h"
#include "velocity_kinematics_T.h"
#include "kinematics.h"
#include "kinematics_jacobian.h"
#include "kinematics_velocity.h"
#include "kinematics_velocity_jacobian.h"

void M_func_wrapper(double* x_in, double* M_out) {
    const double* args[1] = {x_in};
    double* res[1] = {M_out};
    long long int iw[0];
    double w[0];
    M_func(args, res, iw, w, 0);
}

void C_func_wrapper(double* x_in, double* C_out) {
    const double* args[1] = {x_in};
    double* res[1] = {C_out};
    long long int iw[0];
    double w[0];
    C_func(args, res, iw, w, 0);
}

void forward_dynamics_wrapper(double* x_in, double* tau_in, double* vdot_out) {
    const double* args[2] = {x_in, tau_in};
    double* res[1] = {vdot_out};
    long long int iw[0];
    double w[0];
    forward_dynamics(args, res, iw, w, 0);
}

void forward_dynamics_deriv_wrapper(double* x_in, double* tau_in, double* dvdot_dx_out, double* dvdout_dtau_out) {
    const double* args[2] = {x_in, tau_in};
    double* res[2] = {dvdot_dx_out, dvdout_dtau_out};
    long long int iw[0];
    double w[0];
    forward_dynamics_deriv(args, res, iw, w, 0);
}

void inverse_dynamics_wrapper(double* x_in, double* vdot_in, double* tau_out) {
    const double* args[2] = {x_in, vdot_in};
    double* res[1] = {tau_out};
    long long int iw[0];
    double w[0];
    inverse_dynamics(args, res, iw, w, 0);
}

void inverse_dynamics_deriv_wrapper(double* x_in, double* vdot_in, double* dtau_dx_out, double* dtau_dv_dot_out) {
    const double* args[2] = {x_in, vdot_in};
    double* res[2] = {dtau_dx_out, dtau_dv_dot_out};
    long long int iw[0];
    double w[0];
    inverse_dynamics_deriv(args, res, iw, w, 0);
}

void velocity_kinematics_wrapper(double* x_in, double* E_out) {
    const double* args[1] = {x_in};
    double* res[1] = {E_out};
    long long int iw[0];
    double w[0];
    velocity_kinematics(args, res, iw, w, 0);
}

void velocity_kinematics_T_wrapper(double* x_in, double* E_T_out) {
    const double* args[1] = {x_in};
    double* res[1] = {E_T_out};
    long long int iw[0];
    double w[0];
    velocity_kinematics_T(args, res, iw, w, 0);
}

void kinematics_wrapper(double* x_in, double* locs_out) {
    const double* args[1] = {x_in};
    double* res[1] = {locs_out};
    long long int iw[0];
    double w[0];
    kinematics(args, res, iw, w, 0);
}

void kinematics_jacobian_wrapper(double* x_in, double* J_out) {
    const double* args[1] = {x_in};
    double* res[1] = {J_out};
    long long int iw[0];
    double w[0];
    kinematics_jacobian(args, res, iw, w, 0);
}

void kinematics_velocity_wrapper(double* x_in, double* locs_dot_out) {
    const double* args[1] = {x_in};
    double* res[1] = {locs_dot_out};
    long long int iw[0];
    double w[0];
    kinematics_velocity(args, res, iw, w, 0);
}

void kinematics_velocity_jacobian_wrapper(double* x_in, double* J_dot_out) {
    const double* args[1] = {x_in};
    double* res[1] = {J_dot_out};
    long long int iw[0];
    double w[0];
    kinematics_velocity_jacobian(args, res, iw, w, 0);
}


// void forward_dynamics(double* q_in, double* qdot_in, double* tau_in, double* qddot_out) {
//     const double* args[3] = {q_in, qdot_in, tau_in};
//     double* res[1] = {qddot_out};
//     long long int iw[0];
//     double w[0];
//     eval_forward_dynamics(args, res, iw, w, 0);
// }

// void rk4(double* q_in, double* qdot_in, double* tau_in, double* h_in, double* q_out, double* qdot_out) {
//     const double* args[4] = {q_in, qdot_in, tau_in, h_in};
//     double* res[2] = {q_out, qdot_out};
//     long long int iw[0];
//     double w[0];
//     eval_rk4(args, res, iw, w, 0);
// }

// void rk4_derivatives(double* q_in, double* qdot_in, double* tau_in, double* h_in, 
//                     double* q_jac_qout, double* q_jac_qdotout, double* q_jac_uout,
//                     double* qdot_jac_qout, double* qdot_jac_qdoutout, double* qdot_jac_uout) {
//     const double* args[4] = {q_in, qdot_in, tau_in, h_in};
//     double* res[6] = {q_jac_qout, q_jac_qdotout, q_jac_uout, qdot_jac_qout, qdot_jac_qdoutout, qdot_jac_uout};
//     long long int iw[0];
//     double w[0];
//     eval_rk4_derivatives(args, res, iw, w, 0);
// }