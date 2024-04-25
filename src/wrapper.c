#include "M_func.h"
#include "C_func.h"
#include "forward_dynamics.h"
#include "inverse_dynamics.h"

void M_func_wrapper(double* x_in, double* M_out) {
    const double* args[1] = {x_in};
    double* res[1] = {M_out};
    long long int iw[0];
    double w[0];
    M_func(args, res, iw, w, 0);
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