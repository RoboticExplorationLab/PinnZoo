#include "contact_res.h"
#include "contact_res_deriv.h"

void contact_res_wrapper(double* delta_x_k_in, double* u_k_in, double* f_k_in, double* delta_x_next_in,
                         double dt, double alpha, double* res_out) {
    const double* args[6] = {delta_x_k_in, u_k_in, f_k_in, delta_x_next_in, &dt, &alpha};
    double* res[1] = {res_out};
    long long int iw[0];
    double w[0];
    contact_res(args, res, iw, w, 0);                       
}

void contact_res_deriv_wrapper(double* delta_x_k_in, double* u_k_in, double* f_k_in, double* delta_x_next_in,
                         double dt, double alpha, double* J_out) {
    const double* args[6] = {delta_x_k_in, u_k_in, f_k_in, delta_x_next_in, &dt, &alpha};
    double* res[1] = {J_out};
    long long int iw[0];
    double w[0];
    contact_res_deriv(args, res, iw, w, 0);                       
}