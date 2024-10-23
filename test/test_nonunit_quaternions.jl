using Test
using PinnZoo
using FiniteDiff
using FiniteDifferences
using LinearAlgebra
using Random
using Plots
using FiniteDifferences

model = Go1()

# Euler using inverse dynamics
function dynamics_residual(model, x_k, u_k, x_next, h)
    q_k, v_k = x_k[1:model.nq], x_k[model.nq + 1:end]
    q_next, v_next = x_next[1:model.nq], x_next[model.nq + 1:end]

    # Position residual
    q_res = q_next - (q_k + h*velocity_kinematics(model, q_k)*v_k)

    # Velocity residual
    # v_next - v_k - h(inv(M)*(τ - C)) -> M*(v_next - v_k)/h + C - τ
    B = B_func(model)
    A = kinematics_velocity_jacobian(model, x_k)[:, model.nq + 1:end]
    v_res = inverse_dynamics(model, x_k, (v_next - v_k)/h) - [B A']*u_k 

    return [q_res; v_res]
end

function dynamics_residual_jacobian(model, x_k, u_k, x_next, h)
    q_k, v_k = x_k[1:model.nq], x_k[model.nq + 1:end]
    q_next, v_next = x_next[1:model.nq], x_next[model.nq + 1:end]

    nu = length(u_k)
    J = zeros(model.nx, model.nx*2 + length(u_k))

    # Position residual jacobian
    J[1:model.nq, :] = hcat(-[I(model.nq) h*velocity_kinematics(model, q_k)] - h*velocity_kinematics_jvp_deriv(model, q_k, v_k),
            zeros(model.nq, nu), [I(model.nq) zeros(model.nq, model.nv)])

    # Velocity residual jacobian
    B = B_func(model)
    A = kinematics_velocity_jacobian(model, x_k)[:, model.nq + 1:end]
    dx, dv̇ = inverse_dynamics_deriv(model, x_k, (v_next - v_k)/h)
    J[model.nq + 1:end, :] = hcat(dx - dv̇*[zeros(model.nv, model.nq) I(model.nv)]/h - kinematics_force_jacobian(model, x_k, u_k[13:end]),
            -[B A'], dv̇*[zeros(model.nv, model.nq) I(model.nv)]/h)

    return J
end

x1 = randn(model.nx)
x2 = randn(model.nx)
x2[4:7] /= norm(x2[4:7]*2)
nu = 12 + 12
u = randn(nu)
dynamics_residual(model, x1, u, x2, 0.05)

J1 = dynamics_residual_jacobian(model, x1, u, x2, 0.05)
J2 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), z -> dynamics_residual(model, z[1:model.nx], z[model.nx .+ (1:nu)],
        z[model.nx + nu .+ (1:model.nx)], 0.05), copy([x1; u; x2]))[1]
norm(J1 - J2, Inf)

