using Test
using Revise
using PinnZoo
import PinnZoo: accel_constrained_contact_dyn
using RigidBodyDynamics
using FiniteDiff
using LinearAlgebra
using Random

model = Nadia(kinematics_ori = :AxisAngle, fixed_arms = true)

x1 = randn_state(model)
u = zeros(model.nu);
λ = zeros(kinematics_size(model));
x2 = randn_state(model);
dt = 0.1;
α = 1/0.01;

# x1 = x2;
x1[model.nq + 1:end] .= 0

Δx1 = [x1[1:3]; quat_to_axis_angle(x1[4:7]); x1[8:end]];
Δx2 = [x2[1:3]; quat_to_axis_angle(x2[4:7]); x2[8:end]];

output = accel_constrained_contact_dyn(model, Δx1, u, Δx2, α, dt);

M = M_func(model, x1);
C = C_func(model, x1);
B = B_func(model);
J = kinematics_jacobian(model, x1)[:, 1:model.nq]*velocity_kinematics(model, x1);
J_dot = kinematics_velocity_jacobian(model, x1)*error_jacobian(model, x1)

Δx = state_error(model, x2, x1)

res = [
    Δx[1:model.nv] - dt*Δx1[model.nv + 1:end]
    M*Δx[model.nv + 1:end]/dt + C - B*u - J'*λ
    J_dot*[Δx1[model.nv + 1:end]; Δx[model.nv + 1:end]/dt] - 2*α*kinematics(model, x1) - α^2*kinematics_velocity(model, x1)
]

print(output - res)