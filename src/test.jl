using Pkg; Pkg.activate(joinpath(@__DIR__, ".."))
using Libdl
using LinearAlgebra
using ForwardDiff
using FiniteDiff
using RigidBodyDynamics

abstract type PinnZooModel end

struct Cartpole <: PinnZooModel
    nq
    nv
    nx
    M_func_ptr::Ptr{Nothing}
    C_func_ptr::Ptr{Nothing}
    forward_dynamics_ptr::Ptr{Nothing}
    inverse_dynamics_ptr::Ptr{Nothing}
    kinematics_bodies::Vector{String}
    kinematics_ptr::Ptr{Nothing}
    kinematics_jacobian_ptr::Ptr{Nothing}
    function Cartpole()
        lib = dlopen(joinpath(@__DIR__, "build/libdynamics.so"))

        # Dynamics
        M_func_ptr = dlsym(lib, :M_func_wrapper)
        C_func_ptr = dlsym(lib, :C_func_wrapper)
        forward_dynamics_ptr = dlsym(lib, :forward_dynamics_wrapper)
        inverse_dynamics_ptr = dlsym(lib, :inverse_dynamics_wrapper)

        # Kinematics
        kinematics_bodies = ["pole_tip"]
        kinematics_ptr = dlsym(lib, :kinematics_wrapper)
        kinematics_jacobian_ptr = dlsym(lib, :kinematics_jacobian_wrapper)
        return new(
            2, 2, 2 + 2,
            M_func_ptr, C_func_ptr, forward_dynamics_ptr, inverse_dynamics_ptr,
            kinematics_bodies, kinematics_ptr, kinematics_jacobian_ptr
        )
    end
end

function M_func(model::PinnZooModel, x::Vector{Float64})
    M = zeros(model.nv, model.nv)
    ccall(model.M_func_ptr, Cvoid, (Ptr{Cdouble}, Ref{Cdouble}), x, M)
    return M
end

function C_func(model::PinnZooModel, x::Vector{Float64})
    C = zeros(model.nv)
    ccall(model.C_func_ptr, Cvoid, (Ptr{Cdouble}, Ref{Cdouble}), x, C)
    return C
end

function forward_dynamics(model::PinnZooModel, x::Vector{Float64}, τ::Vector{Float64})
    v̇ = zeros(model.nv)
    ccall(model.forward_dynamics_ptr, Cvoid, (Ptr{Cdouble}, Ptr{Cdouble}, Ref{Cdouble}), x, τ, v̇)
    return v̇
end

function inverse_dynamics(model::PinnZooModel, x::Vector{Float64}, v̇::Vector{Float64})
    τ = zeros(model.nv)
    ccall(model.inverse_dynamics_ptr, Cvoid, (Ptr{Cdouble}, Ptr{Cdouble}, 
            Ref{Cdouble}), x, v̇, τ)
    return τ
end

function kinematics(model::PinnZooModel, x::Vector{Float64})
    locs = zeros(3*length(model.kinematics_bodies))
    ccall(model.kinematics_ptr, Cvoid, (Ptr{Cdouble}, Ref{Cdouble}), x, locs)
    return locs
end

function kinematics_jacobian(model::PinnZooModel, x::Vector{Float64})
    J = zeros(3*length(model.kinematics_bodies), model.nx)
    ccall(model.kinematics_jacobian_ptr, Cvoid, (Ptr{Cdouble}, Ref{Cdouble}), x, J)
    return J
end

# Get code-gen model
model = Cartpole()

# Build RigidBodyDynamics.jl model and necessary helpers for testing
robot = parse_urdf(joinpath(@__DIR__, "../cartpole.urdf"), remove_fixed_tree_joints=false)
state = MechanismState(robot)
dyn_res = DynamicsResult(robot)
function kinematics(x)
    set_configuration!(state, x[1:model.nq])
    return vcat([
        translation(relative_transform(state, default_frame(findbody(robot, body)), root_frame(robot))) 
        for body in model.kinematics_bodies]...)
end

# Make random state to test with
x = randn(model.nx)
v̇ = randn(model.nv)
τ = randn(model.nv)
set_configuration!(state, x[1:model.nq])
set_velocity!(state, x[model.nq + 1:end])

# Test mass matrix
M1 = M_func(model, x)
M2 = mass_matrix(state)
display(norm(M1 - M2, Inf))

# Test coriolis matrix
C1 = C_func(model, x)
C2 = dynamics_bias(state)
display(norm(C1 - C2, Inf))

# Test forward dynamics 
v̇1 = forward_dynamics(model, x, τ)
dynamics!(dyn_res, state, τ)
v̇2 = dyn_res.v̇
display(norm(v̇1 - v̇2, Inf))

# Test inverse dynamics
τ1 = inverse_dynamics(model, x, v̇);
dyn_res.v̇[:] = v̇ # inverse_dynamics needs a Segmented_Vector, this is a workaround
τ2 = RigidBodyDynamics.inverse_dynamics(state, dyn_res.v̇)
display(norm(τ1 - τ2, Inf))

# Test kinematics
locs1 = kinematics(model, x)
locs2 = kinematics(x)
display(norm(locs1 - locs2, Inf))

# Test kinematics jacobian
J1 = kinematics_jacobian(model, x)
J2 = FiniteDiff.finite_difference_jacobian(kinematics, x)
display(norm(J1 - J2, Inf))


