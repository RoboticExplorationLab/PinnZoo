module PinnZoo
    using Libdl
    using LinearAlgebra

    abstract type PinnZooModel end
    abstract type Quadruped <: PinnZooModel end

    const SHARED_LIBRARY_DIR = joinpath(@__DIR__, "../build")
    const MODEL_DIR = joinpath(@__DIR__, "../models")

    # Include model files here
    include(joinpath(MODEL_DIR, "cartpole/cartpole.jl"))
    include(joinpath(MODEL_DIR, "unitree_go1/go1.jl"))
    include(joinpath(MODEL_DIR, "unitree_go2/go2.jl"))

    # Defaults for models
    is_floating(model::PinnZooModel) = false

    # Generate state (override if state vector contains a quaternion)
    zero_state(model::PinnZooModel) = zeros(model.nx)
    randn_state(model::PinnZooModel) = randn(model.nx)
    init_state(model::PinnZooModel) = zeros(model.nx) # Define for each model if desired

    # Dynamics functions
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
    
    # Kinematics functions
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

    # Quaternion helpers
    include(joinpath(@__DIR__, "quaternions.jl"))
    export quat_to_axis_angle, axis_angle_to_quat, quat_conjugate, skew, L_mult,
        R_mult, attitude_jacobian, quat_to_rot

    export PinnZooModel, Quadruped
    export is_floating, zero_state, init_state, rand_state
    export M_func, C_func, forward_dynamics, inverse_dynamics
    export kinematics, kinematics_jacobian

    export Cartpole, Go1, Go2
end