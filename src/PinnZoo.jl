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

    function forward_dynamics_deriv(model::PinnZooModel, x::Vector{Float64}, τ::Vector{Float64})
        dv̇_dx, dv̇_dτ = zeros(model.nv, model.nx), zeros(model.nv, model.nv)
        ccall(model.forward_dynamics_deriv_ptr, Cvoid, (Ptr{Cdouble}, Ptr{Cdouble}, Ref{Cdouble}, Ref{Cdouble}), 
                x, τ, dv̇_dx, dv̇_dτ)
        return dv̇_dx, dv̇_dτ
    end
    
    function inverse_dynamics(model::PinnZooModel, x::Vector{Float64}, v̇::Vector{Float64})
        τ = zeros(model.nv)
        ccall(model.inverse_dynamics_ptr, Cvoid, (Ptr{Cdouble}, Ptr{Cdouble}, 
                Ref{Cdouble}), x, v̇, τ)
        return τ
    end

    function inverse_dynamics_deriv(model::PinnZooModel, x::Vector{Float64}, v̇::Vector{Float64})
        dτ_dx, dτ_dv̇ = zeros(model.nv, model.nx), zeros(model.nv, model.nv)
        ccall(model.inverse_dynamics_deriv_ptr, Cvoid, (Ptr{Cdouble}, Ptr{Cdouble}, 
                Ref{Cdouble}, Ref{Cdouble}), x, v̇, dτ_dx, dτ_dv̇)
        return dτ_dx, dτ_dv̇
    end

    function velocity_kinematics(model::PinnZooModel, x::Vector{Float64})
        E = zeros(model.nq, model.nv)
        ccall(model.velocity_kinematics_ptr, Cvoid, (Ptr{Cdouble}, Ref{Cdouble}), x, E)
        return E
    end
    
    function velocity_kinematics_T(model::PinnZooModel, x::Vector{Float64})
        E_T = zeros(model.nv, model.nq)
        ccall(model.velocity_kinematics_T_ptr, Cvoid, (Ptr{Cdouble}, Ref{Cdouble}), x, E_T)
        return E_T
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

    function kinematics_velocity(model::PinnZooModel, x::Vector{Float64})
        locs_dot = zeros(3*length(model.kinematics_bodies))
        ccall(model.kinematics_velocity_ptr, Cvoid, (Ptr{Cdouble}, Ref{Cdouble}), x, locs_dot)
        return locs_dot
    end
    
    function kinematics_velocity_jacobian(model::PinnZooModel, x::Vector{Float64})
        J_dot = zeros(3*length(model.kinematics_bodies), model.nx)
        ccall(model.kinematics_velocity_jacobian_ptr, Cvoid, (Ptr{Cdouble}, Ref{Cdouble}), x, J_dot)
        return J_dot
    end

    # Quaternion helpers
    include(joinpath(@__DIR__, "quaternions.jl"))
    export quat_to_axis_angle, axis_angle_to_quat, quat_conjugate, skew, L_mult,
        R_mult, attitude_jacobian, quat_to_rot

    # Quadruped specific functions
    include(joinpath(@__DIR__, "quadruped.jl"))
    export error_jacobian, error_jacobian_T, apply_Δx, state_error
    export B_func, fix_joint_limits, inverse_kinematics, nearest_ik

    export PinnZooModel, Quadruped
    export is_floating, zero_state, init_state, randn_state
    export M_func, C_func, forward_dynamics, forward_dynamics_deriv, inverse_dynamics, inverse_dynamics_deriv
    export velocity_kinematics, velocity_kinematics_T
    export kinematics, kinematics_jacobian, kinematics_velocity, kinematics_velocity_jacobian

    export Cartpole, Go1, Go2
end