module PinnZoo
    using Libdl
    
    abstract type PinnZooModel end

    const SHARED_LIBRARY_DIR = joinpath(@__DIR__, "../build")

    # Include model files here
    include(joinpath(@__DIR__, "models/cartpole/cartpole.jl"))

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
end