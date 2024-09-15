@doc raw""" 
    M_func(model::PinnZooModel, x::Vector{Float64})

Return the mass matrix of the model as a function of the configuration (x[1:model.nq])
"""
function M_func(model::PinnZooModel, x::Vector{Float64})
    M = zeros(model.nv, model.nv)
    ccall(model.M_func_ptr, Cvoid, (Ptr{Cdouble}, Ref{Cdouble}), x, M)
    return M
end

@doc raw"""
    C_func(model::PinnZooModel, x::Vector{Float64})

Return the dynamics bias of the model (coriolos + centrifugal + gravitational forces)
"""
function C_func(model::PinnZooModel, x::Vector{Float64})
    C = zeros(model.nv)
    ccall(model.C_func_ptr, Cvoid, (Ptr{Cdouble}, Ref{Cdouble}), x, C)
    return C
end

@doc raw"""
    forward_dynamics(model::PinnZooModel, x::Vector{Float64}, τ::Vector{Float64})

Return the forward dynamics v̇ = M(x) \ (τ - C) where M is the mass matrix and C is the
dynamics bias vector.
"""
function forward_dynamics(model::PinnZooModel, x::Vector{Float64}, τ::Vector{Float64})
    v̇ = zeros(model.nv)
    ccall(model.forward_dynamics_ptr, Cvoid, (Ptr{Cdouble}, Ptr{Cdouble}, Ref{Cdouble}), x, τ, v̇)
    return v̇
end

@doc raw"""
    forward_dynamics_deriv(model::PinnZooModel, x::Vector{Float64}, τ::Vector{Float64})

Return a tuple of derivatives of the forward dynamics (v̇) with respect to x and τ
"""
function forward_dynamics_deriv(model::PinnZooModel, x::Vector{Float64}, τ::Vector{Float64})
    dv̇_dx, dv̇_dτ = zeros(model.nv, model.nx), zeros(model.nv, model.nv)
    ccall(model.forward_dynamics_deriv_ptr, Cvoid, (Ptr{Cdouble}, Ptr{Cdouble}, Ref{Cdouble}, Ref{Cdouble}), 
            x, τ, dv̇_dx, dv̇_dτ)
    return dv̇_dx, dv̇_dτ
end

@doc raw"""
    inverse_dynamics(model::PinnZooModel, x::Vector{Float64}, v̇::Vector{Float64})

Return the inverse dynamics τ = M⩒ + C where M is the mass matrix and C is the dynamics
bias vector
"""
function inverse_dynamics(model::PinnZooModel, x::Vector{Float64}, v̇::Vector{Float64})
    τ = zeros(model.nv)
    ccall(model.inverse_dynamics_ptr, Cvoid, (Ptr{Cdouble}, Ptr{Cdouble}, 
            Ref{Cdouble}), x, v̇, τ)
    return τ
end

@doc raw"""
    inverse_dynamics_deriv(model::PinnZooModel, x::Vector{Float64}, v̇::Vector{Float64}

Return a tuple of derivatives of the inverse dynamics (τ) with respect to x and v̇ 
"""
function inverse_dynamics_deriv(model::PinnZooModel, x::Vector{Float64}, v̇::Vector{Float64})
    dτ_dx, dτ_dv̇ = zeros(model.nv, model.nx), zeros(model.nv, model.nv)
    ccall(model.inverse_dynamics_deriv_ptr, Cvoid, (Ptr{Cdouble}, Ptr{Cdouble}, 
            Ref{Cdouble}, Ref{Cdouble}), x, v̇, dτ_dx, dτ_dv̇)
    return dτ_dx, dτ_dv̇
end

@doc raw"""
    velocity_kinematics(model::PinnZooModel, x::Vector{Float64})

Return the matrix E mapping v to q̇, i.e. q̇ = E(q)v, where q = x[1:model.nq]. This mapping is exact,
i.e. E_T*E = I where E_T comes from velocity_kinematics_T. For an explanation for why Eᵀ != E_T refer to TODO
"""
function velocity_kinematics(model::PinnZooModel, x::Vector{Float64})
    E = zeros(model.nq, model.nv)
    ccall(model.velocity_kinematics_ptr, Cvoid, (Ptr{Cdouble}, Ref{Cdouble}), x, E)
    return E
end

@doc raw"""
    velocity_kinematics_T(model::PinnZooModel, x::Vector{Float64})

Return the matrix E_T projecting q̇ onto v, i.e. v = E_T(q)q̇, where q = x[1:model.nq]. This respects the
tangent space structure. For a further explanation, as well as details on why E_T != Eᵀ refer to TODO
"""
function velocity_kinematics_T(model::PinnZooModel, x::Vector{Float64})
    E_T = zeros(model.nv, model.nq)
    ccall(model.velocity_kinematics_T_ptr, Cvoid, (Ptr{Cdouble}, Ref{Cdouble}), x, E_T)
    return E_T
end