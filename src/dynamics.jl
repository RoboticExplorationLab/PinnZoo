@doc raw""" 
    M_func(model::PinnZooModel, x::AbstractVector{Float64})

Return the mass matrix of the model as a function of the configuration (x[1:model.nq])
"""
function M_func(model::PinnZooModel, x::AbstractVector{Float64})
    M = zeros(model.nv, model.nv)
    ccall(model.M_func_ptr, Cvoid, (Ptr{Cdouble}, Ref{Cdouble}), x, M)
    return M
end

@doc raw"""
    C_func(model::PinnZooModel, x::AbstractVector{Float64})

Return the dynamics bias of the model (coriolos + centrifugal + gravitational forces)
"""
function C_func(model::PinnZooModel, x::AbstractVector{Float64})
    C = zeros(model.nv)
    ccall(model.C_func_ptr, Cvoid, (Ptr{Cdouble}, Ref{Cdouble}), x, C)
    return C
end

@doc raw"""
    forward_dynamics(model::PinnZooModel, x::AbstractVector{Float64}, τ::AbstractVector{Float64})

Return the forward dynamics v̇ = M(x) \ (τ - C) where M is the mass matrix and C is the
dynamics bias vector.
"""
function forward_dynamics(model::PinnZooModel, x::AbstractVector{Float64}, τ::AbstractVector{Float64})
    v̇ = zeros(model.nv)
    ccall(model.forward_dynamics_ptr, Cvoid, (Ptr{Cdouble}, Ptr{Cdouble}, Ref{Cdouble}), x, τ, v̇)
    return v̇
end

@doc raw"""
    forward_dynamics_deriv(model::PinnZooModel, x::AbstractVector{Float64}, τ::AbstractVector{Float64})

Return a tuple of derivatives of the forward dynamics (v̇) with respect to x and τ
"""
function forward_dynamics_deriv(model::PinnZooModel, x::AbstractVector{Float64}, τ::AbstractVector{Float64})
    dv̇_dx, dv̇_dτ = zeros(model.nv, model.nx), zeros(model.nv, model.nv)
    ccall(model.forward_dynamics_deriv_ptr, Cvoid, (Ptr{Cdouble}, Ptr{Cdouble}, Ref{Cdouble}, Ref{Cdouble}), 
            x, τ, dv̇_dx, dv̇_dτ)
    return dv̇_dx, dv̇_dτ
end

@doc raw"""
    inverse_dynamics(model::PinnZooModel, x::AbstractVector{Float64}, v̇::AbstractVector{Float64})

Return the inverse dynamics τ = M⩒ + C where M is the mass matrix and C is the dynamics
bias vector
"""
function inverse_dynamics(model::PinnZooModel, x::AbstractVector{Float64}, v̇::AbstractVector{Float64})
    τ = zeros(model.nv)
    ccall(model.inverse_dynamics_ptr, Cvoid, (Ptr{Cdouble}, Ptr{Cdouble}, 
            Ref{Cdouble}), x, v̇, τ)
    return τ
end

@doc raw"""
    inverse_dynamics_deriv(model::PinnZooModel, x::AbstractVector{Float64}, v̇::AbstractVector{Float64}

Return a tuple of derivatives of the inverse dynamics (τ) with respect to x and v̇ 
"""
function inverse_dynamics_deriv(model::PinnZooModel, x::AbstractVector{Float64}, v̇::AbstractVector{Float64})
    dτ_dx, dτ_dv̇ = zeros(model.nv, model.nx), zeros(model.nv, model.nv)
    ccall(model.inverse_dynamics_deriv_ptr, Cvoid, (Ptr{Cdouble}, Ptr{Cdouble}, 
            Ref{Cdouble}, Ref{Cdouble}), x, v̇, dτ_dx, dτ_dv̇)
    return dτ_dx, dτ_dv̇
end

@doc raw"""
    velocity_kinematics(model::PinnZooModel, x::AbstractVector{Float64})

Return the matrix E mapping v to q̇, i.e. q̇ = E(q)v, where q = x[1:model.nq]. This mapping is exact,
i.e. E_T*E = I where E_T comes from velocity_kinematics_T. For an explanation for why Eᵀ != E_T refer to TODO
"""
function velocity_kinematics(model::PinnZooModel, x::AbstractVector{Float64})
    E = zeros(model.nq, model.nv)
    ccall(model.velocity_kinematics_ptr, Cvoid, (Ptr{Cdouble}, Ref{Cdouble}), x, E)
    return E
end

@doc raw"""
    velocity_kinematics_T(model::PinnZooModel, x::AbstractVector{Float64})

Return the matrix E_T projecting q̇ onto v, i.e. v = E_T(q)q̇, where q = x[1:model.nq]. This respects the
tangent space structure. For a further explanation, as well as details on why E_T != Eᵀ refer to TODO
"""
function velocity_kinematics_T(model::PinnZooModel, x::AbstractVector{Float64})
    E_T = zeros(model.nv, model.nq)
    ccall(model.velocity_kinematics_T_ptr, Cvoid, (Ptr{Cdouble}, Ref{Cdouble}), x, E_T)
    return E_T
end

@doc raw"""
    apply_Δx(model::PinnZooFloatingBaseModel, x_k, Δx)

Return Δx added to x while respecting the configuration space/tangent space relationship (i.e. do quaternion multiplication,
rotate Δbody pos from body frame to world frame). Δx should be model.nv*2, x_k should be model.nx. Floating base rotation in Δx should be axis angle.
"""
function apply_Δx(model::PinnZooFloatingBaseModel, x_k, Δx)
    x_next = zeros(promote_type(eltype(x_k), eltype(Δx)), length(x_k))
    x_next[1:3] = x_k[1:3] + quat_to_rot(x_k[4:7])*Δx[1:3]
    x_next[4:7] = L_mult(x_k[4:7])*axis_angle_to_quat(Δx[4:6])
    x_next[8:end] = x_k[8:end] + Δx[7:end]
    return x_next
end
apply_Δx(model::PinnZooModel, x_k, Δx) = x_k + Δx

@doc raw"""
    state_error(model::PinnZooFloatingBaseModel, x, x0)

Return the state_error between x and x0, using axis-angles for quaternion error, and representing body position error in the
body frame (matches with body velocity convention).
"""
function state_error(model::PinnZooFloatingBaseModel, x, x0)
    return [
        quat_to_rot(x0[4:7])'*(x[1:3] - x0[1:3])
        quat_to_axis_angle(L_mult(x0[4:7])'*x[4:7])
        x[8:end] - x0[8:end]
    ]
end
state_error(model::PinnZooModel, x, x0) = x - x0

@doc raw"""
    error_jacobian(model::PinnZooFloatingBaseModel, x)

Return the jacobian mapping Δx to x where Δx is in the tangent space (look at state_error for our choice
of tangent space).
"""
function error_jacobian(model::PinnZooFloatingBaseModel, x)
    return [
        velocity_kinematics(model, x) zeros(model.nq, length(x) - model.nq)
        zeros(length(x) - model.nq, model.nv) I(length(x) - model.nq)
    ]
end
error_jacobian(model::PinnZooModel, x) = 1.0I(length(x))

@doc raw"""
    error_jacobian_T(model::PinnZooFloatingBaseModel, x)

Return the jacobian mapping x to Δx where Δx is in the tangent space (look at state_error for our choice
of tangent space).
"""
function error_jacobian_T(model::PinnZooFloatingBaseModel, x)
    return [
        velocity_kinematics_T(model, x) zeros(model.nv, length(x) - model.nq)
        zeros(length(x) - model.nq, model.nq) I(length(x) - model.nq)
    ]
end
error_jacobian_T(model::PinnZooModel, x) = 1.0I(length(x))

