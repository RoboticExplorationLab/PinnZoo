import ForwardDiff as FD

@doc raw"""
    kinematics_size(model::PinnZooModel)
Returns the size of the kinematics vector usually 3*model.nc, but is 7*model.nc if 
model.kinematics_ori exists and is Quaternion, and 6*model.nc if it is AxisAngle
"""
kinematics_size(model::PinnZooModel) = hasproperty(model, :kinematics_ori) ? kinematics_size(model, Val(model.kinematics_ori)) : 3*length(model.kinematics_bodies)
kinematics_size(model::PinnZooModel, ::Val{:None}) = 3*length(model.kinematics_bodies)
kinematics_size(model::PinnZooModel, ::Val{:Quaternion}) = 7*length(model.kinematics_bodies)
kinematics_size(model::PinnZooModel, ::Val{:AxisAngle}) = 6*length(model.kinematics_bodies)

@doc raw"""
    kinematics_dim(model::PinnZooModel)
Returns the dim of the kinematics vector for each element in model.kinematics_bodies. Usually 3 (pos x, y, z) but 7 if
model.kinematics_ori = :Quaternion and 6 if model.kinematics_ori = :AxisAngle
"""
kinematics_dim(model::PinnZooModel) = hasproperty(model, :kinematics_ori) ? kinematics_dim(model, Val(model.kinematics_ori)) : 3
kinematics_dim(model::PinnZooModel, ::Val{:None}) = 3
kinematics_dim(model::PinnZooModel, ::Val{:Quaternion}) = 7
kinematics_dim(model::PinnZooModel, ::Val{:AxisAngle}) = 6

@doc raw"""
    kinematics(model::PinnZooModel, x::AbstractVector{Float64})

Return a list of the locations of each body in model.kinematics_bodies in the world frame.
"""
function kinematics(model::PinnZooModel, x::AbstractVector{Float64})
    locs = zeros(kinematics_size(model))
    ccall(model.kinematics_ptr, Cvoid, (Ptr{Cdouble}, Ref{Cdouble}), x, locs)
    return locs
end

@doc raw"""
    kinematics(model::PinnZooModel, x::AbstractVector{T1}, id::Int, point::AbstractVector{T2}) where {T1 <: Real, T2 <: Real}

Returns the location of the point (body frame) on the body identified by id (model.kinematics_bodies) in the world frame.
NOTE: only works for kinematics_ori = :Quaternion
"""
function kinematics(model::PinnZooModel, x::AbstractVector{T1}, id::Int, point::AbstractVector{T2}) where {T1 <: Real, T2 <: Real}
    return _kinematics(id, point, kinematics(model, x))
end

@doc raw"""
    _kinematics(id::Int, point::AbstractVector, locs::AbstractVector)

Internal helper. 
Returns the location of the point (body frame) on the body identified by id (model.kinematics_bodies) in the world frame.
NOTE: only works for kinematics_ori = :Quaternion
"""
function _kinematics(id::Int, point::AbstractVector{T1}, locs::AbstractVector{T2}) where {T1 <: Real, T2 <: Real}
    pos, rot = locs[(id - 1)*7 .+ (1:3)], quat_to_rot(locs[(id - 1)*7 .+ (4:7)])
    return pos + rot*point
end

@doc raw"""
    kinematics_rotation(model::PinnZooModel, x::AbstractVector{Float64})    

Return a list of rotation matrices in the world frame.
"""
function kinematics_rotation(model::PinnZooModel, x::AbstractVector{T}) where T <: Real
    if !hasproperty(model, :kinematics_ori) || model.kinematics_ori == :None
        @error "kinematics_rotation is only supported for kinematics functions with rotations"
    end

    locs = kinematics(model, x)
    return _kinematics_rotation(model, x, locs)
end

function _kinematics_rotation(model::PinnZooModel, x::AbstractVector{T}, locs::AbstractVector) where T <: Real
    if model.kinematics_ori == :Quaternion
        quats = [locs[(k - 1)*7 .+ (4:7)] for k in 1:length(model.kinematics_bodies)]
        return vcat([quat_to_rot(quat) for quat in quats]...)
    elseif model.kinematics_ori == :AxisAngle
        aas = [locs[(k - 1)*6 .+ (4:6)] for k in 1:length(model.kinematics_bodies)]
        return vcat([quat_to_rot(axis_angle_to_quat(aa)) for aa in aas]...)
    end
end

@doc raw"""
    kinematics_jacobian(model::PinnZooModel, x::AbstractVector{Float64})

Return the jacobian of the kinematics function with respect to x (not projected into the body rotation tangent space).
"""
function kinematics_jacobian(model::PinnZooModel, x::AbstractVector{Float64})
    J = zeros(kinematics_size(model), model.nx)
    ccall(model.kinematics_jacobian_ptr, Cvoid, (Ptr{Cdouble}, Ref{Cdouble}), x, J)
    return J
end

@doc raw"""
    kinematics_jacobian(model::PinnZooModel, x::AbstractVector{Float64}, id::Int, point::Vector{Float64})

Return the jacobian of a point (body frame) on the body in the world frame with respect to x (not projected into the body rotation tangent space). 
The body is identified by id (model.kinematics_bodies). 
"""
function kinematics_jacobian(model::PinnZooModel, x::AbstractVector{Float64}, id::Int, point::Vector{Float64})
    locs = kinematics(model, x)
    return FD.jacobian(_locs -> _kinematics(id, point, _locs), locs)*kinematics_jacobian(model, x)
end

@doc raw"""
    kinematics_hvp(model::PinnZooModel, x::AbstractVector{Float64}, dx::AbstractVector{Float64})

Return the hessian-vector product of the kinematics function or d/dx kinematics_jacobian * dx
"""
function kinematics_hvp(model::PinnZooModel, x::AbstractVector{Float64}, dx::AbstractVector{Float64})
    J_dx = zeros(kinematics_size(model), model.nx)
    ccall(model.kinematics_hvp_ptr, Cvoid, (Ptr{Cdouble}, Ptr{Cdouble}, Ref{Cdouble}), x, dx, J_dx)
    return J_dx
end

@doc raw"""
    kinematics_jacobianTvp(model::PinnZooModel, x::AbstractVector{Float64}, λ::AbstractVector{Float64})

Returns the nv-dim jacobian-transpose vector product J(x)'*λ, where J(x) maps joint velocities into kinematics velocities
"""
function kinematics_jacobianTvp(model::PinnZooModel, x::AbstractVector{Float64}, λ::AbstractVector{Float64})
    J = kinematics_velocity_jacobian(model, x)
    return J[:, model.nq + 1:end]'*λ
end

@doc raw"""
    kinematics_velocity(model::PinnZooModel, x::AbstractVector{Float64})

Return a list of the instantaneous linear velocities of each body in model.kinematics_bodies in the world frame.
This is J(q)*E(q)*v
"""
function kinematics_velocity(model::PinnZooModel, x::AbstractVector{Float64})
    locs_dot = zeros(kinematics_size(model))
    ccall(model.kinematics_velocity_ptr, Cvoid, (Ptr{Cdouble}, Ref{Cdouble}), x, locs_dot)
    return locs_dot
end

@doc raw"""
    kinematics_velocity(model::PinnZooModel, x::AbstractVector{T1}, id::Int, point::AbstractVector{T2}) where {T1 <: Real, T2 <: Real}

Returns the instantaneous linear velocities of the point (body frame) on the body identified by id (model.kinematics_bodies) in the world frame.
This is J(q)*E(q)*v
"""
function kinematics_velocity(model::PinnZooModel, x::AbstractVector{<:Real}, id::Int, point::AbstractVector{<:Real})
    locs = kinematics(model, x)
    return FD.jacobian(_locs -> _kinematics(id, point, _locs), locs) * kinematics_velocity(model, x)
end

@doc raw"""
    kinematics_velocity_analytical(model::PinnZooModel, x::AbstractVector{<:Real}, id::Int, point::AbstractVector{<:Real})

Returns the instantaneous linear velocity of the point (body frame) on the body identified by id in the world
frame, computed via the rigid body propagation formula $$v_P = v_b + \omega_{world}\times r$$ where $$v_b$$ is
the world-frame linear velocity of the body's origin (read from kinematics\_velocity), $$r = R(q)\cdot point$$
is the lever arm in the world frame, and $$\omega_{world} = R(q)\cdot 2 G_b(q)^T \dot{q}$$ is the world-frame
angular velocity recovered from the body's quaternion rate via attitude\_jacobian.
NOTE: only works for kinematics_ori = :Quaternion
"""
function kinematics_velocity_analytical(model::PinnZooModel, x::AbstractVector{<:Real}, id::Int, point::AbstractVector{<:Real})
    locs         = kinematics(model, x)
    twist_bodies = kinematics_velocity(model, x)
    q  = locs[(id - 1)*7 .+ (4:7)] # [qw, qx, qy, qz]
    vb = twist_bodies[(id - 1)*7 .+ (1:3)] # d(p_body)/dt in world frame
    dq = twist_bodies[(id - 1)*7 .+ (4:7)] # d(quat)/dt
    omega_body  = 2 * attitude_jacobian(q)' * dq # Body-frame angular velocity:  ω_body = 2·G_body(q)ᵀ · dq
    rot = quat_to_rot(q) 
    omega_world = rot * omega_body # World-frame angular velocity:  ω_world = R(q) · ω_body
    r = rot * point # Lever arm in world frame
    return vb + cross(omega_world, r) # Rigid body propagation: v_p = v_origin + ω_world × r
end

@doc raw"""
    kinematics_velocity_jacobian(model::PinnZooModel, x::AbstractVector{Float64})

Return the jacobian of the kinematics\_velocity function with respect to x (not in the tangent space). If $$J_q$$ is the
derivative of the kinematics with respect to $$q$$, this jacobian $$J$$ = [$$\dot{J_q}$$ $$J_q$$] where $$\dot{J_q} = \frac{\partial}{\partial q} J_qv$$ and $$J_qv$$ is
equal to kinematics\_velocity. This also means that $$J\dot{x}$$ expresses the constraint at the acceleration level, i.e. $$\dot{J_q}\dot{q} + J_q\dot{v} = 0$$

"""
function kinematics_velocity_jacobian(model::PinnZooModel, x::AbstractVector{Float64})
    J_dot = zeros(kinematics_size(model), model.nx)
    ccall(model.kinematics_velocity_jacobian_ptr, Cvoid, (Ptr{Cdouble}, Ref{Cdouble}), x, J_dot)
    return J_dot
end

@doc raw"""
    kinematics_velocity_jacobian_analytical(model::PinnZooModel, x::AbstractVector{Float64}, id::Int, point::AbstractVector{Float64})

Return the jacobian (3 x nx matrix) of the world-frame linear velocity of the point (body frame) on the body
identified by id, with respect to x. Computed via the rigid body Jacobian propagation formula
$$J_{v_P} = J_{v_b} - [r]_\times J_{\omega_{world}} + [\omega_{world}]_\times J_r$$, where $$J_{v_b}$$ is pulled
directly from kinematics\_velocity\_jacobian(model, x), $$r = R(q)\cdot point$$, $$\omega_{world}$$ is the body's
world-frame angular velocity, and $$J_{\omega_{world}} = \frac{\partial \omega_{world}}{\partial x}$$ and
$$J_r = \frac{\partial r}{\partial x}$$ are computed by ForwardDiff.
NOTE: only works for kinematics_ori = :Quaternion
"""
function kinematics_velocity_jacobian_analytical(model::PinnZooModel, x::AbstractVector{Float64}, id::Int, point::AbstractVector{Float64})
    locs     = kinematics(model, x)
    locs_dot = kinematics_velocity(model, x)
    q  = locs[(id - 1)*7 .+ (4:7)] # body's world-frame quat [qw, qx, qy, qz]
    dq = locs_dot[(id - 1)*7 .+ (4:7)] # its time derivative
    rot = quat_to_rot(q)
    omega_body  = 2 * attitude_jacobian(q)' * dq # body-frame angular velocity
    omega_world = rot * omega_body # world-frame angular velocity
    r = rot * point # lever arm in world frame
    J_p = kinematics_velocity_jacobian(model, x)[(id - 1)*7 .+ (1:3), :] # J_p = ∂(d p_body / dt) / ∂x 
    # J_omega_world = ∂ω_world / ∂x
    function omega_world_of(_x)
        _locs     = kinematics(model, _x)
        _locs_dot = kinematics_velocity(model, _x)
        _q  = _locs[(id - 1)*7 .+ (4:7)]
        _dq = _locs_dot[(id - 1)*7 .+ (4:7)]
        return quat_to_rot(_q) * (2 * attitude_jacobian(_q)' * _dq)
    end
    J_omega_world = FD.jacobian(omega_world_of, x)
    # J_r = ∂(R(q_body_world)·point) / ∂x
    function r_of(_x)
        _q = kinematics(model, _x)[(id - 1)*7 .+ (4:7)]
        return quat_to_rot(_q) * point
    end
    J_r = FD.jacobian(r_of, x)
    #   v_p = vb + ω_world × r
    #   d v_p / dx = J_p − [r]× · J_ω_world + [ω_world]× · J_r
    return J_p - skew(r) * J_omega_world + skew(omega_world) * J_r
end

@doc raw"""
    kinematics_velocity_jacobian(model::PinnZooModel, x::AbstractVector{Float64}, id::Int, point::AbstractVector{Float64})

Return the jacobian of the kinematics\_velocity function with respect to x (not in the tangent space). If $$J_q$$ is the
derivative of the kinematics with respect to $$q$$, this jacobian $$J$$ = [$$\dot{J_q}$$ $$J_q$$] where $$\dot{J_q} = \frac{\partial}{\partial q} J_qv$$ and $$J_qv$$ is
equal to kinematics\_velocity. This also means that $$J\dot{x}$$ expresses the constraint at the acceleration level, i.e. $$\dot{J_q}\dot{q} + J_q\dot{v} = 0$$

"""
# function kinematics_velocity_jacobian(model::PinnZooModel, x::AbstractVector{Float64}, id::Int, point::AbstractVector{Float64})
#     locs = kinematics(model, x)
#     return FD.jacobian(_locs -> _kinematics(id, point, _locs), locs)*kinematics_velocity_jacobian(model, x)
# end

function kinematics_velocity_jacobian(model::PinnZooModel, x::AbstractVector{Float64}, id::Int, point::AbstractVector{Float64})
    locs = kinematics(model, x)
    locs_dot = kinematics_velocity(model, x)
    A_func = _locs -> _kinematics(id, point, _locs) # A(locs) = ∂(point_in_world)/∂locs.  
    A = FD.jacobian(A_func, locs)
    #   (dA/dx) * locs_dot  =  d/dlocs [ A(locs) * locs_dot_const ]  *  dlocs/dx
    dAv_dlocs = FD.jacobian(_locs -> FD.jacobian(A_func, _locs) * locs_dot, locs)
    return A * kinematics_velocity_jacobian(model, x) + dAv_dlocs * kinematics_jacobian(model, x)
end

@doc raw"""
    kinematics_velocity_hvp(model::PinnZooModel, x::AbstractVector{Float64}, mult::AbstractVector{Float64})

Return the hessian-vector product of kinematics\_velocity, i.e. $$\frac{d}{dx}(\text{kinematics\_velocity\_jacobian}(x))\cdot \text{mult}$$,
as a kinematics_size x nx matrix.
"""
function kinematics_velocity_hvp(model::PinnZooModel, x::AbstractVector{Float64}, mult::AbstractVector{Float64})
    J_dot = zeros(kinematics_size(model), model.nx)
    ccall(model.kinematics_velocity_jacobian_ptr, Cvoid, (Ptr{Cdouble}, Ref{Cdouble}), x, J_dot)
    return J_dot
end

@doc raw"""
    kinematics_force_jacobian(model::PinnZooModel, x::AbstractVector{Float64}, λ::AbstractVector{Float64})

Return the jacobian (nv x nx matrix) of (J(x))'*λ with respect to x, where J(x) maps joint velocities into kinematics velocities
"""
function kinematics_force_jacobian(model::PinnZooModel, x::AbstractVector{Float64}, λ::AbstractVector{Float64})
    J = zeros(model.nv, model.nx)
    ccall(model.kinematics_force_jacobian_ptr, Cvoid, (Ptr{Cdouble}, Ptr{Cdouble}, Ref{Cdouble}), x, λ, J)
    return J
end

@doc raw"""
    kinematics_force_hvp_ptr(model::PinnZooModel, x::AbstractVector{Float64}, λ::AbstractVector{Float64})

Return the jacobian of (d/dx J(x, λ))*mult w.r.t x and λ where J(x, λ) is kinematics_force_jacobian(model, x, λ)
"""
function kinematics_force_hvp(model::PinnZooModel, x::AbstractVector{Float64}, λ::AbstractVector{Float64}, mult::AbstractVector{Float64})
    @assert !hasproperty(model, :kinematics_ori) || model.kinematics_ori != :AxisAngle "hvp is currently not supported for axis angles"
    J_x, J_λ = zeros(model.nv, model.nx), zeros(model.nv, kinematics_size(model))
    ccall(model.kinematics_force_hvp_ptr, Cvoid, (Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}, Ref{Cdouble}, Ref{Cdouble}), x, λ, 
                                                        mult, J_x, J_λ)
    return J_x, J_λ
end

@doc raw"""
    kinematics_force_hTvp_ptr(model::PinnZooModel, x::AbstractVector{Float64}, λ::AbstractVector{Float64})

Return the jacobian of J(x, λ)'*mult w.r.t x and λ where J(x, λ) is kinematics_force_jacobian(model, x, λ)
"""
function kinematics_force_hTvp(model::PinnZooModel, x::AbstractVector{Float64}, λ::AbstractVector{Float64}, mult::AbstractVector{Float64})
    @assert !hasproperty(model, :kinematics_ori) || model.kinematics_ori != :AxisAngle "hTvp is currently not supported for axis angles"
    J_x, J_λ = zeros(model.nx, model.nx), zeros(model.nx, kinematics_size(model))
    ccall(model.kinematics_force_hTvp_ptr, Cvoid, (Ptr{Cdouble}, Ptr{Cdouble}, Ptr{Cdouble}, Ref{Cdouble}, Ref{Cdouble}), x, λ, 
                                                        mult, J_x, J_λ)
    return J_x, J_λ
end