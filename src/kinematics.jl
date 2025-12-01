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

Return the jacobian of the kinematics function with respect to x (not projected into the tangent space).
"""
function kinematics_jacobian(model::PinnZooModel, x::AbstractVector{Float64})
    J = zeros(kinematics_size(model), model.nx)
    ccall(model.kinematics_jacobian_ptr, Cvoid, (Ptr{Cdouble}, Ref{Cdouble}), x, J)
    return J
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
    kinematics_force_jacobian(model::PinnZooModel, x::AbstractVector{Float64}, λ::AbstractVector{Float64})

Return the jacobian (nv x nx matrix) of (J(x))'*λ with respect to x, where J(x) maps joint velocities into kinematics velocities
"""
function kinematics_force_jacobian(model::PinnZooModel, x::AbstractVector{Float64}, λ::AbstractVector{Float64})
    J = zeros(model.nv, model.nx)
    ccall(model.kinematics_force_jacobian_ptr, Cvoid, (Ptr{Cdouble}, Ptr{Cdouble}, Ref{Cdouble}), x, λ, J)
    return J
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