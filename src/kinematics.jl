@doc raw"""
    kinematics_size(model::PinnZooModel)
Returns the size of the kinematics vector usually 3*model.nc, but is 7*model.nc if 
model.kinematics_ori exists and is true
"""
kinematics_size(model::PinnZooModel) = hasproperty(model, :kinematics_ori) ? kinematics_size(model, Val(model.kinematics_ori)) : 3*length(model.kinematics_bodies)
kinematics_size(model::PinnZooModel, ::Val{:None}) = 3*length(model.kinematics_bodies)
kinematics_size(model::PinnZooModel, ::Val{:Quaternion}) = 7*length(model.kinematics_bodies)
kinematics_size(model::PinnZooModel, ::Val{:AxisAngle}) = 6*length(model.kinematics_bodies)

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
    kinematics_jacobian(model::PinnZooModel, x::AbstractVector{Float64})

Return the jacobian of the kinematics function with respect to x (not projected into the tangent space).
"""
function kinematics_jacobian(model::PinnZooModel, x::AbstractVector{Float64})
    J = zeros(kinematics_size(model), model.nx)
    ccall(model.kinematics_jacobian_ptr, Cvoid, (Ptr{Cdouble}, Ref{Cdouble}), x, J)
    return J
end

@doc raw"""
    kinematics_velocity(model::PinnZooModel, x::AbstractVector{Float64})

Return a list of the instantaneous linear velocities of each body in model.kinematics_bodies in the world frame.
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