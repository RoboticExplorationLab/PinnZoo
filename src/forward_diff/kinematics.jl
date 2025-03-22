# Wrap derivatives from Pinocchio so they can be used with ForwardDiff
import ForwardDiff: Dual, value, partials

# kinematics
function kinematics(model::PinnZooModel, x::AbstractVector{Dual{T,N,V}}) where {T,N,V}
    x_value, x_partials = [value(x_elem) for x_elem in x], hcat([partials(x_elem) for x_elem in x])

    f = kinematics(model, x_value)
    df_dx = kinematics_jacobian(model, x_value)*x_partials
    return [Dual{T}(f[k], df_dx[k]) for k in eachindex(f)]
end

# kinematics_velocity
function kinematics_velocity(model::PinnZooModel, x::AbstractVector{Dual{T,N,V}}) where {T,N,V}
    x_value, x_partials = [value(x_elem) for x_elem in x], hcat([partials(x_elem) for x_elem in x])

    f = kinematics_velocity(model, x_value)
    df_dx = kinematics_velocity_jacobian(model, x_value)*x_partials
    return [Dual{T}(f[k], df_dx[k]) for k in eachindex(f)]
end

# TODO add kinematics_jTvp (jacobian-transpose-vector product) to use with kinematics_force_jacobian (i.e. d_dx J(x)'λ)