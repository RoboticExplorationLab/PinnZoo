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

function kinematics_jacobianTvp(model::PinnZooModel, x::AbstractVector{Dual{T,N,V}}, λ::AbstractVector{Float64}) where {T,N,V}
    x_value, x_partials = [value(x_elem) for x_elem in x], hcat([partials(x_elem) for x_elem in x])

    f = kinematics_jacobianTvp(model, x_value, λ)
    df_dx = kinematics_force_jacobian(model, x_value, λ)*x_partials
    return [Dual{T}(f[k], df_dx[k]) for k in eachindex(f)]
end

function kinematics_jacobianTvp(model::PinnZooModel, x::AbstractVector{Float64}, λ::AbstractVector{Dual{T,N,V}}) where {T,N,V}
    λ_value, λ_partials = [value(λ_elem) for λ_elem in λ], hcat([partials(λ_elem) for λ_elem in λ])

    f = kinematics_jacobianTvp(model, x, λ_value)
    df_dλ = kinematics_velocity_jacobian(model, x)[:, model.nq + 1:end]'*λ_partials
    return [Dual{T}(f[k], df_dλ[k]) for k in eachindex(f)]
end

function kinematics_jacobianTvp(model::PinnZooModel, x::AbstractVector{Dual{T,N,V}}, λ::AbstractVector{Dual{T,N,V}}) where {T,N,V}
    x_value, x_partials = [value(x_elem) for x_elem in x], hcat([partials(x_elem) for x_elem in x])
    λ_value, λ_partials = [value(λ_elem) for λ_elem in λ], hcat([partials(λ_elem) for λ_elem in λ])

    f = kinematics_jacobianTvp(model, x_value, λ_value)
    df_dx = kinematics_force_jacobian(model, x_value, λ_value)*x_partials
    df_dλ = kinematics_velocity_jacobian(model, x_value)[:, model.nq + 1:end]'*λ_partials
    return [Dual{T}(f[k], df_dx[k] + df_dλ[k]) for k in eachindex(f)]
end

# TODO add kinematics_jTvp (jacobian-transpose-vector product) to use with kinematics_force_jacobian (i.e. d_dx J(x)'λ)
