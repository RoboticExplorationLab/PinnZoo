# Wrap derivatives from Pinocchio so they can be used with ForwardDiff
import ForwardDiff: Dual, value, partials, Partials

# forward_dynamics
function forward_dynamics(model::PinnZooModel, x::AbstractVector{Dual{T,N,V}}, τ::AbstractVector{Float64}) where {T,N,V}
    x_value, x_partials = [value(x_elem) for x_elem in x], hcat([partials(x_elem) for x_elem in x])

    f = forward_dynamics(model, x_value, τ)
    df_dx = forward_dynamics_deriv(model, x_value, τ)[1]*x_partials
    return [Dual{T}(f[k], df_dx[k]) for k in eachindex(f)]
end
function forward_dynamics(model::PinnZooModel, x::AbstractVector{Float64}, τ::AbstractVector{Dual{T,N,V}}) where {T,N,V}
    τ_value, τ_partials = [value(τ_elem) for τ_elem in τ], hcat([partials(τ_elem) for τ_elem in τ])
    
    f = forward_dynamics(model, x, τ_value)
    df_dτ = forward_dynamics_deriv(model, x, τ_value)[2]*τ_partials
    return [Dual{T}(f[k], df_dτ[k]) for k in eachindex(f)]
end
function forward_dynamics(model::PinnZooModel, x::AbstractVector{Dual{T,N,V}}, τ::AbstractVector{Dual{T,N,V}}) where {T,N,V}
    x_value, x_partials = [value(x_elem) for x_elem in x], hcat([partials(x_elem) for x_elem in x])
    τ_value, τ_partials = [value(τ_elem) for τ_elem in τ], hcat([partials(τ_elem) for τ_elem in τ])

    f = forward_dynamics(model, x_value, τ_value)
    (df_dx, df_dτ) = forward_dynamics_deriv(model, x_value, τ_value).*(x_partials, τ_partials)
    return [Dual{T}(f[k], df_dx[k] + df_dτ[k]) for k in eachindex(f)]
end

# dynamics
function dynamics(model::PinnZooModel, x::AbstractVector{Dual{T,N,V}}, τ::AbstractVector{Float64}) where {T,N,V}
    x_value, x_partials = [value(x_elem) for x_elem in x], hcat([partials(x_elem) for x_elem in x])

    f = dynamics(model, x_value, τ)
    df_dx = dynamics_deriv(model, x_value, τ)[1]*x_partials
    return [Dual{T}(f[k], df_dx[k]) for k in eachindex(f)]
end
function dynamics(model::PinnZooModel, x::AbstractVector{Float64}, τ::AbstractVector{Dual{T,N,V}}) where {T,N,V}
    τ_value, τ_partials = [value(τ_elem) for τ_elem in τ], hcat([partials(τ_elem) for τ_elem in τ])
    
    f = dynamics(model, x, τ_value)
    df_dτ = dynamics_deriv(model, x, τ_value)[2]*τ_partials
    return [Dual{T}(f[k], df_dτ[k]) for k in eachindex(f)]
end
function dynamics(model::PinnZooModel, x::AbstractVector{Dual{T,N,V}}, τ::AbstractVector{Dual{T,N,V}}) where {T,N,V}
    x_value, x_partials = [value(x_elem) for x_elem in x], hcat([partials(x_elem) for x_elem in x])
    τ_value, τ_partials = [value(τ_elem) for τ_elem in τ], hcat([partials(τ_elem) for τ_elem in τ])

    f = dynamics(model, x_value, τ_value)
    (df_dx, df_dτ) = dynamics_deriv(model, x_value, τ_value).*(x_partials, τ_partials)
    
    return [Dual{T}(f[k], df_dx[k] + df_dτ[k]) for k in eachindex(f)]
end;

# inverse_dynamics
function inverse_dynamics(model::PinnZooModel, x::AbstractVector{Dual{T,N,V}}, v̇::AbstractVector{Float64}) where {T,N,V}
    x_value, x_partials = [value(x_elem) for x_elem in x], hcat([partials(x_elem) for x_elem in x])

    f = inverse_dynamics(model, x_value, v̇)
    df_dx = inverse_dynamics_deriv(model, x_value, v̇)[1]*x_partials
    return [Dual{T}(f[k], df_dx[k]) for k in eachindex(f)]
end
function inverse_dynamics(model::PinnZooModel, x::AbstractVector{Float64}, v̇::AbstractVector{Dual{T,N,V}}) where {T,N,V}
    v̇_value, v̇_partials = [value(v̇_elem) for v̇_elem in v̇], hcat([partials(v̇_elem) for v̇_elem in v̇])
    
    f = inverse_dynamics(model, x, v̇_value)
    df_dv̇ = inverse_dynamics_deriv(model, x, v̇_value)[2]*v̇_partials
    return [Dual{T}(f[k], df_dv̇[k]) for k in eachindex(f)]
end
function inverse_dynamics(model::PinnZooModel, x::AbstractVector{Dual{T,N,V}}, v̇::AbstractVector{Dual{T,N,V}}) where {T,N,V}
    x_value, x_partials = [value(x_elem) for x_elem in x], hcat([partials(x_elem) for x_elem in x])
    v̇_value, v̇_partials = [value(v̇_elem) for v̇_elem in v̇], hcat([partials(v̇_elem) for v̇_elem in v̇])

    f = inverse_dynamics(model, x_value, v̇_value)
    (df_dx, df_dv̇) = inverse_dynamics_deriv(model, x_value, v̇_value).*(x_partials, v̇_partials)
    
    return [Dual{T}(f[k], df_dx[k] + df_dv̇[k]) for k in eachindex(f)]
end;
