
# forward_dynamics
function forward_dynamics(model::PinnZooModel, x::AbstractVector{Dual{T,N,V}}, τ::AbstractVector{Float64}) where {T,N,V}
    terms(x) = forward_dynamics(model, x, τ), forward_dynamics_deriv(model, x, τ)[1]
    return jac_wrapper(x, terms)
end
function forward_dynamics(model::PinnZooModel, x::AbstractVector{Float64}, τ::AbstractVector{Dual{T,N,V}}) where {T,N,V}
    terms(τ) = forward_dynamics(model, x, τ), forward_dynamics_deriv(model, x, τ)[2]
    return jac_wrapper(τ, terms)
end
function forward_dynamics(model::PinnZooModel, x::AbstractVector{Dual{T,N,V}}, τ::AbstractVector{Dual{T,N,V}}) where {T,N,V}
    nx = length(x)
    function terms(z)
        x, τ = z[1:nx], z[nx + 1:end]
        f = forward_dynamics(model, x, τ)
        f_dx, f_dτ = forward_dynamics_deriv(model, x, τ)
        return f, [f_dx f_dτ]
    end
    return jac_wrapper([x; τ], terms)
end

# dynamics
function dynamics(model::PinnZooModel, x::AbstractVector{Dual{T,N,V}}, τ::AbstractVector{Float64}) where {T,N,V}
    terms(x) = dynamics(model, x, τ), dynamics_deriv(model, x, τ)[1]
    return jac_wrapper(x, terms)
end
function dynamics(model::PinnZooModel, x::AbstractVector{Float64}, τ::AbstractVector{Dual{T,N,V}}) where {T,N,V}
    terms(τ) = dynamics(model, x, τ), dynamics_deriv(model, x, τ)[2]
    return jac_wrapper(τ, terms)
end
function dynamics(model::PinnZooModel, x::AbstractVector{Dual{T,N,V}}, τ::AbstractVector{Dual{T,N,V}}) where {T,N,V}
    nx = length(x)
    function terms(z)
        x, τ = z[1:nx], z[nx + 1:end]
        f = dynamics(model, x, τ)
        f_dx, f_dτ = dynamics_deriv(model, x, τ)
        return f, [f_dx f_dτ]
    end
    return jac_wrapper([x; τ], terms)
end;

# inverse_dynamics
function inverse_dynamics(model::PinnZooModel, x::AbstractVector{Dual{T,N,V}}, v̇::AbstractVector{Float64}) where {T,N,V}
    terms(x) = inverse_dynamics(model, x, v̇), inverse_dynamics_deriv(model, x, v̇)[1]
    return jac_wrapper(x, terms)
end
function inverse_dynamics(model::PinnZooModel, x::AbstractVector{Float64}, v̇::AbstractVector{Dual{T,N,V}}) where {T,N,V}
    terms(v̇) = inverse_dynamics(model, x, v̇),  inverse_dynamics_deriv(model, x, v̇)[2]
    return jac_wrapper(v̇, terms)
end
function inverse_dynamics(model::PinnZooModel, x::AbstractVector{Dual{T,N,V}}, v̇::AbstractVector{Dual{T,N,V}}) where {T,N,V}
    nx = length(x)
    function terms(z)
        x, v̇ = z[1:nx], z[nx + 1:end]
        f = inverse_dynamics(model, x, v̇)
        f_dx, f_dv̇ = inverse_dynamics_deriv(model, x, v̇)
        return f, [f_dx f_dv̇]
    end
    return jac_wrapper([x; v̇], terms)
end;
