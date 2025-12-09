
# forward_dynamics
function forward_dynamics(model::PinnZooModel, x::AbstractVector{Dual{T,N,V}}, τ::AbstractVector{Float64}) where {T,N,V}
    terms(_x) = forward_dynamics(model, _x, τ), forward_dynamics_deriv(model, _x, τ)[1]
    return jac_wrapper(x, terms)
end
function forward_dynamics(model::PinnZooModel, x::AbstractVector{Float64}, τ::AbstractVector{Dual{T,N,V}}) where {T,N,V}
    terms(_τ) = forward_dynamics(model, x, _τ), forward_dynamics_deriv(model, x, _τ)[2]
    return jac_wrapper(τ, terms)
end
function forward_dynamics(model::PinnZooModel, x::AbstractVector{Dual{T,N,V}}, τ::AbstractVector{Dual{T,N,V}}) where {T,N,V}
    nx = length(x)
    function terms(_z)
        _x, _τ = _z[1:nx], _z[nx + 1:end]
        f = forward_dynamics(model, _x, _τ)
        f_dx, f_dτ = forward_dynamics_deriv(model, _x, _τ)
        return f, [f_dx f_dτ]
    end
    return jac_wrapper([x; τ], terms)
end

# dynamics
function dynamics(model::PinnZooModel, x::AbstractVector{Dual{T,N,V}}, τ::AbstractVector{Float64}) where {T,N,V}
    terms(_x) = dynamics(model, _x, τ), dynamics_deriv(model, _x, τ)[1]
    return jac_wrapper(x, terms)
end
function dynamics(model::PinnZooModel, x::AbstractVector{Float64}, τ::AbstractVector{Dual{T,N,V}}) where {T,N,V}
    terms(_τ) = dynamics(model, x, _τ), dynamics_deriv(model, x, _τ)[2]
    return jac_wrapper(τ, terms)
end
function dynamics(model::PinnZooModel, x::AbstractVector{Dual{T,N,V}}, τ::AbstractVector{Dual{T,N,V}}) where {T,N,V}
    nx = length(x)
    function terms(_z)
        _x, _τ = _z[1:nx], _z[nx + 1:end]
        f = dynamics(model, _x, _τ)
        f_dx, f_dτ = dynamics_deriv(model, _x, _τ)
        return f, [f_dx f_dτ]
    end
    return jac_wrapper([x; τ], terms)
end;

# inverse_dynamics jacobian
function inverse_dynamics(model::PinnZooModel, x::AbstractVector{Dual{T,N,V}}, v̇::AbstractVector{Float64}) where {T,N,V}
    terms(_x) = inverse_dynamics(model, _x, v̇), inverse_dynamics_deriv(model, _x, v̇)[1]
    return jac_wrapper(x, terms)
end
function inverse_dynamics(model::PinnZooModel, x::AbstractVector{Float64}, v̇::AbstractVector{Dual{T,N,V}}) where {T,N,V}
    terms(_v̇) = inverse_dynamics(model, x, _v̇),  inverse_dynamics_deriv(model, x, _v̇)[2]
    return jac_wrapper(v̇, terms)
end
function inverse_dynamics(model::PinnZooModel, x::AbstractVector{Dual{T,N,V}}, v̇::AbstractVector{Dual{T,N,V}}) where {T,N,V}
    nx = length(x)
    function terms(_z)
        _x, _v̇ = _z[1:nx], _z[nx + 1:end]
        f = inverse_dynamics(model, _x, _v̇)
        f_dx, f_dv̇ = inverse_dynamics_deriv(model, _x, _v̇)
        return f, [f_dx f_dv̇]
    end
    return jac_wrapper([x; v̇], terms)
end;

# inverse dynamics hessian of μ'*ID(x, v̇)
function inverse_dynamics(model::PinnZooModel, x::AbstractVector{Dual{T1, Dual{T2, V2, N2}, N1}}, v̇::AbstractVector{Float64}) where {T1, T2, V2, N2, N1}
    terms(_x) = inverse_dynamics(model, _x, v̇), inverse_dynamics_deriv(model, _x, v̇)[1], dx -> inverse_dynamics_hvp(model, _x, v̇, dx)[1]
    return hess_wrapper(x, terms)
end

function inverse_dynamics(model::PinnZooModel, x::AbstractVector{Float64}, v̇::AbstractVector{Dual{T1, Dual{T2, V2, N2}, N1}}) where {T1, T2, V2, N2, N1}
    terms(_v̇) = inverse_dynamics(model, x, _v̇), inverse_dynamics_deriv(model, x, _v̇)[2], dv̇ -> zeros(model.nv, model.nv)
    return hess_wrapper(v̇, terms)
end

function inverse_dynamics(model::PinnZooModel, x::AbstractVector{Dual{T1, Dual{T2, V2, N2}, N1}}, v̇::AbstractVector{Dual{T1, Dual{T2, V2, N2}, N1}}) where {T1, T2, V2, N2, N1}
    function terms(_z)
        _x, _v̇ = _z[1:model.nx], _z[model.nx + 1:end]
        f = inverse_dynamics(model, _x, _v̇)
        df = hcat(inverse_dynamics_deriv(model, _x, _v̇)...)
        function ddf(dz)
            dx, dv̇ = dz[1:model.nx], dz[model.nx + 1:end]
            H1, H2 = inverse_dynamics_hvp(model, _x, _v̇, dx)
            H3 = M_jvp(model, _x, dv̇)
            return [H1 + H3 H2]
        end
        return f, df, ddf
    end
    return hess_wrapper([x; v̇], terms)
end
