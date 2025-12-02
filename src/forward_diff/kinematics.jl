# kinematics
function kinematics(model::PinnZooModel, x::AbstractVector{Dual{T,V,N}}) where {T,V,N}
    terms(x) = kinematics(model, x), kinematics_jacobian(model, x)
    return jac_wrapper(x, terms)
end

function kinematics_rotation(model::PinnZooModel, x::AbstractVector{Dual{T,V,N}}) where {T,V,N}
    function terms(x)
        locs = kinematics(model, x)
        f = _kinematics_rotation(model, x, locs)
        df = jacobian(_locs -> _kinematics_rotation(model, x, _locs), locs)*kinematics_jacobian(model, x)
        return f, df
    end
    return jac_wrapper(x, terms)
end

# kinematics_velocity
function kinematics_velocity(model::PinnZooModel, x::AbstractVector{Dual{T,V,N}}) where {T,V,N}
    terms(x) = kinematics_velocity(model, x), kinematics_velocity_jacobian(model, x)
    return jac_wrapper(x, terms)
end

function kinematics_jacobianTvp(model::PinnZooModel, x::AbstractVector{Dual{T,V,N}}, λ::AbstractVector{Float64}) where {T,V,N}
    terms(x) = kinematics_jacobianTvp(model, x, λ), kinematics_force_jacobian(model, x, λ)
    return jac_wrapper(x, terms)
end

function kinematics_jacobianTvp(model::PinnZooModel, x::AbstractVector{Float64}, λ::AbstractVector{Dual{T,V,N}}) where {T,V,N}
    terms(λ) = kinematics_jacobianTvp(model, x, λ), kinematics_velocity_jacobian(model, x)[:, model.nq + 1:end]
    return jac_wrapper(λ, terms)
end

function kinematics_jacobianTvp(model::PinnZooModel, x::AbstractVector{Dual{T,V,N}}, λ::AbstractVector{Dual{T,V,N}}) where {T,V,N}
    nx = length(x)
    function terms(z)
        x, λ = z[1:nx], z[nx + 1:end]
        f = kinematics_jacobianTvp(model, x, λ)
        f_dx = kinematics_force_jacobian(model, x, λ)'
        f_dλ = kinematics_velocity_jacobian(model, x)[:, model.nq + 1:end]
        return f, [f_dx f_dλ]
    end
    return jac_wrapper([x; λ], terms)
end
