import ForwardDiff: jacobian, Dual, value, partials

function jac_wrapper(x::AbstractVector{Dual{T, V, N}}, terms::Function) where {T, V, N}
    # Each dual number x_i = a_i + Σ b_ij*ϵ_ij = a_i + b_i'ϵ_i where x_i ∈ ℝ^n, b_i ∈ ℝ^p
    # x_value is an AbstractVector{V}, x_partials is a n×1 matrix of Partials of size p
    # if you do hcat([]...) instead x_partials will be a pxn matrix of type V
    # this later option can be used in the future to support gradient-vector products
    x_value, x_partials = [value(x_elem) for x_elem in x], hcat([partials(x_elem) for x_elem in x])

    f, df = terms(x_value)
    df_dx = df*x_partials
    return [Dual{T}(f[k], df_dx[k]) for k in eachindex(f)]
end