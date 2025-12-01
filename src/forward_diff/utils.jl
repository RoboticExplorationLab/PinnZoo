import ForwardDiff: jacobian, Dual, value, partials

function jac_wrapper(x::AbstractVector{Dual{T, V, N}}, evaluator::Function) where {T, V, N}
    x_value, x_partials = [value(x_elem) for x_elem in x], hcat([partials(x_elem) for x_elem in x])

    f, df = evaluator(x_value)
    df_dx = df*x_partials
    return [Dual{T}(f[k], df_dx[k]) for k in eachindex(f)]
end