import ForwardDiff: jacobian, Dual, Partials, value, partials

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

# wraps f, df, ddf_func = terms(a) to make a function f(a) compatible with ForwardDiff.hessian where
# a - n-dim vector
# f - scalar or m-dim vector
# df - jacobian of f as an m×n matrix (can be 1×n adjoint vector if f is scalar)
# ddf_func(c) - d/da (df*c) as an m×n matrix (can be 1×n adjoint vector if f is scalar) where c is an n-dim vector
function hess_wrapper(x::AbstractVector{Dual{T1, Dual{T2, V2, N2}, N1}}, terms::Function) where {T1, T2, V2, N2, N1}
    # A 2nd derivative dual number is x = (a + bϵ₁) + (c + dϵ₁)ϵ₂. 
    # with ϵ₁² = ϵ₂² = 0
    # The expension of f(x) is f(a) + (df'b)ϵ₁ + (df'c)ϵ₂ + (df'd + c'ddf*b)ϵ₁ϵ₂
    # giving the output dual number res = (f(a) + (df'b)ϵ₁) + (df'c + (df'd + c'ddf*b)ϵ₁)ϵ₂

    # Extract initial dual number coefficients
    num_p = length(partials(x[1]))
    a = [value(value(x_elem)) for x_elem in x]
    b = hcat([partials(value(x_elem)) for x_elem in x]...)
    c = [[value(partials(x_elem)[j]) for x_elem in x] for j in 1:num_p]
    d_T = [hcat([partials.(partials(x_elem)[j]) for x_elem in x]) for j in 1:num_p]

    # Get value, gradient, and hessian-vector product function
    f, df, ddf_func = terms(a)
    
    # Compute new dual number coefficients
    a_new = f
    b_new = df*b' 
    c_new = df*hcat(c...) # we shouldn't need to compute this because of symmetry, but forward diff chunks up partials which breaks

    # d is a bit more complicated since it consists of d/dx(df/dx b)*c + d/dx(df/dx c)*b
    dx_df_b_c = [hcat([ddf_func(b[i, :])*c[j] for i in 1:num_p]...) for j in 1:num_p]
    dx_df_c_b = [ddf_func(c[j])*b' for j in 1:num_p]
    d_new = cat(([hcat(df*d...)' for d in d_T] + 0.5*(dx_df_b_c + dx_df_c_b))..., dims=3)

    # Build vector
    nf = length(a_new)
    val = [Dual{T2}(a_new[k], b_new[k, :]...) for k in 1:nf]
    partial = [Partials(Tuple(Dual{T2}.(c_new[k, :], eachrow(d_new[k, :, :])...))) for k in 1:nf]
    f = [Dual{T1}(v, p...) for (v, p) in zip(val, partial)]
    return isa(a_new, Vector) ? f : f[1] # Handle hessians of scalars
end