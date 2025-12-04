using LinearAlgebra
import ForwardDiff as FD
import ForwardDiff: jacobian, Dual, value, partials, Partials

nx = 3
idx = [3; 3; 2:3]
function c_func(x, record = false)
    global vals_int
    res = x[idx]*x[idx]'*ones(length(idx))
    if record
        push!(vals_int, copy(x))
        push!(vals_int, copy(res))
    end
    return res
end
function lag2(x, λ)
    res = c_func(x, true)'*λ
    return res
end
function lag3(x, λ)
    global vals
    res = c_func(x, false)'*λ
    push!(vals, copy(x))
    push!(vals, copy(res))
    return res
end
l(x, λ) = c_func(x, false)'*λ
vals, vals_int = [], []

function hess_wrapper_scalar(x::AbstractVector{Dual{T1, Dual{T2, V2, N2}, N1}}, terms::Function) where {T1, T2, V2, N2, N1}
    # A 2nd derivative dual number is x = (a + bϵ₁) + (c + dϵ₁)ϵ₂. 
    # with ϵ₁² = ϵ₂² = 0
    # The expension of f(x) is f(a) + (df'b)ϵ₁ + (df'c)ϵ₂ + (df'd + c'ddf*b)ϵ₁ϵ₂
    # giving the output dual number res = (f(a) + (df'b)ϵ₁) + (df'c + (df'd + c'ddf*b)ϵ₁)ϵ₂

    # Extract initial dual number coefficients
    a = [value(value(x_elem)) for x_elem in x]
    b = hcat([partials(value(x_elem)) for x_elem in x]...)
    c = hcat([value.(partials(x_elem)) for x_elem in x]...)
    d = hcat([partials.(partials(x_elem)) for x_elem in x]...)

    # Get value, gradient, and hessian-vector product function
    f, df, ddf_func = terms(a)
    
    # Compute new dual number coefficients
    a_new = f
    b_new = b*df'
    c_new = c*df'

    # Final coefficient size depends on whether the function is vector-valued
    if isa(f, Vector)
        d_new = [vcat(d_elem'*df_int'...) for d_elem in eachrow(d)] + [ddf_int(c_elem)*b' for c_elem in eachrow(c)]
        new_val = [Dual{T2}(a_elem, b_elem...) for (a_elem, b_elem) in zip(a_new, b_new)]
        new_partials = [Dual{T2}(c_elem, d_elem...) for (c_elem, d_elem) in zip(c_elem, d_elem)]
        return [Dual{T1}(v, p...) for (v, p) in zip(new_val, new_partials)]
    else
        d_new = d*df' + [b*ddf_func(c_elem) for c_elem in eachrow(c)]

        # Build dual
        new_val = Dual{T2}(a_new, b_new...)
        new_partials = [Dual{T2}(c_elem, d_elem...) for (c_elem, d_elem) in zip(c_new, d_new)]
        return Dual{T1}(new_val, new_partials...)
    end
end

x, λ = randn(nx), randn(length(idx))
function lag(x::AbstractVector{Dual{T1, Dual{T2, V2, N2}, N1}}, λ) where {T1, T2, V2, N2, N1}
    terms(x) = l(x, λ), FD.gradient(_x -> l(_x, λ), x)', c -> FD.hessian(_x ->  l(_x, λ), x)'*c
    return hess_wrapper_scalar(x, terms)
end
test_func(x) = lag(x, λ)
norm(FD.hessian(test_func, x) - FD.hessian(_x -> lag3(_x, λ), x), Inf)

# Test lagrangian hessian
vals = []; FD.hessian(_x -> lag3(_x, λ), x)

x_in = vals[1]
a = [value(value(x_elem)) for x_elem in x_in]
b = hcat([partials(value(x_elem)) for x_elem in x_in]...)
c = hcat([value.(partials(x_elem)) for x_elem in x_in]...)
d = hcat([partials.(partials(x_elem)) for x_elem in x_in]...)

T1, T2 = typeof(vals[2]).parameters[1], typeof(vals[2]).parameters[2].parameters[1]
f_a = l(a, λ)
df = FD.gradient(_x -> l(_x, λ), a)
ddf(c) = FD.gradient(_x -> FD.gradient(_x -> l(_x, λ), _x)'*c, a)
b_df_a = b*df
c_df_a = c*df
b_ddf_a_c = d*df + [b*ddf(c_elem) for c_elem in eachrow(c)]
val = Dual{T2}(f_a, b_df_a...)
partial = [Dual{T2}(f1, f2...) for (f1, f2) in zip(c_df_a, b_ddf_a_c)]
res = Dual{T1}(val, partial...)

# Test internal constraint derivative
nx = 13
idx = [1; 10;]#rand(1:12, 100)
x, λ = randn(nx), randn(length(idx))
vals_int = []; FD.hessian(_x -> lag2(_x, λ), x)
x_in = vals_int[3]
num_p = length(partials(x_in[1]))
a = [value(value(x_elem)) for x_elem in x_in]
b = hcat([partials(value(x_elem)) for x_elem in x_in]...)
c = [[value(partials(x_elem)[j]) for x_elem in x_in] for j in 1:num_p]
d_T = [hcat([partials.(partials(x_elem)[j]) for x_elem in x_in]) for j in 1:num_p]

# Comparing b_ij to c_ij
b_ij = hcat([[partials(value(vals_int[3][i]))[j] for j in 1:num_p] for i in 1:length(vals_int[3])]...)
c_ij = hcat([[value(partials(vals_int[3][i])[j]) for j in 1:num_p] for i in 1:length(vals_int[3])]...)
norm(b_ij - c_ij, Inf)

T1_int, T2_int = eltype(vals_int[1]).parameters[1], eltype(vals_int[1]).parameters[2].parameters[1]
f_a_int = c_func(a)
df_int = FD.jacobian(_x -> c_func(_x), a)
ddf_int(c) = FD.jacobian(_x -> FD.jacobian(_x -> c_func(_x), _x)*c, a)
df_b_int = df_int*b' # don't need?
df_c_int = df_int*hcat(c...) # iterate rows when building value(partials(...)) and partials(value(...))
df_d_int = [hcat(df_int*d...)' for d in d_T]
dx_df_b_c_int = [hcat([ddf_int(b[i, :])*c[j] for i in 1:num_p]...) for j in 1:num_p]
dx_df_c_b_int = [ddf_int(c[j])*b' for j in 1:num_p]
ddf_total_int = df_d_int + 0.5*(dx_df_b_c_int + dx_df_c_b_int)

val_int = [Dual{T2_int}(f_a_int[k], df_b_int[k, :]...) for k in 1:2]
test = cat(ddf_total_int..., dims=3)
partial_int = [Partials(Tuple(Dual{T2_int}.(df_c_int[k, :], eachrow(test[k, :, :])...))) for k in 1:2]
res_int = [Dual{T1_int}(v, p...) for (v, p) in zip(val_int, partial_int)]

# Quick test to see if it works for 1-D
f_a_int, df_int = f_a, FD.gradient(_x -> l(_x, λ), a)'
ddf_int(c) = FD.gradient(_x -> FD.gradient(_x -> l(_x, λ), _x)'*c, a)'

df_b_int = df_int*b' # don't need?
df_c_int = df_int*hcat(c...) # iterate rows when building value(partials(...)) and partials(value(...))
df_d_int = [hcat(df_int*d...)' for d in d_T]
dx_df_b_c_int = [hcat([ddf_int(b[i, :])*c[j] for i in 1:num_p]...) for j in 1:num_p]
dx_df_c_b_int = [ddf_int(c[j])*b' for j in 1:num_p]
ddf_total_int = df_d_int + 0.5*(dx_df_b_c_int + dx_df_c_b_int)

val_int = [Dual{T2_int}(f_a_int[k], df_b_int[k, :]...) for k in 1:1]
test = cat(ddf_total_int..., dims=3)
partial_int = [Partials(Tuple(Dual{T2_int}.(df_c_int[k, :], eachrow(test[k, :, :])...))) for k in 1:1]
res_int = [Dual{T1_int}(v, p...) for (v, p) in zip(val_int, partial_int)]

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
function c_func2(x::AbstractVector{Dual{T1, Dual{T2, V2, N2}, N1}}) where {T1, T2, V2, N2, N1}
    dc_func(x) = sum(x)*I(nx) + x * ones(nx)'
    ddc_func(x, c) = c*ones(nx)' + sum(c)*I(nx)
    terms(x) = c_func(x), dc_func(x), c -> ddc_func(x, c)
    return hess_wrapper(x, terms)
end
nx = 5
idx = 1:nx
x, λ = randn(nx), randn(length(idx))
test1, test2 = _x -> λ'*c_func(_x, true), _x -> λ'*c_func2(_x)
vals_int = []; H1 = FD.hessian(test1, x)
H2 = FD.hessian(test2, x)
norm(H1 - H2, Inf)

# Check if wrapper works for scalar functions
function lag4(x::AbstractVector{Dual{T1, Dual{T2, V2, N2}, N1}}, λ) where {T1, T2, V2, N2, N1}
    terms(x) = l(x, λ), FD.gradient(_x -> l(_x, λ), x)', c -> FD.gradient(_x -> FD.gradient(_x -> l(_x, λ), _x)'*c, x)'
    return hess_wrapper(x, terms)
end
H1 = FD.hessian(_x -> lag4(_x, λ), x)
H2 = FD.hessian(_x -> l(_x, λ), x)
norm(H1 - H2, Inf)

# Try with different sizes
nx = 100
idx = 1:nx
x, λ = randn(nx), randn(length(idx))
vals_int = []; H1 = FD.hessian(test1, x)
H2 = FD.hessian(test2, x)
norm(H1 - H2, Inf)

@profview H2 = FD.hessian(test2, x)

using BenchmarkTools
@benchmark FD.hessian(test1, x)
@benchmark FD.hessian(test2, x)