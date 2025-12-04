using LinearAlgebra
using ForwardDiff
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
    terms(x) = l(x, λ), ForwardDiff.gradient(_x -> l(_x, λ), x)', c -> ForwardDiff.hessian(_x ->  l(_x, λ), x)'*c
    return hess_wrapper_scalar(x, terms)
end
test_func(x) = lag(x, λ)
norm(ForwardDiff.hessian(test_func, x) - ForwardDiff.hessian(_x -> lag3(_x, λ), x), Inf)

# Test lagrangian hessian
vals = []; ForwardDiff.hessian(_x -> lag3(_x, λ), x)

x_in = vals[1]
a = [value(value(x_elem)) for x_elem in x_in]
b = hcat([partials(value(x_elem)) for x_elem in x_in]...)
c = hcat([value.(partials(x_elem)) for x_elem in x_in]...)
d = hcat([partials.(partials(x_elem)) for x_elem in x_in]...)

T1, T2 = typeof(vals[2]).parameters[1], typeof(vals[2]).parameters[2].parameters[1]
f_a = l(a, λ)
df = ForwardDiff.gradient(_x -> l(_x, λ), a)
ddf(c) = ForwardDiff.gradient(_x -> ForwardDiff.gradient(_x -> l(_x, λ), _x)'*c, a)
b_df_a = b*df
c_df_a = c*df
b_ddf_a_c = d*df + [b*ddf(c_elem) for c_elem in eachrow(c)]
val = Dual{T2}(f_a, b_df_a...)
partial = [Dual{T2}(f1, f2...) for (f1, f2) in zip(c_df_a, b_ddf_a_c)]
res = Dual{T1}(val, partial...)

# Test internal constraint derivative
vals_int = []; ForwardDiff.hessian(_x -> lag2(_x, λ), x)
x_in = vals[1]
num_p = length(partials(x_in[1]))
a = [value(value(x_elem)) for x_elem in x_in]
b_T = hcat([partials(value(x_elem)) for x_elem in x_in])
c = [[value(partials(x_elem)[j]) for x_elem in x_in] for j in 1:num_p]
d_T = [hcat([partials.(partials(x_elem)[j]) for x_elem in x_in]) for j in 1:num_p]

# Comparing b_ij to c_ij
b_ij = hcat([[partials(value(vals_int[2][i]))[j] for j in 1:num_p] for i in 1:length(vals_int[2])]...)
c_ij = hcat([[value(partials(vals_int[2][i])[j]) for j in 1:num_p] for i in 1:length(vals_int[2])]...)
norm(b_ij - c_ij, Inf)

T1_int, T2_int = eltype(vals_int[1]).parameters[1], eltype(vals_int[1]).parameters[2].parameters[1]
@assert vals[1] == vals_int[1]
f_a_int = c_func(a)
df_int = ForwardDiff.jacobian(_x -> c_func(_x), a)
ddf_int(c) = ForwardDiff.jacobian(_x -> ForwardDiff.jacobian(_x -> c_func(_x), _x)*c, a)
df_b_int = df_int*b_T # don't need?
df_c_int = df_int*hcat(c...) # iterate rows when building value(partials(...)) and partials(value(...))
df_d_int = [df_int*d for d in d_T]
dx_df_c_b_int = [ddf_int(c[j])*b_T for j in 1:num_p]
ddf_total_int = df_d_int + dx_df_c_b_int

val_int = [Dual{T2_int}(f_a_int[k], df_b_int[k]...) for k in 1:2]
test = cat(ddf_total_int..., dims=3)
partial_int = [Dual{T2_int}.(df_b_int[k], test[k, :, :]...) for k in 1:2]
res_int = [Dual{T1_int}(v, p...) for (v, p) in zip(val_int, partial_int)]

# Quick test to see if it works for 1-D
f_a_int, df_int = f_a, ForwardDiff.gradient(_x -> l(_x, λ), a)'
ddf_int(c) = ForwardDiff.gradient(_x -> ForwardDiff.gradient(_x -> l(_x, λ), _x)'*c, a)'

df_b_int = df_int*b_T # don't need?
df_c_int = df_int*hcat(c...) # iterate rows when building value(partials(...)) and partials(value(...))
df_d_int = [df_int*d for d in d_T]
dx_df_c_b_int = [ddf_int(c[j])*b_T for j in 1:num_p]
ddf_total_int = df_d_int + dx_df_c_b_int

val_int = [Dual{T2_int}(f_a_int[k], df_c_int[k, :]...) for k in 1:1]
test = cat(ddf_total_int..., dims=3)
partial_int = [Dual{T2_int}.(df_c_int[k, :], test[k, :, :]...) for k in 1:1]
res_int = [Dual{T1_int}(v, p...) for (v, p) in zip(val_int, partial_int)]

function hess_wrapper(x::AbstractVector{Dual{T1, Dual{T2, V2, N2}, N1}}, terms::Function) where {T1, T2, V2, N2, N1}
    # A 2nd derivative dual number is x = (a + bϵ₁) + (c + dϵ₁)ϵ₂. 
    # with ϵ₁² = ϵ₂² = 0
    # The expension of f(x) is f(a) + (df'b)ϵ₁ + (df'c)ϵ₂ + (df'd + c'ddf*b)ϵ₁ϵ₂
    # giving the output dual number res = (f(a) + (df'b)ϵ₁) + (df'c + (df'd + c'ddf*b)ϵ₁)ϵ₂

    # Extract initial dual number coefficients
    num_p = length(partials(x[1]))
    a = [value(value(x_elem)) for x_elem in x]
    b_T = hcat([partials(value(x_elem)) for x_elem in x])
    c = [[value(partials(x_elem)[j]) for x_elem in x] for j in 1:num_p]
    d_T = [hcat([partials.(partials(x_elem)[j]) for x_elem in x]) for j in 1:num_p]

    # Get value, gradient, and hessian-vector product function
    f, df, ddf_func = terms(a)
    
    # Compute new dual number coefficients
    a_new = f
    b_new = df*b_T # from symmetry this is equal to c_new, so we don't need to compute that
    d_new = cat(([df*d for d in d_T] + [ddf_func(c[j])*b_T for j in 1:num_p])..., dims=3)

    # Build vector
    nf = length(a_new)
    val = [Dual{T2}(a_new[k], b_new[k]...) for k in 1:nf]
    partial = [Dual{T2}.(b_new[k], d_new[k, :, :]...) for k in 1:nf]
    f = [Dual{T1}(v, p...) for (v, p) in zip(val, partial)]
    return f
end
function c_func2(x::AbstractVector{Dual{T1, Dual{T2, V2, N2}, N1}}) where {T1, T2, V2, N2, N1}
    terms(x) = c_func(x), ForwardDiff.jacobian(_x -> c_func(_x), x), c -> ForwardDiff.jacobian(_x -> ForwardDiff.jacobian(_x -> c_func(_x), _x)*c, x)
    return hess_wrapper(x, terms)
end
test1, test2 = _x -> λ'*c_func(_x, true), _x -> λ'*c_func2(_x)
vals_int = []; H1 = ForwardDiff.hessian(test1, x)
H2 = ForwardDiff.hessian(test2, x)
norm(H1 - H2, Inf)

# Try with different sizes
nx = 12
idx = rand(1:12, 100)
x, λ = randn(nx), randn(length(idx))
vals_int = []; H1 = ForwardDiff.hessian(test1, x)
H2 = ForwardDiff.hessian(test2, x)
norm(H1 - H2, Inf)

k = 3
b_ij = hcat([[partials(value(vals_int[k][i]))[j] for j in 1:num_p] for i in 1:length(vals_int[k])]...)
c_ij = hcat([[value(partials(vals_int[k][i])[j]) for j in 1:num_p] for i in 1:length(vals_int[k])]...)
norm(b_ij - c_ij, Inf)
