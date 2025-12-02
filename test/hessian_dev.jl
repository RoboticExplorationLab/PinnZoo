using LinearAlgebra
using ForwardDiff
import ForwardDiff: jacobian, Dual, value, partials, Partials


nx = 16
idx = 1:2
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

function hess_wrapper(x::AbstractVector{Dual{T1, Dual{T2, V2, N2}, N1}}, terms::Function) where {T1, T2, V2, N2, N1}
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
    b_new = b*df
    c_new = c*df

    # Final coefficient size depends on whether the function is vector-valued
    if isa(f, Vector)
        @error "unsupported"
    else
        d_new = d*df + [b*ddf_func(c_elem) for c_elem in eachrow(c)]

        # Build dual
        new_val = Dual{T2}(a_new, b_new...)
        new_partials = [Dual{T2}(c_elem, d_elem...) for (c_elem, d_elem) in zip(c_new, d_new)]
        return Dual{T1}(new_val, new_partials...)
    end
end

x, λ = randn(nx), randn(length(idx))
function lag(x::AbstractVector{Dual{T1, Dual{T2, V2, N2}, N1}}, λ) where {T1, T2, V2, N2, N1}
    terms(x) = l(x, λ), ForwardDiff.gradient(_x -> l(_x, λ), x), c -> ForwardDiff.hessian(_x ->  l(_x, λ), x)'*c
    return hess_wrapper(x, terms)
end; ForwardDiff.hessian(_x -> lag(_x, λ), x)
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
res ≈ vals[2]

# Test internal constraint derivative
vals_int = []; ForwardDiff.hessian(_x -> lag2(_x, λ), x)
T1_int, T2_int = eltype(vals_int[1]).parameters[1], eltype(vals_int[1]).parameters[2].parameters[1]
@assert vals[1] == vals_int[1]
f_a_int = c_func(a)
df_int = ForwardDiff.jacobian(_x -> c_func(_x), a)
ddf_int(c) = ForwardDiff.jacobian(_x -> ForwardDiff.jacobian(_x -> c_func(_x), a)*c, a)
b_df_a_int = b*df_int'
c_df_a_int = c*df_int'
t1 = [vcat(d_elem'*df_int'...) for d_elem in eachrow(d)]
t2 = [ddf_int(c_elem)*b' for c_elem in eachrow(c)]
b_ddf_a_c_int = t1 + t2
val_int = [Dual{T2_int}(f1, f2...) for (f1, f2) in zip(f_a_int, c_df_a_int)]
partial_int = [Dual{T2}(f1, f2...) for (f1, f2) in zip(c_df_a_int, b_ddf_a_c_int)]
res_int = [Dual{T1}(v, p...) for (v, p) in zip(val_int, partial_int)]
res_int == vals_int[2]