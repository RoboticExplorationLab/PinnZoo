using LinearAlgebra
using ForwardDiff
import ForwardDiff: jacobian, Dual, value, partials, Partials

function c_func(x, record = false)
    global vals_int
    res = x.*x
    if record
        push!(vals_int, x)
        push!(vals_int, res)
    end
    return x.*x
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

x, λ = randn(100), randn(100)
function lag(x::AbstractVector{Dual{T1, Dual{T2, V2, N2}, N1}}, λ) where {T1, T2, V2, N2, N1}
    # A 2nd derivative dual number is x = (a + bϵ₁) + (c + dϵ₁)ϵ₂. 
    # with ϵ₁² = ϵ₂² = 0
    # The expension of f(x) is f(a) + (df'b)ϵ₁ + (df'c)ϵ₂ + (df'd + c'ddf*b)ϵ₁ϵ₂
    # giving the output dual number res = (f(a) + (df'b)ϵ₁) + (df'c + (df'd + c'ddf*b)ϵ₁)ϵ₂

    # Extract initial dual number coefficients
    a = [value(value(x_elem)) for x_elem in x]
    b = hcat([partials(value(x_elem)) for x_elem in x]...)
    c = hcat([value.(partials(x_elem)) for x_elem in x]...)
    d = hcat([partials.(partials(x_elem)) for x_elem in x]...)

    # Compute function, gradient, and hessian
    f = (a.*a)'*λ
    df = 2*diagm(a)'*λ
    ddf(c) = 2*diagm(λ)*c

    # Compute new dual number coefficients
    a_new = f
    b_new = b*df
    c_new = c*df
    d_new = d*df + [b*ddf(c_elem) for c_elem in eachrow(c)]

    # Build new dual number
    new_val = Dual{T2}(a_new, b_new...)
    new_partials = [Dual{T2}(c_elem, d_elem...) for (c_elem, d_elem) in zip(c_new, d_new)]
    return Dual{T1}(new_val, new_partials...)
end; ForwardDiff.hessian(_x -> lag(_x, λ), x)
norm(ForwardDiff.hessian(_x -> lag(_x, λ), x) - ForwardDiff.hessian(_x -> lag3(_x, λ), x), Inf)

# Test lagrangian hessian
vals = []; ForwardDiff.hessian(_x -> lag3(_x, λ), x)

x_in = vals[1]
a = [value(value(x_elem)) for x_elem in x_in]
b = hcat([partials(value(x_elem)) for x_elem in x_in]...)
c = hcat([value.(partials(x_elem)) for x_elem in x_in]...)
d = hcat([partials.(partials(x_elem)) for x_elem in x_in]...)

T1, T2 = typeof(vals[2]).parameters[1], typeof(vals[2]).parameters[2].parameters[1]
f_a = (a.*a)'*λ
b_df_a = b*(2*diagm(a)'*λ)
c_df_a = c*(2*diagm(a)'*λ)
t1 = d*(2*diagm(a)'*λ)
t2 = [b*2*diagm(λ)*c_elem for c_elem in eachrow(c)]
b_ddf_a_c = d*(2*diagm(a)'*λ) + [b*2*diagm(λ)*c_elem for c_elem in eachrow(c)]
val = Dual{T2}(f_a, b_df_a...)
partial = [Dual{T2}(f1, f2...) for (f1, f2) in zip(c_df_a, b_ddf_a_c)]
res = Dual{T1}(val, partial...)
res == vals[2]

# Test internal constraint derivative
# vals_int = []; ForwardDiff.hessian(_x -> lag2(_x, λ), x)
# T1_int, T2_int = eltype(vals_int[2]).parameters[1], eltype(vals_int[2]).parameters[2].parameters[1]
# @assert vals[1] == vals_int[1]
# f_a_int = a.*a
# b_df_a_int = (2*diagm(a))'*b
# c_df_a_int = hcat(c...)*(2*diagm(a)')
# t1 = hcat((2*diagm(a))'*d...)
# t2 = hcat([2*diagm(hcat(c...)[i, :])*hcat(b...)[i, :] for i in 1:length(c[1])]...)
# b_ddf_a_c_int = 2*diagm(a)'*d + [2*diagm(hcat(c...)[:, i])'*hcat(b...) for i in 1:length(c)]
# val_int = [Dual{T2_int}(f1, f2...) for (f1, f2) in zip(f_a_int, c_df_a_int)]
# partial_int =  [[Dual{T2_int}(c_df_a_int[i][j], b_ddf_a_c_int[i][j]) for j in 1:length(c_df_a_int[i])] for i in 1:length(c_df_a_int)]
# res_int = [Dual{T1}(v, p...) for (v, p) in zip(val_int, partial_int)]
# res_int == vals_int[2]