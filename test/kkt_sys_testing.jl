using Revise
using PinnZoo
import ForwardDiff as FD
using LinearAlgebra

model = Pineapple()

# Dummy constraint to test lagrangian hessian
function constraint(model, x, f, v̇)
    return [inverse_dynamics(model, x, v̇) + kinematics_jacobianTvp(model, x, f);]
end

x = randn_state(model)
f = randn(kinematics_size(model))
v̇ = randn(model.nv)
μ = ones(length(constraint(model, x, f, v̇)))

# Test seperate hessians
FD.hessian(_tmp -> μ'*constraint(model, _tmp, f, v̇), x)
FD.hessian(_tmp -> μ'*constraint(model, x, _tmp, v̇), f)
FD.hessian(_tmp -> μ'*constraint(model, x, f, _tmp), v̇)

# Test combined
x_idx = 1:model.nx
f_idx = x_idx[end] .+ (1:kinematics_size(model))
v̇_idx = f_idx[end] .+ (1:model.nv)
FD.hessian(_z -> μ'*constraint(model, _z[x_idx], _z[f_idx], _z[v̇_idx]), [x; f; v̇])

# Harder dummy constraint to test lagrangian hessian
function constraint(model, x, f, v̇)
    return [inverse_dynamics(model, x, v̇) + kinematics_jacobianTvp(model, x, f);
            kinematics(model, x)]
end
μ = ones(length(constraint(model, x, f, v̇)))

# Test seperate hessians
FD.hessian(_tmp -> μ'*constraint(model, _tmp, f, v̇), x)
FD.hessian(_tmp -> μ'*constraint(model, x, _tmp, v̇), f)
FD.hessian(_tmp -> μ'*constraint(model, x, f, _tmp), v̇)

# Test combined
x_idx = 1:model.nx
f_idx = x_idx[end] .+ (1:kinematics_size(model))
v̇_idx = f_idx[end] .+ (1:model.nv)
FD.hessian(_z -> μ'*constraint(model, _z[x_idx], _z[f_idx], _z[v̇_idx]), [x; f; v̇])