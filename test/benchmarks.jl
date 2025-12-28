using Revise
using PinnZoo
using BenchmarkTools
using Printf
using Statistics

model = Go2()

x = randn_state(model)
v = randn(model.nv)

# Benchmark dynamics
benchmarks = [
    (:M_func,                 () -> M_func(model, x)),
    (:M_jvp,                  () -> M_jvp(model, x, x)),
    (:C_func,                 () -> C_func(model, x)),
    (:dynamics,               () -> dynamics(model, x, v)),
    (:dynamics_deriv,         () -> dynamics_deriv(model, x, v)),
    (:forward_dynamics,       () -> forward_dynamics(model, x, v)),
    (:forward_dynamics_deriv, () -> forward_dynamics_deriv(model, x, v)),
    (:inverse_dynamics,       () -> inverse_dynamics(model, x, v)),
    (:inverse_dynamics_deriv, () -> inverse_dynamics_deriv(model, x, v)),
    (:inverse_dynamics_hvp,   () -> inverse_dynamics_hvp(model, x, v, x)),
    (:inverse_dynamics_hTvp,  () -> inverse_dynamics_hTvp(model, x, v, v)),
]

println("| Function | Median | Q1 – Q3 |")
println("|----------|--------|-----------|")
for (name, f) in benchmarks
    t = @benchmark($f()).times
    q = quantile(t, (0.25, 0.5, 0.75))
    unit, q = q[2] ≥ 1e5 ? ("ms", q./1e6) : q[2] ≥ 1e2 ? ("μs", q./1e3) : ("ns", q)
    @printf("| [`%s`](@ref) | %.3g %s | (%.3g–%.3g) %s |\n", String(name), q[2], unit, q[1], q[3], unit)
end

# Benchmark kinematics
λ = randn(kinematics_size(model))
benchmarks = [
    (:kinematics,                     () -> kinematics(model, x)),
    # (:kinematics_rotation,            () -> kinematics_rotation(model, x)),
    (:kinematics_jacobian,            () -> kinematics_jacobian(model, x)),
    (:kinematics_hvp,                 () -> kinematics_hvp(model, x, x)),
    (:kinematics_velocity,            () -> kinematics_velocity(model, x)),
    (:kinematics_velocity_jacobian,   () -> kinematics_velocity_jacobian(model, x)),
    (:kinematics_force_jacobian,      () -> kinematics_force_jacobian(model, x, λ)),
    (:kinematics_jacobianTvp,         () -> kinematics_jacobianTvp(model, x, λ)),
    (:kinematics_force_hvp,           () -> kinematics_force_hvp(model, x, λ, x)),
    (:kinematics_force_hTvp,          () -> kinematics_force_hTvp(model, x, λ, x)),
]

println("| Function | Median | Q1 – Q3 |")
println("|----------|--------|-----------|")
for (name, f) in benchmarks
    t = @benchmark($f()).times
    q = quantile(t, (0.25, 0.5, 0.75))
    unit, q = q[2] ≥ 1e5 ? ("ms", q./1e6) : q[2] ≥ 1e2 ? ("μs", q./1e3) : ("ns", q)
    @printf("| [`%s`](@ref) | %.3g %s | (%.3g–%.3g) %s |\n", String(name), q[2], unit, q[1], q[3], unit)
end

# Benchmark ForwardDiff jacobians
using ForwardDiff
benchmarks = [
    (:kinematics,               () -> ForwardDiff.jacobian(_x -> kinematics(model, _x), x), :x),
    (:kinematics_velocity,      () -> ForwardDiff.jacobian(_x -> kinematics_velocity(model, _x), x), :x),
    (:kinematics_jacobianTvp,   () -> ForwardDiff.jacobian(_x -> kinematics_jacobianTvp(model, _x, λ), x), :x),
    (:kinematics_jacobianTvp,   () -> ForwardDiff.jacobian(_λ -> kinematics_jacobianTvp(model, x, _λ), λ), :λ),
    (:dynamics,                 () -> ForwardDiff.jacobian(_x -> dynamics(model, _x, v), x), :x),
    (:dynamics,                 () -> ForwardDiff.jacobian(_v -> dynamics(model, x, _v), v), :τ),
    (:forward_dynamics,         () -> ForwardDiff.jacobian(_x -> forward_dynamics(model, _x, v), x), :x),
    (:forward_dynamics,         () -> ForwardDiff.jacobian(_v̇ -> forward_dynamics(model, x, _v̇), v), :v̇),
    (:inverse_dynamics,         () -> ForwardDiff.jacobian(_x -> inverse_dynamics(model, _x, v), x), :x),
    (:inverse_dynamics,         () -> ForwardDiff.jacobian(_τ -> inverse_dynamics(model, x, _τ), v), :τ),
]

println("| Function | w.r.t | Median | Q1 – Q3 |")
println("|----------|----|--------|-----------|")
for (name, f, wrt) in benchmarks
    t = @benchmark($f()).times
    q = quantile(t, (0.25, 0.5, 0.75))
    unit, q = q[2] ≥ 1e5 ? ("ms", q./1e6) : q[2] ≥ 1e2 ? ("μs", q./1e3) : ("ns", q)
    @printf("| [`%s`](@ref) | %s | %.3g %s | (%.3g–%.3g) %s |\n", String(name), String(wrt), q[2], unit, q[1], q[3], unit)
end

# Benchmark ForwardDiff hessians
benchmarks = [
    (:kinematics,               () -> ForwardDiff.hessian(_x -> λ'*kinematics(model, _x), x), :x),
    (:kinematics_jacobianTvp,   () -> ForwardDiff.hessian(_x -> v'*kinematics_jacobianTvp(model, _x, λ), x), :x),
    (:kinematics_jacobianTvp,   () -> ForwardDiff.hessian(_λ -> v'*kinematics_jacobianTvp(model, x, _λ), λ), :λ),
    (:inverse_dynamics,         () -> ForwardDiff.hessian(_x -> v'*inverse_dynamics(model, _x, v), x), :x),
    (:inverse_dynamics,         () -> ForwardDiff.hessian(_τ -> v'*inverse_dynamics(model, x, _τ), v), :τ),
]

println("| Function | w.r.t | Median | Q1 – Q3 |")
println("|----------|----|--------|-----------|")
for (name, f, wrt) in benchmarks
    t = @benchmark($f()).times
    q = quantile(t, (0.25, 0.5, 0.75))
    unit, q = q[2] ≥ 1e5 ? ("ms", q./1e6) : q[2] ≥ 1e2 ? ("μs", q./1e3) : ("ns", q)
    @printf("| [`%s`](@ref) | %s | %.3g %s | (%.3g–%.3g) %s |\n", String(name), String(wrt), q[2], unit, q[1], q[3], unit)
end

func = _x -> v'*inverse_dynamics(model, _x, v)
@profview ForwardDiff.hessian(func, x)

