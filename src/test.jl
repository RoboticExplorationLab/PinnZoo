using Pkg; Pkg.activate(joinpath(@__DIR__, ".."))
using RigidBodyDynamics
using Libdl
using LinearAlgebra

lib = dlopen(joinpath(@__DIR__, "build/libdynamics.so"))
M_func = dlsym(lib, :M_func_wrapper)
robot = parse_urdf(joinpath(@__DIR__, "../cartpole.urdf"))

state = MechanismState(robot)
x = [configuration(state); velocity(state)]
display(x)
M1 = zeros(2, 2)
ccall(M_func, Cvoid, (Ptr{Cdouble}, Ref{Cdouble}), x, M1)
M2 = mass_matrix(state)
display(M1)
display(M2)
display(norm(M1 - M2, Inf))
