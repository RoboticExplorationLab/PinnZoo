using Test
using PinnZoo
using RigidBodyDynamics
using FiniteDiff
using LinearAlgebra
using Random

include(joinpath(@__DIR__, "default_func_tests.jl"))

# Run default tests
test_default_functions(Cartpole())