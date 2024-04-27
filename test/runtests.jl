using Test
using PinnZoo
using RigidBodyDynamics
using FiniteDiff
using LinearAlgebra
using Random

include(joinpath(@__DIR__, "default_func_tests.jl"))

# Cartpole
@testset "Cartpole" begin
    test_default_functions(Cartpole())
end

# Go1
@testset "Unitree Go1" begin
    test_default_functions(Go1())
end

# Go2
@testset "Unitree Go2" begin
    test_default_functions(Go2())
end