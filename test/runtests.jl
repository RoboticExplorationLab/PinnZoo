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

# Nadia
@testset "IHMC Nadia" begin
    test_default_functions(Nadia())
    test_default_functions(Nadia(nc_per_foot = 4))
    # test_default_functions(Nadia(kinematics_ori = true))
    @test_throws "specified configuration is not supported" Nadia(nc_per_foot = 4, kinematics_ori = true)
end