using Test
using PinnZoo
using RigidBodyDynamics
using FiniteDiff
using LinearAlgebra
using Random

include(joinpath(@__DIR__, "default_func_tests.jl"))
include(joinpath(@__DIR__, "forward_diff_tests.jl"))

@testset verbose=true "PinnZoo" begin
    # Pendulum
    @testset "Pendulum" begin
        test_default_functions(Pendulum())
    end

    # Double Pendulum
    @testset "Double Pendulum" begin
        test_default_functions(DoublePendulum())
    end

    # Cartpole
    @testset "Cartpole" begin
        test_default_functions(Cartpole())
    end

     # Double Cartpole
     @testset "Double Cartpole" begin
        test_default_functions(DoubleCartpole())
    end

    # Rigidbody
    @testset "RigidBody" begin
        test_default_functions(PinnZoo.RigidBody())
    end

    # Quadrotor
    @testset "Quadrotor" begin
        test_default_functions(Quadrotor())
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
    @testset verbose=true "IHMC Nadia" begin
        @testset "default" test_default_functions(Nadia())
        @testset "nc=4" test_default_functions(Nadia(nc_per_foot = 4))
        @testset "nc=4 fixed_arms" test_default_functions(Nadia(nc_per_foot = 4, fixed_arms = true))
        @testset "quat" test_default_functions(Nadia(kinematics_ori = :Quaternion))
        @testset "aa" test_default_functions(Nadia(kinematics_ori = :AxisAngle))
        @testset "aa fixed_arms" test_default_functions(Nadia(kinematics_ori = :AxisAngle, fixed_arms = true))
        @testset "aa fixed_arms fixed_spine" test_default_functions(Nadia(kinematics_ori = :AxisAngle, fixed_arms = true, fixed_spine = true))
        @test_throws "specified configuration is not supported" Nadia(nc_per_foot = 4, kinematics_ori = :Quaternion)
    end

    # Pineapple
    @testset verbose=true "Pineapple" begin
        @testset "v0 default" test_default_functions(Pineapple(version=:v0))
        @testset "v0 kin_ori=none" test_default_functions(Pineapple(kinematics_ori=:None, version=:v0))
        @testset "v1 default" test_default_functions(Pineapple())
        @testset "v1 kin_ori=none" test_default_functions(Pineapple(kinematics_ori=:None))
        @testset "v2 default" test_default_functions(Pineapple(version=:v2))
        @testset "v2 kin_ori=none" test_default_functions(Pineapple(kinematics_ori=:None, version=:v2))
        @testset "v3 default" test_default_functions(Pineapple(version=:v3))
        @testset "v3_gripper default" test_default_functions(Pineapple(version=:v3_gripper))
    end

    # ForwardDiff compatability
    @testset "ForwardDiff" begin
        test_forward_diff();
    end
end
