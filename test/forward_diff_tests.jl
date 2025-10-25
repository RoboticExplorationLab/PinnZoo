using ForwardDiff
using FiniteDifferences
using PinnZoo

function test_forward_diff()
    model = Nadia(kinematics_ori = :Quaternion);
    x = randn_state(model);
    u = randn(model.nv);
    
    # Test kinematics
    J1 = ForwardDiff.jacobian(_x -> kinematics(model, _x), x)
    J2 = kinematics_jacobian(model, x)
    @test J1 == J2 # This should be exact

    # Test kinematics_velocity
    J1 = ForwardDiff.jacobian(_x -> kinematics_velocity(model, _x), x)
    J2 = kinematics_velocity_jacobian(model, x)
    @test J1 == J2 # This should be exact

    # Test kinematics rotation if applicable
    if hasproperty(model, :kinematics_ori) && !isnothing(model.kinematics_ori) && model.kinematics_ori != :None
        J1 = ForwardDiff.jacobian(_x -> kinematics_rotation(model, _x), x)
        J2 = ForwardDiff.jacobian(_locs -> PinnZoo._kinematics_rotation(model, x, _locs), kinematics(model, x))
        J2 = J2*kinematics_jacobian(model, x)
        J3 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _x -> kinematics_rotation(model, _x), x)[1]
        @test norm(J1 - J2, Inf) < 1e-12
        @test norm(J2 - J3, Inf) < 1e-7 # Make sure chain rule is correct
    end

    # Test forward_dynamics
    J_x, J_u = forward_dynamics_deriv(model, x, u)
    J = ForwardDiff.jacobian(_x -> forward_dynamics(model, _x, u), x)
    @test J == J_x # This should be exact
    J = ForwardDiff.jacobian(_u -> forward_dynamics(model, x, _u), u)
    @test J == J_u # This should be exact
    J = ForwardDiff.jacobian(_y -> forward_dynamics(model, _y[1:model.nx], _y[model.nx + 1:end]), [x; u])
    @test J == [J_x J_u] # This should be exact

    # Test dynamics
    J_x, J_u = dynamics_deriv(model, x, u)
    J = ForwardDiff.jacobian(_x -> dynamics(model, _x, u), x)
    @test J == J_x # This should be exact
    J = ForwardDiff.jacobian(_u -> dynamics(model, x, _u), u)
    @test J == J_u # This should be exact
    J = ForwardDiff.jacobian(_y -> dynamics(model, _y[1:model.nx], _y[model.nx + 1:end]), [x; u])
    @test J == [J_x J_u] # This should be exact

    # Test inverse_dynamics
    J_x, J_u = inverse_dynamics_deriv(model, x, u)
    J = ForwardDiff.jacobian(_x -> PinnZoo.inverse_dynamics(model, _x, u), x)
    @test J == J_x # This should be exact
    J = ForwardDiff.jacobian(_u -> PinnZoo.inverse_dynamics(model, x, _u), u)
    @test J == J_u # This should be exact
    J = ForwardDiff.jacobian(_y -> PinnZoo.inverse_dynamics(model, _y[1:model.nx], _y[model.nx + 1:end]), [x; u])
    @test J == [J_x J_u] # This should be exact
end
