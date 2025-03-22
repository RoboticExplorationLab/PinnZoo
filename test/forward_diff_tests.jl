using ForwardDiff
using PinnZoo

function test_forward_diff()
    model = Go1();
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
