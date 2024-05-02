
function test_default_functions(model::PinnZooModel)
    # Make random state to test with
    Random.seed!(1)
    x = randn_state(model)
    v̇ = randn(model.nv)
    τ = randn(model.nv)

    # Make sure all functions can be called without error
    @test_nowarn let 
        # Default dynamics functions
        M_func(model, x)
        C_func(model, x)
        forward_dynamics(model, x, τ)
        PinnZoo.inverse_dynamics(model, x, v̇)
        velocity_kinematics(model, x)
        velocity_kinematics_T(model, x)

        # Default kinematics functions
        PinnZoo.kinematics(model, x)
        kinematics_jacobian(model, x)
        kinematics_velocity(model, x)
        kinematics_velocity_jacobian(model, x)
    end

    if model.nq != model.nv
        @warn "RigidBodyDynamics.jl tests do not support floating base joints yet"
        return
    end

    # Build RigidBodyDynamics.jl model and necessary helpers for testing
    robot = parse_urdf(model.urdf_path, remove_fixed_tree_joints=false, floating=is_floating(model))
    state = MechanismState(robot)
    dyn_res = DynamicsResult(robot)
    function kinematics(x)
        set_configuration!(state, x[1:model.nq])
        return vcat([
            translation(relative_transform(state, default_frame(findbody(robot, body)), root_frame(robot))) 
            for body in model.kinematics_bodies]...)
    end
    set_configuration!(state, x[1:model.nq])
    set_velocity!(state, x[model.nq + 1:end])

    # Test mass matrix
    M1 = M_func(model, x)
    M2 = mass_matrix(state)
    @test norm(M1 - M2, Inf) < 1e-12

    # Test coriolis matrix
    C1 = C_func(model, x)
    C2 = dynamics_bias(state)
    @test norm(C1 - C2, Inf) < 1e-12

    # Test forward dynamics 
    v̇1 = forward_dynamics(model, x, τ)
    dynamics!(dyn_res, state, τ)
    v̇2 = dyn_res.v̇
    @test norm(v̇1 - v̇2, Inf) < 1e-12

    # Test forward dynamics derivatives
    J1 = FiniteDiff.finite_difference_jacobian(_x -> forward_dynamics(model, _x, τ), x)
    J2 = FiniteDiff.finite_difference_jacobian(_τ -> forward_dynamics(model, x, _τ), τ)
    J3, J4 = forward_dynamics_deriv(model, x, τ)
    @test norm(J1 - J3, Inf) < 1e-6
    @test norm(J2 - J4, Inf) < 1e-6

    # Test inverse dynamics
    τ1 = PinnZoo.inverse_dynamics(model, x, v̇);
    dyn_res.v̇[:] = v̇ # inverse_dynamics needs a Segmented_Vector, this is a workaround
    τ2 = RigidBodyDynamics.inverse_dynamics(state, dyn_res.v̇)
    @test norm(τ1 - τ2, Inf) < 1e-12

    # Test inverse dynamics derivatives
    J1 = FiniteDiff.finite_difference_jacobian(_x -> PinnZoo.inverse_dynamics(model, _x, v̇), x)
    J2 = FiniteDiff.finite_difference_jacobian(_v̇ -> PinnZoo.inverse_dynamics(model, x, _v̇), v̇)
    J3, J4 = inverse_dynamics_deriv(model, x, v̇)
    @test norm(J1 - J3, Inf) < 1e-6
    @test norm(J2 - J4, Inf) < 1e-6

    # Test velocity kinematics (TODO fix for q̇ != v)
    @test norm(velocity_kinematics(model, x) - I(model.nq)) < 1e-12
    @test norm(velocity_kinematics_T(model, x) - I(model.nq)) < 1e-12

    # Test kinematics
    locs1 = PinnZoo.kinematics(model, x)
    locs2 = kinematics(x)
    @test norm(locs1 - locs2, Inf) < 1e-12

    # Test kinematics jacobian
    J1 = kinematics_jacobian(model, x)
    J2 = FiniteDiff.finite_difference_jacobian(kinematics, x)
    @test norm(J1 - J2, Inf) < 1e-6

    # Test kinematics velocity (TODO fix for q̇ != v)
    locs_dot1 = kinematics_velocity(model, x)
    locs_dot2 = kinematics_jacobian(model, x)[:, 1:model.nq]*x[model.nq + 1:end]
    @test norm(locs_dot1 - locs_dot2) < 1e-12

    # Test kinematics velocity jacobian (TODO fix for q̇ != v)
    J_dot1 = kinematics_velocity_jacobian(model, x)
    J_dot2 = FiniteDiff.finite_difference_jacobian(
        _x -> kinematics_jacobian(model, _x)[:, 1:model.nq]*_x[model.nq + 1:end], x)
    @test norm(J_dot1 - J_dot2) < 1e-6
end

