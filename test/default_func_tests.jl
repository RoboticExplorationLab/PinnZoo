
function test_default_functions(model::PinnZooModel)
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

    # Make random state to test with
    Random.seed!(1)
    x = randn(model.nx)
    v̇ = randn(model.nv)
    τ = randn(model.nv)
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

    # Test inverse dynamics
    τ1 = PinnZoo.inverse_dynamics(model, x, v̇);
    dyn_res.v̇[:] = v̇ # inverse_dynamics needs a Segmented_Vector, this is a workaround
    τ2 = RigidBodyDynamics.inverse_dynamics(state, dyn_res.v̇)
    @test norm(τ1 - τ2, Inf) < 1e-12

    # Test kinematics
    locs1 = PinnZoo.kinematics(model, x)
    locs2 = kinematics(x)
    @test norm(locs1 - locs2, Inf) < 1e-12

    # Test kinematics jacobian
    J1 = kinematics_jacobian(model, x)
    J2 = FiniteDiff.finite_difference_jacobian(kinematics, x)
    @test norm(J1 - J2, Inf) < 1e-6
end

