using Test
using PinnZoo
using RigidBodyDynamics
using FiniteDiff
using FiniteDifferences
using LinearAlgebra
using Random

function test_default_functions(model::PinnZooModel)
    # Check the model is a supported type
    @assert model.nq == model.nv || (typeof(model) <: PinnZooFloatingBaseModel && model.nq == model.nv + 1)

    Random.seed!(1)
    # Test with initial state
    x = init_state(model)
    test_default_functions(model, x)

    # Test with random state
    x = randn_state(model)
    test_default_functions(model, x)
end

function test_default_functions(model::PinnZooModel, x::Vector{Float64})
    v̇ = randn(model.nv)
    τ = randn(model.nv)
    λ = randn(kinematics_size(model))

    # Make sure all functions can be called without error
    @test_nowarn let 
        # Default dynamics functions
        M_func(model, x)
        C_func(model, x)
        dynamics(model, x, τ)
        forward_dynamics(model, x, τ)
        PinnZoo.inverse_dynamics(model, x, v̇)
        velocity_kinematics(model, x)
        velocity_kinematics_T(model, x)
        velocity_kinematics_jvp_deriv(model, x, x[model.nq + 1:end])
        velocity_kinematics_T_jvp_deriv(model, x, x[1:model.nq])

        # Default kinematics functions
        if length(model.kinematics_bodies) != 0
            PinnZoo.kinematics(model, x)
            kinematics_jacobian(model, x)
            kinematics_velocity(model, x)
            kinematics_velocity_jacobian(model, x)
            kinematics_force_jacobian(model, x, λ)
        end
    end

    # Build RigidBodyDynamics.jl model and necessary helpers for testing
    robot = parse_urdf(model.urdf_path, remove_fixed_tree_joints=false, floating=is_floating(model))
    state = MechanismState(robot)
    dyn_res = DynamicsResult(robot)

    # Generate state order for rigidBodyDynamics
    state = MechanismState(robot)
    config_names =  [Symbol(joints(robot)[id].name) for id in state.q_index_to_joint_id]
    vel_names = [Symbol(joints(robot)[id].name) for id in state.v_index_to_joint_id]
    for joint in joints(robot) # Fix floating base
        if typeof(joint.joint_type) <: QuaternionFloating
            config_names[state.qranges[joint]] = [:q_w, :q_x, :q_y, :q_z, :x, :y, :z]
            vel_names[state.vranges[joint]] = [:ang_v_x, :ang_v_y, :ang_v_z, :lin_v_x, :lin_v_y, :lin_v_z]
        end
    end    
    torque_names = [name for name in vel_names if name in model.orders[:nominal].torque_names]
    model.orders[:rigidBodyDynamics] = StateOrder(config_names, vel_names, torque_names)
    generate_conversions(model.orders, model.conversions)

    # Make rbd versions of random states
    x_rbd = change_order(model, x, :nominal, :rigidBodyDynamics)
    v̇_rbd = change_order(model, v̇, :nominal, :rigidBodyDynamics)
    τ_rbd = change_order(model, τ, :nominal, :rigidBodyDynamics)

    # Helper function to get kinematics (only position supported right now)
    function kinematics(x)
        set_configuration!(state, change_order(model, x[1:model.nq], :nominal, :rigidBodyDynamics))
        if hasproperty(model, :kinematics_ori) && model.kinematics_ori == :Quaternion
            return vcat([
                (frame = relative_transform(state, default_frame(findbody(robot, string(body))), root_frame(robot));
                 q = RigidBodyDynamics.QuatRotation(rotation(frame));
                 [translation(frame); q.w; q.x; q.y; q.z])
                 for body in model.kinematics_bodies]...)
        elseif hasproperty(model, :kinematics_ori) && model.kinematics_ori == :AxisAngle
            return vcat([
                (frame = relative_transform(state, default_frame(findbody(robot, string(body))), root_frame(robot));
                 q = RigidBodyDynamics.QuatRotation(rotation(frame));
                 [translation(frame); quat_to_axis_angle([q.w; q.x; q.y; q.z])])
                 for body in model.kinematics_bodies]...)
        else
            return vcat([
                translation(relative_transform(state, default_frame(findbody(robot, string(body))), root_frame(robot))) 
                for body in model.kinematics_bodies]...)
        end
    end

    # Set configuration and velocity
    set_configuration!(state, x_rbd[1:model.nq])
    set_velocity!(state, x_rbd[model.nq + 1:end])

    # Test mass matrix
    M1 = M_func(model, x)
    M2 = change_order(model, Matrix(mass_matrix(state)), :rigidBodyDynamics, :nominal)
    @test norm(M1 - M2, Inf) < 1e-12

    # Make sure mass matrix is positive definite
    @test isposdef(M1)

    # Test coriolis matrix
    C1 = C_func(model, x)
    C2 = change_order(model, dynamics_bias(state), :rigidBodyDynamics, :nominal)
    @test norm(C1 - C2, Inf) < 1e-12

    # Test forward dynamics 
    v̇1 = forward_dynamics(model, x, τ)
    dynamics!(dyn_res, state, τ_rbd)
    v̇2 = change_order(model, dyn_res.v̇, :rigidBodyDynamics, :nominal)
    @test norm(v̇1 - v̇2, Inf) < 1e-9

    # Test forward dynamics derivatives
    J1 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _x -> forward_dynamics(model, _x, τ), copy(x))[1]
    J2 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _τ -> forward_dynamics(model, x, _τ), copy(τ))[1]
    J3, J4 = forward_dynamics_deriv(model, x, τ)
    @test norm(J1 - J3, Inf) < 5e-5
    @test norm(J2 - J4, Inf) < 1e-8

    # Test dynamics
    ẋ1 = dynamics(model, x, τ)
    ẋ2 = [x[model.nq + 1:end]; forward_dynamics(model, x, τ)]
    @test norm(ẋ1 - ẋ2) < 1e-9

    # Test dynamics derivatives
    J1 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _x -> dynamics(model, _x, τ), copy(x))[1]
    J2 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _τ -> dynamics(model, x, _τ), copy(τ))[1]
    J3, J4 = dynamics_deriv(model, x, τ)
    @test norm(J1 - J3, Inf) < 5e-5
    @test norm(J2 - J4, Inf) < 5e-10

    # Test inverse dynamics
    τ1 = PinnZoo.inverse_dynamics(model, x, v̇);
    dyn_res.v̇[:] = v̇_rbd # inverse_dynamics needs a Segmented_Vector, this is a workaround
    τ2 = change_order(model, RigidBodyDynamics.inverse_dynamics(state, dyn_res.v̇), :rigidBodyDynamics, :nominal)
    @test norm(τ1 - τ2, Inf) < 1e-12

    # Test inverse dynamics derivatives
    J1 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _x -> PinnZoo.inverse_dynamics(model, _x, v̇), copy(x))[1]
    J2 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _v̇ -> PinnZoo.inverse_dynamics(model, x, _v̇), copy(v̇))[1]
    J3, J4 = inverse_dynamics_deriv(model, x, v̇)
    @test norm(J1 - J3, Inf) < 1e-4
    @test norm(J2 - J4, Inf) < 1e-4

    # Test velocity kinematics
    if typeof(model) <: PinnZooFloatingBaseModel
        E = zeros(model.nq, model.nv);
        E[1:3, 1:3] = quat_to_rot(x[4:7])
        E[4:7, 4:6] = 0.5*attitude_jacobian(x[4:7])
        E[8:end, 7:end] = I(model.nq - 7)
        @test norm(velocity_kinematics(model, x) - E) < 1e-12
        E_T = zeros(model.nv, model.nq);
        E_T[1:3, 1:3] = quat_to_rot(x[4:7])'
        E_T[4:6, 4:7] = 2*attitude_jacobian(x[4:7])'
        E_T[7:end, 8:end] = I(model.nq - 7)
        @test norm(velocity_kinematics_T(model, x) - E_T) < 1e-12
    elseif model.nq != model.nv
        @warn "velocity kinematics test is currently unsupported for models with quaternions that are not part of a floating base or continuous joints (nq != nv)"
    else
        @test norm(velocity_kinematics(model, x) - I(model.nq)) < 1e-12
        @test norm(velocity_kinematics_T(model, x) - I(model.nq)) < 1e-12
    end

    # Test velocity kinematics jacobian-vector product derivatives
    test_x = randn_state(model)
    J1 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(2, 1), _x -> velocity_kinematics(model, _x)*test_x[model.nq + 1:end], copy(x))[1]
    J2 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(2, 1), _x -> velocity_kinematics_T(model, _x)*test_x[1:model.nq], copy(x))[1]
    J3 = velocity_kinematics_jvp_deriv(model, x, test_x[model.nq + 1:end])
    J4 = velocity_kinematics_T_jvp_deriv(model, x, test_x[1:model.nq])
    @test norm(J1 - J3, Inf) < 1e-6
    @test norm(J2 - J4, Inf) < 1e-6

    if length(model.kinematics_bodies) == 0
        return
    end

    # Test kinematics
    locs1 = PinnZoo.kinematics(model, x)
    locs2 = kinematics(x)
    @test norm(locs1 - locs2, Inf) < 1e-12

    # Test kinematics jacobian
    J1 = kinematics_jacobian(model, x)
    J2 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(2, 1), _x -> PinnZoo.kinematics(model, _x), x)[1]
    @test norm(J1 - J2, Inf) < 1e-6

    # Test kinematics velocity
    locs_dot1 = kinematics_velocity(model, x)
    locs_dot2 = kinematics_jacobian(model, x)[:, 1:model.nq]*velocity_kinematics(model, x)*x[model.nq + 1:end]
    @test norm(locs_dot1 - locs_dot2) < 1e-12

    # Test kinematics velocity jacobian
    J_dot1 = kinematics_velocity_jacobian(model, x)
    J_dot2 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), 
        _x -> kinematics_jacobian(model, _x)[:, 1:model.nq]*velocity_kinematics(model, _x)*_x[model.nq + 1:end], copy(x))[1]
    @test norm(J_dot1 - J_dot2) < 2e-6     
    
    # Test kinematics force jacobian
    J_dot1 = kinematics_force_jacobian(model, x, λ)
    J_dot2 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(2, 1),  _x -> kinematics_velocity_jacobian(model, _x)[:, model.nq + 1:end]'*λ, copy(x))[1]
    @test norm(J_dot1 - J_dot2) < 5e-5 

    # If this is a floating base model, check apply_Δx, state_error and error_jacobains
    if typeof(model) <: PinnZooFloatingBaseModel
        Δx = randn(model.nv*2)
        x2 = x + randn(model.nx)
        x2[4:7] = normalize(x2[4:7])

        @test norm(state_error(model, apply_Δx(model, x, Δx), x) - Δx, Inf) < 1e-12
        @test norm(apply_Δx(model, x, state_error(model, x2, x)) - x2, Inf) < 1e-12

        E2 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _Δx -> apply_Δx(model, x, _Δx), zeros(model.nv*2))[1]
        E_T2 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _x -> state_error(model, _x, x), copy(x))[1]

        @test norm(error_jacobian(model, x) - E2, Inf) < 1e-7
        @test norm(error_jacobian_T(model, x) - E_T2, Inf) < 1e-7
    end
end

