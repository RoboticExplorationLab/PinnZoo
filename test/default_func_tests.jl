using Test
using PinnZoo
using RigidBodyDynamics
using FiniteDiff
using FiniteDifferences
using LinearAlgebra
using Random
import ForwardDiff as FD
using StaticArrays

function test_default_functions(model::PinnZooModel)
    # Check the model is a supported type
    @assert model.nq == model.nv || (typeof(model) <: PinnZooFloatingBaseModel && model.nq == model.nv + 1)

    Random.seed!(1)
    # Test with initial state
    x = init_state(model)
    test_default_functions(model, x)

    # Test with random state
    for _ in 1:10
        x = randn_state(model)
        test_default_functions(model, x)
    end
end
# model = Pineapple(version=:v1)
# x = init_state(model)
# x = [0.0, 0.0, 0.24999999998590466, 0.9999998721986809, 8.14276208556384e-8, 0.0005055714519148044, -1.0655900172952339e-7, 1.2986234043060987e-5, 0.8303450607969001, -1.4965402647260369, -2.0809621854359284e-12, 1.2986316712952231e-5, 0.8303556396534936, -1.4965604026502244, -2.0767343402642347e-12, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

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
            if hasproperty(model, :kinematics_ori) && !isnothing(model.kinematics_ori) && model.kinematics_ori != :None
                PinnZoo.kinematics_rotation(model, x)
            end
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

    function rbd_state(model, x)
        state = MechanismState{eltype(x)}(robot)
        x_rbd = change_order(model, x, :nominal, :rigidBodyDynamics)
        set_configuration!(state, x_rbd[1:model.nq])
        set_velocity!(state, x_rbd[model.nq + 1:end])
        return state
    end

    # Make rbd versions of random states
    x_rbd = change_order(model, x, :nominal, :rigidBodyDynamics)
    v̇_rbd = change_order(model, v̇, :nominal, :rigidBodyDynamics)
    τ_rbd = change_order(model, τ, :nominal, :rigidBodyDynamics)

    # Set configuration and velocity
    set_configuration!(state, x_rbd[1:model.nq])
    set_velocity!(state, x_rbd[model.nq + 1:end])

    # Test mass matrix
    M1 = M_func(model, x)
    M2 = change_order(model, Matrix(mass_matrix(state)), :rigidBodyDynamics, :nominal)
    @test norm(M1 - M2, Inf) < 1e-10

    # Make sure mass matrix is positive definite
    @test isposdef(M1)

    # Test mass matrix jacobian-vector product d/dx(Mv̇)
    M1 = M_jvp(model, x, v̇)
    M2 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _x -> M_func(model, _x)*v̇, copy(x))[1]
    @test norm(M1 - M2, Inf) < 1e-10

    # Test coriolis matrix
    C1 = C_func(model, x)
    C2 = change_order(model, dynamics_bias(state), :rigidBodyDynamics, :nominal)
    @test norm(C1 - C2, Inf) < 1e-10

    # Test forward dynamics 
    v̇1 = forward_dynamics(model, x, τ)
    dynamics!(dyn_res, state, τ_rbd)
    v̇2 = change_order(model, dyn_res.v̇, :rigidBodyDynamics, :nominal)
    @test norm(v̇1 - v̇2, Inf) < 1e-7

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
    @test norm(τ1 - τ2, Inf) < 1e-10

    # Test inverse dynamics derivatives
    J1 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _x -> PinnZoo.inverse_dynamics(model, _x, v̇), copy(x))[1]
    J2 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _v̇ -> PinnZoo.inverse_dynamics(model, x, _v̇), copy(v̇))[1]
    J3, J4 = inverse_dynamics_deriv(model, x, v̇)
    @test norm(J1 - J3, Inf) < 1e-4
    @test norm(J2 - J4, Inf) < 1e-4

    # Test derivatives of inverse dynamics second derivative product
    mult = randn(model.nx)
    J1 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _x -> inverse_dynamics_deriv(model, _x, v̇)[1]*mult, copy(x))[1]
    J2 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _v̇ -> inverse_dynamics_deriv(model, x, _v̇)[1]*mult, copy(v̇))[1]
    J3, J4 = inverse_dynamics_hvp(model, x, v̇, mult)
    @test norm(J1 - J3, Inf) < 1e-4
    @test norm(J2 - J4, Inf) < 1e-4

    # Test derivatives of inverse dynamics second derivative jacobian product
    mult = randn(model.nv)
    J1 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _x -> inverse_dynamics_deriv(model, _x, v̇)[1]'*mult, copy(x))[1]
    J2 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _x -> inverse_dynamics_deriv(model, _x, v̇)[2]'*mult, copy(x))[1]
    J3, J4 = inverse_dynamics_hTvp(model, x, v̇, mult)
    @test norm(J1 - J3, Inf) < 1e-4
    @test norm(J2 - J4, Inf) < 1e-4

    # Test velocity kinematics
    if typeof(model) <: PinnZooFloatingBaseModel
        E = zeros(model.nq, model.nv);
        E[1:3, 1:3] = quat_to_rot(x[4:7])
        E[4:7, 4:6] = 0.5*attitude_jacobian(x[4:7])
        E[8:end, 7:end] = I(model.nq - 7)
        @test norm(velocity_kinematics(model, x) - E) < 1e-10
        E_T = zeros(model.nv, model.nq);
        E_T[1:3, 1:3] = quat_to_rot(x[4:7])'
        E_T[4:6, 4:7] = 2*attitude_jacobian(x[4:7])'
        E_T[7:end, 8:end] = I(model.nq - 7)
        @test norm(velocity_kinematics_T(model, x) - E_T) < 1e-10
    elseif model.nq != model.nv
        @warn "velocity kinematics test is currently unsupported for models with quaternions that are not part of a floating base or continuous joints (nq != nv)"
    else
        @test norm(velocity_kinematics(model, x) - I(model.nq)) < 1e-10
        @test norm(velocity_kinematics_T(model, x) - I(model.nq)) < 1e-10
    end

    # Test velocity kinematics jacobian-vector product derivatives
    test_x = randn_state(model)
    J1 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _x -> velocity_kinematics(model, _x)*test_x[model.nq + 1:end], copy(x))[1]
    J2 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _x -> velocity_kinematics_T(model, _x)*test_x[1:model.nq], copy(x))[1]
    J3 = velocity_kinematics_jvp_deriv(model, x, test_x[model.nq + 1:end])
    J4 = velocity_kinematics_T_jvp_deriv(model, x, test_x[1:model.nq])
    @test norm(J1 - J3, Inf) < 1e-6
    @test norm(J2 - J4, Inf) < 1e-6

    if length(model.kinematics_bodies) == 0
        return
    end

    # Helper function to get kinematics
    function kinematics(x)
        state = rbd_state(model, x)

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

    # Helper function to get kinematics rotations
    function kinematics_rotation(x)
        state = rbd_state(model, x)
        set_configuration!(state, change_order(model, x[1:model.nq], :nominal, :rigidBodyDynamics))  

        return vcat([
            (frame = relative_transform(state, default_frame(findbody(robot, string(body))), root_frame(robot));
             q = RigidBodyDynamics.QuatRotation(rotation(frame));
             quat_to_rot([q.w; q.x; q.y; q.z]))
             for body in model.kinematics_bodies]...)  # For all bodies
    end

    # Test kinematics
    locs1 = PinnZoo.kinematics(model, x)
    locs2 = kinematics(x)
    @test norm(locs1 - locs2, Inf) < 1e-10

    # Test kinematics jacobian
    J1 = kinematics_jacobian(model, x)
    J2 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _x -> PinnZoo.kinematics(model, _x), x)[1]
    @test norm(J1 - J2, Inf) < 1e-6

    # Test kinematics hessian-vector product
    dx = randn(model.nx)
    J1 = kinematics_hvp(model, x, dx)
    J2 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _x -> kinematics_jacobian(model, _x)*dx, x)[1] 
    @test norm(J1 - J2, Inf) < 1e-6

    # Test kinematics velocity
    locs_dot1 = kinematics_velocity(model, x)
    locs_dot2 = kinematics_jacobian(model, x)[:, 1:model.nq]*velocity_kinematics(model, x)*x[model.nq + 1:end]
    @test norm(locs_dot1 - locs_dot2) < 1e-10

    # Test kinematics velocity jacobian
    J_dot1 = kinematics_velocity_jacobian(model, x)
    J_dot2 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), 
        _x -> kinematics_jacobian(model, _x)[:, 1:model.nq]*velocity_kinematics(model, _x)*_x[model.nq + 1:end], copy(x))[1]
    @test norm(J_dot1 - J_dot2) < 2e-6     
    
    # Test kinematics force jacobian
    J_dot1 = kinematics_force_jacobian(model, x, λ)
    J_dot2 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1),  _x -> kinematics_velocity_jacobian(model, _x)[:, model.nq + 1:end]'*λ, copy(x))[1]
    @test norm(J_dot1 - J_dot2) < 5e-5 

    # Test kinematics force hvp and hTvp
    if hasproperty(model, :kinematics_ori) && model.kinematics_ori == :AxisAngle # Not supported, has NaNs
    else
        mult = randn(model.nx)
        J1 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _x -> kinematics_force_jacobian(model, _x, λ)*mult, x)[1]
        J2 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _λ -> kinematics_force_jacobian(model, x, _λ)*mult, λ)[1]
        J3, J4 = kinematics_force_hvp(model, x, λ, mult)
        @test norm(J1 - J3, Inf) < 1e-6
        @test norm(J2 - J4, Inf) < 1e-6

        mult = randn(model.nv)
        J1 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _x -> kinematics_force_jacobian(model, _x, λ)'*mult, x)[1]
        J2 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _λ -> kinematics_force_jacobian(model, x, _λ)'*mult, λ)[1]
        J3, J4 = kinematics_force_hTvp(model, x, λ, mult)
        @test norm(J1 - J3, Inf) < 1e-6
        @test norm(J2 - J4, Inf) < 1e-6
    end

    # Test kinematics rotation
    if hasproperty(model, :kinematics_ori) && !isnothing(model.kinematics_ori) && model.kinematics_ori != :None
        rots1 = PinnZoo.kinematics_rotation(model, x)
        rots2 = kinematics_rotation(x)
        @test norm(rots1 - rots2, Inf) < 1e-10
    end

    # Test kinematics for points on rigidbody (only for kinematics_ori = :Quaternion for now)
    if hasproperty(model, :kinematics_ori) && model.kinematics_ori == :Quaternion
        function kinematics(x, id, point)
            state = rbd_state(model, x)
            body_frame = default_frame(findbody(robot, string(model.kinematics_bodies[id])))
            p_body = Point3D(body_frame, SVector{3}(point))
            p_world = transform(state, p_body, root_frame(robot))
            return p_world.v
        end
        # Test kinematics
        point = randn(3)
        locs1 = PinnZoo.kinematics(model, x, 1, point)
        locs2 = kinematics(x, 1, point)
        @test norm(locs1[1:3] - locs2, Inf) < 1e-10

        # Test kinematics velocity
        v1 = PinnZoo.kinematics_velocity(model, x, 1, point)
        v2 = FD.jacobian(_x -> kinematics(_x, 1, point), x)[:, 1:model.nq]*error_jacobian(model, x[1:model.nq])*x[model.nq + 1:end]
        v3 = PinnZoo.kinematics_velocity_analytical(model, x, 1, point)
        @test norm(v1[1:3] - v2, Inf) < 1e-10
        @test norm(v1 - v3, Inf) < 1e-10

        # Test kinematics jacobian
        J1 = kinematics_jacobian(model, x, 1, point)
        J2 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _x -> PinnZoo.kinematics(model, _x, 1, point), copy(x))[1]
        @test norm(J1 - J2, Inf) < 1e-6

        # Test kinematics velocity jacobian
        J_dot1 = kinematics_velocity_jacobian(model, x,  1, point)
        J_dot2 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _x -> PinnZoo.kinematics_velocity(model, _x, 1, point), copy(x))[1]
        J_dot3 = PinnZoo.kinematics_velocity_jacobian_analytical(model, x, 1, point)
        @test norm(J_dot1 - J_dot2) < 2e-6  
        @test norm(J_dot1 - J_dot3) < 2e-6  

        # Quat block of the per-point functions must match the body's quat block
        @test PinnZoo.kinematics(model, x, 1, point)[4:7] ==
              PinnZoo.kinematics(model, x)[(1-1)*7 .+ (4:7)]

        @test PinnZoo.kinematics_velocity(model, x, 1, point)[4:7] ==
              PinnZoo.kinematics_velocity(model, x)[(1-1)*7 .+ (4:7)]

        @test PinnZoo.kinematics_jacobian(model, x, 1, point)[4:7, :] ==
              PinnZoo.kinematics_jacobian(model, x)[(1-1)*7 .+ (4:7), :]

        @test PinnZoo.kinematics_velocity_jacobian(model, x, 1, point)[4:7, :] ==
              PinnZoo.kinematics_velocity_jacobian(model, x)[(1-1)*7 .+ (4:7), :]

        λ_p = randn(7) # 7 entries per point (3 force + 4 quat-force)
        JTλ_1 = PinnZoo.kinematics_jacobianTvp(model, x, 1, point, λ_p)
        JTλ_2 = PinnZoo.kinematics_velocity_jacobian(model, x, 1, point)[:, model.nq + 1:end]' * λ_p
        @test norm(JTλ_1 - JTλ_2, Inf) < 1e-10

        # Differentiability check: ForwardDiff matches FiniteDifferences
        dx_truth = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1),
            _x -> PinnZoo.kinematics_jacobianTvp(model, _x, 1, point, λ_p), copy(x))[1]
        dx_fwd = FD.jacobian(_x -> PinnZoo.kinematics_jacobianTvp(model, _x, 1, point, λ_p), x)
        @test norm(dx_truth - dx_fwd, Inf) < 2e-6
    end

    # If this is a floating base model, check apply_Δx, state_error and error_jacobains
    if typeof(model) <: PinnZooFloatingBaseModel
        Δx = randn(model.nv*2)
        x2 = x + randn(model.nx)
        x2[4:7] = normalize(x2[4:7])

        @test norm(state_error(model, apply_Δx(model, x, Δx), x) - Δx, Inf) < 1e-10
        @test norm(apply_Δx(model, x, state_error(model, x2, x)) - x2, Inf) < 1e-10

        E2 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _Δx -> apply_Δx(model, x, _Δx), zeros(model.nv*2))[1]
        E_T2 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _x -> state_error(model, _x, x), copy(x))[1]

        @test norm(error_jacobian(model, x) - E2, Inf) < 1e-7
        @test norm(error_jacobian_T(model, x) - E_T2, Inf) < 1e-7
        
        # Test the derivatives of the jacobian vector products
        J1 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _x -> error_jacobian(model, _x)*Δx, copy(x))[1]
        J2 = error_jacobian_jvp_deriv(model, x, Δx)
        @test norm(J1 - J2, Inf) < 1e-6
        J1 = FiniteDifferences.jacobian(FiniteDifferences.central_fdm(5, 1), _x -> error_jacobian_T(model, _x)*x2, copy(x))[1]
        J2 = error_jacobian_T_jvp_deriv(model, x, x2)
        @test norm(J1 - J2, Inf) < 1e-6
    end

    # ── Pineapple-specific wheel helpers ─────────────────────────────────
    # Guard on :Quaternion: get_wheel_contour calls kinematics_rotation,
    # which is only supported for kinematics_ori = :Quaternion / :AxisAngle
    if model isa Pineapple && model.kinematics_ori == :Quaternion
        nc = 2
        normals = [[0,0,1.0]; [0,0,1.0]]
        pos_inds = vcat([(i-1)*7 .+ (1:3) for i in 1:nc]...)

        # Resolve contact points the same way wheel_* does
        r = model.wheel_radius
        contours = [get_wheel_contour(model, x, i, normals) for i in 1:nc]
        points = [[-r*sin(σ); 0; -r*cos(σ)] for σ in contours]

        # wheel_kinematics: stack per-point kinematics
        @test norm(wheel_kinematics(model, x, normals) -
                   vcat([PinnZoo.kinematics(model, x, i, points[i]) for i in 1:nc]...), Inf) < 1e-10

        # wheel_velocity: stack per-point velocity
        @test norm(wheel_velocity(model, x, normals) -
                   vcat([PinnZoo.kinematics_velocity(model, x, i, points[i]) for i in 1:nc]...), Inf) < 1e-10

        # wheel_jacobian position block: FiniteDifferences on the wheel_kinematics POSITION rows
        wJ_pos = wheel_jacobian(model, x, normals)[pos_inds, :]
        wJ_truth = vcat([FiniteDifferences.jacobian(
                              FiniteDifferences.central_fdm(5, 1),
                              _x -> PinnZoo.kinematics(model, _x, i, points[i])[1:3],
                              copy(x))[1] for i in 1:nc]...)
        @test norm(wJ_pos - wJ_truth, Inf) < 1e-6

        # wheel_velocity_jacobian position block: same idea
        wJv_pos = wheel_velocity_jacobian(model, x, normals)[pos_inds, :]
        wJv_truth = vcat([FiniteDifferences.jacobian(
                              FiniteDifferences.central_fdm(5, 1),
                              _x -> PinnZoo.kinematics_velocity(model, _x, i, points[i])[1:3],
                              copy(x))[1] for i in 1:nc]...)
        @test norm(wJv_pos - wJv_truth, Inf) < 2e-6

        # wheel_jacobianTvp: sum of per-wheel kinematics_jacobianTvp on λ
        λ_w = randn(7*nc)
        τ_wheel = wheel_jacobianTvp(model, x, λ_w, normals)
        τ_per_pt = sum(PinnZoo.kinematics_jacobianTvp(model, x, i, points[i], λ_w[(i-1)*7 .+ (1:7)]) for i in 1:nc)
        @test norm(τ_wheel - τ_per_pt, Inf) < 1e-10
    end
end