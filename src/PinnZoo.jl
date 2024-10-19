module PinnZoo
    using Libdl
    using LinearAlgebra

    abstract type PinnZooModel end
    abstract type PinnZooFloatingBaseModel <: PinnZooModel end

    const SHARED_LIBRARY_DIR = joinpath(@__DIR__, "../build")
    const MODEL_DIR = joinpath(@__DIR__, "../models")

    # Vector order functions (helps with conversions between our convention, MuJoCo, and RigidBodyDynamics)
    include(joinpath(@__DIR__, "vector_orders.jl"))

    # Dynamics functions (from codegen)
    include(joinpath(@__DIR__, "dynamics.jl"))
    
    # Kinematics functions (from codegen)
    include(joinpath(@__DIR__, "kinematics.jl"))

    # Quaternion helpers
    include(joinpath(@__DIR__, "quaternions.jl"))

    # Quadruped specific functions (TODO: some of this can be more general)
    include(joinpath(@__DIR__, "quadruped.jl"))

    # Include model files here
    include(joinpath(@__DIR__, "model_macro.jl"))
    include(joinpath(MODEL_DIR, "pendulum/pendulum.jl"))
    include(joinpath(MODEL_DIR, "double_pendulum/double_pendulum.jl"))
    include(joinpath(MODEL_DIR, "cartpole/cartpole.jl"))
    include(joinpath(MODEL_DIR, "unitree_go1/go1.jl"))
    include(joinpath(MODEL_DIR, "unitree_go2/go2.jl"))
    include(joinpath(MODEL_DIR, "nadia/nadia.jl"))

    # Defaults for models
    """
        is_floating(model::PinnZooModel)
    
    Return whether the model includes a floating base joint
    """
    is_floating(model::PinnZooModel) = false
    is_floating(model::PinnZooFloatingBaseModel) = true

    # Generate state (override if state vector contains a quaternion)
    """
        zero_state(model::PinnZooModel)

    Return the neutral state of the model.
    """
    zero_state(model::PinnZooModel) = zeros(model.nx)
    zero_state(model::PinnZooFloatingBaseModel) =  [zeros(3); 1; zeros(model.nx - 4)]

    """
        randn_state(model::PinnZooModel)

    Return a state vector where every coordinate is drawn from a normal distribution
    """
    randn_state(model::PinnZooModel) = randn(model.nx)
    randn_state(model::PinnZooFloatingBaseModel) = [randn(3); normalize(randn(4)); randn(model.nx - 7)]

    """
        init_state(model::PinnZooModel)

    Return a custom initial state for the model (like standing) if defined, otherwise returns all zeros
    """
    init_state(model::PinnZooModel) = zero_state(model) # Define for each model if desired
    
    ## Exports
    # Types
    export PinnZooModel, PinnZooFloatingBaseModel, Quadruped

    # Conversions
    export StateOrder, ConversionIndices, generate_conversions
    export change_order, change_order!, change_orders, change_orders!

    # Helpers, dynamics, kinematics
    export is_floating, zero_state, init_state, randn_state
    export M_func, C_func, dynamics, dynamics_deriv, forward_dynamics, forward_dynamics_deriv, inverse_dynamics, inverse_dynamics_deriv
    export velocity_kinematics, velocity_kinematics_T
    export kinematics_size
    export kinematics, kinematics_jacobian, kinematics_velocity, kinematics_velocity_jacobian
    export error_jacobian, error_jacobian_T, apply_Δx, state_error

    # Quaternion helpers
    export quat_to_axis_angle, axis_angle_to_quat, quat_conjugate, skew, L_mult,
        R_mult, attitude_jacobian, quat_to_rot

    # Quadruped specific functions
    export Quadruped
    export B_func, fix_joint_limits, inverse_kinematics, nearest_ik

    export Pendulum, DoublePendulum, Cartpole, Go1, Go2, Nadia
end