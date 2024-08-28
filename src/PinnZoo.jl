module PinnZoo
    using Libdl
    using LinearAlgebra

    abstract type PinnZooModel end

    const SHARED_LIBRARY_DIR = joinpath(@__DIR__, "../build")
    const MODEL_DIR = joinpath(@__DIR__, "../models")

    # Dynamics functions (from codegen)
    include(joinpath(@__DIR__, "dynamics.jl"))
    
    # Kinematics functions (from codegen)
    include(joinpath(@__DIR__, "kinematics.jl"))

    # Quaternion helpers
    include(joinpath(@__DIR__, "quaternions.jl"))

    # Quadruped specific functions (TODO: some of this can be more general)
    include(joinpath(@__DIR__, "quadruped.jl"))

    # Include model files here
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

    # Generate state (override if state vector contains a quaternion)
    """
        zero_state(model::PinnZooModel)

    Return the neutral state of the model.
    !!! warning
        This currently defaults to all zeros unless overridden and will be wrong if there is a quaternion in the state.
    """
    zero_state(model::PinnZooModel) = zeros(model.nx)

    """
        randn_state(model::PinnZooModel)

    Return a state vector where every coordinate is drawn from a normal distribution
    !!! warning
        This currently does not respect the unit norm constraint on quaternions unless overridden
    """
    randn_state(model::PinnZooModel) = randn(model.nx)

    """
        init_state(model::PinnZooModel)

    Return a custom initial state for the model (like standing) if defined, otherwise returns all zeros
    !!! warning
        This currently defaults to all zeros unless overridden and will be wrong if there is a quaternion in the state.
    """
    init_state(model::PinnZooModel) = zeros(model.nx) # Define for each model if desired
    
    ## Exports
    # Types
    export PinnZooModel

    # Code-genned functions
    export is_floating, zero_state, init_state, randn_state
    export M_func, C_func, forward_dynamics, forward_dynamics_deriv, inverse_dynamics, inverse_dynamics_deriv
    export velocity_kinematics, velocity_kinematics_T
    export kinematics, kinematics_jacobian, kinematics_velocity, kinematics_velocity_jacobian

    # Quaternion helpers
    export quat_to_axis_angle, axis_angle_to_quat, quat_conjugate, skew, L_mult,
        R_mult, attitude_jacobian, quat_to_rot

    # Quadruped specific functions
    export Quadruped
    export error_jacobian, error_jacobian_T, apply_Δx, state_error
    export B_func, fix_joint_limits, inverse_kinematics, nearest_ik

    export PinnZooModel, Quadruped
    export is_floating, zero_state, init_state, randn_state
    export M_func, C_func, forward_dynamics, forward_dynamics_deriv, inverse_dynamics, inverse_dynamics_deriv
    export velocity_kinematics, velocity_kinematics_T
    export kinematics, kinematics_jacobian, kinematics_velocity, kinematics_velocity_jacobian

    export Cartpole, Go1, Go2, Nadia
end