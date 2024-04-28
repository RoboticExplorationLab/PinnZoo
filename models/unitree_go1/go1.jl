struct Go1 <: Quadruped
    urdf_path::String
    state_order::Vector{String}
    nq
    nv
    nx
    nu
    μ::Float64 # Friction coefficient
    torque_limits::Vector{Float64}
    joint_limits::Matrix{Float64}
    M_func_ptr::Ptr{Nothing}
    C_func_ptr::Ptr{Nothing}
    forward_dynamics_ptr::Ptr{Nothing}
    inverse_dynamics_ptr::Ptr{Nothing}
    velocity_kinematics_ptr::Ptr{Nothing}
    kinematics_bodies::Vector{String}
    kinematics_ptr::Ptr{Nothing}
    kinematics_jacobian_ptr::Ptr{Nothing}
    function Go1(; μ = 0.3)
        local lib
        try
            lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libunitree_go1.so"))
        catch e
            @error "Unitree Go1 dynamics library wasn't found. Did you compile it using CMake?"
        end

        # Path to URDF (useful for visualization/testing)
        urdf_path = joinpath(MODEL_DIR, "unitree_go1/go1.urdf")

        # Definition of state_order
        state_order = ["x", "y", "z", "q_w", "q_x", "q_y", "q_z", 
                "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", 
                "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", 
                "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", 
                "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint", 
                "lin_v_x", "lin_v_y", "lin_v_z", "ang_v_x", "ang_v_y", "ang_v_z", 
                "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", 
                "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", 
                "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", 
                "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"]

        # Dynamics
        M_func_ptr = dlsym(lib, :M_func_wrapper)
        C_func_ptr = dlsym(lib, :C_func_wrapper)
        forward_dynamics_ptr = dlsym(lib, :forward_dynamics_wrapper)
        inverse_dynamics_ptr = dlsym(lib, :inverse_dynamics_wrapper)
        velocity_kinematics_ptr = dlsym(lib, :velocity_kinematics_wrapper)

        # Kinematics
        kinematics_bodies = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]
        kinematics_ptr = dlsym(lib, :kinematics_wrapper)
        kinematics_jacobian_ptr = dlsym(lib, :kinematics_jacobian_wrapper)

        # Limits
        torque_limits = 23.7*ones(12)
        joint_limits = [repeat([-Inf Inf], 7); repeat([-0.802851 0.802851; -1.0472 4.18879; -2.69653 -0.916298], 4)]

        return new(
            urdf_path, state_order,
            19, 18, 18 + 19, 12,
            μ, torque_limits, joint_limits,
            M_func_ptr, C_func_ptr, forward_dynamics_ptr, inverse_dynamics_ptr, velocity_kinematics_ptr,
            kinematics_bodies, kinematics_ptr, kinematics_jacobian_ptr
        )
    end
end

is_floating(model::Go1) = true
zero_state(model::Go1) = [zeros(3); 1; zeros(model.nx - 4)]
rand_state(model::Go1) = [randn(3); normalize(randn(4)); randn(model.nx - 7)]
function init_state(model::Go1)
    x = zero_state(model)
    x[3] = 0.058
    x[8:3:19] = [-0.55; 0.55; -0.55; 0.55]
    x[9:3:19] .= 64*pi/180
    x[10:3:19] .= -155*pi/180
    return x
end

function inverse_kinematics(model::Go1, x, foot_locs)
    # Leg variables
    hip_to_thigh = 0.08
    thigh_length = 0.213
    calf_length = 0.213

    # Hip positions in the body frame
    hip_local_pos = [
        [0.1881, 0.04675, 0],
        [0.1881, -0.04675, 0],
        [-0.1881, 0.04675, 0],
        [-0.1881, -0.04675, 0]]

    # Base transformations
    base_rot = quat_to_rot(x[4:7])
    base_pos = base_rot'*x[1:3]

    # Results storage
    foot_q = zeros(12, 2);

    for foot_ind = 1:4
        # Calculate desired foot position in the hip frame
        hip_trans = -hip_local_pos[foot_ind] - base_pos;
        foot_pos = base_rot'*foot_locs[(foot_ind - 1)*3 .+ (1:3)] + hip_trans

        # Reflect right feet to use same IK for all four feet
        if foot_ind % 2 == 1
            foot_pos[2] = -foot_pos[2]
        end

        # Extract x, y, z pos of foot to make code cleaner
        x, y, z = foot_pos

        # Storage for each solution
        leg_solns = zeros(3, 2)

        #--------- Calculate hip angle ----------
        # Calculate "radius" of circle from the thigh pivot to the foot
        L_squared = y^2 + z^2 - hip_to_thigh^2
        L = 0
        if (L_squared > 1e-12)
            L = sqrt(L_squared) # Prevent numerical issues if L is close to 0
        end

        # Solve linear system in cos(theta), sin(theta) relating leg vector
        # before and after hip rotation. There are two solutions corresponding to
        # (hip_to_thigh, L) and (hip_to_thigh, -L)
        cos_theta = (hip_to_thigh*y - L*z) / (L^2 + hip_to_thigh^2)
        sin_theta = (L*y + hip_to_thigh*z) / (L^2 + hip_to_thigh^2)
        leg_solns[1, 1] = atan(sin_theta, cos_theta); # First solution

        cos_theta = (hip_to_thigh*y + L*z) / (L^2 + hip_to_thigh^2)
        sin_theta = (-L*y + hip_to_thigh*z) / (L^2 + hip_to_thigh^2)
        leg_solns[1, 2] = atan(sin_theta, cos_theta); # Second solution

        # ------ Calculate thigh and calf angle for each possible hip angle ------------
        for soln = 1:2
            # Undo hip rotation on z-axis to deal with planar xz calf-thigh relationship
            z_rot = sin(-leg_solns[1, soln])*y + cos(-leg_solns[1, soln])*z

            # Calf angle first
            leg_solns[3, soln] = acos(
                (thigh_length^2 + calf_length^2 - x^2 - z_rot^2) / (2*thigh_length^2)) - π
            
            # Thigh angle
            leg_solns[2, soln] = -leg_solns[3, soln] / 2 - atan(z_rot, x) - π/2
        end

        # Flip hip angles for right feet
        if foot_ind % 2 == 1
            leg_solns[1, 1] *= -1
            leg_solns[1, 2] *= -1
        end

        # Write soln
        foot_q[(foot_ind - 1)*3 .+ (1:3), :] = leg_solns        
    end

    return foot_q
end