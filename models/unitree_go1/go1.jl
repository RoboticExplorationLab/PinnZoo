@create_pinnzoo_model struct Go1 <: Quadruped
    μ::Float64
    torque_limits::Vector{Float64}
    joint_limits::Matrix{Float64}
    function Go1(; μ = 0.3)
        lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libunitree_go1"))

        # Limits
        torque_limits = 23.7*ones(12)
        joint_limits = [repeat([-Inf Inf], 7); repeat([-0.802851 0.802851; -1.0472 4.18879; -2.69653 -0.916298], 4)]
        return new(μ, torque_limits, joint_limits)
    end
end

@doc raw"""
    Go1(; μ = 0.3) <: Quadruped

Return the Unitree Go1 dynamics and kinematics model
""" Go1

@doc raw"""
    init_state(model::Go1)

Return state vector with the robot on the ground in the starting pose
"""
function init_state(model::Go1)
    x = zero_state(model)
    x[3] = 0.058
    x[8:3:19] = [-0.55; 0.55; -0.55; 0.55]
    x[9:3:19] .= 64*pi/180
    x[10:3:19] .= -155*pi/180
    return x
end

@doc raw"""
    inverse_kinematics(model::Go1, x, foot_locs)

Return 12 by 2 matrix containing the 2 possible joint angle solutions that put the feet at foot_locs given
the body pose in x. Will be populated with NaN or Inf if solutions don't exist.
"""
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
        if foot_ind % 2 == 0
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
        if foot_ind % 2 == 0
            leg_solns[1, 1] *= -1
            leg_solns[1, 2] *= -1
        end

        # Write soln
        foot_q[(foot_ind - 1)*3 .+ (1:3), :] = leg_solns        
    end

    return foot_q
end