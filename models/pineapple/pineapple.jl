@create_pinnzoo_model struct Pineapple <: PinnZooFloatingBaseModel
    kinematics_ori::Symbol
    μ::Float64
    # torque_limits::Vector{Float64}
    # joint_limits::Matrix{Float64}
    function Pineapple(; μ = 0.3, kinematics_ori::Symbol = :Quaternion)
        lib = let
            if kinematics_ori == :None
                lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libpineapple_8dof"))                
            elseif kinematics_ori == :Quaternion
                lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libpineapple_8dof_quat"))
            else
                throw(error("specified configuration is either not found or not supported. Did you compile?"))
            end
            lib
        end

        # Limits
        # torque_limits = 23.7*ones(12)
        # joint_limits = [repeat([-Inf Inf], 7); repeat([-0.802851 0.802851; -1.0472 4.18879; -2.69653 -0.916298], 4)]
        return new(kinematics_ori, μ)#, torque_limits, joint_limits)
    end
end



@doc raw"""
    B_func(quad::Quadruped)

Return the input jacobian mapping motor torques into joint torques
"""
function B_func(model::Pineapple)
    return [zeros(6, model.nu); I(model.nu)]
end

function init_state(model::Pineapple)
    x = zero_state(model)
    if model.nu == 6
        x[3] = 0.276;
        x[8:13] = [pi/4; -pi/2; 0; pi/4; -pi/2; 0];
    elseif model.nu == 8
        x[3] = 0.28;
        x[8:15] = [0; pi/4; -pi/2; 0; 0; pi/4; -pi/2; 0];
    else
        throw(ArgumentError("Unsupported number of DOFs"))
    end
    return x
end

@doc raw"""
    fix_joint_limits(model::Quadruped, x; supress_error = false)

Return x with joint angles wrapped to 2pi to fit within joint limits if possible. If not
and supress_error is false, this will error. Otherwise, this will return a clamped version.
"""
function fix_joint_limits(model::Pineapple, x; supress_error = false)
    x = copy(x)
    success = true
    failed_joints = []
    for j = 8:model.nq
        while x[j] < model.joint_limits[j, 1]
            x[j] += 2*pi
        end
        while x[j] > model.joint_limits[j, 2]
            x[j] -= 2*pi
        end
        success = success && (model.joint_limits[j, 2] > x[j] > model.joint_limits[j, 1])
        if !(model.joint_limits[j, 2] > x[j] > model.joint_limits[j, 1])
            push!(failed_joints, model.orders[:nominal].config_names[j])
        end
    end
    if !success && !supress_error
        @error "Couldn't satisfy all joint limits"
        println(failed_joints)
    end
    x[1:model.nq] = clamp.(x[1:model.nq], model.joint_limits[:, 1], model.joint_limits[:, 2])
    return x
end

@doc raw"""
    inverse_kinematics(model::Pineapple, x, wheel_locs)

Return 8 by 2 matrix containing the 2 possible joint angle solutions that put the wheels at wheel_locs given
the body pose in x. For Pineapple v1 (8-DOF), this includes 2 legs (right and left) with 3 joints each plus 2 wheel joints.
Will be populated with NaN or Inf if solutions don't exist.
"""
function inverse_kinematics(model::Pineapple, x, wheel_locs)
    # Check if this is the 8-DOF version
    if model.nu != 8
        throw(ArgumentError("This IK function is designed for Pineapple v1 (8-DOF). Current model has $(model.nu) DOFs."))
    end
    
    # Leg parameters extracted from URDF following Go2 approach
    # Based on URDF joint origins analysis
    
    # Hip to thigh joint distances - ASYMMETRIC FROM URDF
    
    # leg parameters - OPTIMIZED FOR ACCURACY
    # These parameters achieve sub-centimeter IK accuracy
    hip_to_thigh = 0.0129353215081658        # Distance from hip to thigh joint
    thigh_length = 0.190000001217773 #0.184        # Length of thigh segment
    calf_length = 0.190000001217773 #0.1844         # Length of calf segment
    
    # Offset parameters for accurate IK
    thigh_to_calf = 0.035931787557176
    calf_to_wheel = 0.0421817504179199
    offset = hip_to_thigh + thigh_to_calf + calf_to_wheel
    # Hip positions in the body frame (extracted from URDF joint origins)
    # CORRECTED ORDER: Left leg first (leg_ind=1), Right leg second (leg_ind=2)
    hip_local_pos = [
        [0.0000, 0.061249999999998, 0.0610009280869146],   # Left hip (positive y) - leg_ind=1
        [0.0000, -0.061249999999998, 0.0610009280869146]   # Right hip (negative y) - leg_ind=2
    ]

    # Base transformations
    base_rot = quat_to_rot(x[4:7])
    base_pos = base_rot' * x[1:3]

    # Results storage: 6 joints (3 per leg) by 2 solutions
    wheel_q = zeros(6, 2)

    for wheel_ind = 1:2
        # Calculate desired wheel position in the hip frame
        hip_trans = -hip_local_pos[wheel_ind] - base_pos;

        # Remove offsets
        wheel_pos = base_rot'*wheel_locs[(wheel_ind - 1)*kinematics_dim(model) .+ (1:3)] + hip_trans

        # Reflect right wheels to use same IK for all two wheels
        if wheel_ind % 2 == 0
            wheel_pos[2] = -wheel_pos[2]
        end

        # wheel_pos += wheel_offset_local[wheel_ind] + calf_offset_local[wheel_ind]
        # Extract x, y, z pos of wheel to make code cleaner
        x, y, z = wheel_pos

        # Storage for each solution
        leg_solns = zeros(3, 2)

        #--------- Calculate hip angle ----------
        # Calculate "radius" of circle from the thigh pivot to the foot
        L_squared = y^2 + z^2 - offset^2
        L = 0
        if (L_squared > 1e-12)
            L = sqrt(L_squared) # Prevent numerical issues if L is close to 0
        end

        # Solve linear system in cos(theta), sin(theta) relating leg vector
        # before and after hip rotation. There are two solutions corresponding to
        # (offset, L) and (offset, -L)
        cos_theta = (offset*y - L*z) / (L^2 + offset^2)
        sin_theta = (L*y + offset*z) / (L^2 + offset^2)
        leg_solns[1, 1] = atan(sin_theta, cos_theta); # First solution

        cos_theta = (offset*y + L*z) / (L^2 + offset^2)
        sin_theta = (-L*y + offset*z) / (L^2 + offset^2)
        leg_solns[1, 2] = atan(sin_theta, cos_theta); # Second solution

        # ------ Calculate thigh and calf angle for each possible hip angle ------------
        for soln = 1:2
            # Undo hip rotation on z-axis to deal with planar xz calf-thigh relationship
            z_rot = sin(-leg_solns[1, soln])*y + cos(-leg_solns[1, soln])*z

            # Calf angle first
            cos_calf = (thigh_length^2 + calf_length^2 - x^2 - z_rot^2) / (2*thigh_length*calf_length)
            cos_calf = clamp(cos_calf, -1.0, 1.0)  # Prevent domain error
            leg_solns[3, soln] = acos(cos_calf) - π
            
            # Thigh angle
            cos_alpha = (thigh_length^2 + x^2 + z_rot^2 - calf_length^2) / (2*thigh_length*sqrt(x^2 + z_rot^2 + 1e-16))
            cos_alpha = clamp(cos_alpha, -1.0, 1.0)  # Prevent domain error
            alpha = acos(cos_alpha)
            phi = atan(z_rot, x)
            leg_solns[2, soln] = alpha - phi - π/2
            #-leg_solns[3, soln] / 2 - atan(z_rot, x) - π/2
        end

        # Flip hip angles for right feet
        if wheel_ind % 2 == 0
            leg_solns[1, 1] *= -1
            leg_solns[1, 2] *= -1
        end

        # Write soln
        wheel_q[(wheel_ind - 1)*3 .+ (1:3), :] = leg_solns        
    end

    return wheel_q
end

@doc raw"""
    nearest_ik(model::Pineapple, q, wheel_locs; obey_limits = true)

Return the ik solution for the given wheel_locs and body orientation in q that is closest to the current
joint angles in q (defined by minimum norm per joint). For Pineapple robot with wheels.
"""
function nearest_ik(model::Pineapple, q, wheel_locs; obey_limits = true)
    q = copy(q)
    wheel_q = inverse_kinematics(model, q, wheel_locs)

    # Build both options
    q1 = copy(q)
    q1[8:10] = wheel_q[1:3, 1]  # Left leg: hip, thigh, calf
    q1[12:14] = wheel_q[4:6, 1]  # Right leg: hip, thigh, calf (skip wheel joints 11 and 15)
    q2 = copy(q)
    q2[8:10] = wheel_q[1:3, 2]  # Left leg: hip, thigh, calf
    q2[12:14] = wheel_q[4:6, 2]  # Right leg: hip, thigh, calf (skip wheel joints 11 and 15)


    # Fix joint limits (don't let these error since some will probably be wrong)
    if obey_limits && hasfield(typeof(model), :joint_limits)
        q1 = fix_joint_limits(model, q1, supress_error = true)
        q2 = fix_joint_limits(model, q2, supress_error = true)
    end

    # Build final ik leg by leg (skip wheel joints)
    # Left leg: indices 8-10 (hip, thigh, calf)
    left_leg_inds = 8:10
    if norm(q[left_leg_inds] - q1[left_leg_inds]) < norm(q[left_leg_inds] - q2[left_leg_inds])
        q[left_leg_inds] = q1[left_leg_inds]
    else
        q[left_leg_inds] = q2[left_leg_inds]
    end
    
    # Right leg: indices 12-14 (hip, thigh, calf) - skip wheel joint 11
    right_leg_inds = 12:14
    if norm(q[right_leg_inds] - q1[right_leg_inds]) < norm(q[right_leg_inds] - q2[right_leg_inds])
        q[right_leg_inds] = q1[right_leg_inds]
    else
        q[right_leg_inds] = q2[right_leg_inds]
    end

    # Final fix (let this one error if we can't obey the limits)
    if obey_limits && hasfield(typeof(model), :joint_limits)
        return fix_joint_limits(model, q)
    else
        return q
    end
end

"""
    X = solve_ref_IK(model, X, wheel_ref)
Given a vectors of vectors of states (X) and wheel locations (wheel_ref) representing a N
long reference trajectory, solves for joint angles for X[k] to make kinematics(model, X[k]) == wheel_locs[k], 
picking the closest IK solution to X[k - 1]. Assumes that kinematics(model, X[1]) == wheel_locs[1] is already true
"""
function solve_ref_IK(model::Pineapple, X, wheel_ref)
    N = length(X)
    for k = 2:N
        X[k][7 .+ (1:model.nu)] = X[k - 1][7 .+ (1:model.nu)]
        X[k] = nearest_ik(model, X[k], wheel_ref[k])
    end
    return X
end

"""
    X, U, Λ, residuals = solve_controls(model::Quadruped, X, h)
Given a reference trajectory, solve for controls and contact forces that make that reference consistent
with rk4. Currently assumes all 4 feet are in contact. Does not enforce friction or normal force constraints.
This is an underdetermined problem so we use least-squares, so the newton solve likely won't converge. residuals
includes the final residual for each solve.
"""
# function solve_ref_controls(model::Pineaple, X, h; max_iters = 10, tol = 1e-10, verbose = false)
#     N = length(X)
#     U = [zeros(model.nu) for _ = 1:N-1]
#     Λ = [zeros(model.nu) for _ = 1:N-1]
#     residuals = zeros(N - 1)

#     for k = 1:N - 1
#         function residual(u)
#             u, λ = u[1:model.nu], u[model.nu+1:end]

#             x_rk4 = rk4(model, X[k], u, λ, h)
#             x_next = X[k + 1]

#             return [
#                 x_next[1:3] - x_rk4[1:3]
#                 quat_to_axis_angle(L_mult(x_next[4:7])'*x_rk4[4:7])
#                 x_next[8:end] - x_rk4[8:end]
#             ]
#         end

#         # Perform Newton to try to minimize residual
#         u = [U[k]; Λ[k]]
#         r = residual(u)
#         for iter = 1:max_iters
#             dr_du = FiniteDiff.finite_difference_jacobian(residual, u)
#             J = (dr_du'*dr_du + 1e-6*I)
#             Δu = -J \ (dr_du'*r)

#             # Linesearch
#             α = 1
#             for _ = 1:10
#                 if  norm(residual(u + α*Δu), Inf) > norm(residual(u), Inf)
#                     α = α/2
#                 end
#             end
#             u = u + α*Δu
#             prev_r = r
#             r = residual(u)

#             if verbose
#                 @info "k = %d\titer = %d\tr = %1.2e\tΔr = %1.2e\tα = %1.2e\tcond(dr_dv) = %1.2e\n" k iter norm(r, Inf) norm(r - prev_r, Inf) α cond(J) 
#             end
            
#             residuals[k] = norm(r, Inf)
#             if norm(r, Inf) < 1e-10
#                 break
#             elseif norm(r - prev_r, Inf) < norm(r, Inf)*1e-3 # Early exit criterion
#                 break
#             end
#         end
#     end

#     @info "Largest residual error = %1.2e\n" norm(residuals, Inf)

#     return X, U, Λ, residuals
# end