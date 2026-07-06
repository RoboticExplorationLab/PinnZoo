_pineapple_default_wheel_radius(version::Symbol) =
    version === :v0 ? 0.07  :
    version === :v1 ? 0.07  :
    version === :v2 ? 0.075 :
    version === :v3 ? 0.0925 :
    version === :v3_gripper ? 0.0925 :
    error("No default wheel_radius for version=$(version). Pass wheel_radius= kwarg explicitly.")

@create_pinnzoo_model struct Pineapple <: PinnZooFloatingBaseModel
    kinematics_ori::Symbol
    version::Symbol
    μ::Float64
    wheel_radius::Float64
    hwd_x_inds::Vector{Int64}
    hwd_u_inds::Vector{Int64}

    function Pineapple(; μ = 0.3, kinematics_ori::Symbol = :Quaternion, version::Symbol=:v1, wheel_radius::Float64 = _pineapple_default_wheel_radius(version))
        
        # version ∈ (:v0, :v1, :v2) || error("Unsupported version=$(version). Use :v0, :v1, or :v2.")
        
        lib = let
            if kinematics_ori == :None
                lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libpineapple_$(version)"))                
            elseif kinematics_ori == :Quaternion
                lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libpineapple_quat_$(version)"))
            else
                throw(error("specified configuration is either not found or not supported. Did you compile?"))
            end
            lib
        end 
         
        # --- Version-dependent hardware index mapping ---
        hwd_x_inds_by_version = Dict{Symbol, Vector{Int}}(
            :v0 => [1:7; 7 .+ (1:6); 19 .+ (1:6); 19 + 6 .+ (1:6)],  
            :v1 => [1:7; 7 .+ (1:8); 19 .+ (1:6); 19 + 6 .+ (1:8)],  # default
            :v2 => [1:7; 7 .+ (1:8); 19 .+ (1:6); 19 + 6 .+ (1:8)],  
            :v3 => [1:7; 7 .+ (1:13); 19 .+ (1:6); 19 + 6 .+ (1:13)], # TODO: confirm these hardware indices for v3  
            :v3_gripper => [1:7; 7 .+ (1:15); 19 .+ (1:6); 19 + 6 .+ (1:15)] # TODO: confirm these hardware indices for v3_gripper  
        )

        hwd_u_inds_by_version = Dict{Symbol, Vector{Int}}(
            :v0 => collect(1:6),  
            :v1 => collect(1:8),  # default
            :v2 => collect(1:8),
            :v3 => collect(1:13),
            :v3_gripper => collect(1:15)  
        )

        hwd_x_inds = get(hwd_x_inds_by_version, version, nothing)
        hwd_x_inds === nothing && error("Missing hwd_x_inds mapping for version=$(version).")

        hwd_u_inds = get(hwd_u_inds_by_version, version, nothing)
        hwd_u_inds === nothing && error("Missing hwd_u_inds mapping for version=$(version).")

        return new(kinematics_ori, version, μ, wheel_radius, hwd_x_inds, hwd_u_inds)
    end
end

@doc raw"""
    Pineapple(; μ = 0.3, kinematics_ori::Symbol = :Quaternion,
                version::Symbol = :v1, wheel_radius = nothing) <: PinnZooFloatingBaseModel

Return the Pineapple wheeled biped dynamics and kinematics model.

`wheel_radius` defaults to the version's nominal value (`v0`/`v1`: 0.07, `v2`: 0.075, `v3`: 0.0925)
pass it explicitly to override.
""" Pineapple

@doc raw"""
    get_wheel_contour(rot_normal::AbstractVector)

Return the contour angle $$\sigma = \text{atan2}(n_x, n_z)$$ locating the wheel's contact point at
$$[-r\sin\sigma;\; 0;\; -r\cos\sigma]$$ in body frame, where rot\_normal $$= (n_x, n_y, n_z)$$ is the
floor normal in the wheel body frame.
"""
function get_contour(rot_normal)
    return atan(rot_normal[1], rot_normal[3])
end

@doc raw"""
    get_contour(model::Pineapple, x, wheel_ind::Int, normals = [[0,0,1.0]; [0,0,1.0]])

Return the contour angle σ for wheel wheel\_ind, rotating the world-frame floor normal
normals[(wheel\_ind-1)*3 .+ (1:3)] into the wheel body frame via kinematics\_rotation.
NOTE: only works for kinematics_ori = :Quaternion
"""
function get_wheel_contour(model::Pineapple, x, wheel_ind, normals = [[0,0,1.0]; [0,0,1.0]])
    rot = kinematics_rotation(model, x)
    rot = reshape(vec(rot), 3*length(model.kinematics_bodies), 3)[(wheel_ind - 1)*3 .+ (1:3), :]
    return get_contour(rot'*normals[(wheel_ind - 1)*3 .+ (1:3)])
end

@doc raw"""
    wheel_kinematics(model::Pineapple, x, normals = [[0,0,1.0]; [0,0,1.0]])

Return the stacked (3*nc) world-frame positions of every wheel's contact point at radius
model.wheel_radius and floor normals.
"""
function wheel_kinematics(model::Pineapple, x::AbstractVector{T}, normals = [[0,0,1.0]; [0,0,1.0]]) where T <: Real
    r = model.wheel_radius
    nc = 2 # TODO: assumes wheels are first two kinematics_bodies
    contours = [get_wheel_contour(model, x, i, normals) for i in 1:nc]
    points   = [[-r*sin(σ); 0; -r*cos(σ)] for σ in contours]
    return vcat([kinematics(model, x, i, points[i]) for i in 1:nc]...)
end

@doc raw"""
    wheel_jacobian(model::Pineapple, x, normals = [[0,0,1.0]; [0,0,1.0]])

Return the (3*nc × nx) jacobian of wheel\_kinematics with respect to x.
"""

function wheel_jacobian(model::Pineapple, x::AbstractVector{Float64}, normals = [[0,0,1.0]; [0,0,1.0]])
    r = model.wheel_radius
    nc = 2 # TODO: assumes wheels are first two kinematics_bodies
    contours = [get_wheel_contour(model, x, i, normals) for i in 1:nc]
    points   = [[-r*sin(σ); 0; -r*cos(σ)] for σ in contours]
    return vcat([kinematics_jacobian(model, x, i, points[i]) for i in 1:nc]...)
end

@doc raw"""
    wheel_velocity(model::Pineapple, x::AbstractVector{<:Real},
                   normals = [[0,0,1.0]; [0,0,1.0]])

Return the stacked (3*nc) world-frame velocities of every wheel's contact point.
"""
function wheel_velocity(model::Pineapple, x::AbstractVector{<:Real}, normals = [[0,0,1.0]; [0,0,1.0]])
    r = model.wheel_radius
    nc = 2 # TODO: assumes wheels are first two kinematics_bodies
    contours = [get_wheel_contour(model, x, i, normals) for i in 1:nc]
    points   = [[-r*sin(σ); 0; -r*cos(σ)] for σ in contours]
    return vcat([kinematics_velocity(model, x, i, points[i]) for i in 1:nc]...)
end

@doc raw"""
    wheel_velocity_jacobian(model::Pineapple, x::AbstractVector{Float64},
                            normals = [[0,0,1.0]; [0,0,1.0]])

Return the (3*nc × nx) jacobian of wheel\_velocity with respect to x.
"""
function wheel_velocity_jacobian(model::Pineapple, x::AbstractVector{Float64}, normals = [[0,0,1.0]; [0,0,1.0]])
    r = model.wheel_radius
    nc = 2 # TODO: assumes wheels are first two kinematics_bodies
    contours = [get_wheel_contour(model, x, i, normals) for i in 1:nc]
    points   = [[-r*sin(σ); 0; -r*cos(σ)] for σ in contours]
    return vcat([kinematics_velocity_jacobian(model, x, i, points[i]) for i in 1:nc]...)
end

@doc raw"""
    wheel_jacobianTvp(model::Pineapple, x::AbstractVector{<:Real},
                      λ::AbstractVector{<:Real},
                      normals = [[0,0,1.0]; [0,0,1.0]])

Return the `nv`-dim joint-space torque from contact wrenches `λ` (a `3*nc`-vector, 3 per wheel)
applied at the current wheel contact points. Equivalent to
`sum(kinematics_jacobianTvp(model, x, i, point_i, λ_i) for i in 1:nc)`, but resolves the contact
points first. Differentiable in both `x` and `λ`.
"""
function wheel_jacobianTvp(model::Pineapple, x::AbstractVector{<:Real}, λ::AbstractVector{<:Real}, normals = [[0,0,1.0]; [0,0,1.0]])
    r = model.wheel_radius
    nc = 2 # TODO: assumes wheels are first two kinematics_bodies
    contours = [get_wheel_contour(model, x, i, normals) for i in 1:nc]
    points   = [[-r*sin(σ); 0; -r*cos(σ)] for σ in contours]
    return sum(kinematics_jacobianTvp(model, x, i, points[i], λ[(i-1)*7 .+ (1:7)]) for i in 1:nc)
end

@doc raw"""
    B_func(model::Pineapple)

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
    elseif model.nu == 13
        x[3] = 0.3677;
        x[8:20] = [zeros(5); 0; pi/4; -pi/2; 0; 0; pi/4; -pi/2; 0];
    elseif model.nu == 15
        x[3] = 0.3677;
        x[8:22] = [zeros(7); 0; pi/4; -pi/2; 0; 0; pi/4; -pi/2; 0];
    else
        throw(ArgumentError("Unsupported number of DOFs"))
    end
    return x
end

@doc raw"""
    fix_joint_limits(model::Pineapple, x; supress_error = false)

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
function inverse_kinematics(model::Pineapple, x::AbstractVector, wheel_locs::AbstractVector)
    # Check if this is the 8-DOF version
    if model.nu != 8
        throw(ArgumentError("This IK function is designed for Pineapple v1 (8-DOF). Current model has $(model.nu) DOFs."))
    end
    
    # Leg parameters extracted directly from pineapple_v1.urdf joint origins.
    # All values below are the joint-origin xyz of the corresponding child link
    # in its parent's frame; see URDF lines around L_hip_joint, L_thigh_joint,
    # L_calf_joint, L_wheel_joint.
    thigh_length   =  0.165       # |L_calf_joint.origin.z|   (calf joint Z below thigh axis)
    calf_length    =  0.165       # |L_wheel_joint.origin.z|  (wheel joint Z below calf axis)
    hip_to_thigh   =  0.01995     #  L_thigh_joint.origin.y   (thigh joint Y offset from hip axis)
    thigh_to_calf  =  0.0383      #  L_calf_joint.origin.y    (calf joint Y offset from thigh axis)
    calf_to_wheel  =  0.02705     #  L_wheel_joint.origin.y   (wheel joint Y offset from calf axis)
    thigh_x_offset = -0.05675     #  L_thigh_joint.origin.x   (thigh axis X offset behind hip axis)
    offset         = hip_to_thigh + thigh_to_calf + calf_to_wheel   # total Y offset hip→wheel
    # Hip joint positions in the body frame (L_hip_joint and R_hip_joint origins)
    hip_local_pos = [
        [ 0.047159,  0.06125, 0.066001],   # Left hip  (leg_ind=1)
        [ 0.047159, -0.06125, 0.066001]    # Right hip (leg_ind=2)
    ]

    # Base transformations
    base_rot = quat_to_rot(x[4:7])
    base_pos = base_rot' * x[1:3]
    #base_pos = base_pos .+ [0, 0, 0.07248523];  # Adjust for any offsets if needed
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
        # The planar 2-link IK is rooted at the thigh axis, which sits at
        # x = thigh_x_offset in the hip frame.  So shift x before solving.
        x_eff = x - thigh_x_offset

        for soln = 1:2
            # Undo hip rotation to deal with planar xz calf-thigh relationship
            z_rot = sin(-leg_solns[1, soln])*y + cos(-leg_solns[1, soln])*z

            # Calf angle (law of cosines on the 2-link chain rooted at thigh axis)
            cos_calf = (thigh_length^2 + calf_length^2 - x_eff^2 - z_rot^2) / (2*thigh_length*calf_length)
            cos_calf = clamp(cos_calf, -1.0, 1.0)
            leg_solns[3, soln] = acos(cos_calf) - π

            # Thigh angle
            cos_alpha = (thigh_length^2 + x_eff^2 + z_rot^2 - calf_length^2) / (2*thigh_length*sqrt(x_eff^2 + z_rot^2 + 1e-16))
            cos_alpha = clamp(cos_alpha, -1.0, 1.0)
            alpha = acos(cos_alpha)
            phi   = atan(z_rot, x_eff)
            leg_solns[2, soln] = alpha - phi - π/2
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
function nearest_ik(model::Pineapple, q::AbstractVector, wheel_locs::AbstractVector; obey_limits = true)
    q = copy(q)
    wheel_q = inverse_kinematics(model, q, wheel_locs)
    # @show wheel_q

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
function solve_ref_IK(model::Pineapple, X::AbstractVector, wheel_ref::AbstractVector)
    N = length(X)
    for k = 2:N
        X[k][7 .+ (1:model.nu)] = X[k - 1][7 .+ (1:model.nu)]
        X[k] = nearest_ik(model, X[k], wheel_ref[k])
    end
    return X
end