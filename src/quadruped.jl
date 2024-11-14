# Functions that are generic to Quadruped models (specifically Unitree Go1 and Go2)
abstract type Quadruped <: PinnZooFloatingBaseModel end

@doc raw"""
    B_func(quad::Quadruped)

Return the input jacobian mapping motor torques into joint torques
"""
function B_func(quad::Quadruped)
    return [zeros(6, 12); I(12)]
end

@doc raw"""
    fix_joint_limits(model::Quadruped, x; supress_error = false)

Return x with joint angles wrapped to 2pi to fit within joint limits if possible. If not
and supress_error is false, this will error. Otherwise, this will return a clamped version.
"""
function fix_joint_limits(model::Quadruped, x; supress_error = false)
    x = copy(x)
    success = true
    failed_joints = []
    for j = 8:19
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
    nearest_ik(model::Quadruped, q, foot_locs; obey_limits = true)

Return the ik solution for the given foot_locs and body orientation in q that is closest to the current
joint angles in q (defined by minimum norm per joint).
"""
function nearest_ik(model::Quadruped, q, foot_locs; obey_limits = true)
    q = copy(q)
    foot_q = inverse_kinematics(model, q, foot_locs)

    # Build both options
    q1 = copy(q)
    q1[7 .+ (1:12)] = foot_q[:, 1]
    q2 = copy(q)
    q2[7 .+ (1:12)] = foot_q[:, 2]


    # Fix joint limits (don't let these error since some will probably be wrong)
    if obey_limits
        q1 = fix_joint_limits(model, q1, supress_error = true)
        q2 = fix_joint_limits(model, q2, supress_error = true)
    end

    # Build final ik leg by leg
    for i = 1:4
        foot_inds = 7 + 3*(i - 1) .+ (1:3)
        if norm(q[foot_inds] - q1[foot_inds]) < norm(q[foot_inds] - q2[foot_inds])
            q[foot_inds] = q1[foot_inds]
        else
            q[foot_inds] = q2[foot_inds]
        end
    end

    # Final fix (let this one error if we can't obey the limits)
    if obey_limits
        return fix_joint_limits(model, q)
    else
        return q
    end
end