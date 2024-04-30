# Functions that are generic to Quadruped models (specifically Unitree Go1 and Go2)
function error_jacobian(quad::Quadruped, x)

    return [
        velocity_kinematics(quad, x) zeros(quad.nq, length(x) - quad.nq)
        zeros(length(x) - quad.nq, quad.nv) I(length(x) - quad.nq)
    ]
end

function error_jacobian_T(quad::Quadruped, x)
    return [
        velocity_kinematics_T(quad, x) zeros(quad.nv, length(x) - quad.nq)
        zeros(length(x) - quad.nq, quad.nq) I(length(x) - quad.nq)
    ]
end

function apply_Δx(quad::Quadruped, x_k, Δx)
    x_next = zeros(promote_type(eltype(x_k), eltype(Δx)), length(x_k))
    x_next[1:3] = x_k[1:3] + quat_to_rot(x_k[4:7])*Δx[1:3]
    x_next[4:7] = L_mult(x_k[4:7])*axis_angle_to_quat(Δx[4:6])
    x_next[8:end] = x_k[8:end] + Δx[7:end]
    return x_next
end

function state_error(quad::Quadruped, x, x0)
    return [
        quat_to_rot(x0[4:7])'*(x[1:3] - x0[1:3])
        quat_to_axis_angle(L_mult(x0[4:7])'*x[4:7])
        x[8:end] - x0[8:end]
    ]
end

# Input jacobian
function B_func(quad::Quadruped)
    return [zeros(6, 12); I(12)]
end

# Attempt to wrap joints into joint limits
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
            push!(failed_joints, model.state_order[j])
        end
    end
    if !success && !supress_error
        @error "Couldn't satisfy all joint limits"
        println(failed_joints)
    end
    x[1:model.nq] = clamp.(x[1:model.nq], model.joint_limits[:, 1], model.joint_limits[:, 2])
    return x
end

# Solve for ik nearest to the given reference
function nearest_ik(model::Quadruped, q, foot_locs)
    q = copy(q)
    foot_q = inverse_kinematics(model, q, foot_locs)

    # Build both options
    q1 = copy(q)
    q1[7 .+ (1:12)] = foot_q[:, 1]
    q2 = copy(q)
    q2[7 .+ (1:12)] = foot_q[:, 2]

    # Fix joint limits (don't let these error since some will probably be wrong)
    q1 = fix_joint_limits(model, q1, supress_error = true)
    q2 = fix_joint_limits(model, q2, supress_error = true)

    # Build final ik leg by leg
    for i = 1:4
        foot_inds = 7 + 3*(i - 1) .+ (1:3)
        if norm(q[foot_inds] - q1[foot_inds]) < norm(q[foot_inds] - q2[foot_inds])
            q[foot_inds] = q1[foot_inds]
        else
            q[foot_inds] = q2[foot_inds]
        end
    end
    return fix_joint_limits(model, q) # Let this one error
end