# Functions that are generic to Quadruped models (specifically Unitree Go1 and Go2)
function error_jacobian(quad::Quadruped, x)
    E = zeros(length(x), length(x) - 1)
    E[1:3, 1:3] = I(3)
    E[4:7, 4:6] = 0.5*attitude_jacobian(x[4:7])
    E[8:end, 7:end] = I(length(x) - 7)
    return E
end

function error_jacobian_T(quad::Quadruped, x)
    E = zeros(length(x), length(x) - 1)
    E[1:3, 1:3] = I(3)
    E[4:7, 4:6] = 2*attitude_jacobian(x[4:7])
    E[8:end, 7:end] = I(length(x) - 7)
    return E'
end

function apply_Δx(quad::Quadruped, x_k, Δx)
    x_next = zeros(promote_type(eltype(x_k), eltype(Δx)), length(x_k))
    x_next[1:3] = x_k[1:3] + Δx[1:3]
    x_next[4:7] = L_mult(x_k[4:7])*axis_angle_to_quat(Δx[4:6])
    x_next[8:end] = x_k[8:end] + Δx[7:end]
    return x_next
end

function state_error(quad::Quadruped, x, x0)
    return [
        x[1:3] - x0[1:3]
        quat_to_axis_angle(L_mult(x0[4:7])'*x[4:7])
        x[8:end] - x0[8:end]
    ]
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
            push!(failed_joints, model.configNames[j])
        end
    end
    if !success && !supress_error
        @error "Couldn't satisfy all joint limits"
        println(failed_joints)
    end
    x[1:model.nq] = clamp.(x[1:model.nq], model.joint_limits[:, 1], model.joint_limits[:, 2])
    return x
end