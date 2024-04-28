# Functions that are generic to Quadruped models (specifically Unitree Go1 and Go2)
function error_jacobian(quad::Quadruped, x)
    E = zeros(length(x), length(x) - 1)
    E[1:3, 1:3] = I(3)
    E[4:7, 4:6] = 0.5*G(x[4:7])
    E[8:end, 7:end] = I(length(x) - 7)
    return E
end

function error_jacobian_T(quad::Quadruped, x)
    E = zeros(length(x), length(x) - 1)
    E[1:3, 1:3] = I(3)
    E[4:7, 4:6] = 2*G(x[4:7])
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