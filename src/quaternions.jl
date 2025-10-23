# Quaternion functions based around/derived from axis-angle as the three parameter representation

@doc raw"""
    quat_to_axis_angle(q; tol = 1e-12)

Return the axis angle corresponding to the provided quaternion
"""
function quat_to_axis_angle(q; tol = 1e-12)
    qs = q[1]
    qv = q[2:4]
    norm_qv = norm(qv)

    if norm_qv >= tol
        θ = 2*atan(norm_qv, qs)
        return θ*qv/norm_qv
    else
        return 2*qv # Makes sure ForwardDiff works when q is close to/at [1; 0; 0; 0]
    end
end

@doc raw"""
    axis_angle_to_quat(ω; tol = 1e-12)

Return the quaternion corresponding to the provided axis angle
"""
function axis_angle_to_quat(ω; tol = 1e-12)
    norm_ω = norm(ω)
    return [cos(norm_ω/2); ω*0.5*sinc(norm_ω/π/2)] # sinc(x) = sin(πx)/(πx) 
end

@doc raw"""
    quat_conjugate(q)

Return the conjugate of the given quaternion (negates the velocity part)
"""
quat_conjugate(q) = [q[1]; -q[2:4]]

@doc raw"""
    skew(v)

Return a matrix M such that $$v \times x = Mx$$ where $$\times$$ denotes the cross product
"""
skew(v) = [0 -v[3] v[2]; v[3] 0 -v[1]; -v[2] v[1] 0]

@doc raw"""
    L_mult(q)

Return a matrix representation of left quaternion multiplication, i.e. $$q1 \cdot q2 = L(q1)q2$$
where $$\cdot$$ is quaternion multiplication.
"""
function L_mult(q)
    qs = q[1]
    qv = q[2:4]
    return [qs -qv'; qv qs*I + skew(qv)]
end

@doc raw"""
    R_mult(q)

Return a matrix representation of right quaternion multiplication, i.e. $$q1 \cdot q2 = R(q2)q1$$
where $$\cdot$$ is quaternion multiplication.
"""
function R_mult(q)
    qs = q[1]
    qv = q[2:4]
    return [qs -qv'; qv qs*I - skew(qv)]
end

@doc raw"""
    attitude_jacobian(q)

Return the attitude jacobian G define as $$\dot{q} = 0.5G\omega$$, mapping angular velocity
into quaternion time derivative.
"""
function attitude_jacobian(q)
    qs = q[1]
    qv = q[2:4]
    return [-qv'; qs*I + skew(qv)]
end

@doc raw"""
    quat_to_rot(q)

Return a rotation matrix R that q represents, defined by $$\hat{p}^+ = q\hat{p}q^\dagger$$ where $$\hat{p}$$
turns $$p$$ into a quaternion with zero scalar part and $$p$$ as the vector part, and $$^\dagger$$ is the quaternion conjugate.
"""
function quat_to_rot(q)
    skew_qv = skew(q[2:4])
    return 1.0I + 2*q[1]*skew_qv + 2*skew_qv^2
end

@doc raw"""
    rot_to_quat(R)

Convert a 3x3 rotation matrix `R` ∈ SO(3) to a unit quaternion.
Returns a 4-element vector [qw, qx, qy, qz].
"""
function rot_to_quat(R)
    tr = R[1,1] + R[2,2] + R[3,3]

    if tr > 0
        S = sqrt(tr + 1.0) * 2  # S = 4 * qw
        qw = 0.25 * S
        qx = (R[3,2] - R[2,3]) / S
        qy = (R[1,3] - R[3,1]) / S
        qz = (R[2,1] - R[1,2]) / S
    elseif R[1,1] > R[2,2] && R[1,1] > R[3,3]
        S = sqrt(1.0 + R[1,1] - R[2,2] - R[3,3]) * 2
        qw = (R[3,2] - R[2,3]) / S
        qx = 0.25 * S
        qy = (R[1,2] + R[2,1]) / S
        qz = (R[1,3] + R[3,1]) / S
    elseif R[2,2] > R[3,3]
        S = sqrt(1.0 + R[2,2] - R[1,1] - R[3,3]) * 2
        qw = (R[1,3] - R[3,1]) / S
        qx = (R[1,2] + R[2,1]) / S
        qy = 0.25 * S
        qz = (R[2,3] + R[3,2]) / S
    else
        S = sqrt(1.0 + R[3,3] - R[1,1] - R[2,2]) * 2
        qw = (R[2,1] - R[1,2]) / S
        qx = (R[1,3] + R[3,1]) / S
        qy = (R[2,3] + R[3,2]) / S
        qz = 0.25 * S
    end

    return normalize([qw, qx, qy, qz])
end
