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
        return zeros(3)
    end
end

@doc raw"""
    axis_angle_to_quat(ω; tol = 1e-12)

Return the quaternion corresponding to the provided axis angle
"""
function axis_angle_to_quat(ω; tol = 1e-12)
    norm_ω = norm(ω)
    
    if norm_ω >= tol
        return [cos(norm_ω/2); ω/norm_ω*sin(norm_ω/2)]
    else
        return [1; 0; 0; 0]
    end
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