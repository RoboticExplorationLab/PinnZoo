# Quaternion functions based around/derived from axis-angle as the three parameter representation
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

function axis_angle_to_quat(ω; tol = 1e-12)
    norm_ω = norm(ω)
    
    if norm_ω >= tol
        return [cos(norm_ω/2); ω/norm_ω*sin(norm_ω/2)]
    else
        return [1; 0; 0; 0]
    end
end

quat_conjugate(q) = [q[1]; -q[2:4]]

skew(v) = [0 -v[3] v[2]; v[3] 0 -v[1]; -v[2] v[1] 0]

function L_mult(q)
    qs = q[1]
    qv = q[2:4]
    return [qs -qv'; qv qs*I + skew(qv)]
end

function R_mult(q)
    qs = q[1]
    qv = q[2:4]
    return [qs -qv'; qv qs*I - skew(qv)]
end

function attitude_jacobian(q)
    qs = q[1]
    qv = q[2:4]
    return [-qv'; qs*I + skew(qv)]
end

# Derived from p̂₊ = qp̂q† -> Hp₊ = L_mult(q)R_mult(q)'Hp
function quat_to_rot(q)
    skew_qv = skew(q[2:4])
    return 1.0I + 2*q[1]*skew_qv + 2*skew_qv^2
end