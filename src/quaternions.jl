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
    rotmat_to_quat(R) -> q

Convert a 3×3 rotation matrix `R` ∈ SO(3) to a unit quaternion `q = (w,x,y,z)`
(scalar-first). Uses a trace-based, numerically stable branch; output is
normalized and sign-canonicalized (`w ≥ 0`).

**Conventions:** column-vector math (R maps basis vectors as columns).  
**Args:** `R::AbstractMatrix{<:Real}` of size (3,3).  
**Returns:** `Vector{Float64}` length 4, `(w,x,y,z)`.

**Example**
```julia
q = rotmat_to_quat(Matrix(I,3,3))  # -> [1.0, 0.0, 0.0, 0.0]

"""

function rotmat_to_quat(R::AbstractMatrix{<:Real})
    @assert size(R) == (3,3)
    R00, R01, R02 = R[1,1], R[1,2], R[1,3]
    R10, R11, R12 = R[2,1], R[2,2], R[2,3]
    R20, R21, R22 = R[3,1], R[3,2], R[3,3]

    t = R00 + R11 + R22
    if t > 0
        S = 2 * sqrt(t + 1)
        w = 0.25 * S
        x = (R21 - R12) / S
        y = (R02 - R20) / S
        z = (R10 - R01) / S
    elseif (R00 > R11) && (R00 > R22)
        S = 2 * sqrt(1 + R00 - R11 - R22)
        w = (R21 - R12) / S
        x = 0.25 * S
        y = (R01 + R10) / S
        z = (R02 + R20) / S
    elseif R11 > R22
        S = 2 * sqrt(1 + R11 - R00 - R22)
        w = (R02 - R20) / S
        x = (R01 + R10) / S
        y = 0.25 * S
        z = (R12 + R21) / S
    else
        S = 2 * sqrt(1 + R22 - R00 - R11)
        w = (R10 - R01) / S
        x = (R02 + R20) / S
        y = (R12 + R21) / S
        z = 0.25 * S
    end

    q = [w, x, y, z]
    q ./= sqrt(sum(abs2, q))  # normalize
    if q[1] < 0
        q .*= -1
    end
    return q  # (w, x, y, z)
end



