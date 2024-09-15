# PinnZoo.jl Documentation

```@contents
```

### Conventions
- Quaternions use \[$$q_w$$ $$q_x$$ $$q_y$$ $$q_z$$\] order and represent body to world rotations, using Hamilton's convention $$i^2 = j^2 = k^2 = -1$$
- Floating base joints use \[$$x$$ $$y$$ $$z$$ $$q_w$$ $$q_x$$ $$q_y$$ $$q_z$$\]$ order where the position is in the world frame
- Linear and angular velocities corresponding to a floating base joint are in the body frame

### Inputs/Variables
- nq $\rightarrow$ # of configuration variables (1 per revolute/prismatic joint, 7 for floating joint)
- nv $\rightarrow$ # of velocity variables. Also the number of degrees of freedom (1 per revolute/prismatic joint, 6 for floating joint)
- nx $\rightarrow$ # of states, nq + nv
- nc $\rightarrow$ # of points on the robot that kinematics were generated for
- q $\rightarrow$ configuration vector, size nq.
- x $\rightarrow$ state vector, size nx. The first nq elements are the configuration and the last nv elements are velocity.
- x_dot $\rightarrow$ state vector derivative, size nx. The first nq elements are the derivative of the configuration with respect to time, and the last nv are the acceleration (derivative of velocity). 
- tau $\rightarrow$ generalized force vector, size nv. Represents forces/torques at each degree of freedom.

For the dynamics, we assume the following manipulator equation and velocity kinematics:

$$M(x)\dot{v} + C(x) = \tau$$
$$\dot{q} = E(q)v$$

### Velocity Kinematics
E(q) is typically the identity except in the following cases (not an exhaustive list):

When the configuration includes a quaternion, E(q) includes the mapping from angular velocity into a quaternion time derivative, respecting $$\dot{q}q = 0$$, the unit norm
constraint at the velocity level. Because angular velocities are related to axis-angles, there is a factor of 2 that shows up, so the mapping back from $$v$$ to $$\dot{q}$$ which
we refer to as $$E\_T$$ is not equal to $$E^T$$.

When the translation velocity of a floating is in the body frame, but the position is in the world frame, so E(q) includes a body-to-world rotation matrix.

## Index

```@index
```

## Dynamics Functions
```@docs
M_func
C_func
forward_dynamics
forward_dynamics_deriv
inverse_dynamics
inverse_dynamics_deriv
```

## Kinematics Functions
```@docs
kinematics
kinematics_jacobian
kinematics_velocity
kinematics_velocity_jacobian
```

## Utility Functions
```@docs
is_floating
zero_state
randn_state
init_state
```

## Quaternion functions
```@docs
quat_to_axis_angle
axis_angle_to_quat
quat_conjugate
skew
L_mult
R_mult
attitude_jacobian
quat_to_rot
```