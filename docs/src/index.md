# PinnZoo.jl

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

## Order Conversion Functions

The functions below can be used as helpers to convert between different vector orders to help interfacing with different dynamics packages.
For example, to use a configuration vector from this package in mujoco, you can do
```
q_mujoco = change_order(model, q_pinnzo, :nominal, :mujoco)
```

```@docs
StateOrder
ConversionIndices
change_order!
change_order
change_orders!
change_orders
generate_conversions
```

## Dynamics Functions

```@docs
M_func
M_jvp
C_func
dynamics
dynamics_deriv
forward_dynamics
forward_dynamics_deriv
inverse_dynamics
inverse_dynamics_deriv
inverse_dynamics_hvp
inverse_dynamics_hTvp
velocity_kinematics
velocity_kinematics_T
velocity_kinematics_jvp_deriv
velocity_kinematics_T_jvp_deriv
state_error
apply_Δx
error_jacobian
error_jacobian_T
error_jacobian_jvp_deriv
error_jacobian_T_jvp_deriv
```

## Kinematics Functions

```@docs
kinematics
kinematics_rotation
kinematics_jacobian
kinematics_hvp
kinematics_velocity
kinematics_velocity_jacobian
kinematics_force_jacobian
kinematics_jacobianTvp
kinematics_force_hvp
kinematics_force_hTvp
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
rot_to_quat
```

## Model specific functions

TODO: Generalize state error and related functions to all models
```@docs
B_func
fix_joint_limits
inverse_kinematics
nearest_ik
```

# Models

### Pendulum

```@docs
Pendulum
```

### Double Pendulum

```@docs
DoublePendulum
```

### Cartpole

```@docs
Cartpole
```

### Double Cartpole

```@docs
DoubleCartpole
```

### RigidBody

```@docs
RigidBody
```

### Quadrotor

```@docs
Quadrotor
```

### Unitree Go1

```@docs
Go1
init_state(model::Go1)
inverse_kinematics(model::Go1, x, foot_locs)
```

### Unitree Go2

```@docs
Go2
init_state(model::Go2)
inverse_kinematics(model::Go2, x, foot_locs)
```

### IHMC Nadia

```@docs
Nadia
```

### Pineapple

```@docs
Pineapple
```

# ForwardDiff.jl Compatability

We have made many of the PinnZoo functions compatible with ForwardDiff.jl to enable building more complex constraints and objectives in Julia on
top of Pinocchio/PinnZoo without having to derive custom derivatives or using slow finite differences. To support this, sometimes certain calls
need to be used to trigger the appropriate derivatives. 

### Functions that support ForwardDiff.jacobian

- [`kinematics`](@ref)  
- [`kinematics_rotation`](@ref)  
- [`kinematics_velocity`](@ref)  
- [`kinematics_jacobianTvp`](@ref) - use this for J(x)'λ (mapping kinematics forces into generalized forces)  
- [`forward_dynamics`](@ref)  
- [`dynamics`](@ref)  
- [`inverse_dynamics`](@ref)  

### Functions that support ForwardDiff.hessian

You can use ForwardDiff.hessian on scalar functions (for example, Lagrangians in optimization) that use any of the following internally.

- [`kinematics`](@ref)  
- [`kinematics_rotation`](@ref)  
- [`kinematics_jacobianTvp`](@ref) - use this for J(x)'λ (mapping kinematics forces into generalized forces)  
- [`inverse_dynamics`](@ref)  

# Benchmarking
Generated using `tests/benchmarks/jl` with the [`Go2`](@ref) model
### Dynamics
| Function | Median | Q1 – Q3 |
|----------|--------|-----------|
| [`M_func`](@ref) | 1.07 μs | (1.05–1.13) μs |
| [`M_jvp`](@ref) | 2.75 μs | (2.65–2.93) μs |
| [`C_func`](@ref) | 0.879 μs | (0.826–0.995) μs |
| [`dynamics`](@ref) | 2.18 μs | (2–2.38) μs |
| [`dynamics_deriv`](@ref) | 28.9 μs | (27.6–31.9) μs |
| [`forward_dynamics`](@ref) | 2.34 μs | (2.13–2.47) μs |
| [`forward_dynamics_deriv`](@ref) | 30.8 μs | (28–33.3) μs |
| [`inverse_dynamics`](@ref) | 1.03 μs | (0.902–1.12) μs |
| [`inverse_dynamics_deriv`](@ref) | 10.3 μs | (9.96–10.8) μs |
| [`inverse_dynamics_hvp`](@ref) | 24.8 μs | (23–27.5) μs |
| [`inverse_dynamics_hTvp`](@ref) | 20.1 μs | (19.3–21.6) μs |

### Kinematics
| Function | Median | Q1 – Q3 |
|----------|--------|-----------|
| [`kinematics`](@ref) | 0.201 μs | (0.193–0.218) μs |
| [`kinematics_jacobian`](@ref) | 1.14 μs | (1.11–1.2) μs |
| [`kinematics_hvp`](@ref) | 2.01 μs | (1.95–2.15) μs |
| [`kinematics_velocity`](@ref) | 0.458 μs | (0.412–0.499) μs |
| [`kinematics_velocity_jacobian`](@ref) | 2.68 μs | (2.53–3.04) μs |
| [`kinematics_force_jacobian`](@ref) | 2.29 μs | (2.23–2.41) μs |
| [`kinematics_jacobianTvp`](@ref) | 3.17 μs | (2.95–3.52) μs |
| [`kinematics_force_hvp`](@ref) | 6.03 μs | (5.73–6.55) μs |
| [`kinematics_force_hTvp`](@ref) | 7.94 μs | (7.29–8.62) μs |

### ForwardDiff Jacobians
Currently slow due to allocations
| Function | w.r.t | Median | Q1 – Q3 |
|----------|----|--------|-----------|
| [`kinematics`](@ref) | x | 23 μs | (21.6–24.4) μs |
| [`kinematics_velocity`](@ref) | x | 29.5 μs | (28.4–32.5) μs |
| [`kinematics_jacobianTvp`](@ref) | x | 39.3 μs | (38.2–41.7) μs |
| [`kinematics_jacobianTvp`](@ref) | λ | 8.55 μs | (8.31–9.24) μs |
| [`dynamics`](@ref) | x | 0.159 ms | (0.152–0.171) ms |
| [`dynamics`](@ref) | τ | 87 μs | (79.2–100) μs |
| [`forward_dynamics`](@ref) | x | 0.148 ms | (0.142–0.16) ms |
| [`forward_dynamics`](@ref) | v̇ | 71.1 μs | (68.1–75.1) μs |
| [`inverse_dynamics`](@ref) | x | 66.5 μs | (64–71.8) μs |
| [`inverse_dynamics`](@ref) | τ | 32.4 μs | (30.8–36.1) μs |

### ForwardDiff Hessians
| Function | w.r.t | Median | Q1 – Q3 |
|----------|----|--------|-----------|
| [`kinematics`](@ref) | x | 7.07 ms | (6.55–8.35) ms |
| [`kinematics_jacobianTvp`](@ref) | x | 16.5 ms | (15.3–17.2) ms |
| [`kinematics_jacobianTvp`](@ref) | λ | 0.272 ms | (0.259–0.291) ms |
| [`inverse_dynamics`](@ref) | x | 52.2 ms | (50.9–53.5) ms |
| [`inverse_dynamics`](@ref) | τ | 0.974 ms | (0.936–1.05) ms |
