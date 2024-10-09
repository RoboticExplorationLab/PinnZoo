# PinnZoo (WIP)

PinnZoo contains fast dependency-free C code for dynamics and kinematics functions for various robots (defined by URDFs) generated using Pinocchio and CasADI, along with a wrapper to generate a shared library and call the code from Julia. 

*Note: You do not need to install Pinocchio or CasADI to use the models, the models are dependency free. You only need them to generate a new model.*

Models can be found in the models directory. Each model folder should include the following:
- the URDF
- generate.py (file used to generate the C code)
- a generated_code directory where the C code is
- a <model_name>.jl file which wraps the C code so it can be called from Julia
- a README that provides basic model details, such as state vector order, bodies that kinematics were generated for, and anything else that may need clarification.
- any additional C or Julia files for model specific functions/behaviors (should be documented in README)

Refer to the Generated Code Conventions section below to see what dynamics and kinematics are included for each model. There are many ways you can use these functions, either linking them into your own C or C++ project, or calling them from Python or Julia. We currently provide a Julia wrapper for each model, which you can see how to use in the Get Started section below.

# Get Started (Julia)
First, clone the repository, and run the following commands in the terminal. You will need CMake and C compiler to be installed.
```
cd PinnZoo
mkdir build
cd build
cmake ..
cmake --build .
```

*Note: If you'd only like to compile code for a certain model instead of all possible models, use `cmake --build . --target <model_name>`.*

Once you've finished compiling, you can install PinnZoo in your Julia environment with the following commands
```
using Pkg
Pkg.develop(path="<path to PinnZoo folder>")
```

Here are some examples of basic usage:
```
using PinnZoo
model = Cartpole()
x = randn_state(model) # Can also use zero_state(model)
u = randn(model.nv) # All DoFs are actuated by default

M = M_func(model, x) # Mass matrix
C = C_func(model, x) # Bias/nonlinear term
v_dot = forward_dynamics(model, x, u) # Solves for accelerations using manipulator equation (ABA)
u_new = inverse_dynamics(model, x, v_dot) # Solves for torques using manipulator equation (RNEA)

locs = kinematics(model, x) # For cartpole, location of the pole tip in the world frame
J = kinematics_jacobian(model, x) # Jacobian of the pole tip w.r.t the state vector
```

# Supported Models
Below is a list of models that we have generate code for
- cartpole
- go1 (currently failing tests due to state order mismatch with RigidBodyDynamics.jl)
- go2 (currently failing tests due to state order mismatch with RigidBodyDynamics.jl)

# Dependencies
To compile and use the generate code, you'll need to install CMake and a C++ compiler such as GCC or Clang.

To generate code for a new model, please reach out to Arun Bishop. You'll need to install Pinocchio and CasADI. At the time of writing this, Pinocchio v3 has not been released (April 2024), so you'll need to install it
from a specific conda channel. You can install casadi using from condaforge.

```
conda install -c olivier.roussel pinocchio=2.99
conda install casadi
```

# Generated Code Conventions
By default each model should include the following set of functions defined on the following inputs. For some models (such as the quadruped), there are additional functions, which should be listed in a README.md file in that models folder.

### Conventions
- Quaternions use [q_w; q_x; q_y; q_z] order and represent body to world rotations
- Floating base joints use [x; y; z; q_w; q_x; q_y; q_z] order where the position is in the world frame
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

$$M(x)v + C(x) = tau$$
$$\dot{q} = E(q)v$$

$E(q)$ is typically the identity except in the following cases (not an exhaustive list):
- The configuration vector includes a quaternion, so E(q) includes the mapping from angular velocity into a quaternion time derivative.
- Translational velocity of a floating body is in the body frame, but the position is in the world frame, so E(q) includes a body-to-world rotation matrix.

### Dynamics Functions
- M = M_func(x) $\rightarrow$ returns nv x nv mass matrix
- C = C_func(x) $\rightarrow$ returns nv x 1 dynamics bias (centrifugal, coriolis, and gravity terms)
- x_dot = forward_dyn(x, tau) $\rightarrow$ solves for x_dot using the manipulator equation and velocity kinematics.
- tau = inverse_dyn(x, x_dot) $\rightarrow$ solves for tau using the manipulator equation (the q_dot part of x_dot is unused)
- forward_dyn_derivatives(x, tau) $\rightarrow$ returns two nx x nv matrices corresponding to the derivative of x_dot with respect to x and tau.
- inverse_dyn_derivatives(x, x_dot) $\rightarrow$ returns two nv x nx matrices corresponding to the derivative of tau with respect to x and x_dot. Since tau doesn't depend on the first nq elements of x_dot, the corresponding derivatives will be zero.

### Kinematics Functions
When generating code you can specify a set of bodies in the URDF to generate the kinematics for. By default this will produce a list of world positions for each body in the order specified when generating the code. This order should be listed in the README.md file for the model
- kinematics(x) $\rightarrow$ returns a vector of world positions for points on the robot specified during code generation
- kinematics_jacobian(x) $\rightarrow$ jacobian of the kinematics function with respect to x (the velocity portion will be zero by default).
- kinematics_velocity(x) $\rightarrow$ returns the velocity in the world frame for points on the robot specified during code generation
- kinematics_velocity_jacobian(x) $\rightarrow$ jacobian of the kinematics_velocity function with respect to x. The first nq columns will be the same as the kinematics_jacobian. The last nv columns will be the same as the time derivative of the kinematics_jacobian.

### Regenerating code
You can run the following in the PinnZoo directory to re-generate all the generated code if a change is made to symbolic_generator.py
`find models -type f -name generate.py -exec python {} \;`
# TODO
- Generalize error_state and apply_Δx functions for any model with a quaternion in the state
- Fix tests for quadruped (more general solution for joint order mismatches between RigidBodyDynamics.jl and Pinocchio)