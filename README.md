## Python Install
At the time of writing this Pinocchio v3 has not been released (April 2024), so you'll need to install it
from a specific conda channel. You can install casadi using from condaforge.

```
conda install -c olivier.roussel pinocchio=2.99
conda install casadi
```

# PinnZoo (WIP)

PinnZoo contains fast dependency-free C code for the dynamics and kinematics for various robots (defined by URDFs) generated using Pinocchio and CasADI, along with a wrapper to generate a shared library and call the code from Julia. 

*Note: You do not need to install Pinocchio, Eigen, or CasADI to use the models, they are dependency free. You only need them to generate a new model.*

Models can be found in the models directory. Each model folder should include the following:
- the URDF
- generate.cpp (to load the URDF and generate dynamics)
- a gen_code directory with the generated code
- a CMakeLists.txt to compile the generated code into a shared library
- a Julia file that defines a wrapper that can be used to call the code from Julia

*Note: The generated files are sometimes large for large state dimension, so compiling for the first time can take minutes*

To use a robot model, either include this repository as a submodule or copy the revelant model folder to your repository. If using the Julia wrapper, make sure to compile the target in the CMakeLists.txt. You may need to edit paths in the Julia wrapper so that it can find wherever you compile the shared library to.

If you'd like to add a model, please reach out to arunleob@cmu.edu for now. Instructions for how to do this are a work in progress. Finally, this was thrown together very quickly and is not using the most optimized way to generate these files (for example, I currently force everything to be dense). Suggestions and assistance on how to improve this are more than welcome!

# Get Started
TODO Usage example

*Note: Please read the following section which details the dynamics that are generated and the conventions used.*

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

### Utilities
- E(x) $\rightarrow$ 



# Dependencies
*This is only needed for generating models, not for using them*

To generate a model you need to have CMake, Eigen, Pinocchio, and CasADI installed. The following instructions for Pinocchio and CasADI were tested on Ubuntu 20.04 LTS, but may need to be modified to account for difference Python versions for example.

### Install Pinocchio
Install dependencies
```
sudo apt install -qqy lsb-release gnupg2 curl
```
Add robotpkg as source repository to apt
```
echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list
```
Register the authentication certificate of robotpkg
```
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
```
Run apt-get update to fetch the package descriptions
```
sudo apt-get update
```
Install Pinocchio
```
sudo apt install -qqy robotpkg-py38-pinocchio 
```
If you're using Jammy, you can try robotpgk-py310-pinocchio.

Finally, setup environment variables (copy the following lines into ~/.bashrc to have the env variables persist between sessions).
```
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.8/site-packages:$PYTHONPATH # Adapt your desired python version here
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
```
These instructions were copied from [Pinocchio's Documentation](https://stack-of-tasks.github.io/pinocchio/download.html) 
on February 8th, 2023 for a specific Python version (3.8). Installation instructions may have changed since then.

### Install CasADI
These instructions were copied from [CasADI's Documentation](https://github.com/casadi/casadi/wiki/InstallationLinux) on June 18th, 2023.
```
sudo apt-get install gcc g++ gfortran git cmake liblapack-dev pkg-config --install-recommends -y
sudo apt-get install coinor-libipopt-dev -y # Install IPOPT (optional)
git clone https://github.com/casadi/casadi.git -b main casadi
cd casadi && git checkout 3.6.3
mkdir build && cd build
cmake ..
make
sudo make install
```


