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

Refer to the docs (currently under docs/build/index.html) for detials on the generated functions. While we wrap them in Julia, there are many ways you can use these functions, such as linking them into your own C or C++ project, or calling them from Python. An example of using the Julia wrapper is in the Get Started section below.

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
x_dot = dynamics(model, x, u) # adds q_dot = E(q)v_dot to forward_dynamics
u_new = inverse_dynamics(model, x, v_dot) # Solves for torques using manipulator equation (RNEA)

locs = kinematics(model, x) # For cartpole, location of the pole tip in the world frame
J = kinematics_jacobian(model, x) # Jacobian of the pole tip w.r.t the state vector
```
Check out the documentation (currently under docs/build/index.html) for details on conventions and the functions available

# Helpful info

### Adding a model
To add a model, do the following:
- Create a folder under models with the model name, with the urdf, a generate.py file and a <name>.jl file. Copy models/pendulum for a basic model,
  but make the parent type PinnZooFloatingBaseModel if it is a floating base (look at unitree_go1, unitree_go2 or Nadia for more). If curious, look at src/model_macro.jl to see what the macro is doing, it mainly adds a bunch of C pointers to the generated code to your struct, as well as
  fetching some model info.
- Run your generate.py file to generate the code
- Modify CMakeLists.txt to add your shared library
- cd into build and run `cmake ..` and `cmake --build . --target <model_name>`
- Modify src/PinnZoo.jl to include your model and export it
- Modify test/runtests.jl to include your model
- Run `julia -t auto test/runtests.jl` and make sure your model passes

### Regenerating code
You can run the following in the PinnZoo directory to re-generate all the generated code if a change is made to symbolic_generator.py
`find models -type f -name generate.py -exec python {} \;`
Make sure to run the PinnZoo test suite after regenerating.

### Generating models
You can convert from urdf to mjcf using MuJoCo in more recent versions by loading the model and using mj_saveLastXML.

# TODO
- Generalize error_state and apply_Δx functions for any model with a quaternion in the state
- Fix velocity_kinematics, kinematics_velocity, kinematics_velocity_jacobian tests
- Fix forward_dynamics_deriv (failing tests)