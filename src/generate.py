from symbolic_generator import SymbolicGenerator

def generate_c_functions(urdf_path, floating = True):
    
    # Load the URDF model
    if floating:
        robot = RobotWrapper.BuildFromURDF(urdf_path, root_joint = pin.JointModelFreeFlyer())
    else:
        robot = RobotWrapper.BuildFromURDF(urdf_path)

    # Get dimensions
    nq = robot.model.nq #- 1 # Don't include Pinocchio's "universe" joint
    nv = robot.model.nv

    # Print statistics
    print("Loaded robot model from:", urdf_path)
    print("\t# of configs =", nq)
    print("\t# of DoFs =", nv)
    print([robot.model.frames[i].name for i in range(robot.model.nframes)])

    # Create symbolic model
    cmodel = cpin.Model(robot.model)
    cdata = cmodel.createData()

    # Define the state vector as a CasADi symbol
    x = cs.SX.sym('x', cmodel.nq + cmodel.nv)
    q = x[:cmodel.nq]
    v = x[cmodel.nq:]

    lst= []
    while True:
        print(f"Current list: {lst}")
        lst.append(input("Enter new element: "))  
        sys.stdout.write("\033[K") # Clear to the end of line
        sys.stdout.write("\033[F") # Cursor up one line
        sys.stdout.write("\033[K") # Clear to the end of line
        sys.stdout.write("\033[F") # Cursor up one line


    print([name for name in robot.model.names[1:robot.model.nq+1]])


    # Make symbolic models
    cmodel = cpin.Model(robot.model)
    cdata = cmodel.createData()

    # Define the state vector as a CasADi symbol
    x = cs.SX.sym('x', cmodel.nq + cmodel.nv)
    q = x[:cmodel.nq]
    v = x[cmodel.nq:]

    # Compute the mass matrix using Pinocchio
    M = cpin.crba(cmodel, cdata, q)

    # Compute the inverse dynamics using Pinocchio
    # tau = cpin.rnea(cmodel, cdata, q, v)

    # Compute the Jacobian of the inverse dynamics using CasADi
    # J = cs.jacobian(tau, x)

    # print(M)
    print(M.is_dense())

    tau = cpin.rnea(cmodel, cdata, q, v, v)
    print(tau.is_dense())

    J = cs.densify(cs.jacobian(tau, x))
    print(J.is_dense())
    print(M.size())


    # Create CasADi functions
    mass_matrix_func = cs.Function('mass_matrix', [q], [M], ['q'], ['M'])
    # # inverse_dynamics_func = cs.Function('inverse_dynamics', [x], [tau], ['x'], ['tau'])
    # # jacobian_func = cs.Function('jacobian', [x], [J], ['x'], ['J'])

    mass_matrix_func.generate('f.c')

    # # Generate C code
    # c_code_mass_matrix = cs.ccode(mass_matrix_func)
    # # c_code_inverse_dynamics = cs.ccode(inverse_dynamics_func)
    # # c_code_jacobian = cs.ccode(jacobian_func)

    # # Write the C code to files
    # with open('mass_matrix.c', 'w') as f:
    #     f.write(c_code_mass_matrix)
    # # with open('inverse_dynamics.c', 'w') as f:
    #     # f.write(c_code_inverse_dynamics)
    # # with open('jacobian.c', 'w') as f:
    #     # f.write(c_code_jacobian)

    # print("C functions have been written to 'mass_matrix.c', 'inverse_dynamics.c', and 'jacobian.c'")

# Call the function with the path to your URDF file
symb_gen = SymbolicGenerator('../cartpole.urdf', ".", ['pole_tip'], floating = False)

# symb_gen = SymbolicGenerator('../go1.urdf', ".",
#                              mesh_dir = ".",
#                              kinematics_bodies= ["FL_foot", "FR_foot", "RL_foot", "RR_foot"], 
#                              floating = True)

symb_gen.generate()
