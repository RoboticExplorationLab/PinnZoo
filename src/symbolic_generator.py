
import pinocchio as pin
import pinocchio.casadi as cpin
import casadi as cs
from pinocchio.robot_wrapper import RobotWrapper
import sys, os
import numpy as np

class SymbolicGenerator:
    def __init__(self, urdf_path, gen_dir = "./generated_code", 
                 kinematics_bodies = [], floating = False, mesh_dir=".", actuated_dofs = None):
        # Load the URDF model
        if floating:
            self.robot = RobotWrapper.BuildFromURDF(urdf_path, mesh_dir, root_joint = pin.JointModelFreeFlyer())
        else:
            self.robot = RobotWrapper.BuildFromURDF(urdf_path, mesh_dir)
        self.model = self.robot.model
        self.data = self.model.createData()

        # Get dimensions
        self.nq = self.robot.model.nq
        self.nv = self.robot.model.nv
        self.nx = self.nq + self.nv
        self.bodies = [self.model.frames[i].name for i in range(2, self.model.nframes)]

         # Set up symbolic model
        self.cmodel = cpin.Model(self.model)
        self.cdata = self.cmodel.createData()

        # Set up symbolic variables
        self.x = cs.SX.sym('x', self.nx)
        self.x_dot = cs.SX.sym('x_dot', self.nx)
        self.tau = cs.SX.sym('tau', self.nv)

        # Fix the order of any quaternion variables to agree with our conventions (w, x, y, z) not (x, y, z, w)
        # and define the state vector order
        self.define_state_order()

        # Define actuators
        if actuated_dofs is None:
            self.torque_order = self.state_order[self.nq:]
        else:
            self.torque_order = self.state_order[self.nq:][actuated_dofs]

        # Create directory to save files to if it doesn't exist
        self.gen_dir = os.path.abspath(gen_dir)
        if not os.path.exists(self.gen_dir):
            os.makedirs(self.gen_dir)

        # Print model parameters
        print("Loaded robot model from:", urdf_path)
        print("----------- Model Details -----------")
        print("\tFloating base =", "true" if floating else "false")
        print("\t# of configs =", self.nq)
        print("\t# of DoFs =", self.nv)
        print("\n\tBodies:",self.bodies)
        print("\n\tJoints:",[name for name in self.model.names])
        if len(kinematics_bodies) > 0:
            print("\n\tKinematics:",kinematics_bodies)
        
        # Print state vector order
        print("\n\tState vector order:", self.state_order)
        print("\n\tTorque order:", self.torque_order)

        # Check that the bodies in kinematics_bodies actually exist
        for body in kinematics_bodies:
            if not body in self.bodies:
                print(f"\nERROR: Couldn't find body with name \"{body}\", check that it exists in the list above.")
                sys.exit()
        self.kinematics_bodies = kinematics_bodies

    def generate(self):
        print("\nGenerating code")

        # Define config and velocity vectors to pass to Pinocchio functions
        # pinn_x and x may have different orders (i.e. our convention vs Pinocchios for quaternions)
        # refer to define_state_order
        self.q = self.pinn_x[:self.nq]
        self.v = self.pinn_x[self.nq:]
        self.q_dot = self.pinn_x_dot[:self.nq]
        self.v_dot = self.pinn_x_dot[self.nq:]

        # Generation options
        self.gen_opts = dict(with_header = True)

        # Change directory for output
        orig_dir = os.getcwd()
        os.chdir(self.gen_dir)

        self.generate_order_functions()

        self.generate_dynamics()

        if len(self.kinematics_bodies) > 0:
            self.generate_kinematics()

        print("Finished generating to", self.gen_dir)
        os.chdir(orig_dir)

    def generate_dynamics(self):
        # Mass matrix
        M = cs.densify(cpin.crba(self.cmodel, self.cdata, self.q))

        # Coriolis matrix
        C = cs.densify(cpin.nonLinearEffects(self.cmodel, self.cdata, self.q, self.v))

        # Forward dynamics
        v_dot_out = cs.densify(cpin.aba(self.cmodel, self.cdata, self.q, self.v, self.tau))
        dv_dot_out_dx = cs.densify(cs.jacobian(v_dot_out, self.x))
        dv_dot_out_dtau = cs.densify(cs.jacobian(v_dot_out, self.tau))
        
        # Inverse dynamics
        tau_out = cs.densify(cpin.rnea(self.cmodel, self.cdata, self.q, self.v, self.v_dot))
        dtau_dx = cs.densify(cs.jacobian(tau_out, self.x))
        dtau_dv_dot = cs.densify(cs.jacobian(tau_out, self.v_dot))

        # Create CasADI functions
        m_func = cs.Function("M_func", [self.x], [M])
        c_func = cs.Function("C_func", [self.x], [C])
        forward_dynamics_func = cs.Function("forward_dynamics", [self.x, self.tau], [v_dot_out])
        forward_dynamics_deriv_func = cs.Function("forward_dynamics_deriv", [self.x, self.tau], 
                                                  [dv_dot_out_dx, dv_dot_out_dtau])
        inverse_dynamics_func = cs.Function("inverse_dynamics", [self.x, self.v_dot], [tau_out])
        inverse_dynamics_deriv_func = cs.Function("inverse_dynamics_deriv", [self.x, self.v_dot],
                                                  [dtau_dx, dtau_dv_dot])

        # Generate files
        m_func.generate("M_func.c", self.gen_opts)
        c_func.generate("C_func.c", self.gen_opts)
        forward_dynamics_func.generate("forward_dynamics.c", self.gen_opts)
        forward_dynamics_deriv_func.generate("forward_dynamics_deriv.c", self.gen_opts)
        inverse_dynamics_func.generate("inverse_dynamics.c", self.gen_opts)
        inverse_dynamics_deriv_func.generate("inverse_dynamics_deriv.c", self.gen_opts)

        print("Generated dynamics")

    def generate_kinematics(self):
        # Perform forward kinematics with the symbolic configuration
        cpin.forwardKinematics(self.cmodel, self.cdata, self.q)
        cpin.updateFramePlacements(self.cmodel, self.cdata)

        # Generate velocity kinematics, E(q) such that q_dot = E(q)v
        self.generate_velocity_kinematics()

        # Forward kinematics (world frame by default)
        locs = []
        for body in self.kinematics_bodies:
            locs.append(self.cdata.oMf[self.cmodel.getFrameId(body)].translation)
        locs = cs.vertcat(*locs)

        # Forward kinematics jacobian
        J = cs.densify(cs.jacobian(locs, self.x)) # v block should be zero

        # Kinematics velocity (world frame by default)
        locs_dot = J[:, :self.nq]@self.E@self.v

        # # Kinematics velocity jacobian
        J_dot = cs.densify(cs.jacobian(locs_dot, self.x))

        # Create CasADI functions
        kinematics = cs.Function("kinematics", [self.x], [locs])
        kinematics_jacobian = cs.Function("kinematics_jacobian", [self.x], [J])
        kinematics_velocity = cs.Function("kinematics_velocity", [self.x], [locs_dot])
        kinematics_velocity_jacobian = cs.Function("kinematics_velocity_jacobian", [self.x], [J_dot])

        # Generate files
        kinematics.generate("kinematics.c", self.gen_opts)
        kinematics_jacobian.generate("kinematics_jacobian.c", self.gen_opts)
        kinematics_velocity.generate("kinematics_velocity.c", self.gen_opts)
        kinematics_velocity_jacobian.generate("kinematics_velocity_jacobian.c", self.gen_opts)

        print("Generated kinematics")

    def generate_order_functions(self):
        with open('vector_orders.c', 'w') as f:
            f.write('#include <stdio.h>\n\n')
            f.write('const char* config_names[] = {\n')
            for name in self.state_order[:self.nq]:
                f.write(f'    "{name}",\n')
            f.write('};\n\n')
            f.write('const char* vel_names[] = {\n')
            for name in self.state_order[self.nq:]:
                f.write(f'    "{name}",\n')
            f.write('};\n\n')
            f.write('const char* torque_names[] = {\n')
            for name in self.torque_order:
                f.write(f'    "{name}",\n')
            f.write('};\n\n')
            f.write('const char** get_config_order() {\n')
            f.write('    return config_names;\n')
            f.write('}\n')
            f.write('const char** get_vel_order() {\n')
            f.write('    return vel_names;\n')
            f.write('}\n')
            f.write('const char** get_torque_order() {\n')
            f.write('    return torque_names;\n')
            f.write('}\n')


    def define_state_order(self):
        # Define state order
        self.state_order = [self.model.names[self.q_idx_to_jnt_idx(i)] for i in range(self.model.nq)] + \
                           [self.model.names[self.v_idx_to_jnt_idx(i)] for i in range(self.model.nv)]
        
        # Fix quaternions, detect unsupported joints
        self.pinn_x = cs.SX(self.x)
        self.pinn_x_dot = cs.SX(self.x_dot)
        for jnt_idx in range(self.model.njoints):
            joint = self.model.joints[jnt_idx]
            if joint.nq == 1: 
                continue
            elif joint.nq == 7: # Modify symbolic order to match with our convention by swapping q_w and q_x
                self.pinn_x[joint.idx_q + 3:joint.idx_q + 6], self.pinn_x[joint.idx_q + 6] = \
                        self.pinn_x[joint.idx_q + 4:joint.idx_q + 7], self.pinn_x[joint.idx_q + 3]
                self.pinn_x_dot[joint.idx_q + 3:joint.idx_q + 6], self.pinn_x_dot[joint.idx_q + 6] = \
                        self.pinn_x_dot[joint.idx_q + 4:joint.idx_q + 7], self.pinn_x_dot[joint.idx_q + 3]
                self.state_order[joint.idx_q:joint.idx_q + joint.nq] = ["x", "y", "z", "q_w", "q_x", "q_y", "q_z"]
                self.state_order[self.model.nq + joint.idx_v:self.model.nq + joint.idx_v + joint.nv] = ["lin_v_x", "lin_v_y", "lin_v_z", "ang_v_x", "ang_v_y", "ang_v_z"]
            else:
                print(f"ERROR: Encountered an unsupported joint named \"{self.model.names[jnt_idx]}\"")
                print("This error occurs when the joint has a number of configuration variables (joint.nq)"
                      " that is not 1 or 7 (floating base), and we aren't sure it is supported. A common"
                      " culprit is using a \"continuous\" joint instead of \"revolute\". Pinocchio represents"
                      "  \"continuous\" with two configuration variables, sin(theta) and cos(theta). This "
                      "causes issues with libraries like RigidBodyDynamics.jl, and also results in a "
                      "loss of information (no revolution count). If this isn't the case that you are "
                      "in, please reach out to Arun Bishop.")
                sys.exit()

    def q_idx_to_jnt_idx(self, q_idx):
        for jnt_idx in range(self.model.njoints):
            joint = self.model.joints[jnt_idx]
            if joint.idx_q <= q_idx < joint.idx_q + joint.nq:
                return jnt_idx
            
    def v_idx_to_jnt_idx(self, v_idx):
        for jnt_idx in range(self.model.njoints):
            joint = self.model.joints[jnt_idx]
            if joint.idx_v <= v_idx < joint.idx_v + joint.nv:
                return jnt_idx
            
    # Generates E(q) such that q_dot = E(q)v
    def generate_velocity_kinematics(self):
        self.E = cs.SX.zeros(self.nq, self.nv)
        self.E_T = cs.SX.zeros(self.nv, self.nq)

        for jnt_idx in range(self.model.njoints):
            joint = self.model.joints[jnt_idx]
            if joint.nq == 1: # Trivial relationship, q_dot = v
                self.E[joint.idx_q, joint.idx_v] = 1
                self.E_T[joint.idx_v, joint.idx_q] = 1
            elif joint.nq == 7: # Floating base
                E_jnt = cs.SX.zeros(7, 6)
                E_T_jnt = cs.SX.zeros(6, 7)

                # Extract quaterion, build cross product matrix for the vector part
                quat = self.x[joint.idx_q + 3:joint.idx_q + 7]
                skew_v = cs.vertcat(
                    cs.horzcat(0, -quat[3], quat[2]),
                    cs.horzcat(quat[3], 0, -quat[1]),
                    cs.horzcat(-quat[2], quat[1], 0))
            
                # Create rotation matrix
                rot_mat = cs.SX.eye(3) + 2*quat[0]*skew_v + 2*skew_v@skew_v

                # Rotation to rotate linear velocity from body to world
                E_jnt[:3, :3] = rot_mat
                E_T_jnt[:3, :3] = rot_mat.T

                # Create attitude jacobian
                attitude_jacobian = cs.vertcat(-quat[1:4].T, quat[0]*cs.SX.eye(3) - skew_v)

                # Attitude jacobian to convert angular velocity to quaternion time derivative
                E_jnt[3:, 3:] = 0.5*attitude_jacobian
                E_T_jnt[3:, 3:] = 2*attitude_jacobian.T

                self.E[joint.idx_q:joint.idx_q + joint.nq, joint.idx_v:joint.idx_v + joint.nv] = E_jnt
                self.E_T[joint.idx_v:joint.idx_v + joint.nv, joint.idx_q:joint.idx_q + joint.nq] = E_T_jnt
            else:
                print(f"ERROR: Encountered an unsupported joint named \"{self.model.names[jnt_idx]}\"")
                print("This error occurs when the joint has a number of configuration variables (joint.nq)"
                      " that is not 1 or 7 (floating base), and we aren't sure it is supported. A common"
                      " culprit is using a \"continuous\" joint instead of \"revolute\". Pinocchio represents"
                      "  \"continuous\" with two configuration variables, sin(theta) and cos(theta). This "
                      "causes issues with libraries like RigidBodyDynamics.jl, and also results in a "
                      "loss of information (no revolution count). If this isn't the case that you are "
                      "in, please reach out to Arun Bishop.")
                sys.exit()

        velocity_kinematics = cs.Function("velocity_kinematics", [self.x], [self.E])
        velocity_kinematics_T = cs.Function("velocity_kinematics_T", [self.x], [self.E_T])
        velocity_kinematics.generate("velocity_kinematics.c", self.gen_opts)
        velocity_kinematics_T.generate("velocity_kinematics_T.c", self.gen_opts)

