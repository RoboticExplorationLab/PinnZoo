
import pinocchio as pin
import pinocchio.casadi as cpin
import casadi as cs
from pinocchio.robot_wrapper import RobotWrapper
import sys, os
import numpy as np

class SymbolicGenerator:
    def __init__(self, urdf_path, gen_dir = "./generated_code", 
                 kinematics_bodies = [], floating = False, mesh_dir="."):
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

        # Check that the bodies in kinematics_bodies actually exist
        for body in kinematics_bodies:
            if not body in self.bodies:
                print(f"\nERROR: Couldn't find body with name \"{body}\", check that it exists in the list above.")
                sys.exit()
        self.kinematics_bodies = kinematics_bodies

    def generate(self):
        print("\nGenerating code")

        # Define config and velocity vectors to pass to Pinocchio functions
        self.q = self.x[:self.nq]
        self.v = self.x[self.nq:]
        self.q_dot = self.x_dot[:self.nq]
        self.v_dot = self.x_dot[self.nq:]

        # Generation options
        self.gen_opts = dict(with_header = True)

        # Change directory for output
        orig_dir = os.getcwd()
        os.chdir(self.gen_dir)

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
        
        # Inverse dynamics
        tau_out = cs.densify(cpin.rnea(self.cmodel, self.cdata, self.q, self.v, self.v_dot))

        # Create CasADI functions
        m_func = cs.Function("M_func", [self.x], [M])
        c_func = cs.Function("C_func", [self.x], [C])
        forward_dynamics_func = cs.Function("forward_dynamics", [self.x, self.tau], [v_dot_out])
        inverse_dynamics_func = cs.Function("inverse_dynamics", [self.x, self.v_dot], [tau_out])

        # Generate files
        orig_dir = os.getcwd()
        os.chdir(self.gen_dir)
        m_func.generate("M_func.c", self.gen_opts)
        c_func.generate("C_func.c", self.gen_opts)
        forward_dynamics_func.generate("forward_dynamics.c", self.gen_opts)
        inverse_dynamics_func.generate("inverse_dynamics.c", self.gen_opts)
        os.chdir(orig_dir)

        print("Generated dynamics")

    def generate_kinematics(self):
        # Perform forward kinematics with the symbolic configuration
        cpin.forwardKinematics(self.cmodel, self.cdata, self.q)
        cpin.updateFramePlacements(self.cmodel, self.cdata)

        # Forward kinematics (world frame by default)
        locs = []
        for body in self.kinematics_bodies:
            locs.append(self.cdata.oMf[self.cmodel.getFrameId(body)].translation)
        locs = cs.vertcat(*locs)

        # Forward kinematics jacobian
        J = cs.densify(cs.jacobian(locs, self.x)) # v block should be zero


        # Kinematics velocity (world frame by default)
        # locs_dot = J@self.x_dot # only uses the q_dot part

        # # Kinematics velocity jacobian
        # J_dot = cs.jacobian(locs_dot, self.x) # v block is zero (but should maybe be J(q)E(q))

        # Create CasADI functions
        kinematics = cs.Function("kinematics", [self.x], [locs])
        kinematics_jacobian = cs.Function("kinematics_jacobian", [self.x], [J])
        # kinematics_velocity = cs.Function("kinematics_velocity", [self.x], [locs])
        # kinematics_velocity_jacobian = cs.Function("kinematics_velocity_jacobian", [self.x], [locs])

        # Generate files
        orig_dir = os.getcwd()
        os.chdir(self.gen_dir)
        kinematics.generate("kinematics.c", self.gen_opts)
        kinematics_jacobian.generate("kinematics_jacobian.c", self.gen_opts)
        # kinematics_velocity.generate("kinematics_velocity.c", self.gen_opts)
        # kinematics_velocity_jacobian.generate("kinematics_velocity_jacobian.c", self.gen_opts)
        os.chdir(orig_dir)

        print("Generated kinematics")

    def define_state_order(self):
        # Define state order
        self.state_order = [self.model.names[self.q_idx_to_jnt_idx(i)] for i in range(self.model.nq)] + \
                           [self.model.names[self.v_idx_to_jnt_idx(i)] for i in range(self.model.nv)]
        
        # Fix quaternions, detect unsupported joints
        for jnt_idx in range(self.model.njoints):
            joint = self.model.joints[jnt_idx]
            if joint.nq == 1: 
                continue
            elif joint.nq == 7: # Modify symbolic order to match with our convention by swapping q_w and q_x
                self.x[joint.idx_q + 4:joint.idx_q + 7], self.x[joint.idx_q + 3] = \
                        self.x[joint.idx_q + 3:joint.idx_q + 6], self.x[joint.idx_q + 6]
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
            
    # def v_idx_to_jnt(self, v_idx):
    #     for j_idx 
    #     for joint in self.model.joints:
    #         if joint.idx_v <= v_idx < joint.idx_v + joint.nv:
    #             return joint

