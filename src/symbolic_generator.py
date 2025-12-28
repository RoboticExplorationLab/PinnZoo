import pinocchio as pin
import pinocchio.casadi as cpin
import casadi as cs
from pinocchio.robot_wrapper import RobotWrapper
import sys, os
import numpy as np
from enum import Enum

# Defininition for kinematics orientation options
KinematicsOrientation = Enum('KinematicsOrientation', ['NONE', 'Quaternion', 'AxisAngle'])

class SymbolicGenerator:
    def __init__(self, urdf_path, gen_dir = "./generated_code", 
                 kinematics_bodies = [], floating = False, mesh_dir=".", actuated_dofs = None, kinematics_ori = KinematicsOrientation.NONE,
                 write_files = True):
        # Load the URDF model
        self.urdf_path = urdf_path
        if floating:
            self.model = RobotWrapper.BuildFromURDF(urdf_path, mesh_dir, root_joint = pin.JointModelFreeFlyer()).model
        else:
            self.model = RobotWrapper.BuildFromURDF(urdf_path, mesh_dir).model

        # (untested) If make_fixed isn't empty, remove the joints in make_fixed from the model
        # Get the ID of all existing joints
        # if len(make_fixed) != 0:
        #     q_fixed = pin.neutral(self.model)
        #     jointsToFix = []
        #     for i in range(len(make_fixed)):
        #         name = make_fixed[i][0]
        #         config = make_fixed[i][1]
        #         if self.model.existJointName(name):
        #             jointsToFix.append(self.model.getJointId(name))
        #             q_fixed[self.model.joints[self.model.getJointId(name)].idx_q] = config
        #         else:
        #             raise Exception('Error: joint ' + str(name) + ' not found in model!')
                    
        #     self.model = pin.buildReducedModel(self.model, jointsToFix, q_fixed)

        self.data = self.model.createData()

        # Get dimensions
        self.nq = self.model.nq
        self.nv = self.model.nv
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
        self.write_files = write_files
        if self.write_files:
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
            print("\n\tOrienation representation:",kinematics_ori.name)
        
        # Print state vector order
        print("\n\tState vector order:", self.state_order)
        print("\n\tTorque order:", self.torque_order)

        # Check that the bodies in kinematics_bodies actually exist
        for body in kinematics_bodies:
            if not body in self.bodies:
                print(f"\nERROR: Couldn't find body with name \"{body}\", check that it exists in the list above.")
                sys.exit()
        self.kinematics_bodies = kinematics_bodies
        self.kinematics_ori = kinematics_ori

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
        if self.write_files:
            self.orig_dir = os.getcwd()
            os.chdir(self.gen_dir)

        if self.write_files:
            self.generate_order_functions()

        self.generate_dynamics()

        if len(self.kinematics_bodies) > 0:
            self.generate_kinematics()

        if self.write_files:
            print("Finished generating to", self.gen_dir)
            os.chdir(self.orig_dir)

    def generate_dynamics(self):
        # Mass matrix
        self.M = cs.densify(cpin.crba(self.cmodel, self.cdata, self.q))

        # Mass matrix jacobian-vector product with respect to x
        self.dv_dot = cs.SX.sym('dv_dot', self.nv)
        self.dx_M_dv = cs.densify(cs.jacobian(self.M@self.dv_dot, self.x))

        # Coriolis matrix
        self.C = cs.densify(cpin.nonLinearEffects(self.cmodel, self.cdata, self.q, self.v))

        # Generate velocity kinematics, E(q) such that q_dot = E(q)v
        self.generate_velocity_kinematics()

        # Forward dynamics
        self.v_dot_out = cs.densify(cpin.aba(self.cmodel, self.cdata, self.q, self.v, self.tau))
        self.dv_dot_out_dx = cs.densify(cs.jacobian(self.v_dot_out, self.x))
        self.dv_dot_out_dtau = cs.densify(cs.jacobian(self.v_dot_out, self.tau))

        # Dynamics (x_dot = f(x, u))
        self.x_dot_out = cs.vertcat(self.v, self.v_dot_out)
        self.dx_dot_out_dx = cs.densify(cs.jacobian(self.x_dot_out, self.x))
        self.dx_dot_out_dtau = cs.densify(cs.jacobian(self.x_dot_out, self.tau))

        # Inverse dynamics
        self.tau_out = cs.densify(cpin.rnea(self.cmodel, self.cdata, self.q, self.v, self.v_dot))
        self.dtau_dx = cs.densify(cs.jacobian(self.tau_out, self.x))
        self.dtau_dv_dot = cs.densify(cs.jacobian(self.tau_out, self.v_dot))

        # Inverse dynamics hessian-vector product
        self.tau_mult_out = cs.SX.sym('tau_mult', self.nx)
        self.dtau_dxlam_dx = cs.densify(cs.jacobian(cs.jtimes(self.tau_out, self.x, self.tau_mult_out), self.x))
        self.dtau_dxlam_dv_dot = cs.densify(cs.jacobian(cs.jtimes(self.tau_out, self.x, self.tau_mult_out), self.v_dot))

        # Inverse dynamics hessian-transpose-vector product
        self.tauT_mult_out = cs.SX.sym('tau_mult', self.nv)
        self.dtau_dxTlam_dx = cs.densify(cs.jacobian(cs.jtimes(self.tau_out, self.x, self.tauT_mult_out, True), self.x))
        self.dtau_dv_dotTlam_dx = cs.densify(cs.jacobian(cs.jtimes(self.tau_out, self.v_dot, self.tauT_mult_out, True), self.x))

        # Create CasADI functions
        self.m_func = cs.Function("M_func", [self.x], [self.M])
        self.m_jvp = cs.Function("M_jvp", [self.x, self.dv_dot], [self.dx_M_dv])
        self.c_func = cs.Function("C_func", [self.x], [self.C])
        self.forward_dynamics = cs.Function("forward_dynamics", [self.x, self.tau], [self.v_dot_out])
        self.forward_dynamics_deriv = cs.Function("forward_dynamics_deriv", [self.x, self.tau], 
                                                  [self.dv_dot_out_dx, self.dv_dot_out_dtau])
        self.dynamics = cs.Function("dynamics", [self.x, self.tau], [self.x_dot_out])
        self.dynamics_deriv = cs.Function("dynamics_deriv", [self.x, self.tau], 
                                                  [self.dx_dot_out_dx, self.dx_dot_out_dtau])
        self.inverse_dynamics = cs.Function("inverse_dynamics", [self.x, self.v_dot], [self.tau_out])
        self.inverse_dynamics_deriv = cs.Function("inverse_dynamics_deriv", [self.x, self.v_dot],
                                                  [self.dtau_dx, self.dtau_dv_dot])
        self.inverse_dynamics_hvp = cs.Function("inverse_dynamics_hvp", [self.x, self.v_dot, self.tau_mult_out],
                                                  [self.dtau_dxlam_dx, self.dtau_dxlam_dv_dot])
        self.inverse_dynamics_hTvp = cs.Function("inverse_dynamics_hTvp", [self.x, self.v_dot, self.tauT_mult_out],
                                                  [self.dtau_dxTlam_dx, self.dtau_dv_dotTlam_dx])

        # Generate files
        if self.write_files:
            self.m_func.generate("M_func.c", self.gen_opts)
            self.m_jvp.generate("M_jvp.c", self.gen_opts)
            self.c_func.generate("C_func.c", self.gen_opts)
            self.forward_dynamics.generate("forward_dynamics.c", self.gen_opts)
            self.forward_dynamics_deriv.generate("forward_dynamics_deriv.c", self.gen_opts)
            self.dynamics.generate("dynamics.c", self.gen_opts)
            self.dynamics_deriv.generate("dynamics_deriv.c", self.gen_opts)
            self.inverse_dynamics.generate("inverse_dynamics.c", self.gen_opts)
            self.inverse_dynamics_deriv.generate("inverse_dynamics_deriv.c", self.gen_opts)
            self.inverse_dynamics_hvp.generate("inverse_dynamics_hvp.c", self.gen_opts)
            self.inverse_dynamics_hTvp.generate("inverse_dynamics_hTvp.c", self.gen_opts)

        print("Generated dynamics")

    def generate_kinematics(self):
        # Perform forward kinematics with the symbolic configuration
        cpin.forwardKinematics(self.cmodel, self.cdata, self.q)
        cpin.updateFramePlacements(self.cmodel, self.cdata)

        # Forward kinematics (world frame by default)
        kinematics = []
        for body in self.kinematics_bodies:
            frame_id = self.cmodel.getFrameId(body)
            placement = self.cdata.oMf[frame_id]
            kinematics.append(placement.translation)
            if self.kinematics_ori == KinematicsOrientation.Quaternion:
                quat = self.rotation_matrix_to_quaternion(placement.rotation)
                kinematics.append(quat)
            elif self.kinematics_ori == KinematicsOrientation.AxisAngle:
                quat = self.rotation_matrix_to_quaternion(placement.rotation)
                aa = self.quaternion_to_axis_angle(quat)
                kinematics.append(aa)

        self.kinematics = cs.vertcat(*kinematics)

        # Forward kinematics jacobian
        self.J = cs.jacobian(self.kinematics, self.x) # v block is zero

        # Kinematics hessian-vector product
        self.dx = cs.SX.sym('dx', self.nx)
        self.J_dx = cs.densify(cs.jacobian(cs.jtimes(self.kinematics, self.x, self.dx), self.x))

        # Kinematics velocity (world frame by default)
        self.kinematics_dot = cs.densify(cs.jtimes(self.kinematics, self.x[:self.nq], self.E@self.v))
        # self.kinematics_dot = self.J[:, :self.nq]@self.E@self.v

        # Kinematics velocity jacobian
        self.J_dot = cs.densify(cs.jacobian(self.kinematics_dot, self.x))

        # Kinematics force jacobian
        self.force = cs.SX.sym('force', self.kinematics.numel())
        self.q_f = cs.densify(self.E.T@cs.jtimes(self.kinematics, self.x[:self.nq], self.force, True))
        # self.q_f = self.E.T@self.J[:, :self.nq].T@self.force
        self.J_f = cs.densify(cs.jacobian(self.q_f, self.x))

        # Kinematics force jacobian hessian-vector product
        # Expansion is f(x + dx) = f(x) + df_dx * dx + 0.5*d_dx(df_dx * dx) * dx
        # f(x) = J(x)'f -> df_dx = d/dx (J(x)'f)
        # Compute d/dx and d/df of (d/dx (J(x)'*f) * mult) where mult = dx
        # d/df (d/dx (J(x)'*f) * mult)
        # TODO: if we do d/dx (d/df J(x)'*f * f_mult) = d/dx (J(x)'*f_mult) this will be cheaper but require two multipliers
        self.J_f_mult = cs.SX.sym("J_f_mult_out", self.nx)
        self.J_f_dx = cs.densify(cs.jacobian(cs.jtimes(self.q_f, self.x, self.J_f_mult), self.x))
        self.J_f_df = cs.densify(cs.jacobian(cs.jtimes(self.q_f, self.x, self.J_f_mult), self.force))

        # Kinematics force jacobian hessian-transpose-vector product
        # Scalar term is mult'(J(x)'*f) = mult'*J(x)'*f
        # Jacobian is d/dx (J(x)'*f)'*mult
        # d/dx and d/df of (d/dx (J(x)'*f)'*mult)
        # d/df (d/dx (J(x)'*f)'*mult) = d/dx (d/df (J(x)'*f)'*mult) = d/dx (mult'*J(x)') = d/dx (J(x)*mult)'
        self.J_fT_mult = cs.SX.sym('J_fT_mult_out', self.nv)
        if self.kinematics_ori == KinematicsOrientation.AxisAngle: # jtimes has NaNs
            self.J_fT_dx = cs.densify(cs.jacobian(self.J_f.T@self.J_fT_mult, self.x))
        else:
            self.J_fT_dx = cs.densify(cs.jacobian(cs.jtimes(self.q_f, self.x, self.J_fT_mult, True), self.x)) # has NaNs for axis angles
        self.J_fT_df = cs.densify(cs.jacobian(cs.jtimes(self.kinematics, self.x[:self.nq],self.E@self.J_fT_mult), self.x)).T

        # Create CasADI functions
        kinematics = cs.Function("kinematics", [self.x], [self.kinematics])
        kinematics_jacobian = cs.Function("kinematics_jacobian", [self.x], [self.J])
        kinematics_hvp = cs.Function("kinematics_hvp", [self.x, self.dx], [self.J_dx])
        kinematics_velocity = cs.Function("kinematics_velocity", [self.x], [self.kinematics_dot])
        kinematics_velocity_jacobian = cs.Function("kinematics_velocity_jacobian", [self.x], [self.J_dot])
        kinematics_force_jacobian = cs.Function("kinematics_force_jacobian", [self.x, self.force], [self.J_f])
        self.kinematics_force_hvp = cs.Function("kinematics_force_hvp", [self.x, self.force, self.J_f_mult], [self.J_f_dx, self.J_f_df])
        self.kinematics_force_hTvp = cs.Function("kinematics_force_hTvp", [self.x, self.force, self.J_fT_mult], [self.J_fT_dx, self.J_fT_df])

        # Generate files
        if self.write_files:
            kinematics.generate("kinematics.c", self.gen_opts)
            kinematics_jacobian.generate("kinematics_jacobian.c", self.gen_opts)
            kinematics_hvp.generate("kinematics_hvp.c", self.gen_opts)
            kinematics_velocity.generate("kinematics_velocity.c", self.gen_opts)
            kinematics_velocity_jacobian.generate("kinematics_velocity_jacobian.c", self.gen_opts)
            kinematics_force_jacobian.generate("kinematics_force_jacobian.c", self.gen_opts)
            self.kinematics_force_hvp.generate("kinematics_force_hvp.c", self.gen_opts)
            self.kinematics_force_hTvp.generate("kinematics_force_hTvp.c", self.gen_opts)

        print("Generated kinematics")

    def generate_order_functions(self):
        with open('vector_orders.c', 'w') as f:
            f.write('#include <stdio.h>\n\n')
            f.write('const char* config_names[] = {\n')
            for name in self.state_order[:self.nq]:
                f.write(f'    "{name}",\n')
            f.write('    NULL\n};\n\n')
            f.write('const char* vel_names[] = {\n')
            for name in self.state_order[self.nq:]:
                f.write(f'    "{name}",\n')
            f.write('    NULL\n};\n\n')
            f.write('const char* torque_names[] = {\n')
            for name in self.torque_order:
                f.write(f'    "{name}",\n')
            f.write('    NULL\n};\n\n')
            f.write('const char* kinematics_bodies[] = {\n')
            for name in self.kinematics_bodies:
                f.write(f'    "{name}",\n')
            f.write('    NULL\n};\n\n')
            f.write('const char** get_config_order() {\n')
            f.write('    return config_names;\n')
            f.write('}\n')
            f.write('const char** get_vel_order() {\n')
            f.write('    return vel_names;\n')
            f.write('}\n')
            f.write('const char** get_torque_order() {\n')
            f.write('    return torque_names;\n')
            f.write('}\n')
            f.write('const char** get_kinematics_bodies() {\n')
            f.write('    return kinematics_bodies;\n')
            f.write('}\n')
            f.write('const char* get_urdf_path() {\n')

            # Get urdf relative path
            urdf_full_path = self.orig_dir+"/"+self.urdf_path
            parts = urdf_full_path.split(os.sep)
            models_index = len(parts) - 1 - parts[::-1].index('models')
            subpath = os.sep.join(parts[models_index + 1:])
            
            f.write(f'    return "{subpath}";\n')
            f.write('}\n');


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
        self.E, self.E_T = self.build_velocity_kinematics(self.x)

        # Generate jacobian-vector product derivatives
        q_in = cs.SX.sym('q_in', self.nq)
        v_in = cs.SX.sym('v_in', self.nv)
        E_jvp_dx = cs.densify(cs.jacobian(self.E@v_in, self.x))
        E_T_jvp_dx = cs.densify(cs.jacobian(self.E_T@q_in, self.x))

        self.velocity_kinematics = cs.Function("velocity_kinematics", [self.x], [self.E])
        self.velocity_kinematics_T = cs.Function("velocity_kinematics_T", [self.x], [self.E_T])
        self.velocity_kinematics_jvp_deriv = cs.Function("velocity_kinematics_jvp_deriv", [self.x, v_in], [E_jvp_dx])
        self.velocity_kinematics_T_jvp_deriv = cs.Function("velocity_kinematics_T_jvp_deriv", [self.x, q_in], [E_T_jvp_dx])

        if self.write_files:
            self.velocity_kinematics.generate("velocity_kinematics.c", self.gen_opts)
            self.velocity_kinematics_T.generate("velocity_kinematics_T.c", self.gen_opts)
            self.velocity_kinematics_jvp_deriv.generate("velocity_kinematics_jvp_deriv.c", self.gen_opts)
            self.velocity_kinematics_T_jvp_deriv.generate("velocity_kinematics_T_jvp_deriv.c", self.gen_opts)

    def build_velocity_kinematics(self, x):
        E = cs.SX.zeros(self.nq, self.nv)
        E_T = cs.SX.zeros(self.nv, self.nq)

        for jnt_idx in range(self.model.njoints):
            joint = self.model.joints[jnt_idx]
            if joint.nq == 1: # Trivial relationship, q_dot = v
                E[joint.idx_q, joint.idx_v] = 1
                E_T[joint.idx_v, joint.idx_q] = 1
            elif joint.nq == 7: # Floating base
                E_jnt = cs.SX.zeros(7, 6)
                E_T_jnt = cs.SX.zeros(6, 7)

                # Extract quaterion, build cross product matrix for the vector part
                quat = x[joint.idx_q + 3:joint.idx_q + 7]
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
                attitude_jacobian = cs.vertcat(-quat[1:4].T, quat[0]*cs.SX.eye(3) + skew_v)

                # Attitude jacobian to convert angular velocity to quaternion time derivative
                E_jnt[3:, 3:] = 0.5*attitude_jacobian
                E_T_jnt[3:, 3:] = 2*attitude_jacobian.T

                E[joint.idx_q:joint.idx_q + joint.nq, joint.idx_v:joint.idx_v + joint.nv] = E_jnt
                E_T[joint.idx_v:joint.idx_v + joint.nv, joint.idx_q:joint.idx_q + joint.nq] = E_T_jnt
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

        return E, E_T

    def rotation_matrix_to_quaternion(self, R):
        # return cs.vertcat(qw, qx, qy, qz)
        # Compute candidate values
        a = 1 + R[0,0] + R[1,1] + R[2,2]
        b = 1 + R[0,0] - R[1,1] - R[2,2]
        c = 1 - R[0,0] + R[1,1] - R[2,2]
        d = 1 - R[0,0] - R[1,1] + R[2,2]
        
        # Symbolic max of four values
        max_abcd = cs.fmax(cs.fmax(a,b), cs.fmax(c,d))
        
        a_new = cs.if_else(a == max_abcd, a,
                cs.if_else(b == max_abcd, R[2,1] - R[1,2],
                cs.if_else(c == max_abcd, R[0,2] - R[2,0],
                                          R[1,0] - R[0,1])))
        
        b_new = cs.if_else(a == max_abcd, R[2,1] - R[1,2],
                cs.if_else(b == max_abcd, b,
                cs.if_else(c == max_abcd, R[1,0] + R[0,1],
                                          R[0,2] + R[2,0])))

        c_new = cs.if_else(a == max_abcd, R[0,2] - R[2,0],
                cs.if_else(b == max_abcd, R[1,0] + R[0,1],
                cs.if_else(c == max_abcd, c,
                                          R[2,1] + R[1,2])))

        d_new = cs.if_else(a == max_abcd, R[1,0] - R[0,1],
                cs.if_else(b == max_abcd, R[0,2] + R[2,0],
                cs.if_else(c == max_abcd, R[2,1] + R[1,2],
                                          d)))
        
        q = cs.vertcat(a_new, b_new, c_new, d_new)
        
        # Normalize
        q = q / cs.norm_2(q)
        
        return q * cs.sign(q[0])

    def quaternion_to_axis_angle(self, q):
        """
        Return the axis angle corresponding to the provided quaternion, using a regularized norm to avoid the singularity
        """
        tol = 1e-12
        qs = q[0]
        qv = q[1:4]

        # Regularized norm to avoid singularity
        norm_qv = cs.sqrt(cs.sumsqr(qv) + tol**2)
        theta = 2 * cs.atan2(norm_qv, qs)

        return theta * qv / norm_qv
            
