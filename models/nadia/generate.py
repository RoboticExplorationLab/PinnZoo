import os, sys
os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.abspath("../../src"))
from symbolic_generator import SymbolicGenerator, KinematicsOrientation, cpin, cs
import math

# symb_gen = SymbolicGenerator('nadiaV17.fullRobot.simpleKnees.cycloidArms_mj.urdf', 
#                              floating = True,
#                              kinematics_bodies=['L_C', 'R_C'],
#                              gen_dir="./generated_code/simple_1cp",
#                              actuated_dofs = slice(6,29))
# symb_gen.generate()
# symb_gen = SymbolicGenerator('nadiaV17.fullRobot.simpleKnees.cycloidArms_mj.urdf', 
#                              floating = True,
#                              kinematics_bodies=['L_FL', 'L_FR', 'L_RL', 'L_RR',
#                                                 'R_FL', 'R_FR', 'R_RL', 'R_RR'],
#                              gen_dir="./generated_code/simple_4cp",
#                              actuated_dofs = slice(6,29))
# symb_gen.generate()
# symb_gen = SymbolicGenerator('nadiaV17.fullRobot.simpleKnees.cycloidArms_mj.urdf', 
#                              floating = True,
#                              kinematics_bodies=['L_C', 'R_C'],
#                              kinematics_ori = KinematicsOrientation.AxisAngle,
#                              gen_dir="./generated_code/simple_1cp_aa",
#                              actuated_dofs = slice(6,29))
# symb_gen.generate()
# symb_gen = SymbolicGenerator('nadiaV17.fullRobot.simpleKnees.cycloidArms_mj.urdf', 
#                              floating = True,
#                              kinematics_bodies=['L_C', 'R_C'],
#                              kinematics_ori = KinematicsOrientation.Quaternion,
#                              gen_dir="./generated_code/simple_1cp_quat",
#                              actuated_dofs = slice(6,29))
# symb_gen.generate()
# symb_gen = SymbolicGenerator('nadiaV17.fullRobot.simpleKnees.fixedArms_mj.urdf', 
#                              floating = True,
#                              kinematics_bodies=['L_C', 'R_C'],
#                              kinematics_ori = KinematicsOrientation.AxisAngle,
#                              gen_dir="./generated_code/simple_1cp_aa_no_arms",
#                              actuated_dofs = slice(6,21))
# symb_gen.generate()
symb_gen = SymbolicGenerator('nadiaV17.fullRobot.simpleKnees.fixedArms_mj.urdf', 
                             floating = True,
                             kinematics_bodies=['L_FL', 'L_FR', 'L_RL', 'L_RR',
                                                'R_FL', 'R_FR', 'R_RL', 'R_RR'],
                             gen_dir="./generated_code/simple_4cp_no_arms",
                             actuated_dofs = slice(6,21))
symb_gen.generate()

# Custom residual functions
def both_feet_dynamics_gen(symb_gen):
    nx, nv, nq, nc = symb_gen.nx, symb_gen.nv, symb_gen.nq, len(symb_gen.kinematics_bodies)*6
    cmodel, cdata = symb_gen.cmodel, symb_gen.cdata
    kinematics_bodies = symb_gen.kinematics_bodies

    # Define inputs
    delta_x_k = cs.SX.sym('x_k', nv*2)
    delta_x_next = cs.SX.sym('x_next', nv*2)
    u_k = cs.SX.sym('u_k', nv - 6)
    f_k = cs.SX.sym('f_k', nc)
    dt = cs.SX.sym('dt', 1)
    alpha = cs.SX.sym("alpha", 1)

    # Form full x_k and x_next
    x_k = cs.vertcat(delta_x_k[0:3], cpin.exp3_quat(delta_x_k[3:6]), delta_x_k[6:])
    x_next = cs.vertcat(delta_x_next[0:3], cpin.exp3_quat(delta_x_next[3:6]), delta_x_next[6:])

    # Pull out q and v
    q_k, v_k = x_k[0:nq], x_k[nq:]
    q_next, v_next = x_next[0:nq], x_next[nq:]

    # Pull out body positions and quaternions
    pos_k, quat_k = x_k[0:3], x_k[3:7]
    pos_next, quat_next = x_next[0:3], x_next[3:7]

    # Compute position error
    R = cpin.exp3(delta_x_k[3:6])
    pos_err = R.T@(pos_next - pos_k)
    att_err = cpin.log3(cpin.exp3(delta_x_k[3:6]).T@cpin.exp3(delta_x_next[3:6]))
    q_err = cs.vertcat(pos_err, att_err, delta_x_next[6:nv] - delta_x_k[6:nv])

    # Compute dynamics terms
    v_err = delta_x_next[nv:] - delta_x_k[nv:]
    tau = cpin.rnea(cmodel, cdata, q_k, v_k, v_err/dt)
    Bu_k = cs.vertcat(cs.SX.zeros(6), u_k)

    # Compute kinematics terms
    x_k_sym = cs.SX.sym('x_k_sym', nx) # Fully symbolic for jacobian calculation
    cpin.forwardKinematics(cmodel, cdata, x_k_sym[:nq], x_k_sym[nq:])
    cpin.updateFramePlacements(cmodel, cdata)
    kinematics = []
    kinematics_dot = []
    for body in kinematics_bodies:
        id = cmodel.getFrameId(body)
        kinematics.append(cdata.oMf[id].translation)
        aa = cpin.log3(cdata.oMf[id].rotation)
        kinematics.append(aa)
        kin_vel = cpin.getFrameVelocity(cmodel, cdata, id, cpin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        kinematics_dot.append(kin_vel.linear)
        kinematics_dot.append(kin_vel.angular)
    kinematics = cs.vertcat(*kinematics)
    kinematics_dot = cs.vertcat(*kinematics_dot)
    E = cs.substitute(cs.densify(cs.jacobian(cpin.integrate(cmodel, x_k_sym[:nq], x_k_sym[nq:]), x_k_sym[nq:])), x_k_sym[nq:], cs.SX.zeros(nv))
    J = cs.densify(cs.jacobian(kinematics_dot, x_k_sym))
    J = cs.horzcat(J[:, :nq]@E, J[:, nq:])

    # Replace x_k_sym with x_k now
    kinematics = cs.substitute(kinematics, x_k_sym, x_k)
    kinematics_dot = cs.substitute(kinematics_dot, x_k_sym, x_k)
    J = cs.substitute(J, x_k_sym, x_k)

    # Build residual
    res = cs.densify(cs.vertcat(q_err - dt*v_k, \
        tau - Bu_k - J[:, nv:].T@f_k, \
        J@cs.vertcat(v_k, v_err/dt) - 2*alpha*kinematics - pow(alpha, 2)*kinematics_dot))

    # Get jacobian
    stacked = cs.vertcat(delta_x_k, u_k, f_k, delta_x_next)
    res_J = cs.densify(cs.jacobian(res, stacked))

    # Code generation
    orig_dir = os.getcwd()
    os.chdir(symb_gen.gen_dir)

    gen_opts = dict(with_header = True)

    contact_res_func = cs.Function("contact_res", [delta_x_k, u_k, f_k, delta_x_next, dt, alpha], [res])
    contact_res_deriv_func = cs.Function("contact_res_deriv", [delta_x_k, u_k, f_k, delta_x_next, dt, alpha], [res_J])
    contact_res_func.generate("contact_res.c", gen_opts)
    contact_res_deriv_func.generate("contact_res_deriv.c", symb_gen.gen_opts)

    os.chdir(orig_dir)








both_feet_dynamics_gen(symb_gen)
