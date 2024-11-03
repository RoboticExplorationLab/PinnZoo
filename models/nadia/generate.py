import os, sys
os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.abspath("../../src"))
from symbolic_generator import SymbolicGenerator, KinematicsOrientation
import math

symb_gen = SymbolicGenerator('nadiaV17.fullRobot.simpleKnees.cycloidArms_mj.urdf', 
                             floating = True,
                             kinematics_bodies=['L_C', 'R_C'],
                             gen_dir="./generated_code/simple_1cp",
                             actuated_dofs = slice(6,29))
symb_gen.generate()
symb_gen = SymbolicGenerator('nadiaV17.fullRobot.simpleKnees.cycloidArms_mj.urdf', 
                             floating = True,
                             kinematics_bodies=['L_FL', 'L_FR', 'L_RL', 'L_RR',
                                                'R_FL', 'R_FR', 'R_RL', 'R_RR'],
                             gen_dir="./generated_code/simple_4cp",
                             actuated_dofs = slice(6,29))
symb_gen.generate()
symb_gen = SymbolicGenerator('nadiaV17.fullRobot.simpleKnees.cycloidArms_mj.urdf', 
                             floating = True,
                             kinematics_bodies=['L_C', 'R_C'],
                             kinematics_ori = KinematicsOrientation.AxisAngle,
                             gen_dir="./generated_code/simple_1cp_aa",
                             actuated_dofs = slice(6,29))
symb_gen.generate()
symb_gen = SymbolicGenerator('nadiaV17.fullRobot.simpleKnees.cycloidArms_mj.urdf', 
                             floating = True,
                             kinematics_bodies=['L_C', 'R_C'],
                             kinematics_ori = KinematicsOrientation.Quaternion,
                             gen_dir="./generated_code/simple_1cp_quat",
                             actuated_dofs = slice(6,29))
symb_gen.generate()
symb_gen = SymbolicGenerator('nadiaV17.fullRobot.simpleKnees.fixedArms_mj.urdf', 
                             floating = True,
                             kinematics_bodies=['L_C', 'R_C'],
                             kinematics_ori = KinematicsOrientation.AxisAngle,
                             gen_dir="./generated_code/simple_1cp_aa_no_arms",
                             actuated_dofs = slice(6,21))
symb_gen.generate()