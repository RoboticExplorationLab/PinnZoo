import os, sys
os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.abspath("../../src"))
from symbolic_generator import SymbolicGenerator, KinematicsOrientation

symb_gen = SymbolicGenerator('pineapple_8dof.urdf', 
                             floating = True,
                             kinematics_bodies=['L_wheel', 'R_wheel'],
                             actuated_dofs = slice(6,14),
                             gen_dir="./generated_code/pineapple_8dof")
symb_gen.generate()

symb_gen = SymbolicGenerator('pineapple_6dof.urdf', 
                             floating = True,
                             kinematics_bodies=['L_wheel_link', 'R_wheel_link'],
                             actuated_dofs = slice(6,12),
                             gen_dir="./generated_code/pineapple_6dof")
symb_gen.generate()

symb_gen = SymbolicGenerator('pineapple_8dof.urdf', 
                             floating = True,
                             kinematics_bodies=['L_wheel', 'R_wheel'],
                             actuated_dofs = slice(6,14),
                             kinematics_ori = KinematicsOrientation.Quaternion,
                             gen_dir="./generated_code/pineapple_8dof_quat")
symb_gen.generate()

symb_gen = SymbolicGenerator('pineapple_6dof.urdf', 
                             floating = True,
                             kinematics_bodies=['L_wheel_link', 'R_wheel_link'],
                             actuated_dofs = slice(6,12),
                             kinematics_ori = KinematicsOrientation.Quaternion,
                             gen_dir="./generated_code/pineapple_6dof_quat")
symb_gen.generate()