import os, sys
os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.abspath("../../src"))
from symbolic_generator import SymbolicGenerator

symb_gen = SymbolicGenerator('pineapple1.urdf', 
                             floating = True,
                             kinematics_bodies=['L_wheel_link', 'R_wheel_link'],
                             actuated_dofs = slice(6,12))
symb_gen.generate()