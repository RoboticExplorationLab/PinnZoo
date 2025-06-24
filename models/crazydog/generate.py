import os, sys
os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.abspath("../../src"))
from symbolic_generator import SymbolicGenerator

symb_gen = SymbolicGenerator('crazydog_8dof.urdf', 
                             floating = True,
                             kinematics_bodies=['L_wheel', 'R_wheel'],
                             actuated_dofs = slice(6,14))
symb_gen.generate()