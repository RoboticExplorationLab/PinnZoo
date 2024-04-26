import os, sys
os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.abspath("../../src"))
from symbolic_generator import SymbolicGenerator

symb_gen = SymbolicGenerator('cartpole.urdf', 
                             kinematics_bodies=['pole_tip'])
symb_gen.generate()