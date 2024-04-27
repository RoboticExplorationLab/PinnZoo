import os, sys
os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.abspath("../../src"))
from symbolic_generator import SymbolicGenerator

symb_gen = SymbolicGenerator('go1.urdf', 
                             floating = True,
                             kinematics_bodies=['FL_foot', 'FR_foot', 'RL_foot', 'RR_foot'])
symb_gen.generate()