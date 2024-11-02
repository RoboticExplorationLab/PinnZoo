import os, sys
os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.abspath("../../src"))
from symbolic_generator import SymbolicGenerator, KinematicsOrientation

symb_gen = SymbolicGenerator('quadrotor.urdf', floating = True, kinematics_bodies=["quadrotor"], kinematics_ori=KinematicsOrientation.AxisAngle)
symb_gen.generate()