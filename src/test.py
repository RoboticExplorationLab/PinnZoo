import pinocchio as pin
import numpy as np

 #Initialize a simple model
model = pin.Model()

# Create data required by the algorithms
data = model.createData()

# Set a random configuration
q = pin.neutral(model)
v = pin.utils.rand(model.nv)

# Perform forward kinematics
pin.forwardKinematics(model, data, q, v)

# Linear position in world frame
world_frame_position = data.oMi[1].translation

# Linear velocity in world frame
world_frame_velocity = data.v[1].linear

print("Linear position in world frame: ", world_frame_position)
print("Linear velocity in world frame: ", world_frame_velocity)
print(v)