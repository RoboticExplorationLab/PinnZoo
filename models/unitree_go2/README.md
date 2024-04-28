### Nominal (our) conventions:
   Config vector:
   
       [x, y, z, quat_w, quat_x, quat_y, quat_z, FL_hip, FL_thigh, FL_calf,
        FR_hip, FR_thigh, FR_calf, RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf]
   Velocity vector (body velocities in body frame):
   
       [v_x, v_y, v_z, ω_x, ω_y, ω_z, FL_hip, FL_thigh, FL_calf,
        FR_hip, FR_thigh, FR_calf, RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf]
        
### Pinocchio conventions (only differs in config):
   Config vector:
   
       [x, y, z, quat_x, quat_y, quat_z, quat_w, FL_hip, FL_thigh, FL_calf,
        FR_hip, FR_thigh, FR_calf, RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf]
        
   Velocity vector (body velocities in body frame):
   
       [v_x, v_y, v_z, ω_x, ω_y, ω_z, FL_hip, FL_thigh, FL_calf,
        FR_hip, FR_thigh, FR_calf, RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf]

### RigidBodyDynamics conventions:
   Config vector:
   
       [quat_w, quat_x, quat_y, quat_z, x, y, z, FR_hip, FL_hip, RR_hip, RL_hip,
        FR_thigh, FL_thigh, RR_thigh, RL_thigh, FR_calf, FL_calf, RR_calf, RL_calf]
   Velocity vector (body velocities in body frame):
   
       [ω_x, ω_y, ω_z, v_x, v_y, v_z, FR_hip, FL_hip, RR_hip, RL_hip,
        FR_thigh, FL_thigh, RR_thigh, RL_thigh, FR_calf, FL_calf, RR_calf, RL_calf]

Kinematics are for each foot in the order of: ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]

Additional functions
- inverse_kinematics, returns both inverse kinematics solutions given a state (used for body position and rotation) and foot locations in the world frame