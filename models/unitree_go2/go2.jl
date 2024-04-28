struct Go2 <: Quadruped
    urdf_path::String
    state_order::Vector{String}
    nq
    nv
    nx
    nu
    M_func_ptr::Ptr{Nothing}
    C_func_ptr::Ptr{Nothing}
    forward_dynamics_ptr::Ptr{Nothing}
    inverse_dynamics_ptr::Ptr{Nothing}
    kinematics_bodies::Vector{String}
    kinematics_ptr::Ptr{Nothing}
    kinematics_jacobian_ptr::Ptr{Nothing}
    function Go2()
        local lib
        try
            lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libunitree_go2.so"))
        catch e
            @error "Unitree Go2 dynamics library wasn't found. Did you compile it using CMake?"
        end

        # Path to URDF (useful for visualization/testing)
        urdf_path = joinpath(MODEL_DIR, "unitree_go2/go2.urdf")

        # Definition of state_order
        state_order = ["x", "y", "z", "q_w", "q_x", "q_y", "q_z", 
                "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", 
                "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", 
                "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", 
                "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint", 
                "lin_v_x", "lin_v_y", "lin_v_z", "ang_v_x", "ang_v_y", "ang_v_z", 
                "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", 
                "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", 
                "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", 
                "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"]

        # Dynamics
        M_func_ptr = dlsym(lib, :M_func_wrapper)
        C_func_ptr = dlsym(lib, :C_func_wrapper)
        forward_dynamics_ptr = dlsym(lib, :forward_dynamics_wrapper)
        inverse_dynamics_ptr = dlsym(lib, :inverse_dynamics_wrapper)

        # Kinematics
        kinematics_bodies = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]
        kinematics_ptr = dlsym(lib, :kinematics_wrapper)
        kinematics_jacobian_ptr = dlsym(lib, :kinematics_jacobian_wrapper)
        return new(
            urdf_path, state_order,
            19, 18, 18 + 19, 12,
            M_func_ptr, C_func_ptr, forward_dynamics_ptr, inverse_dynamics_ptr,
            kinematics_bodies, kinematics_ptr, kinematics_jacobian_ptr
        )
    end
end

is_floating(model::Go2) = true
zero_state(model::Go2) = [zeros(3); 1; zeros(model.nx - 4)]
rand_state(model::Go2) = [randn(3); normalize(randn(4)); randn(model.nx - 7)]
function init_state(model::Go2)
    x = zeros(model.nq)
    x[1:3] = [0; 0; 0.08]
    x[4:7] = axis_angle_to_quat(-6*pi/180*[0; 1; 0])
    x[8:3:19] = [0; 0; 20.; -20]*pi/180
    x[9:3:19] .= 75*pi/180
    x[10:3:19] .= -155*pi/180
    return [x; zeros(model.nv)]
end