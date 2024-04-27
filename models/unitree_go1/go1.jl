struct Go1 <: PinnZooModel
    urdf_path::String
    nq
    nv
    nx
    M_func_ptr::Ptr{Nothing}
    C_func_ptr::Ptr{Nothing}
    forward_dynamics_ptr::Ptr{Nothing}
    inverse_dynamics_ptr::Ptr{Nothing}
    kinematics_bodies::Vector{String}
    kinematics_ptr::Ptr{Nothing}
    kinematics_jacobian_ptr::Ptr{Nothing}
    function Go1()
        local lib
        try
            lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libunitree_go1.so"))
        catch e
            @error "Unitree Go1 dynamics library wasn't found. Did you compile it using CMake?"
        end

        # Path to URDF (useful for visualization/testing)
        urdf_path = joinpath(MODEL_DIR, "unitree_go1/go1.urdf")

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
            urdf_path,
            19, 18, 18 + 19,
            M_func_ptr, C_func_ptr, forward_dynamics_ptr, inverse_dynamics_ptr,
            kinematics_bodies, kinematics_ptr, kinematics_jacobian_ptr
        )
    end
end

is_floating(model::Go1) = true