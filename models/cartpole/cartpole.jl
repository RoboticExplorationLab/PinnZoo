struct Cartpole <: PinnZooModel
    urdf_path::String
    nq
    nv
    nx
    M_func_ptr::Ptr{Nothing}
    C_func_ptr::Ptr{Nothing}
    forward_dynamics_ptr::Ptr{Nothing}
    inverse_dynamics_ptr::Ptr{Nothing}
    velocity_kinematics_ptr::Ptr{Nothing}
    kinematics_bodies::Vector{String}
    kinematics_ptr::Ptr{Nothing}
    kinematics_jacobian_ptr::Ptr{Nothing}
    function Cartpole()
        local lib
        try
            lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libcartpole.so"))
        catch e
            @error "Cartpole dynamics library wasn't found. Did you compile it using CMake?"
        end

        # Path to URDF (useful for visualization/testing)
        urdf_path = joinpath(MODEL_DIR, "cartpole/cartpole.urdf")

        # Dynamics
        M_func_ptr = dlsym(lib, :M_func_wrapper)
        C_func_ptr = dlsym(lib, :C_func_wrapper)
        forward_dynamics_ptr = dlsym(lib, :forward_dynamics_wrapper)
        inverse_dynamics_ptr = dlsym(lib, :inverse_dynamics_wrapper)
        velocity_kinematics_ptr = dlsym(lib, :velocity_kinematics_wrapper)

        # Kinematics
        kinematics_bodies = ["pole_tip"]
        kinematics_ptr = dlsym(lib, :kinematics_wrapper)
        kinematics_jacobian_ptr = dlsym(lib, :kinematics_jacobian_wrapper)
        return new(
            urdf_path,
            2, 2, 2 + 2,
            M_func_ptr, C_func_ptr, forward_dynamics_ptr, inverse_dynamics_ptr, velocity_kinematics_ptr,
            kinematics_bodies, kinematics_ptr, kinematics_jacobian_ptr
        )
    end
end