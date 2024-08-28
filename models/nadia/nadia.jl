struct Nadia <: PinnZooModel
    urdf_path::String
    nq
    nv
    nx
    nu
    nc
    μ::Float64 # Friction coefficient
    M_func_ptr::Ptr{Nothing}
    C_func_ptr::Ptr{Nothing}
    forward_dynamics_ptr::Ptr{Nothing}
    forward_dynamics_deriv_ptr::Ptr{Nothing}
    inverse_dynamics_ptr::Ptr{Nothing}
    inverse_dynamics_deriv_ptr::Ptr{Nothing}
    velocity_kinematics_ptr::Ptr{Nothing}
    velocity_kinematics_T_ptr::Ptr{Nothing}
    kinematics_bodies::Vector{String}
    kinematics_ptr::Ptr{Nothing}
    kinematics_jacobian_ptr::Ptr{Nothing}
    kinematics_velocity_ptr::Ptr{Nothing}
    kinematics_velocity_jacobian_ptr::Ptr{Nothing}
    function Nadia(; simple = true, nc_per_foot = 1, μ = 1.0)
        local lib
        try
            if simple && nc_per_foot == 1
                lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libnadia_simple_1cp.so"))
            elseif simple && nc_per_foot == 4
                lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libnadia_simple_1cp.so"))
            else
                println("specified configuration is not yet supported")
                @error ""
            end
        catch e
            @error "Dynamics library wasn't found. Did you compile it using CMake?"
        end

        # Path to URDF (useful for visualization/testing)
        if simple
            urdf_path = joinpath(MODEL_DIR, "nadia/nadiaV17.fullRobot.simpleKnees.cycloidArms_mj.urdf")
        end

        # Dynamics
        M_func_ptr = dlsym(lib, :M_func_wrapper)
        C_func_ptr = dlsym(lib, :C_func_wrapper)
        forward_dynamics_ptr = dlsym(lib, :forward_dynamics_wrapper)
        forward_dynamics_deriv_ptr = dlsym(lib, :forward_dynamics_deriv_wrapper)
        inverse_dynamics_ptr = dlsym(lib, :inverse_dynamics_wrapper)
        inverse_dynamics_deriv_ptr = dlsym(lib, :inverse_dynamics_deriv_wrapper)
        velocity_kinematics_ptr = dlsym(lib, :velocity_kinematics_wrapper)
        velocity_kinematics_T_ptr = dlsym(lib, :velocity_kinematics_T_wrapper)

        # Kinematics
        if nc_per_foot == 1
            kinematics_bodies = ["L_C", "R_C"]
        elseif nc_per_foot == 4
            kinematics_bodies = ["L_FL", "L_FR", "L_RL", "L_RR",
                            "R_FL", "R_FR", "R_RL", "R_RR"]
        end
        kinematics_ptr = dlsym(lib, :kinematics_wrapper)
        kinematics_jacobian_ptr = dlsym(lib, :kinematics_jacobian_wrapper)
        kinematics_velocity_ptr = dlsym(lib, :kinematics_velocity_wrapper)
        kinematics_velocity_jacobian_ptr = dlsym(lib, :kinematics_velocity_jacobian_wrapper)

        return new(
            urdf_path,
            30, 29, 30 + 29, 23, nc_per_foot*2,
            μ,
            M_func_ptr, C_func_ptr, forward_dynamics_ptr, forward_dynamics_deriv_ptr, 
            inverse_dynamics_ptr, inverse_dynamics_deriv_ptr,
            velocity_kinematics_ptr, velocity_kinematics_T_ptr,
            kinematics_bodies, kinematics_ptr, kinematics_jacobian_ptr,
            kinematics_velocity_ptr, kinematics_velocity_jacobian_ptr
        )
    end
end

is_floating(model::Nadia) = true