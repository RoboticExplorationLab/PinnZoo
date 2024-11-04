macro create_pinnzoo_model(expr)
    # Pull out fields from expression
    struct_def = expr.args[2]
    new_fields = expr.args[3].args[1:end - 1]
    constructor_name = expr.args[3].args[end].args[1].args[1]
    constructor_args = expr.args[3].args[end].args[1].args[2:end]
    constructor_lib = expr.args[3].args[end].args[2].args[3]
    constructor_internals = expr.args[3].args[end].args[2].args[4:end-1]
    constructor_return = expr.args[3].args[end].args[2].args[end].args[1].args[2:end]

    return quote
        using Libdl
        struct $struct_def
            lib::Ptr{Nothing}
            urdf_path::String
            nq::Int64
            nv::Int64
            nx::Int64
            nẋ::Int64
            nu::Int64
            nc::Int64
            orders::Dict{Symbol, StateOrder}
            conversions::Dict{Tuple{Symbol, Symbol}, ConversionIndices}
            M_func_ptr::Ptr{Nothing}
            C_func_ptr::Ptr{Nothing}
            forward_dynamics_ptr::Ptr{Nothing}
            forward_dynamics_deriv_ptr::Ptr{Nothing}
            dynamics_ptr::Ptr{Nothing}
            dynamics_deriv_ptr::Ptr{Nothing}
            inverse_dynamics_ptr::Ptr{Nothing}
            inverse_dynamics_deriv_ptr::Ptr{Nothing}
            velocity_kinematics_ptr::Ptr{Nothing}
            velocity_kinematics_T_ptr::Ptr{Nothing}
            velocity_kinematics_jvp_deriv_ptr::Ptr{Nothing}
            velocity_kinematics_T_jvp_deriv_ptr::Ptr{Nothing}
            kinematics_bodies::Vector{Symbol}
            kinematics_ptr::Ptr{Nothing}
            kinematics_jacobian_ptr::Ptr{Nothing}
            kinematics_velocity_ptr::Ptr{Nothing}
            kinematics_velocity_jacobian_ptr::Ptr{Nothing}
            kinematics_force_jacobian_ptr::Ptr{Nothing}
            $(new_fields...)
            function $constructor_name($(constructor_args...))
                # Load the library
                local lib
                try
                    $(constructor_lib)
                catch e
                    throw(e)
                end

                # Get urdf_path
                urdf_path = joinpath(MODEL_DIR, unsafe_string(ccall(dlsym(lib, :get_urdf_path), Ptr{Cchar}, ())))

                # Set up orders and conversions
                orders, conversions = init_conversions(lib)
                nq = length(orders[:nominal].config_names)
                nv = length(orders[:nominal].vel_names)
                nx = nq + nv
                nẋ = nv + nv
                nu = length(orders[:nominal].torque_names)

                # Dynamics
                M_func_ptr = dlsym(lib, :M_func_wrapper)
                C_func_ptr = dlsym(lib, :C_func_wrapper)
                forward_dynamics_ptr = dlsym(lib, :forward_dynamics_wrapper)
                forward_dynamics_deriv_ptr = dlsym(lib, :forward_dynamics_deriv_wrapper)
                dynamics_ptr = dlsym(lib, :dynamics_wrapper)
                dynamics_deriv_ptr = dlsym(lib, :dynamics_deriv_wrapper)
                inverse_dynamics_ptr = dlsym(lib, :inverse_dynamics_wrapper)
                inverse_dynamics_deriv_ptr = dlsym(lib, :inverse_dynamics_deriv_wrapper)
                velocity_kinematics_ptr = dlsym(lib, :velocity_kinematics_wrapper)
                velocity_kinematics_T_ptr = dlsym(lib, :velocity_kinematics_T_wrapper)
                velocity_kinematics_jvp_deriv_ptr = dlsym(lib, :velocity_kinematics_jvp_deriv_wrapper)
                velocity_kinematics_T_jvp_deriv_ptr = dlsym(lib, :velocity_kinematics_T_jvp_deriv_wrapper)

                # Kinematics
                kinematics_bodies = get_order(dlsym(lib, :get_kinematics_bodies))
                nc = length(kinematics_bodies)
                kinematics_ptr = dlsym(lib, :kinematics_wrapper)
                kinematics_jacobian_ptr = dlsym(lib, :kinematics_jacobian_wrapper)
                kinematics_velocity_ptr = dlsym(lib, :kinematics_velocity_wrapper)
                kinematics_velocity_jacobian_ptr = dlsym(lib, :kinematics_velocity_jacobian_wrapper)
                kinematics_force_jacobian_ptr = dlsym(lib, :kinematics_force_jacobian_wrapper)

                $(constructor_internals...)
                return new(
                    lib, urdf_path,
                    nq, nv, nx, nẋ, nu, nc, orders, conversions,
                    M_func_ptr, C_func_ptr, forward_dynamics_ptr, forward_dynamics_deriv_ptr, 
                    dynamics_ptr, dynamics_deriv_ptr, inverse_dynamics_ptr, inverse_dynamics_deriv_ptr,
                    velocity_kinematics_ptr, velocity_kinematics_T_ptr,
                    velocity_kinematics_jvp_deriv_ptr, velocity_kinematics_T_jvp_deriv_ptr,
                    kinematics_bodies, kinematics_ptr, kinematics_jacobian_ptr,
                    kinematics_velocity_ptr, kinematics_velocity_jacobian_ptr,
                    kinematics_force_jacobian_ptr,
                    $(constructor_return...))
            end
        end
    end
end;

