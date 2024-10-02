@doc raw"""
    Nadia(; simple = true, nc_per_foot = 1, μ = 1.0)

Return a Nadia dynamics and kinematics model. Currently supports 1 or 4 contact points per foot,
and only the simple knee (does not support simple = false)
"""
struct Nadia <: PinnZooModel
    urdf_path::String
    nq
    nv
    nx
    nu
    nc
    orders::Dict{Symbol, StateOrder}
    conversions::Dict{Tuple{Symbol, Symbol}, ConversionIndices}
    μ::Float64 # Friction coefficient
    kinematics_ori::Bool # Whether the kinematics include orientation
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
    function Nadia(; simple = true, nc_per_foot = 1, μ = 1.0, kinematics_ori = false)
        local lib
        try
            if simple && nc_per_foot == 1
                lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libnadia_simple_1cp.so"))
            elseif simple && nc_per_foot == 4
                lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libnadia_simple_4cp.so"))
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

        # Set up orders and conversions
        orders, conversions = init_conversions(lib)
        nq = length(orders[:nominal].config_names)
        nv = length(orders[:nominal].vel_names)
        nx = nq + nv
        nu = length(orders[:nominal].torque_names)

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
            nq, nv, nx, nu, nc_per_foot*2, orders, conversions,
            μ, kinematics_ori,
            M_func_ptr, C_func_ptr, forward_dynamics_ptr, forward_dynamics_deriv_ptr, 
            inverse_dynamics_ptr, inverse_dynamics_deriv_ptr,
            velocity_kinematics_ptr, velocity_kinematics_T_ptr,
            kinematics_bodies, kinematics_ptr, kinematics_jacobian_ptr,
            kinematics_velocity_ptr, kinematics_velocity_jacobian_ptr
        )
    end
end

is_floating(model::Nadia) = true

B_func(model::Nadia) = [zeros(6, model.nu); I(model.nu)]

function error_jacobian(model::Nadia, x)
    return [
        velocity_kinematics(model, x) zeros(model.nq, length(x) - model.nq)
        zeros(length(x) - model.nq, model.nv) I(length(x) - model.nq)
    ]
end

function error_jacobian_T(model::Nadia, x)
    return [
        velocity_kinematics_T(model, x) zeros(model.nv, length(x) - model.nq)
        zeros(length(x) - model.nq, model.nq) I(length(x) - model.nq)
    ]
end

function apply_Δx(model::Nadia, x_k, Δx)
    x_next = zeros(promote_type(eltype(x_k), eltype(Δx)), length(x_k))
    x_next[1:3] = x_k[1:3] + quat_to_rot(x_k[4:7])*Δx[1:3]
    x_next[4:7] = L_mult(x_k[4:7])*axis_angle_to_quat(Δx[4:6])
    x_next[8:end] = x_k[8:end] + Δx[7:end]
    return x_next
end

function state_error(model::Nadia, x, x0)
    return [
        quat_to_rot(x0[4:7])'*(x[1:3] - x0[1:3])
        quat_to_axis_angle(L_mult(x0[4:7])'*x[4:7])
        x[8:end] - x0[8:end]
    ]
end