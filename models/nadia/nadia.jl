@create_pinnzoo_model struct Nadia <: PinnZooFloatingBaseModel
    kinematics_ori::Symbol
    μ::Float64 # Friction coefficient
    foot_width::Float64
    foot_depth::Float64
    fixed_arms::Bool
    function Nadia(; simple = true, nc_per_foot = 1, μ::Float64 = 1.0, kinematics_ori::Symbol = :None, 
                fixed_arms::Bool = false, foot_width::Float64 = 0.0295, foot_depth::Float64 = 0.125)
        lib = let 
            if simple && nc_per_foot == 1 && kinematics_ori == :Quaternion && !fixed_arms
                lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libnadia_simple_1cp_quat.so"))
            elseif simple && nc_per_foot == 1 && kinematics_ori == :AxisAngle && !fixed_arms
                lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libnadia_simple_1cp_aa.so"))
            elseif simple && nc_per_foot == 1 && kinematics_ori == :AxisAngle && fixed_arms
                lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libnadia_simple_1cp_aa_no_arms.so"))
            elseif simple && nc_per_foot == 1 && kinematics_ori == :None && !fixed_arms
                lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libnadia_simple_1cp.so"))
            elseif simple && nc_per_foot == 4 && kinematics_ori != :None
                throw(error("specified configuration is not supported"))
            elseif simple && nc_per_foot == 4 && kinematics_ori == :None && !fixed_arms
                lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libnadia_simple_4cp.so"))
            elseif simple && nc_per_foot == 4 && kinematics_ori == :None && fixed_arms
                lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libnadia_simple_4cp_no_arms.so"))
            else
                throw(error("specified configuration is either not found or not supported. Did you compile?"))
            end
            lib
        end

        return new(kinematics_ori, μ, foot_width, foot_depth, fixed_arms)
    end
end

@doc raw"""
    Nadia(; simple = true, nc_per_foot = 1, μ = 1.0, kinematics_ori = :None)

Return a Nadia dynamics and kinematics model. Currently supports 1 or 4 contact points per foot,
and only the simple knee (does not support simple = false). For 1 contact point you can also include
foot orientation (kinematics_ori = :Quaterion or :AxisAngle).
""" Nadia

B_func(model::Nadia) = [zeros(6, model.nu); I(model.nu)]