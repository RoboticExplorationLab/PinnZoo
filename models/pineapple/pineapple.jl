@create_pinnzoo_model struct Pineapple <: PinnZooFloatingBaseModel
    μ::Float64
    # torque_limits::Vector{Float64}
    # joint_limits::Matrix{Float64}
    function Pineapple(; num_dofs = 6, μ = 0.3)
        lib = let
            if num_dofs == 6
                lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libpineapple_6dof"))
            elseif num_dofs == 8
                lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libpineapple_8dof"))
            else
                throw(error("specified configuration is either not found or not supported. Did you compile?"))
            end
            lib
        end

        # Limits
        # torque_limits = 23.7*ones(12)
        # joint_limits = [repeat([-Inf Inf], 7); repeat([-0.802851 0.802851; -1.0472 4.18879; -2.69653 -0.916298], 4)]
        return new(μ)#, torque_limits, joint_limits)
    end
end

@doc raw"""
    B_func(quad::Quadruped)

Return the input jacobian mapping motor torques into joint torques
"""
function B_func(model::Pineapple)
    return [zeros(6, model.nu); I(model.nu)]
end

function init_state(model::Pineapple)
    x = zero_state(model)
    x[3] = 0.37
    x[8:15] = [0; pi/4; -pi/2; 0; 0; pi/4; -pi/2; 0]
    return x
end