@create_pinnzoo_model struct CrazyDog <: PinnZooFloatingBaseModel
    μ::Float64
    # torque_limits::Vector{Float64}
    # joint_limits::Matrix{Float64}
    function CrazyDog(; μ = 0.3)
        lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libcrazydog.so"))

        # Limits
        # torque_limits = 23.7*ones(12)
        # joint_limits = [repeat([-Inf Inf], 7); repeat([-0.802851 0.802851; -1.0472 4.18879; -2.69653 -0.916298], 4)]
        return new(μ)#, torque_limits, joint_limits)
    end
end

function init_state(model::CrazyDog)
    x = zero_state(model)
    x[3] = 0.37
    x[8:15] = [0; pi/4; -pi/2; 0; 0; pi/4; -pi/2; 0]
    return x
end