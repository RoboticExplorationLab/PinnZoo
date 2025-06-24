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