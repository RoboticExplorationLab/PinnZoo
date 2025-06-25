@create_pinnzoo_model struct Pineapple1 <: Biped4
    μ::Float64
    # torque_limits::Vector{Float64}
    # joint_limits::Matrix{Float64}
    function Pineapple1(; μ = 0.3)
        lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libpineapple1.dylib"))

        # Limits
        # torque_limits = 23.7*ones(12)
        # joint_limits = [repeat([-Inf Inf], 7); repeat([-0.802851 0.802851; -1.0472 4.18879; -2.69653 -0.916298], 4)]
        return new(μ)#, torque_limits, joint_limits)
    end
end

function init_state(model::Pineapple1)
    x = zero_state(model)
    x[3] = 0.275
    x[8:13] = [pi/4; -pi/2; 0; pi/4; -pi/2; 0]
    return x
end