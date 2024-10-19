@create_pinnzoo_model struct Quadrotor <: PinnZooFloatingBaseModel
    kf::Float64 # Motor thrust constant
    km::Float64 # Motor torque constant
    L::Float64 # Arn lengths
    function Quadrotor(; L = 0.175, kf = 1.0, km = 0.0245)
        lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libquadrotor.so"))
        return new()
    end
end

@doc raw"""
    Quadrotor() <: PinnZooFloatingBaseModel

Return a Quadrotor dynamics model, with m = 1, I = Diag([0.0046; 0.0046; 0.008])
""" Quadrotor

function B_func(model::Quadrotor)
    kt, km, L = model.kf, model.km, model.L
    B_f = [zeros(2, 4); kf*ones(1, 4)]
    B_τ = [0 L*kt 0 -L*kt; -L*kt 0 L*kt 0; km -km km -km]
    return [B_f; B_τ]
end