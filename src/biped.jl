# Functions that are generic to Quadruped models (specifically Unitree Go1 and Go2)
abstract type Biped4 <: PinnZooFloatingBaseModel end

@doc raw"""
    B_func(bip::Biped)

Return the input jacobian mapping motor torques into joint torques
"""
function B_func(bip::Biped4)
    return [zeros(6, 6); I(6)]
end