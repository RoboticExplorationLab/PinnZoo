@create_pinnzoo_model struct Pendulum <: PinnZooModel
    function Pendulum()
        lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libpendulum"))
        return new()
    end
end

@doc raw"""
    Pendulum() <: PinnZooModel

Return an inverted pendulum dynamics model. θ = 0 is up, m = 1, l = 1, point mass at tip.
Rotation axis is the positive x-axis (positive is counter-clockwise, right hand rule).
""" Pendulum