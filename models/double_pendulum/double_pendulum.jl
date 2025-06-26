@create_pinnzoo_model struct DoublePendulum <: PinnZooModel
    function DoublePendulum()
        lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libdouble_pendulum"))
        return new()
    end
end

@doc raw"""
    DoublePendulum() <: PinnZooModel

Return an inverted double pendulum dynamics model. θ = 0 is up, m = 1, l = 1, point mass at tip for both poles.
Rotation axis is the positive x-axis (positive is counter-clockwise, right hand rule).
""" DoublePendulum