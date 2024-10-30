@create_pinnzoo_model struct DoubleCartpole <: PinnZooModel
    function DoubleCartpole()
        lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libdouble_cartpole.so"))
        return new()
    end
end

@doc raw"""
    DoubleCartpole() <: PinnZooModel

Return a DoubleCartpole dynamics model, cart moves along the y-axis, poles rotates around positive x-axis.
cart m = 1, pole m = 1, l = 1 (mass concentrated at pole tip for both poles).
""" DoubleCartpole