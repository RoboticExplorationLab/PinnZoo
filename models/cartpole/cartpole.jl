@create_pinnzoo_model struct Cartpole <: PinnZooModel
    function Cartpole()
        lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libcartpole.so"))
        return new()
    end
end

@doc raw"""
    Cartpole() <: PinnZooModel

Return a Cartpole dynamics model, cart moves along the y-axis, pole rotates around positive x-axis.
cart m = 1, pole m = 1, l = 1 (mass concentrated at pole tip).
""" Cartpole