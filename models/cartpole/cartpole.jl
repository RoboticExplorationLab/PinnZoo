
@create_pinnzoo_model struct Cartpole <: PinnZooModel
    function Cartpole()
        lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "libcartpole.so"))
        return new()
    end
end

@doc raw"""
    Cartpole() <: PinnZooModel

Return a Cartpole dynamics model
""" Cartpole