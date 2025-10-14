@create_pinnzoo_model struct RigidBody <: PinnZooFloatingBaseModel
    function RigidBody()
        lib = dlopen(joinpath(SHARED_LIBRARY_DIR, "librigidbody"))
        return new()
    end
end

@doc raw"""
    RigidBody() <: PinnZooFloatingBaseModel

Return a RigidBody dynamics model, with m = 1, I = Diag([1.0; 1.0; 1.0]). 
""" RigidBody