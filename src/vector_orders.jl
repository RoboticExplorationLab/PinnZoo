# This file details the different conventions for the state vector for models used by
# us, Pinocchio, and RigidBodyDynamics, and has conversion functions between the conventions.
# 
# There are five types of vectors we usually work with:
#   Config vector: position of the robot and the joint angles
#   Velocity vector: velocity of the robot and the joint velocities
#   State vector: stacked config and velocity vector
#   Error config vector: config vector where the quaternion has been swapped
#                        with a 3 param representation (matches velocity convention)
#   Error state vector: stacked error config and velocity vector
#
#   Note: We assume that the errorConfig vector order is the same as the velocity vector order.
#         This is necessary for the conversion function, which uses size to determine the type of the input.

@doc raw"""
    StateOrder(config_names, vel_names, torque_names = vel_names[6 + 1:end])

Struct containing the orders of the configuration, velocity and torque vectors. Can be used to
help convert between different vector orderings
"""
struct StateOrder
    config_names::Vector{Symbol}
    vel_names::Vector{Symbol}
    torque_names::Vector{Symbol}
    function StateOrder(config_names, vel_names, torque_names = vel_names[6 + 1:end])
        return new(config_names, vel_names, torque_names)
    end
end

@doc raw"""
    ConversionIndices

Holds index vectors mapping model vectors from one form to another, for config, velocity,
state, error_state, and torques.
"""
struct ConversionIndices
    config::Vector{Int}
    velocity::Vector{Int}
    state::Vector{Int}
    error_state::Vector{Int}
    torque::Vector{Int}
end

@doc raw"""
    create_conversion(orig_order::StateOrder, new_order::StateOrder)

Returns a ConversionIndices object that can be used with the change_order functions to map vectors from
orig_order to new_order (for example mapping MuJoCo to RigidBodyDynamics)
"""
function create_conversion(orig_order::StateOrder, new_order::StateOrder)
    config = zeros(length(orig_order.config_names))
    velocity = zeros(length(orig_order.vel_names))
    torque = zeros(length(orig_order.torque_names))

    # Get config indices
    for new_ind = 1:length(new_order.config_names)
        orig_ind = argmax(orig_order.config_names .== new_order.config_names[new_ind])
        config[new_ind] = orig_ind
    end

    # Get velocity indices
    for new_ind = 1:length(new_order.vel_names)
        orig_ind = argmax(orig_order.vel_names .== new_order.vel_names[new_ind])
        velocity[new_ind] = orig_ind
    end

    # Get torque indices
    for new_ind = 1:length(new_order.torque_names)
        orig_ind = argmax(orig_order.torque_names .== new_order.torque_names[new_ind])
        torque[new_ind] = orig_ind
    end

    # State indices
    state = [config; velocity .+ length(config)]

    # Error state indices (assumes error config is in the same order as velocity)
    error_state = [velocity; velocity .+ length(velocity)]

    return ConversionIndices(config, velocity, state, error_state, torque)
end

@doc raw"""
    change_order!(model::PinnZooModel, input::AbstractVector, from::Symbol, to::Symbol)

Gets the relevant ConversionIndices from model.conversions[(from, to)] and converts the input vector order in-place, deducing
which conversion to apply (config, velocity, state, error_state, or torque) based on the input vector size.
"""
function change_order!(model::PinnZooModel, input::AbstractVector, from::Symbol, to::Symbol)
    # Get size
    n = length(input)

    # Get indices
    indices = []
    conversion = model.conversions[(from, to)]
    if n == length(conversion.config) # Config size
        indices = conversion.config
    elseif n == length(conversion.velocity) # Velocity/errorConfig size
        indices = conversion.velocity
    elseif n == length(conversion.state) # State size
        indices = conversion.state
    elseif n == length(conversion.error_state) # Error state size
        indices = conversion.error_state
    elseif n == length(conversion.torque) # Torque state conversion
        indices = conversion.torque
    end

    # Change order in place
    input[:] = input[indices]
    return nothing
end

@doc raw"""
    change_order!(model::PinnZooModel, input::AbstractVector, from::Symbol, to::Symbol; dims= (1, 2))

Gets the relevant ConversionIndices from model.conversions[(from, to)] and converts the input matrix order in-place, deducing
which conversion to apply (config, velocity, state, error_state, or torque) based on the input matrix rows and columns. Default is
to transform both input and output, but the dimensions can be specified using the dims option. 
"""
function change_order!(model::PinnZooModel, input::AbstractMatrix, from::Symbol, to::Symbol; dims = (1, 2))
    if 1 in dims # Move rows around (translates to permuting each column)
        for c = 1:size(input, 2)
            vec = @view input[:, c]
            change_order!(model::PinnZooModel, vec, from, to)
        end
    end
    if 2 in dims # Move columns around (translates to permuting each row)
        for r = 1:size(input, 1)
            vec = @view input[r, :]
            change_order!(model::PinnZooModel, vec, from, to)
        end
    end
    return nothing
end

@doc raw"""
    change_order!(model::PinnZooModel, input::Adjoint, from::Symbol, to::Symbol)

Gets the relevant ConversionIndices from model.conversions[(from, to)] and converts the input vector order in-place, deducing
which conversion to apply (config, velocity, state, error_state, or torque) based on the input vector size.
"""
function change_order!(model::PinnZooModel, input::Adjoint, from::Symbol, to::Symbol)
    vec = @view input[1, :]
    change_order!(model::PinnZooModel, vec, from, to)
    return nothing
end

@doc raw"""
    change_order(model::PinnZooModel, input::AbstractVector, from, to)

Gets the relevant ConversionIndices from model.conversions[(from, to)] and converts the input vector order, deducing
which conversion to apply (config, velocity, state, error_state, or torque) based on the input vector size.
"""
function change_order(model::PinnZooModel, input::AbstractVector, from, to)
    output = copy(input)
    change_order!(model::PinnZooModel, output, from, to)
    return output
end

@doc raw"""
    change_order(model::PinnZooModel, input::AbstractMatrix, from::Symbol, to::Symbol)

Gets the relevant ConversionIndices from model.conversions[(from, to)] and converts the input matrix order in-place, deducing
which conversion to apply (config, velocity, state, error_state, or torque) based on the input matrix rows and columns. Default is
to transform both input and output, but the dimensions can be specified using the dims option. 
"""
function change_order(model::PinnZooModel, input::AbstractMatrix, from, to; dims = (1, 2))
    output = copy(input)
    change_order!(model::PinnZooModel, output, from, to, dims=dims)
    return output
end

@doc raw"""
    change_order!(model::PinnZooModel, input::Adjoint, from::Symbol, to::Symbol)

Gets the relevant ConversionIndices from model.conversions[(from, to)] and converts the input vector order, deducing
which conversion to apply (config, velocity, state, error_state, or torque) based on the input vector size.
"""
function change_order(model::PinnZooModel, input::Adjoint, from, to)
    output = copy(input)
    change_order!(model::PinnZooModel, output, from, to)
    return output
end

@doc raw"""
    change_orders!(model::PinnZooModel, inputs::Vector{<:AbstractArray}, from, to; dims = (1, 2))

Changes ordering in-place according to the appropriate convention for each object in inputs    
"""
function change_orders!(model::PinnZooModel, inputs::Vector{<:AbstractArray}, from, to; dims = (1, 2))
    if typeof(dims) <: Tuple
        dims = [dims for _ = 1:length(inputs)]
    end

    for (input, dim) in zip(inputs, dims)
        if typeof(input) <: AbstractVector || typeof(input) <: Adjoint
            change_order!(model::Biped, input, from, to)
        else
            change_order!(model::Biped, input, from, to, dims = dim)
        end
    end
end

@doc raw"""
    change_orders!(model::PinnZooModel, inputs::Vector{<:AbstractArray}, from, to; dims = (1, 2))

Changes ordering according to the appropriate convention for each object in inputs    
"""
function change_orders(model::PinnZooModel, inputs::Vector{<:AbstractArray}, from, to; dims = (1, 2))
    outputs = deepcopy(inputs)
    change_orders!(model::Biped, outputs, from, to, dims = dims)
    return outputs
end

@doc raw"""
    generate_conversions(orders::Dict{Symbol, StateOrder}, 
            conversions::Dict{Tuple{Symbol, Symbol}, ConversionIndices}=Dict{Tuple{Symbol, Symbol}, ConversionIndices}();
            from_scratch = false)

Given the vector orders specified, creates ConversionIndices objects that can be used with change_orders to convert between each order
in orders (i.e. nominal, Pinocchio, MuJoCo)
"""
function generate_conversions(orders::Dict{Symbol, StateOrder}, 
            conversions::Dict{Tuple{Symbol, Symbol}, ConversionIndices}=Dict{Tuple{Symbol, Symbol}, ConversionIndices}();
            from_scratch = false)
    for (orig_type, orig_order) in orders
        for (new_type, new_order) in orders
            if orig_type == new_type || (haskey(conversions, (orig_type, new_type)) && !from_scratch)
                continue
            end

            # Create conversion indices vectors
            conversion = create_conversion(orig_order, new_order)

            # Check conversions
            @assert orig_order.config_names[conversion.config] == new_order.config_names
            @assert orig_order.vel_names[conversion.velocity] == new_order.vel_names
            @assert orig_order.torque_names[conversion.torque] == new_order.torque_names

            # Add to Dict
            conversions[(orig_type, new_type)] = conversion
        end
    end
    return conversions
end

function init_conversions(lib)
    orders = Dict{Symbol, StateOrder}()
    orders[:nominal] = StateOrder(get_order(dlsym(lib, :get_config_order)),
                                  get_order(dlsym(lib, :get_vel_order)),
                                  get_order(dlsym(lib, :get_torque_order)))

    conversions = generate_conversions(orders) # Will be empty to start

    return orders, conversions
end

# Helper to call the order functions (gets a string array, converts to symbols)
function get_order(func_ptr)
    array_ptr = ccall(func_ptr, Ptr{Ptr{Cchar}}, ())
    i = 1
    string_ptr = unsafe_load(array_ptr, i)
    order = Vector{Symbol}()
    while string_ptr != C_NULL
        push!(order, Symbol(unsafe_string(string_ptr)))
        i = i + 1
        string_ptr = unsafe_load(array_ptr, i)
    end
    return order
end

