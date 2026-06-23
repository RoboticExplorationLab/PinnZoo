module RigidBodyDynamicsExt

using PinnZoo
using RigidBodyDynamics

function _add_rigid_body_dynamics_order!(orders, conversions, urdf_path)
    try
        floating = length(orders[:nominal].config_names) >= 7 && orders[:nominal].config_names[4] == :q_w # Hacky check for whether this model should be floating base
        robot = parse_urdf(urdf_path; remove_fixed_tree_joints=false, floating=floating)
        state = MechanismState(robot)
        config_names = [Symbol(joints(robot)[id].name) for id in state.q_index_to_joint_id]
        vel_names = [Symbol(joints(robot)[id].name) for id in state.v_index_to_joint_id]
        for joint in joints(robot)
            if typeof(joint.joint_type) <: QuaternionFloating
                config_names[state.qranges[joint]] = [:q_w, :q_x, :q_y, :q_z, :x, :y, :z]
                vel_names[state.vranges[joint]] = [:ang_v_x, :ang_v_y, :ang_v_z, :lin_v_x, :lin_v_y, :lin_v_z]
            end
        end
        torque_names = [name for name in vel_names if name in orders[:nominal].torque_names]
        orders[:rigidBodyDynamics] = StateOrder(config_names, vel_names, torque_names)
        generate_conversions(orders, conversions)
    catch err
        @warn "RigidBodyDynamics state order not added; nominal/Pinocchio conversions are unchanged" urdf_path exception=(err,)
    end
    return nothing
end

function __init__()
    push!(PinnZoo.GENERATE_ORDER_HOOKS, _add_rigid_body_dynamics_order!)
end

end
