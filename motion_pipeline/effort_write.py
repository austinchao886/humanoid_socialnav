"""Opt-in write optimization for the pinned zero-gain implicit G1 task only.

No actuator model is replaced. ImplicitActuator returns effort commands unchanged;
PhysX remains responsible for its configured effort limits. Diagnostic torque
buffers retain the same clipped-effort meaning. Do not use with reward managers
that inspect per-actuator computed_effort caches or dynamically changing gains.
"""
def install_zero_gain_effort_writer(robot):
    import torch
    from isaaclab.actuators import ImplicitActuator
    if not robot.actuators or any(type(a) is not ImplicitActuator for a in robot.actuators.values()):
        raise ValueError('requires exclusively standard implicit actuators')
    if torch.count_nonzero(robot.data.joint_stiffness).item() or torch.count_nonzero(robot.data.joint_damping).item():
        raise ValueError('nonzero PhysX joint gains forbid effort-only writes')
    limits=torch.zeros_like(robot.data.joint_effort_target)
    covered=torch.zeros_like(limits,dtype=torch.bool)
    for a in robot.actuators.values():
        if torch.count_nonzero(a.stiffness).item() or torch.count_nonzero(a.damping).item():
            raise ValueError('nonzero actuator gains forbid effort-only writes')
        limits[:,a.joint_indices]=a.effort_limit
        covered[:,a.joint_indices]=True
        robot._data.soft_joint_vel_limits[:,a.joint_indices]=a.velocity_limit
    if not bool(covered.all()):raise ValueError('all joints must have validated implicit actuators')
    original=robot.write_data_to_sim
    def write():
        if robot.has_external_wrench:
            robot.root_physx_view.apply_forces_and_torques_at_position(
                force_data=robot._external_force_b.view(-1,3),
                torque_data=robot._external_torque_b.view(-1,3),position_data=None,
                indices=robot._ALL_INDICES,is_global=False)
        effort=robot._data.joint_effort_target
        robot._joint_effort_target_sim.copy_(effort)
        robot._data.computed_torque.copy_(effort)
        robot._data.applied_torque.copy_(torch.clamp(effort,min=-limits,max=limits))
        robot.root_physx_view.set_dof_actuation_forces(robot._joint_effort_target_sim,robot._ALL_INDICES)
    robot.write_data_to_sim=write
    return original


def install_vectorized_implicit_writer(robot):
    """Batch standard implicit actuator bookkeeping; preserve every PhysX target.

    Applicable only to this zero-reward task with static actuator parameters.
    Unlike the zero-gain optimization, this retains nonzero hand drives.
    """
    import torch
    from isaaclab.actuators import ImplicitActuator
    if not robot.actuators or any(type(a) is not ImplicitActuator for a in robot.actuators.values()):
        raise ValueError('requires exclusively standard implicit actuators')
    limits=torch.zeros_like(robot.data.joint_effort_target)
    stiffness=torch.zeros_like(limits);damping=torch.zeros_like(limits)
    covered=torch.zeros_like(limits,dtype=torch.bool)
    for a in robot.actuators.values():
        if bool(covered[:,a.joint_indices].any()):raise ValueError('overlapping actuators')
        limits[:,a.joint_indices]=a.effort_limit
        stiffness[:,a.joint_indices]=a.stiffness
        damping[:,a.joint_indices]=a.damping
        covered[:,a.joint_indices]=True
        robot._data.soft_joint_vel_limits[:,a.joint_indices]=a.velocity_limit
    if not bool(covered.all()):raise ValueError('uncovered joints')
    original=robot.write_data_to_sim
    def write():
        if robot.has_external_wrench:
            robot.root_physx_view.apply_forces_and_torques_at_position(
                force_data=robot._external_force_b.view(-1,3),
                torque_data=robot._external_torque_b.view(-1,3),position_data=None,
                indices=robot._ALL_INDICES,is_global=False)
        data=robot._data
        effort=data.joint_effort_target
        computed=stiffness*(data.joint_pos_target-data.joint_pos)+damping*(data.joint_vel_target-data.joint_vel)+effort
        data.computed_torque.copy_(computed)
        data.applied_torque.copy_(torch.clamp(computed,min=-limits,max=limits))
        robot._joint_effort_target_sim.copy_(effort)
        robot._joint_pos_target_sim.copy_(data.joint_pos_target)
        robot._joint_vel_target_sim.copy_(data.joint_vel_target)
        view=robot.root_physx_view;indices=robot._ALL_INDICES
        view.set_dof_actuation_forces(robot._joint_effort_target_sim,indices)
        view.set_dof_position_targets(robot._joint_pos_target_sim,indices)
        view.set_dof_velocity_targets(robot._joint_vel_target_sim,indices)
    robot.write_data_to_sim=write
    return original
