#!/usr/bin/env python3
"""Load pineapple_v3_gripper.urdf in MuJoCo and open an interactive viewer."""

from __future__ import annotations

import math
import time
from pathlib import Path

import mujoco
import mujoco.viewer

URDF_PATH = Path(__file__).resolve().parent / "pineapple_v3_gripper.urdf"


def _joint_qpos_address(model: mujoco.MjModel, joint_name: str) -> int:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        raise ValueError(f"Joint not found: {joint_name}")
    return model.jnt_qposadr[joint_id]


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(URDF_PATH))
    data = mujoco.MjData(model)

    left_qpos = _joint_qpos_address(model, "gripper_left_joint")
    right_qpos = _joint_qpos_address(model, "gripper_right_joint")
    left_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "gripper_left_joint")
    open_pos = model.jnt_range[left_joint, 1]

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start = time.time()
        while viewer.is_running():
            # Animate open -> fully closed -> open. q=0 brings the inner faces together.
            phase = 0.5 * (1.0 + math.sin(2.0 * math.pi * 0.25 * (time.time() - start)))
            grip = open_pos * phase
            data.qpos[left_qpos] = grip
            data.qpos[right_qpos] = grip
            mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(0.01)


if __name__ == "__main__":
    main()
