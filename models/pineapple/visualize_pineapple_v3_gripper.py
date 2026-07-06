#!/usr/bin/env python3
"""Visualize pineapple_v3_gripper.urdf in MuJoCo with a pinch open/close demo."""

from __future__ import annotations

import math
import time
from pathlib import Path

import mujoco
import mujoco.viewer

URDF_PATH = Path(__file__).resolve().parent / "pineapple_v3_gripper.urdf"


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(URDF_PATH))
    data = mujoco.MjData(model)

    left_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "gripper_left_joint")
    right_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "gripper_right_joint")
    left_qpos = model.jnt_qposadr[left_id]
    right_qpos = model.jnt_qposadr[right_id]
    open_pos = model.jnt_range[left_id, 1]

    with mujoco.viewer.launch_passive(model, data) as viewer:
        t0 = time.time()
        while viewer.is_running():
            phase = 0.5 * (1.0 + math.sin(2.0 * math.pi * 0.25 * (time.time() - t0)))
            grip = open_pos * phase
            data.qpos[left_qpos] = grip
            data.qpos[right_qpos] = grip
            mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(0.01)


if __name__ == "__main__":
    main()
