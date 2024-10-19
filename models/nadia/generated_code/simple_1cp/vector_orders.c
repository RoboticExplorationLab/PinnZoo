#include <stdio.h>

const char* config_names[] = {
    "x",
    "y",
    "z",
    "q_w",
    "q_x",
    "q_y",
    "q_z",
    "LEFT_HIP_Z",
    "LEFT_HIP_X",
    "LEFT_HIP_Y",
    "LEFT_KNEE_Y",
    "LEFT_ANKLE_Y",
    "LEFT_ANKLE_X",
    "RIGHT_HIP_Z",
    "RIGHT_HIP_X",
    "RIGHT_HIP_Y",
    "RIGHT_KNEE_Y",
    "RIGHT_ANKLE_Y",
    "RIGHT_ANKLE_X",
    "SPINE_Z",
    "SPINE_X",
    "SPINE_Y",
    "LEFT_SHOULDER_Y",
    "LEFT_SHOULDER_X",
    "LEFT_SHOULDER_Z",
    "LEFT_ELBOW_Y",
    "RIGHT_SHOULDER_Y",
    "RIGHT_SHOULDER_X",
    "RIGHT_SHOULDER_Z",
    "RIGHT_ELBOW_Y",
    NULL
};

const char* vel_names[] = {
    "lin_v_x",
    "lin_v_y",
    "lin_v_z",
    "ang_v_x",
    "ang_v_y",
    "ang_v_z",
    "LEFT_HIP_Z",
    "LEFT_HIP_X",
    "LEFT_HIP_Y",
    "LEFT_KNEE_Y",
    "LEFT_ANKLE_Y",
    "LEFT_ANKLE_X",
    "RIGHT_HIP_Z",
    "RIGHT_HIP_X",
    "RIGHT_HIP_Y",
    "RIGHT_KNEE_Y",
    "RIGHT_ANKLE_Y",
    "RIGHT_ANKLE_X",
    "SPINE_Z",
    "SPINE_X",
    "SPINE_Y",
    "LEFT_SHOULDER_Y",
    "LEFT_SHOULDER_X",
    "LEFT_SHOULDER_Z",
    "LEFT_ELBOW_Y",
    "RIGHT_SHOULDER_Y",
    "RIGHT_SHOULDER_X",
    "RIGHT_SHOULDER_Z",
    "RIGHT_ELBOW_Y",
    NULL
};

const char* torque_names[] = {
    "LEFT_HIP_Z",
    "LEFT_HIP_X",
    "LEFT_HIP_Y",
    "LEFT_KNEE_Y",
    "LEFT_ANKLE_Y",
    "LEFT_ANKLE_X",
    "RIGHT_HIP_Z",
    "RIGHT_HIP_X",
    "RIGHT_HIP_Y",
    "RIGHT_KNEE_Y",
    "RIGHT_ANKLE_Y",
    "RIGHT_ANKLE_X",
    "SPINE_Z",
    "SPINE_X",
    "SPINE_Y",
    "LEFT_SHOULDER_Y",
    "LEFT_SHOULDER_X",
    "LEFT_SHOULDER_Z",
    "LEFT_ELBOW_Y",
    "RIGHT_SHOULDER_Y",
    "RIGHT_SHOULDER_X",
    "RIGHT_SHOULDER_Z",
    "RIGHT_ELBOW_Y",
    NULL
};

const char* kinematics_bodies[] = {
    "L_C",
    "R_C",
    NULL
};

const char** get_config_order() {
    return config_names;
}
const char** get_vel_order() {
    return vel_names;
}
const char** get_torque_order() {
    return torque_names;
}
const char** get_kinematics_bodies() {
    return kinematics_bodies;
}
const char* get_urdf_path() {
    return "nadia/nadiaV17.fullRobot.simpleKnees.cycloidArms_mj.urdf";
}
