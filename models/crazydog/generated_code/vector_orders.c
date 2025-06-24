#include <stdio.h>

const char* config_names[] = {
    "x",
    "y",
    "z",
    "q_w",
    "q_x",
    "q_y",
    "q_z",
    "L_base2hip",
    "L_hip2thigh",
    "L_thigh2calf",
    "L_calf2wheel",
    "R_base2hip",
    "R_hip2thigh",
    "R_thigh2calf",
    "R_calf2wheel",
    NULL
};

const char* vel_names[] = {
    "lin_v_x",
    "lin_v_y",
    "lin_v_z",
    "ang_v_x",
    "ang_v_y",
    "ang_v_z",
    "L_base2hip",
    "L_hip2thigh",
    "L_thigh2calf",
    "L_calf2wheel",
    "R_base2hip",
    "R_hip2thigh",
    "R_thigh2calf",
    "R_calf2wheel",
    NULL
};

const char* torque_names[] = {
    "L_base2hip",
    "L_hip2thigh",
    "L_thigh2calf",
    "L_calf2wheel",
    "R_base2hip",
    "R_hip2thigh",
    "R_thigh2calf",
    "R_calf2wheel",
    NULL
};

const char* kinematics_bodies[] = {
    "L_wheel",
    "R_wheel",
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
    return "crazydog/crazydog_8dof.urdf";
}
