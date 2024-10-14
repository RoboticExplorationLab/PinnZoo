#include <stdio.h>

const char* config_names[] = {
    "pole1",
    "pole2",
    NULL
};

const char* vel_names[] = {
    "pole1",
    "pole2",
    NULL
};

const char* torque_names[] = {
    "pole1",
    "pole2",
    NULL
};

const char* kinematics_bodies[] = {
    "pole_tip",
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
    return "double_pendulum/double_pendulum.urdf";
}
