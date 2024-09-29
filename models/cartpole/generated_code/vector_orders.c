#include <stdio.h>

const char* config_names[] = {
    "slider_to_cart",
    "cart_to_pole",
};

const char* vel_names[] = {
    "slider_to_cart",
    "cart_to_pole",
};

const char* torque_names[] = {
    "slider_to_cart",
    "cart_to_pole",
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
