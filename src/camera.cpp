#include "nav2_straightline_planner/camera.hpp"
#include <iostream>

namespace camera{
CAMERA::CAMERA(){
    cam.range = 0;
    cam.pan_speed = 0;
    cam.half_fov = (M_PI / 180) * 0;
    cam.pan_init = (M_PI / 180) * 0;
    cam.period = 2 * M_PI / 0;
    cam.cbarInit.x() = cos(0);
    cam.cbarInit.y() = sin(0);

    cam_r = 0.1;
    cam_bound_n = 100;

    for (int i = 0; i < cam_bound_n; i++) {
        double angi = 2 * M_PI / cam_bound_n * i;
        Vector2d bound_coord(ax + cam_r * cos(angi), ay + cam_r * sin(angi));
        cam.boundary.push_back(bound_coord);
    }

    return cam;
}

void CAMERA::setPosition(double ax, double ay){
    cam.position.x() = ax;
    cam.position.y() = ay;
}

void CAMERA::setRange(double range){
    cam.range = range;
}
void CAMERA::setPanSpeed(double pan_speed){
    cam.pan_speed = pan_speed;
}
void CAMERA::setFOV(double half_fov){
    cam.half_fov = (M_PI / 180) * half_fov;
}
void CAMERA::setPanInit(double pan_init){
    cam.pan_init = (M_PI / 180) * pan_init;
    cam.cbarInit.x() = cos(cam.pan_init);
    cam.cbarInit.y() = sin(cam.pan_init);
}
void CAMERA::setParam(double ax, double ay, double range, double pan_speed, double half_fov, double pan_init){
    std::cout << "setting params" << std::endl;
    cam.setPosition(ax, ay);
    cam.setRange(range);
    cam.setFOV(half_fov);
    cam.setPanInit(pan_init);
}


}