#include <eigen3/Eigen/Dense>
#include <vector>
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include <rclcpp/time.hpp>
#include <string>
#include <cmath>

using namespace std;
using namespace Eigen;
namespace camera {

struct Camera {
    Vector2d position;
    double range;
    double pan_speed;
    double half_fov;
    double pan_init;
    Vector2d cbarInit;
    vector<Vector2d> boundary;
    
};
class CAMERA {
public:
    CAMERA();
    void setPosition(double ax, double ay);
    void setRange(double range);
    void setPanSpeed(double pan_speed);
    void setFOV(double half_fov);
    void setPanInit(double pan_init);
    void setParam(double ax, double ay, double range, double pan_speed, double half_fov, double pan_init);

    Camera cam;
    double period;    
    double cam_r;
    int cam_bound_n;
    vector<Vector2d> cam_bound;
};

}