#include "nav2_straightline_planner/straight_line_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
//#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_straightline_planner/rrt.h"
#include "nav2_straightline_planner/rrtc.h"
#include "nav2_util/node_utils.hpp"
#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <rclcpp/time.hpp>
#include <string>
#include "nav2_straightline_planner/camera.hpp"
#include <visualization_msgs/msg/marker.hpp>

// ADDED

namespace nav2_straightline_planner {

StraightLine::StraightLine()
    : Node("rrt_node_publisher")
{
}

void StraightLine::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;

    publisher_line_list_ = node_->create_publisher<visualization_msgs::msg::Marker>("rrt_nodes", 10);
    publisher_costmap_time = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("costmap_time", 10);

    costmap_ros_ = costmap_ros;
    global_frame_ = costmap_ros->getGlobalFrameID();

    // Parameter initialization
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
    node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void StraightLine::cleanup()
{
    RCLCPP_INFO(
        node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
        name_.c_str());
}

void StraightLine::activate()
{
    RCLCPP_INFO(
        node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
        name_.c_str());
}

void StraightLine::deactivate()
{
    RCLCPP_INFO(
        node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
        name_.c_str());
}

nav_msgs::msg::Path StraightLine::createPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal)
{
    //-------------------------------------------------------------------------
    // get current time
    auto now = node_->get_clock()->now();

    //-------------------------------------------------------------------------
    // get static cost map from costmap_ros_ node
    nav2_costmap_2d::Costmap2D* map = this->costmap_ros_->getCostmap();
    int m = map->getSizeInCellsX();
    int n = map->getSizeInCellsY();

    //-------------------------------------------------------------------------
    // create a new temporary costmap copying the static map
    // that will hold time based obstacle as well
    nav2_costmap_2d::Costmap2D time_map = *map;

    //-------------------------------------------------------------------------
    // assign cost based on time obstacles
    // ---------------------------CAM0-----------------------------------------
    {
        // Place cam0
        double ax, ay;
        ax = 3.0;
        ay = 0.0;

        // Camera Setting
        double range = 1.5;
        double pan_speed = 0.25;
        double half_fov = (M_PI / 180) * 20;
        double pan_init = (M_PI / 180) * 180; // Initial Pan angle
        Vector2d cbarInit(cos(pan_init), sin(pan_init)); // Direction of camera facing

        // Output Camera Position with camera range 0.5
        double cam_r = 0.1;
        int cam_bound_n = 100;
        vector<Vector2d> cam_bound;
        for (int i = 0; i < cam_bound_n; i++) {
            double angi = 2 * M_PI / cam_bound_n * i;
            Vector2d bound_coord(ax + cam_r * cos(angi), ay + cam_r * sin(angi));
            cam_bound.push_back(bound_coord);
        }

        // Costmap for camera boundary
        for (int im = 0; im < m; im++) {
            for (int in = 0; in < n; in++) {
                for (int i = 0; i < cam_bound_n; i++) {
                    Vector2d temp = cam_bound[i];
                    unsigned int mxi, myi;
                    time_map.worldToMap(temp.x(), temp.y(), mxi, myi);
                    time_map.setCost(mxi, myi, 254);
                }
            }
        }

        // Costmap for camera FOV
        Vector2d abar(ax, ay);
        Vector2d cbar;
        double period = 2 * M_PI / pan_speed;

        for (int im = 0; im < m; im++) {
            for (int in = 0; in < n; in++) {
                // if b = (wxi, wyi),
                // find all b s.t. |b-a| < r and |theta| < r
                double wxi, wyi;
                time_map.mapToWorld(im, in, wxi, wyi);
                Vector2d bbar(wxi, wyi);
                Vector2d pbar = bbar - abar;

                // Update cbar over time
                double ti = now.seconds() - (2 * M_PI / pan_speed) * std::floor(now.seconds() / (2 * M_PI / pan_speed));
                cbar.x() = cbarInit.norm() * cos(pan_init + pan_speed * ti);
                cbar.y() = cbarInit.norm() * sin(pan_init + pan_speed * ti);

                // Calculate conditoin
                bool inRange = pbar.norm() < range;
                double dot_product = pbar.x() * cbar.x() + pbar.y() * cbar.y();
                double ang = acos(dot_product / pbar.norm());
                bool inAng = abs(ang) <= half_fov;

                // Assign Cost into Costmap
                unsigned int mxi, myi;
                time_map.worldToMap(wxi, wyi, mxi, myi);

                if (inRange && inAng) {
                    // Extract Cost Data
                    time_map.setCost(mxi, myi, 254);
                } else {
                    // time_map.setCost(mxi, myi, 0);
                }
            }
        }
    }

    // ---------------------------CAM1-----------------------------------------
    {
        // Place cam0
        double ax, ay;
        ax = -1.0;
        ay = 3;

        // Camera Setting
        double range = 2.5;
        double pan_speed = 0.1;
        double half_fov = (M_PI / 180) * 35;
        double pan_init = (M_PI / 180) * 90; // Initial Pan angle
        Vector2d cbarInit(cos(pan_init), sin(pan_init)); // Direction of camera facing

        // Output Camera Position with camera range 0.5
        double cam_r = 0.1;
        int cam_bound_n = 100;
        vector<Vector2d> cam_bound;
        for (int i = 0; i < cam_bound_n; i++) {
            double angi = 2 * M_PI / cam_bound_n * i;
            Vector2d bound_coord(ax + cam_r * cos(angi), ay + cam_r * sin(angi));
            cam_bound.push_back(bound_coord);
        }

        // Costmap for camera boundary
        for (int im = 0; im < m; im++) {
            for (int in = 0; in < n; in++) {
                for (int i = 0; i < cam_bound_n; i++) {
                    Vector2d temp = cam_bound[i];
                    unsigned int mxi, myi;
                    time_map.worldToMap(temp.x(), temp.y(), mxi, myi);
                    time_map.setCost(mxi, myi, 254);
                }
            }
        }

        // Costmap for camera FOV
        Vector2d abar(ax, ay);
        Vector2d cbar;
        double period = 2 * M_PI / pan_speed;

        for (int im = 0; im < m; im++) {
            for (int in = 0; in < n; in++) {
                // if b = (wxi, wyi),
                // find all b s.t. |b-a| < r and |theta| < r
                double wxi, wyi;
                time_map.mapToWorld(im, in, wxi, wyi);
                Vector2d bbar(wxi, wyi);
                Vector2d pbar = bbar - abar;

                // Update cbar over time
                double ti = now.seconds() - (2 * M_PI / pan_speed) * std::floor(now.seconds() / (2 * M_PI / pan_speed));
                cbar.x() = cbarInit.norm() * cos(pan_init + pan_speed * ti);
                cbar.y() = cbarInit.norm() * sin(pan_init + pan_speed * ti);

                // Calculate conditoin
                bool inRange = pbar.norm() < range;
                double dot_product = pbar.x() * cbar.x() + pbar.y() * cbar.y();
                double ang = acos(dot_product / pbar.norm());
                bool inAng = abs(ang) <= half_fov;

                // Assign Cost into Costmap
                unsigned int mxi, myi;
                time_map.worldToMap(wxi, wyi, mxi, myi);

                if (inRange && inAng) {
                    // Extract Cost Data
                    time_map.setCost(mxi, myi, 254);
                } else {
                    // time_map.setCost(mxi, myi, 0);
                }
            }
        }
    }

    // ---------------------------CAM2-----------------------------------------
    {
        // Place cam0
        double ax, ay;
        ax = -3.0;
        ay = 1.0;

        // Camera Setting
        double range = 2;
        double pan_speed = 0.15;
        double half_fov = (M_PI / 180) * 10;
        double pan_init = (M_PI / 180) * 30; // Initial Pan angle
        Vector2d cbarInit(cos(pan_init), sin(pan_init)); // Direction of camera facing

        // Output Camera Position with camera range 0.5
        double cam_r = 0.1;
        int cam_bound_n = 100;
        vector<Vector2d> cam_bound;
        for (int i = 0; i < cam_bound_n; i++) {
            double angi = 2 * M_PI / cam_bound_n * i;
            Vector2d bound_coord(ax + cam_r * cos(angi), ay + cam_r * sin(angi));
            cam_bound.push_back(bound_coord);
        }

        // Costmap for camera boundary
        for (int im = 0; im < m; im++) {
            for (int in = 0; in < n; in++) {
                for (int i = 0; i < cam_bound_n; i++) {
                    Vector2d temp = cam_bound[i];
                    unsigned int mxi, myi;
                    time_map.worldToMap(temp.x(), temp.y(), mxi, myi);
                    time_map.setCost(mxi, myi, 254);
                }
            }
        }

        // Costmap for camera FOV
        Vector2d abar(ax, ay);
        Vector2d cbar;
        double period = 2 * M_PI / pan_speed;

        for (int im = 0; im < m; im++) {
            for (int in = 0; in < n; in++) {
                // if b = (wxi, wyi),
                // find all b s.t. |b-a| < r and |theta| < r
                double wxi, wyi;
                time_map.mapToWorld(im, in, wxi, wyi);
                Vector2d bbar(wxi, wyi);
                Vector2d pbar = bbar - abar;

                // Update cbar over time
                double ti = now.seconds() - (2 * M_PI / pan_speed) * std::floor(now.seconds() / (2 * M_PI / pan_speed));
                cbar.x() = cbarInit.norm() * cos(pan_init + pan_speed * ti);
                cbar.y() = cbarInit.norm() * sin(pan_init + pan_speed * ti);

                // Calculate conditoin
                bool inRange = pbar.norm() < range;
                double dot_product = pbar.x() * cbar.x() + pbar.y() * cbar.y();
                double ang = acos(dot_product / pbar.norm());
                bool inAng = abs(ang) <= half_fov;

                // Assign Cost into Costmap
                unsigned int mxi, myi;
                time_map.worldToMap(wxi, wyi, mxi, myi);

                if (inRange && inAng) {
                    // Extract Cost Data
                    time_map.setCost(mxi, myi, 254);
                } else {
                    // time_map.setCost(mxi, myi, 0);
                }
            }
        }
    }

    //-------------------------------------------------------------------------
    // create collision checker
    GridCollisionChecker collision_checker(&time_map, 72, node_);
    collision_checker.setFootprint(costmap_ros_->getRobotFootprint(), true, 0.0);

    //-------------------------------------------------------------------------
    // send the message to ros
    {
        nav_msgs::msg::OccupancyGrid msg;
        std::vector<int8_t> data;
        for (size_t i = 0; i < n * m; i++) {
            uint8_t c = time_map.getCost(i);
            if (c == 255) {
                c = -1;
            } else {
                c = c * 99 / 254;
            }
            data.push_back(c);
        }
        std::cout << std::endl;
        msg.header.frame_id = "map";
        msg.header.stamp = now;
        msg.data = data;
        msg.info.width = m;
        msg.info.height = n;
        msg.info.resolution = map->getResolution();
        msg.info.map_load_time = node_->get_clock()->now();
        msg.info.origin.position.x = map->getOriginX();
        msg.info.origin.position.y = map->getOriginY();
        this->publisher_costmap_time->publish(msg);
    }

    //-------------------------------------------------------------------------
    // execute RRT
    nav_msgs::msg::Path global_path;

    // Checking if the goal and start state is in the global frame
    if (start.header.frame_id != global_frame_) {
        RCLCPP_ERROR(
            node_->get_logger(), "Planner will only except start position from %s frame",
            global_frame_.c_str());
        return global_path;
    }

    if (goal.header.frame_id != global_frame_) {
        RCLCPP_INFO(
            node_->get_logger(), "Planner will only except goal position from %s frame",
            global_frame_.c_str());
        return global_path;
    }

    global_path.poses.clear();
    global_path.header.stamp = node_->now();
    global_path.header.frame_id = global_frame_;

    // Run RRT
    rrt::RRT rrt;
    rrt.setStart(start);
    
    unsigned int mx, my;
    time_map.worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my);
    double wx_map_end, wy_map_end;
    time_map.mapToWorld(m, n, wx_map_end, wy_map_end);
    double wx_map_init, wy_map_init;
    time_map.mapToWorld(0, 0, wx_map_init, wy_map_init);
    double goalx = (double) goal.pose.position.x;
    double goaly = (double) goal.pose.position.y;


    // IF goal is invalid:
    if (collision_checker.inCollision(mx, my, 0, true)==1) {
        RCLCPP_INFO(
            node_->get_logger(), "Invalid Goal");
        return global_path;
        std::cout << "Invalid" << std::endl;
    }

    rrt.setGoal(goal);

    int max_iter = rrt.max_iter;
    int interpolation = 100;
    for (int i = 0; i < max_iter; i++) {
        rrt::Node* q = rrt.randomSample(std::min(std::min(wx_map_init, wx_map_end), goalx),
                                        std::max(std::max(wx_map_init, wx_map_end), goalx),
                                        std::min(std::min(wy_map_init, wy_map_end), goaly),
                                        std::max(std::max(wy_map_init, wy_map_end), goaly),
                                        goal);
        // rrt::Node* q = rrt.randomSample(start, goal);
        if (q != NULL) {
            // Convert intermediate world coord into map coord
            unsigned int mqx, mqy;
            time_map.worldToMap(q->position.x(), q->position.y(), mqx, mqy);

            // Now check collision
            bool collision_detected = collision_checker.inCollision(mqx, mqy, 0, true);
            if (collision_detected==0) {
                // Find nearest neighbor
                rrt::Node* qnear = rrt.find_neighbor(q->position);

                // Find new vertex qnew
                rrt::Node* qnew = new rrt::Node;
                if (rrt.distance(q->position, qnear->position) > rrt.step_size) {
                    Vector2d qnew_pos = rrt.extend(q, qnear);
                    qnew->position = qnew_pos;
                } else {
                    qnew->position = q->position;
                };

                // Interpolate points in straight path and check for collision
                int interpolated_path_collision = 0;
                for (int ip = 0; ip < interpolation; ip++){
                    double ipX = (qnew->position.x()-qnear->position.x())/interpolation*ip+qnear->position.x();
                    double ipY = (qnew->position.y()-qnear->position.y())/interpolation*ip+qnear->position.y();

                    unsigned int mqix, mqiy;
                    time_map.worldToMap(ipX, ipY, mqix, mqiy);
                    interpolated_path_collision += collision_checker.inCollision(mqix, mqiy, 0, true);
                }
                // std::cout << "Path Collision Check: " << interpolated_path_collision << std::endl;

                if (interpolated_path_collision == 0){
                    rrt.add(qnear, qnew);
                }
            };
        };

        if (rrt.reached()) {
            rrt::Node* qGoal = new rrt::Node;
            qGoal->position.x() = goal.pose.position.x;
            qGoal->position.y() = goal.pose.position.y;
            rrt.path.push_back(qGoal);
            break;
        };
    }

    visualize_tree(rrt.root);

    rrt::Node* qvertex;
    if (rrt.reached()) {
        qvertex = rrt.lastNode;
    } else {
        qvertex = rrt.find_neighbor(rrt.endPos);
    }
    while (qvertex != NULL) {
        rrt.path.push_back(qvertex);
        qvertex = qvertex->parent;
    }

    for (size_t k = 0; k < rrt.path.size(); k++) {
        rrt::Node* currNode = rrt.path[k];
    }

    std::reverse(rrt.path.begin(), rrt.path.end());

    // Find total path length
    // For each path segment:
    for (size_t j = 0; j < (rrt.path.size() - 1); j++) {
        // Update segment start and goal
        rrt::Node* segment_start = rrt.path[j];
        rrt::Node* segment_goal = rrt.path[j + 1];

        // Initialize Intermediate Segment Variables
        geometry_msgs::msg::PoseStamped starti;
        starti.pose.position.x = segment_start->position.x();
        starti.pose.position.y = segment_start->position.y();
        starti.pose.position.z = 0.0;
        starti.pose.orientation.x = 0.0;
        starti.pose.orientation.y = 0.0;
        starti.pose.orientation.z = 0.0;
        starti.pose.orientation.w = 1.0;
        starti.header.stamp = node_->now();
        starti.header.frame_id = global_frame_;

        geometry_msgs::msg::PoseStamped goali;
        goali.pose.position.x = segment_goal->position.x();
        goali.pose.position.y = segment_goal->position.y();
        goali.pose.position.z = 0.0;
        goali.pose.orientation.x = 0.0;
        goali.pose.orientation.y = 0.0;
        goali.pose.orientation.z = 0.0;
        goali.pose.orientation.w = 1.0;
        goali.header.stamp = node_->now();
        goali.header.frame_id = global_frame_;

        // Find interpolation_resolution_ of each segment
        int total_number_of_loop = std::hypot(
                                       goali.pose.position.x - starti.pose.position.x,
                                       goali.pose.position.y - starti.pose.position.y)
            / interpolation_resolution_;
        double x_increment = (goali.pose.position.x - starti.pose.position.x) / total_number_of_loop;
        double y_increment = (goali.pose.position.y - starti.pose.position.y) / total_number_of_loop;

        // publish to global path
        for (int i = 0; i < total_number_of_loop; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = starti.pose.position.x + x_increment * i;
            pose.pose.position.y = starti.pose.position.y + y_increment * i;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            pose.header.stamp = node_->now();
            pose.header.frame_id = global_frame_;
            global_path.poses.push_back(pose);
        }

        geometry_msgs::msg::PoseStamped goal_pose = goali;
        goal_pose.header.stamp = node_->now();
        goal_pose.header.frame_id = global_frame_;
        global_path.poses.push_back(goal_pose);
    }

    return global_path;
}

void StraightLine::visualize_tree(rrt::Node* parent)
{
    auto marker = visualization_msgs::msg::Marker();
    marker.header.stamp.sec = 0;
    marker.header.stamp.nanosec = 0;
    marker.header.frame_id = "map";
    marker.ns = "rrt_tree";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = 0;
    marker.pose = geometry_msgs::msg::Pose();
    marker.scale.set__x(0.02);
    marker.color.a = 0.5;
    marker.color.b = 0.0;
    marker.color.g = 0.0;
    marker.color.r = 1.0;

    visualize_node(&marker, parent, 0);
    RCLCPP_INFO(node_->get_logger(), "Publishing RRT Nodes");
    publisher_line_list_->publish(marker);
}

void StraightLine::visualize_node(visualization_msgs::msg::Marker* marker, rrt::Node* parent, int depth)
{
    auto point_parent = geometry_msgs::msg::Point();
    point_parent.x = parent->position.x();
    point_parent.y = parent->position.y();
    point_parent.z = 0;

    for (size_t i = 0; i < parent->children.size(); i++) {
        rrt::Node* child = parent->children[i];

        if (child == parent) {
            continue;
        }
        auto point_child = geometry_msgs::msg::Point();
        point_child.x = child->position.x();
        point_child.y = child->position.y();
        point_child.z = 0;
        marker->points.push_back(point_parent);
        marker->points.push_back(point_child);
        visualize_node(marker, child, depth + 1);
    }
};

} // namespace nav2_straightline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)

// vi: ts=4 sw=4 et :
