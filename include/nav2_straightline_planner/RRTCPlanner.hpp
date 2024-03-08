
#ifndef NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
#define NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_

#include <memory>
#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <string>
#include "nav2_core/global_planner.hpp"
#include "collision_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rrtc.h"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>

// ADDED

namespace nav2_straightline_planner {

class RRTCPlanner : public nav2_core::GlobalPlanner, rclcpp::Node {
public:
    RRTCPlanner();
    ~RRTCPlanner() = default;

    // plugin configure
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    // plugin cleanup
    void cleanup() override;

    // plugin activate
    void activate() override;

    // plugin deactivate
    void deactivate() override;

    // This method creates path for given start and goal pose.
    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal) override;

    void visualize_tree(rrtc::Node* parent, string tree_name, int counter);
    void visualize_node(visualization_msgs::msg::Marker* marker, rrtc::Node* parent, int depth);

    bool future_collision_checker(int m, int n, double ax, double ay, double range, double pan_speed, double half_fov, double pan_init, nav2_costmap_2d::Costmap2D time_map, rrtc::Node* q);

private:
    // Time Map Publisher
    std::shared_ptr<nav2_costmap_2d::Costmap2DPublisher> time_map_publisher_;

    // TF buffer
    std::shared_ptr<tf2_ros::Buffer> tf_;

    // node ptr
    nav2_util::LifecycleNode::SharedPtr node_;

    // Global Costmap
    // This is a wrapper for a costmap to handle publication
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;


    // The global frame of the costmap
    std::string global_frame_, name_;

    double interpolation_resolution_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_start_list_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_goal_list_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_costmap_time;
    // rrtc::RRTC rrt;
    bool path_exist;
};

} // namespace nav2_straightline_planner

#endif // NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
