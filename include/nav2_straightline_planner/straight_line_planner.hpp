
#ifndef NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
#define NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_

#include <memory>
#include <string>
//#include "rclcpp/rclcpp.hpp"
//#include "geometry_msgs/msg/point.hpp"
//#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
//#include "nav_msgs/msg/path.hpp"
//#include "nav2_util/robot_utils.hpp"
#include "collision_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rrt.h"
#include <visualization_msgs/msg/marker.hpp>

// ADDED

namespace nav2_straightline_planner {

class StraightLine : public nav2_core::GlobalPlanner /*, rclcpp::Node*/ {
public:
    StraightLine();
    ~StraightLine() = default;

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

    void visualize_tree(rrt::Node* parent);
    void visualize_node(visualization_msgs::msg::Marker* marker, rrt::Node* parent, int depth);

private:
    // Collision Checker
    std::shared_ptr<GridCollisionChecker> collision_checker_;

    // TF buffer
    std::shared_ptr<tf2_ros::Buffer> tf_;

    // node ptr
    nav2_util::LifecycleNode::SharedPtr node_;

    // Global Costmap
    //nav2_costmap_2d::Costmap2D* costmap_; 
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    

    // The global frame of the costmap
    std::string global_frame_, name_;

    double interpolation_resolution_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_line_list_;
};

} // namespace nav2_straightline_planner

#endif // NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
