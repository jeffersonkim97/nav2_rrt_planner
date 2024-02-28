#include "nav2_straightline_planner/straight_line_planner.hpp"
//#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_straightline_planner/rrt.h"
#include "nav2_util/node_utils.hpp"
#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <rclcpp/time.hpp>
#include <string>
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

    // Initialize collision checker
    collision_checker_ = std::make_shared<GridCollisionChecker> (costmap_ros->getCostmap(), 72, node_);
    collision_checker_->setFootprint(costmap_ros->getRobotFootprint(), true, 0.0);
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
    // create custom cost map with time dependent part
    nav_msgs::msg::OccupancyGrid msg;
    nav2_costmap_2d::Costmap2D * map = this->costmap_ros_->getCostmap();
    int m = map->getSizeInCellsX();
    int n = map->getSizeInCellsY();
    std::vector<int8_t> data;
    auto now = node_->get_clock()->now();
    for (int i=0;i<n*m;i++) {
        data.push_back(map->getCost(i));
    }

    
    // Place camera
    double ax, ay;
    ax = 2.0;
    ay = 0.0;

    // Camera Setting
    double range = 2;
    double pan_speed = 0.25;
    double half_fov = (M_PI/180)*30;
    double pan_init = 0; // Initial Pan angle
    Vector2d cbarInit(cos(pan_init), sin(pan_init)); // Direction of camera facing
                        
    // Output Camera Position with camera range 0.5
    double  cam_r = 0.1;
    int cam_bound_n = 100;
    vector<Vector2d> cam_bound;
    for (int i = 0; i < cam_bound_n; i++){
        double angi = 2*M_PI/cam_bound_n*i;
        Vector2d bound_coord(ax+cam_r*cos(angi), ay+cam_r*sin(angi));
        cam_bound.push_back(bound_coord);
    }

    // Costmap for camera boundary
    for (int im=0; im<m; im++){
        for (int in=0; in<n; in++){
            for (int i = 0; i < cam_bound_n; i++) {
                Vector2d temp = cam_bound[i];
                unsigned int mxi, myi;
                collision_checker_->getCostmap()->worldToMap(temp.x(), temp.y(), mxi, myi);
                int cam_ind = collision_checker_->getCostmap()->getIndex(mxi, myi);
                data[cam_ind] = 100*(int(now.seconds()));

            }
        }
    
    } 

    std::cout << "Period: " << 2*M_PI/pan_speed << std::endl;
    std::cout << "round down: " << std::floor(now.seconds() / (2*M_PI/pan_speed)) << std::endl;
    std::cout << "subtracted: " << now.seconds() - (2*M_PI/pan_speed)*std::floor(now.seconds()/(2*M_PI/pan_speed)) << std::endl;

    // Costmap for camera FOV
    Vector2d abar(ax, ay);
    Vector2d cbar;
    double period = 2*M_PI/pan_speed;
    
    for (int im=0; im<m; im++){
        for (int in=0; in<n; in++){
            // if b = (wxi, wyi),
            // find all b s.t. |b-a| < r and |theta| < r
            double wxi, wyi;
            collision_checker_->getCostmap()->mapToWorld(im, in, wxi, wyi);
            Vector2d bbar(wxi, wyi);
            Vector2d pbar = bbar - abar;

            // Update cbar over time
            double ti = now.seconds() - (2*M_PI/pan_speed)*std::floor(now.seconds()/(2*M_PI/pan_speed));
            // std::cout << "Scaled Time: " << ti << std::endl;
            // std::cout << "theta: " << pan_init + pan_speed*ti << std::endl;
            cbar.x() = cbarInit.norm()*cos(pan_init + pan_speed*ti);
            cbar.y() = cbarInit.norm()*sin(pan_init + pan_speed*ti);
            std::cout << "x, y: (" << cbar.x() << ", " << cbar.y() << ")" << std::endl;
            std::cout << "Cbar, Cbarinit: " << cbar.norm() << ", " << cbarInit.norm() << std::endl;

            // Calculate conditoin
            bool inRange = pbar.norm() < range;
            double dot_product = pbar.x()*cbar.x() + pbar.y()*cbar.y();
            double ang = acos(dot_product/pbar.norm());
            bool inAng = abs(ang) <= half_fov;
            
            if (inRange && inAng) {
                unsigned int mxi, myi;
                collision_checker_->getCostmap()->worldToMap(wxi, wyi, mxi, myi);
                int fov_ind = collision_checker_->getCostmap()->getIndex(mxi, myi);
                data[fov_ind] = 100*(int(now.seconds()));
            }
        }
    }

    // Initialize collision checker for time map:
    time_checker_ = std::make_shared<GridCollisionChecker> (nav2_costmap_2d::Costmap2D(msg), 72, node_);


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

    std::cout << "Creating Plan" << std::endl;
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

    std::cout << "------Inline Collsion Checker Cost: " << collision_checker_->getCost() << std::endl;

    unsigned int mx, my;
    collision_checker_->getCostmap()->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my) ;
    std::cout << "(mx, my): (" << mx << ", " << my << ")" << std::endl;
    std::cout << "Collision?: " << collision_checker_->inCollision(mx, my, 0, true) << std::endl;
    std::cout << "Cost at mx, my: " << std::isprint(collision_checker_->getCostmap()->getCost(mx, my)) << std::endl;


    // IF goal is invalid:
    if (collision_checker_->inCollision(mx, my, 0, true)){
        RCLCPP_INFO(
            node_->get_logger(), "Invalid Goal");
        return global_path;
        std::cout << "Invalid" << std::endl;
    }
    
    rrt.setGoal(goal);
    
    int max_iter = rrt.max_iter;
    for (int i = 0; i < max_iter; i++) {
        rrt::Node* q = rrt.randomSample(start, goal);
        if (q != NULL) {
            // Convert intermediate world coord into map coord
            unsigned int mqx, mqy;
            collision_checker_->getCostmap()->worldToMap(goal.pose.position.x, goal.pose.position.y, mqx, mqy);

            // Now check collision
            bool collision_detected = collision_checker_->inCollision(mx, my, 0, true);

            if (!collision_detected){

            rrt::Node* qnear = rrt.find_neighbor(q->position);
            if (rrt.distance(q->position, qnear->position) > rrt.step_size) {
                Vector2d qnew_pos = rrt.extend(q, qnear);
                rrt::Node* qnew = new rrt::Node;
                qnew->position = qnew_pos;
                rrt.add(qnear, qnew);
            } else {
                rrt::Node* qnew = new rrt::Node;
                qnew->position = q->position;
                rrt.add(qnear, qnew);
            };
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

        // Find orientation for path segment
        // float yaw = std::atan2(segment_goal->position.y() - segment_start->position.y(), segment_goal->position.x() - segment_start->position.x());
        // std::cout << "Yaw: " << yaw << std::endl;
        // Quaternionf q;
        // q = AngleAxisf(0, Vector3f::UnitX())
        //     * AngleAxisf(0, Vector3f::UnitY())
        //     * AngleAxisf(yaw, Vector3f::UnitZ());
        // std::cout << "Quaternion: " << std::endl << q.coeffs() << std::endl;

        // publish to global path
        for (int i = 0; i < total_number_of_loop; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = starti.pose.position.x + x_increment * i;
            pose.pose.position.y = starti.pose.position.y + y_increment * i;
            pose.pose.position.z = 0.0;
            // pose.pose.orientation.x = q.coeffs()[0];
            // pose.pose.orientation.y = q.coeffs()[1];
            // pose.pose.orientation.z = q.coeffs()[2];
            // pose.pose.orientation.w = q.coeffs()[3];
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
