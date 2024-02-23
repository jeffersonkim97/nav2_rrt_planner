/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Shivang Patel
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"
// #include "rrt.h"

#include "nav2_straightline_planner/straight_line_planner.hpp"

//ADDED
namespace rrt{
RRT::RRT(){
    // obstacles = new Obstacles;
    // RRT Loop Control
    step_size = 5;
    max_iter = 3000;
}

void RRT::setRoot(){
  root = new Node;
  root->parent = NULL;
  root->position = startPos;
  lastNode = root;
  nodes.push_back(root);
}

void RRT::setStart(const geometry_msgs::msg::PoseStamped & start){
    startPos.x() = start.pose.position.x;
    startPos.y() = start.pose.position.y;
}
void RRT::setGoal(const geometry_msgs::msg::PoseStamped & goal){
    endPos.x() = goal.pose.position.x;
    endPos.y() = goal.pose.position.y;
}

Node* RRT::randomSample(){
    Node* sample;
    Vector2f point(drand48() * 100, drand48() * 100);
    if (point.x() >= 0 && point.x() <= 100 && point.y() >= 0 && point.y() <= 100){
        sample = new Node;
        sample->position = point;
        return sample;
    }
    return NULL;
}

int RRT::distance(Vector2f &p, Vector2f &q){
    Vector2f v = p - q;
    return sqrt(powf(v.x(), 2) + powf(v.y(), 2));
}

void RRT::proximity(Vector2f point, float radius, vector<Node *>& out_nodes){
    for(int i = 0; i < (int)nodes.size(); i++){
        double dist = distance(point, nodes[i]->position);
        if (dist < radius) {
            out_nodes.push_back(nodes[i]);
        }
    }
}

Node* RRT::find_neighbor(Vector2f point){
    float minDist = 1e9;
    Node *closest = NULL;
    for(int i = 0; i < (int)nodes.size(); i++){
        float dist = distance(point, nodes[i]->position);
        if (dist < minDist){
            minDist = dist;
            closest = nodes[i];
        }
    }
    return closest;
}

Vector2f RRT::extend(Node *q, Node *qnear){
    Vector2f to = q->position;
    Vector2f from = qnear->position;
    Vector2f intermediate = to - from;
    intermediate = intermediate / intermediate.norm();
    Vector2f ret = from + step_size * intermediate;
    return ret;
}

double RRT::Cost(Node *q){
    return q->cost;
}

double RRT::PathCost(Node *qFrom, Node *qTo){
    return distance(qTo->position, qFrom->position);
}

void RRT::add(Node *qnear, Node *qnew){
    qnew->parent = qnear;
    qnear->children.push_back(qnew);
    nodes.push_back(qnew);
    lastNode = qnew;
}

bool RRT::reached(){
    if (distance(lastNode->position, endPos) < 1){
        return true;
    }
    return false;
}
}
//ADDED


namespace nav2_straightline_planner
{

void StraightLine::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
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
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
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
  rrt::RRT rrt;  // This is causing undefined symbol
  rrt.setRoot();
  rrt.setStart(start);
  rrt.setGoal(goal);
  std::cout << "New Start: (" << rrt.startPos.x() << ", " << rrt.startPos.y() << ")" << std::endl;
  std::cout << "New Goal: (" << rrt.endPos.x() << ", " << rrt.endPos.y() << ")" << std::endl;

  int max_iter = rrt.max_iter;
  for (int i = 0; i < max_iter; i++)
  {
      rrt::Node *q = rrt.randomSample();
      if (q){
          rrt::Node *qnear = rrt.find_neighbor(q->position);
          if (rrt.distance(q->position, qnear->position) > rrt.step_size) {
              Vector2f qnew_pos = rrt.extend(q, qnear);
              rrt::Node *qnew = new rrt::Node;
              qnew->position = qnew_pos;
              rrt.add(qnear, qnew);
          };
      };

      if (rrt.reached()){
          std::cout << "Path Found\n" << std::endl;
          rrt::Node *qGoal;
          qGoal->position.x() = rrt.endPos.x();
          qGoal->position.y() = rrt.endPos.y();
          rrt.path.push_back(qGoal);
          break;
      };
  }

  std::cout << "Compute Path" << std::endl;
  rrt::Node *qvertex;
  if (rrt.reached()) {
      qvertex = rrt.lastNode;
  }
  else
  {
      qvertex = rrt.find_neighbor(rrt.endPos);
  }
  while (qvertex != NULL) {
    rrt.path.push_back(qvertex);
    qvertex = qvertex->parent;
  }

  // TODO
  // - rrt segmentation not working -> why?
  // - How do we set the map size? (need for random sample)

  for (size_t k = 0; k < rrt.path.size(); k++){
      rrt::Node *currNode = rrt.path[k];
      std::cout << "Path Node" << k << ": (" << currNode->position.x() << ", " << currNode->position.y() << ")" << std::endl;
  }

  std::cout << "Path Length: " << rrt.path.size() << std::endl;
  std::cout << "Sort Path" << std::endl;
  std::reverse(rrt.path.begin(), rrt.path.end());
  
  // Find total path length
  // For each path segment:
  std::cout << "Start Segmentation" << std::endl;
  for (size_t j = 0; j < (rrt.path.size()-1); j++){
    std::cout << "Segment Number: " << j << std::endl;
    // Update segment start and goal
    rrt::Node *segment_start = rrt.path[j];
    rrt::Node *segment_goal = rrt.path[j+1];

    // Find interpolation_resolution_ of each segment
    int total_number_of_loop = std::hypot(
    segment_goal->position.x() - segment_start->position.x(),
    segment_goal->position.y() - segment_start->position.y()) /
    0.1;
    double x_increment = (segment_goal->position.x() - segment_start->position.x()) / total_number_of_loop;
    double y_increment = (segment_goal->position.y() - segment_start->position.y()) / total_number_of_loop;

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
      pose.pose.position.x = segment_start->position.x() + x_increment * i;
      pose.pose.position.y = segment_start->position.y() + y_increment * i;
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

    geometry_msgs::msg::PoseStamped goal_pose = goal;
    goal_pose.header.stamp = node_->now();
    goal_pose.header.frame_id = global_frame_;
    global_path.poses.push_back(goal_pose);
  }

  // Test
  // calculating the number of loops for current value of interpolation_resolution_
  // Divide path into 2
  // int total_number_of_loop = std::hypot(
  //   goal.pose.position.x - start.pose.position.x,
  //   goal.pose.position.y - start.pose.position.y) /
  //   interpolation_resolution_;
  // double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
  // double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

  // for (int i = 0; i < total_number_of_loop; ++i) {
  //   geometry_msgs::msg::PoseStamped pose;
  //   pose.pose.position.x = start.pose.position.x + x_increment * i;
  //   pose.pose.position.y = start.pose.position.y + y_increment * i;
  //   pose.pose.position.z = 0.0;
  //   pose.pose.orientation.x = 0.0;
  //   pose.pose.orientation.y = 0.0;
  //   pose.pose.orientation.z = 0.0;
  //   pose.pose.orientation.w = 1.0;
  //   pose.header.stamp = node_->now();
  //   pose.header.frame_id = global_frame_;
  //   global_path.poses.push_back(pose);
  // }

  // geometry_msgs::msg::PoseStamped goal_pose = goal;
  // goal_pose.header.stamp = node_->now();
  // goal_pose.header.frame_id = global_frame_;
  // global_path.poses.push_back(goal_pose);

  return global_path;
}

}  // namespace nav2_straightline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)
