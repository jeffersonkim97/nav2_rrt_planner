#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "nav2_straightline_planner/rrt.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

class RRTNodePublisher : public rclcpp::Node{
    public:
        RRTNodePublisher()
        : Node("rrt_node_publisher"), count_(0){
            publisher_line_list_ = this->create_publisher<visualization_msgs::msg::Marker>("rrt_nodes", 10);
            timer_ = this->create_wall_timer(
                5000ms, std::bind(&RRTNodePublisher::rrt_callback, this)
            );
        }

    private:
        void rrt_callback() {

            rrt::RRT rrt;

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
                    break;
                };
            }
            rrt::Node *qvertex;
            if (rrt.reached()) {
                qvertex = rrt.lastNode;
            }
            else
            {
                qvertex = rrt.find_neighbor(rrt.endPos);
            }
            // generate shortest path to destination.
            int kek = 0;
            while (qvertex != NULL) {
                kek++;
                // std::cout << "path q" << kek << ": (" << qvertex->position.x() << ", " << qvertex->position.y() << ")" << std::endl;
                rrt.path.push_back(qvertex);
                qvertex = qvertex->parent;
            }

            // std::cout << "path length: " << rrt.path.size() << std::endl;

            auto marker = visualization_msgs::msg::Marker();
            marker.header.stamp.sec = 0;
            marker.header.stamp.nanosec = 0;
            marker.header.frame_id="map";
            marker.ns = "while_line";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            marker.action = 0;
            marker.pose = geometry_msgs::msg::Pose();
            marker.scale.set__x(0.08);
            marker.color.a = 1.0;
            marker.color.b = 1.0;
            marker.color.g = 1.0;
            marker.color.r = 1.0;
            visualization(&marker, rrt.root, 0);
            RCLCPP_INFO(this->get_logger(), "Publishing RRT Nodes");
            publisher_line_list_->publish(marker);

            auto path_marker = visualization_msgs::msg::Marker();
            path_marker.header.stamp.sec = 0;
            path_marker.header.stamp.nanosec = 0;
            path_marker.header.frame_id="map";
            path_marker.ns = "path_line";
            path_marker.id = 1;
            path_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            path_marker.action = 0;
            path_marker.pose = geometry_msgs::msg::Pose();
            path_marker.scale.set__x(0.08);
            path_marker.color.a = 1.0;
            path_marker.color.b = 0.0;
            path_marker.color.g = 1.0;
            path_marker.color.r = 0.0;
            path_visualization(&path_marker, rrt.lastNode);
            RCLCPP_INFO(this->get_logger(), "Publishing RRT Path");
            publisher_line_list_->publish(path_marker);
        };

        void visualization(visualization_msgs::msg::Marker *marker, rrt::Node *parent, int depth){
            // std::cout << std::string("    ", depth) << "vis parent: " << parent << " children: " << parent->children.size() << " depth: "<< depth << std::endl;

            auto point_parent = geometry_msgs::msg::Point();
            point_parent.x = parent->position.x();
            point_parent.y = parent->position.y();
            point_parent.z = 0;

            for (size_t i=0; i < parent->children.size(); i++){
                // std::cout << "i: " << i <<std::endl;
                rrt::Node *child = parent->children[i];

                // std::cout << std::string("    ", depth) << "child: " << child << " parent: " << parent << std::endl;
                if (child == parent) {
                    continue;
                }
                auto point_child = geometry_msgs::msg::Point();
                point_child.x = child->position.x();
                point_child.y = child->position.y();
                point_child.z = 0;
                marker->points.push_back(point_parent);
                marker->points.push_back(point_child);
                visualization(marker, child, depth + 1);
            }

        };

        void path_visualization(visualization_msgs::msg::Marker *marker, rrt::Node *child){
            // std::cout << "Child:" << std::endl;
            auto point_child = geometry_msgs::msg::Point();
            point_child.x = child->position.x();
            point_child.y = child->position.y();
            point_child.z = 0;
            // std::cout << "Child Pos: (" << point_child.x << ", " << point_child.y << ")"<< std::endl;


            rrt::Node *parent = child->parent;
            if (parent == NULL) {
                return;
            }
            // std::cout << "Parent:" << std::endl;
            auto point_parent = geometry_msgs::msg::Point();
            point_parent.x = parent->position.x();
            point_parent.y = parent->position.y();
            point_parent.z = 0;
            // std::cout << "Parent Pos: (" << point_parent.x << ", " << point_parent.y << ")"<< std::endl;
            marker->points.push_back(point_child);
            marker->points.push_back(point_parent);
            // std::cout << "Next" << std::endl;
            path_visualization(marker, parent);
        };
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_line_list_;

        size_t count_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RRTNodePublisher>());
    rclcpp::shutdown();
    return 0;
}

// vi: ts=4 sw=4 et
