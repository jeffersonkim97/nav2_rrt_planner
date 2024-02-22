#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rrtc.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

class RRTCNodePublisher : public rclcpp::Node{
    public:
        RRTCNodePublisher()
        : Node("rrtc_node_publisher"), count_(0){
            publisher_line_list_ = this->create_publisher<visualization_msgs::msg::Marker>("rrtp_nodes", 10);
            timer_ = this->create_wall_timer(
                500ms, std::bind(&RRTCNodePublisher::rrtc_callback, this)
            );
            }

    private:
        void rrtc_callback() {

            rrtc::RRTC rrtc;

            int max_iter = rrtc.max_iter;
            // int max_iter = 20;

            for (int i = 0; i < max_iter; i++)
            {
                rrtc::Node *q = rrtc.randomSample();
                if (q){
                    rrtc::Node *qnear = rrtc.find_neighbor(q->position, i);
                    if (rrtc.distance(q->position, qnear->position) > rrtc.step_size) {
                        Vector2f qnew_pos = rrtc.extend(q, qnear);
                        rrtc::Node *qnew = new rrtc::Node;
                        qnew->position = qnew_pos;
                        rrtc.add(qnear, qnew, i);
                    } else{ 
                        rrtc.add(qnear, q, i);
                    };
                };
                
                if (rrtc.reached(i)){
                    std::cout << "End Position: " << rrtc.endPos << std::endl;
                    std::cout << "Destination Reached \n" << std::endl;
                    break;
                };
            }

            // start tree
            auto markerStart = visualization_msgs::msg::Marker();
            markerStart.header.stamp.sec = 0;
            markerStart.header.stamp.nanosec = 0;
            markerStart.header.frame_id="map";
            markerStart.ns = "red_line";
            markerStart.id = 0;
            markerStart.type = visualization_msgs::msg::Marker::LINE_LIST;
            markerStart.action = 0;
            markerStart.pose = geometry_msgs::msg::Pose();
            markerStart.scale.set__x(0.08);
            markerStart.color.a = 1.0;
            markerStart.color.b = 0.0;
            markerStart.color.g = 0.0;
            markerStart.color.r = 1.0;
            visualization(&markerStart, rrtc.rootStart, 0);

            // goal tree
            auto markerGoal = visualization_msgs::msg::Marker();
            markerGoal.header.stamp.sec = 0;
            markerGoal.header.stamp.nanosec = 0;
            markerGoal.header.frame_id="map";
            markerGoal.ns = "blue_line";
            markerGoal.id = 0;
            markerGoal.type = visualization_msgs::msg::Marker::LINE_LIST;
            markerGoal.action = 0;
            markerGoal.pose = geometry_msgs::msg::Pose();
            markerGoal.scale.set__x(0.08);
            markerGoal.color.a = 1.0;
            markerGoal.color.b = 1.0;
            markerGoal.color.g = 0.0;
            markerGoal.color.r = 0.0;
            visualization(&markerGoal, rrtc.rootGoal, 0);
            RCLCPP_INFO(this->get_logger(), "Publishing");
            publisher_line_list_->publish(markerStart);
            publisher_line_list_->publish(markerGoal);

        };

        void visualization(visualization_msgs::msg::Marker *marker, rrtc::Node *parent, int depth){
            auto point_parent = geometry_msgs::msg::Point();
            point_parent.x = parent->position.x();
            point_parent.y = parent->position.y();
            point_parent.z = 0;

            for (size_t i=0; i < parent->children.size(); i++){
                rrtc::Node *child = parent->children[i];

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
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_line_list_;

        size_t count_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RRTCNodePublisher>());
    rclcpp::shutdown();
    return 0;
}

// vi: ts=4 sw=4 et
