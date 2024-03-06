#ifndef RRT_TEST_H
#define RRT_TEST_H
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <eigen3/Eigen/Dense>
#include <random>
#include <vector>
// #include "obstacles.h"

using namespace std;
using namespace Eigen;
namespace rrt {

struct Node {
    vector<Node*> children;
    Node* parent;
    Vector2d position;
    float orientation;
    double cost;
};
class RRT {
public:
    RRT();
    void initialize();
    void setStart(const geometry_msgs::msg::PoseStamped& start);
    void setGoal(const geometry_msgs::msg::PoseStamped& goal);
    Node* randomSample(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal);
    Node* randomSample(const double xmin, const double xmax, const double ymin, const double ymax, const geometry_msgs::msg::PoseStamped& goal);
    Node* find_neighbor(Vector2d point);
    int distance(Vector2d& p, Vector2d& q);
    void proximity(Vector2d point, float radius, vector<Node*>& out_nodes);
    Vector2d extend(Node* q, Node* qnear);
    double Cost(Node* q);
    double PathCost(Node* qFrom, Node* qTo);
    void add(Node* qnear, Node* qnew);
    bool reached();

    vector<Node*> nodes;
    vector<Node*> path;
    Node *root, *lastNode;
    Vector2d endPos;
    std::default_random_engine generator;

    int max_iter;
    double step_size;
};

}
#endif
