#ifndef RRTC_TEST_H
#define RRTC_TEST_H
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <eigen3/Eigen/Dense>
#include <random>
#include <vector>
// #include "obstacles.h"

using namespace std;
using namespace Eigen;
namespace rrtc {

struct Node {
    vector<Node*> children;
    Node* parent;
    Vector2d position;
    float orientation;
    double cost;
};
class RRTC {
public:
    RRTC();
    void initialize();
    void setStart(const geometry_msgs::msg::PoseStamped& start);
    void setGoal(const geometry_msgs::msg::PoseStamped& goal);
    Node* randomSample(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal, int tree_counter);
    Node* find_neighbor(Vector2d point, int tree_counter);
    int distance(Vector2d& p, Vector2d& q);
    void proximity(Vector2d point, float radius, vector<Node*>& out_nodes, int tree_counter);
    Vector2d extend(Node* q, Node* qnear);
    double Cost(Node* q);
    double PathCost(Node* qFrom, Node* qTo);
    void add(Node* qnear, Node* qnew, int tree_counter);
    bool reached(int tree_counter);

    vector<Node*> nodesStart, nodesGoal;
    vector<Node*> path, startPath, goalPath;
    Node *rootStart, *rootGoal, *lastStartNode, *lastGoalNode;
    Vector2d startPos, endPos;
    std::default_random_engine generator;

    int max_iter;
    double step_size;
};

}
#endif
