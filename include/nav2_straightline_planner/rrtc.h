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
    double timestamp;
};
class RRTC {
public:
    RRTC();
    void initialize();
    void setStart(const geometry_msgs::msg::PoseStamped& start, double startTime);
    void setGoal(const geometry_msgs::msg::PoseStamped& goal, double goalTime);
    Node* randomSample(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal, int tree_counter);
    Node* randomSample(double x1, double y1, double x2, double y2, const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal, int tree_counter);
    Node* find_neighbor(Node* point, int tree_counter);
    int distance(Vector2d& p, Vector2d& q);
    Vector2d extend(Node* q, Node* qnear);
    double Cost(Node* q);
    double PathCost(Node* qFrom, Node* qTo);
    void add(Node* qnear, Node* qnew, int tree_counter);
    bool reached(int tree_counter);
    Node* reachedNode(int tree_counter);
    bool reachable(Node* p, Node* q, int tree_counter);

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
