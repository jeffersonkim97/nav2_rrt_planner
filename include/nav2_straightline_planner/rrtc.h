#ifndef RRTC_TEST_H
#define RRTC_TEST_H

#include <iostream>
#include <vector>
#include <string>
#include <vector>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include "constants.h"
#include <stdio.h>
#include <string.h>

using namespace std;
using namespace Eigen;
namespace rrtc{

struct Node {
    vector<Node *> children;
    Node *parent;
    Vector2f position;
    float orientation;
    double cost;
    int counter;
};

class RRTC{
    public:
        RRTC();
        void initialize();
        void setStart(const geometry_msgs::msg::PoseStamped& start);
        void setGoal(const geometry_msgs::msg::PoseStamped& goal);
        Node* randomSample(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal);
        Node* find_neighbor(Vector2f point, int counter);
        int distance(Vector2f &p, Vector2f &q);
        void proximity(Vector2f point, float radius, vector<Node *>& out_nodes, int counter);
        Vector2f extend(Node *q, Node *qnear);
        double Cost(Node *q);
        double PathCost(Node *qFrom, Node *qTo);
        void add(Node *qnear, Node *qnew, int counter);
        bool reached(int counter);

        vector<Node *> nodesStart, nodesGoal;
        vector<Node *> path;
        Node *rootStart, *rootGoal, *lastStartNode, *lastGoalNode;
        Vector2f startPos, endPos;
        std::default_random_engine generator;

        int max_iter;
        int step_size;
};
}
# endif