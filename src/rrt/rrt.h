#ifndef RRT_TEST_H
#define RRT_TEST_H

#include <iostream>
#include <vector>
#include <string>
#include <vector>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include "constants.h"
#include <stdio.h>
#include <string.h>
// #include "obstacles.h"

using namespace std;
using namespace Eigen;
namespace rrt{

struct Node {
    vector<Node *> children;
    Node *parent;
    Vector2f position;
    float orientation;
    double cost;
};

class RRT{
    public:
        RRT();
        void initialize();
        Node* randomSample();
        Node* find_neighbor(Vector2f point);
        int distance(Vector2f &p, Vector2f &q);
        void proximity(Vector2f point, float radius, vector<Node *>& out_nodes);
        Vector2f extend(Node *q, Node *qnear);
        double Cost(Node *q);
        double PathCost(Node *qFrom, Node *qTo);
        void add(Node *qnear, Node *qnew);
        bool reached();

        // Obstacles *obstacles;
        vector<Node *> nodes;
        vector<Node *> path;
        Node *root, *lastNode;
        Vector2f startPos, endPos;

        int max_iter;
        int step_size;
};
}
# endif