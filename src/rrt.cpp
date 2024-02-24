#include "nav2_straightline_planner/rrt.h"

namespace rrt {
RRT::RRT()
{
    // obstacles = new Obstacles;
    // RRT Loop Control
    step_size = 1;
    max_iter = 100;
    
    root = new Node;
    root->parent = NULL;
    lastNode = root;
    nodes.push_back(root);
}

void RRT::setStart(const geometry_msgs::msg::PoseStamped& start)
{
    root->position.x() = start.pose.position.x;
    root->position.y() = start.pose.position.y;
}
void RRT::setGoal(const geometry_msgs::msg::PoseStamped& goal)
{
    endPos.x() = goal.pose.position.x;
    endPos.y() = goal.pose.position.y;
}

Node* RRT::randomSample(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal)
{
    Vector2d point(std::uniform_real_distribution<double>(start.pose.position.x, goal.pose.position.x)(generator),
        std::uniform_real_distribution<double>(start.pose.position.y, goal.pose.position.y)(generator));

    Node* sample = new Node;
    sample->position = point;
    return sample;
}

int RRT::distance(Vector2d& p, Vector2d& q)
{
    Vector2d v = p - q;
    return v.norm();
}

void RRT::proximity(Vector2d point, float radius, vector<Node*>& out_nodes)
{
    for (int i = 0; i < (int)nodes.size(); i++) {
        double dist = distance(point, nodes[i]->position);
        if (dist < radius) {
            out_nodes.push_back(nodes[i]);
        }
    }
}

Node* RRT::find_neighbor(Vector2d point)
{
    float minDist = 1e9;
    Node* closest = NULL;
    for (int i = 0; i < (int)nodes.size(); i++) {
        float dist = distance(point, nodes[i]->position);
        if (dist < minDist) {
            minDist = dist;
            closest = nodes[i];
        }
    }
    return closest;
}

Vector2d RRT::extend(Node* q, Node* qnear)
{
    Vector2d to = q->position;
    Vector2d from = qnear->position;
    Vector2d intermediate = to - from;
    intermediate = intermediate / intermediate.norm();
    Vector2d ret = from + step_size * intermediate;
    return ret;
}

double RRT::Cost(Node* q)
{
    return q->cost;
}

double RRT::PathCost(Node* qFrom, Node* qTo)
{
    return distance(qTo->position, qFrom->position);
}

void RRT::add(Node* qnear, Node* qnew)
{
    qnew->parent = qnear;
    qnear->children.push_back(qnew);
    nodes.push_back(qnew);
    lastNode = qnew;
}

bool RRT::reached()
{
    // std::cout << "distance to goal: " << distance(lastNode->position, endPos) << std::endl;
    // std::cout << "Within Threshold?: " << (distance(lastNode->position, endPos) < 1e-3) << std::endl;
    if (distance(lastNode->position, endPos) < 1e-3) {
        return true;
    }
    return false;
}
}
