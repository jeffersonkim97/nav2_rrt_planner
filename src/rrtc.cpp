#include "nav2_straightline_planner/rrtc.h"
#include <iostream>

namespace rrtc {
RRTC::RRTC()
{
    // obstacles = new Obstacles;
    // RRT Loop Control
    step_size = 1.0;
    max_iter = 1000;

    // Initialize Start Tree
    rootStart = new Node;
    rootStart->parent = NULL;
    lastStartNode = rootStart;
    nodesStart.push_back(rootStart);

    // Initialize Goal Tree
    rootGoal = new Node;
    rootGoal->parent = NULL;
    lastGoalNode = rootGoal;
    nodesGoal.push_back(rootGoal);
}

void RRTC::setStart(const geometry_msgs::msg::PoseStamped& start, double startTime)
{
    startPos.x() = start.pose.position.x;
    startPos.y() = start.pose.position.y;
    rootStart->position.x() = start.pose.position.x;
    rootStart->position.y() = start.pose.position.y;
    rootStart->timestamp = startTime;
}
void RRTC::setGoal(const geometry_msgs::msg::PoseStamped& goal, double goalTime)
{
    endPos.x() = goal.pose.position.x;
    endPos.y() = goal.pose.position.y;
    rootGoal->position.x() = goal.pose.position.x;
    rootGoal->position.y() = goal.pose.position.y;
    rootGoal->timestamp = goalTime;
}

Node* RRTC::randomSample(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal, int tree_counter)
{
    double sampleGoal = std::uniform_real_distribution<double>(0, 1)(generator);
    double buffer = 1;

    Vector2d point;
    if (sampleGoal <= 0.1){
        point.x() = std::uniform_real_distribution<double>(std::min(start.pose.position.x, goal.pose.position.x), std::max(start.pose.position.x, goal.pose.position.x))(generator);
        point.y() = std::uniform_real_distribution<double>(std::min(start.pose.position.y, goal.pose.position.y), std::max(start.pose.position.y, goal.pose.position.y))(generator);
    } else {
        if (tree_counter%2 == 0){
            point.x() = goal.pose.position.x;
            point.y() = goal.pose.position.y;
        } else {
            point.x() = start.pose.position.x;
            point.y() = start.pose.position.y;
        }
        
    }

    Node* sample = new Node;
    sample->position = point;
    return sample;
}

Node* RRTC::randomSample(double x1, double y1, double x2, double y2, const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal, int tree_counter)
{
    double sampleGoal = std::uniform_real_distribution<double>(0, 1)(generator);
    double buffer = 0.5;

    Vector2d point;
    if (sampleGoal <= 0.1){
        double xmin = std::min(x1, goal.pose.position.x-buffer);
        double xmax = std::max(x2, goal.pose.position.x+buffer);
        double ymin = std::min(y1, goal.pose.position.y-buffer);
        double ymax = std::max(y2, goal.pose.position.y+buffer);

        point.x() = std::uniform_real_distribution<double>(xmin, xmax)(generator);
        point.y() = std::uniform_real_distribution<double>(ymin, ymax)(generator);
        // point.x() = std::uniform_real_distribution<double>(std::min(start.pose.position.x, goal.pose.position.x), std::max(start.pose.position.x, goal.pose.position.x))(generator);
        // point.y() = std::uniform_real_distribution<double>(std::min(start.pose.position.y, goal.pose.position.y), std::max(start.pose.position.y, goal.pose.position.y))(generator);

    } else {
        if (tree_counter%2 == 0){
            point.x() = goal.pose.position.x;
            point.y() = goal.pose.position.y;
        } else {
            point.x() = start.pose.position.x;
            point.y() = start.pose.position.y;
        }
        
    }

    Node* sample = new Node;
    sample->position = point;
    return sample;
}


int RRTC::distance(Vector2d& p, Vector2d& q)
{
    return sqrt((p.x()-q.x())*(p.x()-q.x()) + (p.y()-q.y())*(p.y()-q.y()));
}


Node* RRTC::find_neighbor(Node* point, int tree_counter)
{
    float minDist = 1e9;
    Node* closest = NULL;

    if (tree_counter%2 == 0){
        for (int i = 0; i < (int)nodesStart.size(); i++) {
            float dist = (point->position - nodesStart[i]->position).norm();
            if (dist < minDist) {
                minDist = dist;
                closest = nodesStart[i];
            }
        }
    } else {
        for (int i = 0; i < (int)nodesGoal.size(); i++) {
            float dist = (point->position - nodesGoal[i]->position).norm();
            if (dist < minDist) {
                minDist = dist;
                closest = nodesGoal[i];
            }
        }
    }
    
    return closest;
}

bool RRTC::reachable(Node* p, Node* q, int tree_counter){
    double dx = p->position.x() - q->position.x();
    double dy = p->position.y() - q->position.y();
    double dd = sqrt(dx*dx + dy*dy);
    double dt = p->timestamp - q->timestamp;

    if (dd == 0){
        return true;
    } else {
        if (tree_counter%2 == 0 && dt > 0 && dd/dt <= 0.5){
            return true;
        } else if (tree_counter%2 == 1 && dt < 0 && dd/dt <= 0.5){
            return true;
        }
    }
    return false;
}

Vector2d RRTC::extend(Node* q, Node* qnear)
{
    Vector2d to = q->position;
    Vector2d from = qnear->position;
    Vector2d intermediate = to - from;
    intermediate = intermediate / intermediate.norm();
    Vector2d ret = from + step_size * intermediate;
    return ret;
}

double RRTC::Cost(Node* q)
{
    return q->cost;
}

double RRTC::PathCost(Node* qFrom, Node* qTo)
{
    return (qTo->position - qFrom->position).norm();
}

void RRTC::add(Node* qnear, Node* qnew, int tree_counter)
{
    if (tree_counter%2 == 0){
        qnew->parent = qnear;
        qnear->children.push_back(qnew);
        nodesStart.push_back(qnew);
        lastStartNode = qnew;
    } else {
        qnew->parent = qnear;
        qnear->children.push_back(qnew);
        nodesGoal.push_back(qnew);
        lastGoalNode = qnew;
    }
    
}

bool RRTC::reached(int tree_counter)
{
    if (tree_counter%2 == 0){
        for (size_t i = 0; i < nodesGoal.size(); i++){
            if ((lastStartNode->position - nodesGoal[i]->position).norm() < 1e-3) {
                return true;
            }
        }
    } else {
        for (size_t i = 0; i < nodesStart.size(); i++){
            if ((lastGoalNode->position - nodesStart[i]->position).norm() < 1e-3) {
                return true;
            }
        }
    }
    return false;
}

Node* RRTC::reachedNode(int tree_counter)
{
    if (tree_counter%2 == 0){
        for (size_t i = 0; i < nodesGoal.size(); i++){
            if ((lastStartNode->position - nodesGoal[i]->position).norm() < 1e-3) {
                return nodesGoal[i];
            }
        }
    } else {
        for (size_t i = 0; i < nodesStart.size(); i++){
            if ((lastGoalNode->position - nodesStart[i]->position).norm() < 1e-3) {
                return nodesStart[i];
            }
        }
    }
    return NULL;
}
}
