#include "nav2_straightline_planner/rrtc.h"

namespace rrtc {
RRTC::RRTC()
{
    // obstacles = new Obstacles;
    // RRT Loop Control
    step_size = 0.25;
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

void RRTC::setStart(const geometry_msgs::msg::PoseStamped& start)
{
    startPos.x() = start.pose.position.x;
    startPos.y() = start.pose.position.y;
    rootStart->position.x() = start.pose.position.x;
    rootStart->position.y() = start.pose.position.y;
}
void RRTC::setGoal(const geometry_msgs::msg::PoseStamped& goal)
{
    endPos.x() = goal.pose.position.x;
    endPos.y() = goal.pose.position.y;
    rootGoal->position.x() = goal.pose.position.x;
    rootGoal->position.y() = goal.pose.position.y;
}

Node* RRTC::randomSample(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal, int tree_counter)
{
    double sampleGoal = std::uniform_real_distribution<double>(0, 1)(generator);
    double buffer = 0.5;

    Vector2d point;
    if (sampleGoal <= 0.1){
        // point.x() = std::uniform_real_distribution<double>(std::min(start.pose.position.x-buffer, start.pose.position.x+buffer), std::max(goal.pose.position.x-buffer, goal.pose.position.x+buffer))(generator);
        // point.y() = std::uniform_real_distribution<double>(std::min(start.pose.position.y-buffer, start.pose.position.y+buffer), std::max(goal.pose.position.y-buffer, goal.pose.position.y+buffer))(generator);
        // point.x() = std::uniform_real_distribution<double>(start.pose.position.x, goal.pose.position.x)(generator);
        // point.y() = std::uniform_real_distribution<double>(start.pose.position.y, goal.pose.position.y)(generator);
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


int RRTC::distance(Vector2d& p, Vector2d& q)
{
    Vector2d v = p - q;
    return v.norm();
}

void RRTC::proximity(Vector2d point, float radius, vector<Node*>& out_nodes, int tree_counter)
{
    if (tree_counter%2 == 0){
        for (int i = 0; i < (int)nodesStart.size(); i++) {
            double dist = distance(point, nodesStart[i]->position);
            if (dist < radius) {
                out_nodes.push_back(nodesStart[i]);
            }
        }
    } else {
        for (int i = 0; i < (int)nodesGoal.size(); i++) {
            double dist = distance(point, nodesGoal[i]->position);
            if (dist < radius) {
                out_nodes.push_back(nodesGoal[i]);
            }
        }
    }
    
}

Node* RRTC::find_neighbor(Vector2d point, int tree_counter)
{
    float minDist = 1e9;
    Node* closest = NULL;

    if (tree_counter%2 == 0){
        for (int i = 0; i < (int)nodesStart.size(); i++) {
            float dist = distance(point, nodesStart[i]->position);
            if (dist < minDist) {
                minDist = dist;
                closest = nodesStart[i];
            }
        }
    } else {
        for (int i = 0; i < (int)nodesGoal.size(); i++) {
            float dist = distance(point, nodesGoal[i]->position);
            if (dist < minDist) {
                minDist = dist;
                closest = nodesGoal[i];
            }
        }
    }
    
    return closest;
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
    return distance(qTo->position, qFrom->position);
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
            if (distance(lastStartNode->position, nodesGoal[i]->position) < 1e-3) {
                return true;
            }
        }
        return false;
    } else {
        for (size_t i = 0; i < nodesStart.size(); i++){
            if (distance(lastGoalNode->position, nodesStart[i]->position) < 1e-3) {
                return true;
            }
        }
        return false;
    }
    
}
}
