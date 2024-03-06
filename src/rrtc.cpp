#include "nav2_straightline_planner/rrtc.h"

namespace rrtc{
RRTC::RRTC(){
    // RRT Loop Control
    step_size = 0.1;
    max_iter = 1000;

    rootStart = new Node;
    rootStart->parent = NULL;
    rootStart->position = startPos;
    
    rootGoal = new Node;
    rootGoal->parent = NULL;
    rootGoal->position = endPos;
    lastStartNode = rootStart;
    lastGoalNode = rootGoal;
    nodesStart.push_back(rootStart);
    nodesGoal.push_back(rootGoal);
    
    // int counter = 0;
}

void RRTC::setStart(const geometry_msgs::msg::PoseStamped& start)
{
    root->position.x() = start.pose.position.x;
    root->position.y() = start.pose.position.y;
}
void RRTC::setGoal(const geometry_msgs::msg::PoseStamped& goal)
{
    endPos.x() = goal.pose.position.x;
    endPos.y() = goal.pose.position.y;
}

Node* RRTC::randomSample(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal)
{
    double sampleGoal = std::uniform_real_distribution<double>(0, 1)(generator);
    double buffer = 0.5;
    
    Vector2d point;
    if (sampleGoal <= 0.1){
        point.x() = std::uniform_real_distribution<double>(std::min(start.pose.position.x-buffer, start.pose.position.x+buffer), std::max(goal.pose.position.x-buffer, goal.pose.position.x+buffer))(generator);
        point.y() = std::uniform_real_distribution<double>(std::min(start.pose.position.y-buffer, start.pose.position.y+buffer), std::max(goal.pose.position.y-buffer, goal.pose.position.y+buffer))(generator);
    } else {
        point.x() = goal.pose.position.x;
        point.y() = goal.pose.position.y;
    }
    Node* sample;
    sample->position = point;
    return sample;

    // Vector2f point(drand48() * WORLD_WIDTH, drand48() * WORLD_HEIGHT);
    // if (point.x() >= 0 && point.x() <= WORLD_WIDTH && point.y() >= 0 && point.y() <= WORLD_HEIGHT){
    //     sample = new Node;
    //     sample->position = point;
    //     return sample;
    // }
    // return NULL;
}

int RRTC::distance(Vector2f &p, Vector2f &q){
    Vector2f v = p - q;
    return sqrt(powf(v.x(), 2) + powf(v.y(), 2));
}

void RRTC::proximity(Vector2f point, float radius, vector<Node *>& out_nodes, int counter){
    vector<Node *> nodes;
    if (counter % 2 == 0){
        nodes = nodesStart;
    } else {
        nodes = nodesGoal;
    }
    for(int i = 0; i < (int)nodes.size(); i++){
        double dist = distance(point, nodes[i]->position);
        if (dist < radius) {
            out_nodes.push_back(nodes[i]);
        }
    }
}

Node* RRTC::find_neighbor(Vector2f point, int counter){
    float minDist = 1e9;
    Node *closest = NULL;
    vector<Node *> nodes;
    if (counter % 2 == 0){
        nodes = nodesStart;
    } else {
        nodes = nodesGoal;
    }
    for(int i = 0; i < (int)nodes.size(); i++){
        float dist = distance(point, nodes[i]->position);
        if (dist < minDist){
            minDist = dist;
            closest = nodes[i];
        }
    }
    return closest;
}

Vector2f RRTC::extend(Node *q, Node *qnear){
    Vector2f to = q->position;
    Vector2f from = qnear->position;
    Vector2f intermediate = to - from;
    intermediate = intermediate / intermediate.norm();
    Vector2f ret = from + step_size * intermediate;
    return ret;
}

double RRTC::Cost(Node *q){
    return q->cost;
}

double RRTC::PathCost(Node *qFrom, Node *qTo){
    return distance(qTo->position, qFrom->position);
}

void RRTC::add(Node *qnear, Node *qnew, int counter){
    qnew->parent = qnear;
    qnear->children.push_back(qnew);

    if (counter % 2 == 0){
        lastStartNode = qnew;
        nodesStart.push_back(qnew);
    } else {
        lastGoalNode = qnew;
        nodesGoal.push_back(qnew);
    }
}

bool RRTC::reached(int counter){
    // Fix: iteratively go through each vertex and find connection within end threshold
    
    if (counter % 2 == 0){
        for (size_t i = 0; i < nodesGoal.size(); i++){
            Node *goalVertex = nodesGoal[i];
            if (distance(lastStartNode->position, goalVertex->position) < END_DIST_THRESHOLD){
                return true;
            }
        }
        return false;
    } else {
        for (size_t i = 0; i < nodesStart.size(); i++){
            Node *startVertex = nodesStart[i];
            if (distance(lastGoalNode->position, startVertex->position) < END_DIST_THRESHOLD){
                return true;
            }
        }
        return false;
    }
}
}