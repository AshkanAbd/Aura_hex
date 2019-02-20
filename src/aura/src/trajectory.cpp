#include "../include/aura/trajectory.h"

using namespace trajectory;

// Trajectory functions

Trajectory::Trajectory() {
    Trajectory::moves = new std::vector<TrajectoryNode>;
    Trajectory::reverse = new std::vector<TrajectoryNode>;
}

void Trajectory::add(uint time, const geometry_msgs::TwistPtr &twist, int flag) {
    TrajectoryNode node(time, twist, flag);
    Trajectory::moves->push_back(node);
}

void Trajectory::calculate_reverse() {
    TrajectoryNode *node;
    for (const auto &n:*Trajectory::moves) {
        if (n.get_flag() == trajectory::LAND || n.get_flag() == trajectory::TAKEOFF) {
            node = new TrajectoryNode;
            node->set_flag(trajectory::get_reverse(n.get_flag()));
        } else {
            geometry_msgs::TwistPtr twistPtr(new geometry_msgs::Twist);
            trajectory::get_reverse(n.get_twist(), twistPtr);
            node = new TrajectoryNode(n.get_time(), twistPtr);
        }
        Trajectory::moves->push_back(*node);
    }
    Trajectory::index = Trajectory::reverse->size() - 1;
}

int Trajectory::get_next(trajectory::TrajectoryNode &node) {
    if ((Trajectory::reverse->size() > Trajectory::index) && (Trajectory::index > -1)) {
        node = Trajectory::reverse->operator[](Trajectory::index);
        Trajectory::index--;
        return 1;
    }
    return 0;
}

// namespace functions

void trajectory::get_reverse(const geometry_msgs::TwistPtr &src, geometry_msgs::TwistPtr &dst) {
    dst->linear.x = src->linear.x * -1;
    dst->linear.y = src->linear.y * -1;
    dst->linear.z = src->linear.z * -1;
    dst->angular.x = src->angular.x * -1;
    dst->angular.y = src->angular.y * -1;
    dst->angular.z = src->angular.z * -1;
}

int trajectory::get_reverse(int flag) {
    if (flag == trajectory::LAND) return trajectory::TAKEOFF;
    if (flag == trajectory::TAKEOFF) return trajectory::LAND;
    return 0;
}

// TrajectoryNode functions

TrajectoryNode::TrajectoryNode(uint time, const geometry_msgs::TwistPtr &twist, int flag) {
    TrajectoryNode::time = time;
    TrajectoryNode::twist = twist;
    TrajectoryNode::flag = flag;
}

TrajectoryNode::TrajectoryNode() {
    TrajectoryNode::twist = geometry_msgs::TwistPtr(new geometry_msgs::Twist);
}

uint TrajectoryNode::get_time() const {
    return time;
}

void TrajectoryNode::set_time(uint time) {
    TrajectoryNode::time = time;
}

const geometry_msgs::TwistPtr &TrajectoryNode::get_twist() const {
    return twist;
}

void TrajectoryNode::set_twist(const geometry_msgs::TwistPtr &twist) {
    TrajectoryNode::twist = twist;
}

int TrajectoryNode::get_flag() const {
    return flag;
}

void TrajectoryNode::set_flag(int flag) {
    TrajectoryNode::flag = flag;
}
