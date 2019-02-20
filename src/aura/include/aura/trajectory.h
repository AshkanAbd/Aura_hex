#ifndef AURA_TRAJECTORY_HELPER_H
#define AURA_TRAJECTORY_HELPER_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>


namespace trajectory {

    // Trajectory flags

    const static int LAND = 1;
    const static int TAKEOFF = 2;
    const static int LEFT = 3;
    const static int RIGHT = 4;
    const static int UP = 5;
    const static int DOWN = 6;
    const static int ROTATE = 7;
    const static int FORWARD = 8;
    const static int BACKWARD = 9;

    // Structure for each node of Trajectory vector
    struct TrajectoryNode {
    private:
        uint time;
        geometry_msgs::TwistPtr twist;
        int flag;

    public:
        TrajectoryNode(uint time, const geometry_msgs::TwistPtr &twist, int flags = 0);

        TrajectoryNode();

        uint get_time() const;

        void set_time(uint time);

        const geometry_msgs::TwistPtr &get_twist() const;

        void set_twist(const geometry_msgs::TwistPtr &twist);

        int get_flag() const;

        void set_flag(int flag);
    };

    // Trajectory class that saves each move info
    class Trajectory {
    private:
        std::vector<TrajectoryNode> *moves, *reverse;

        ulong index = 0;

    public:
        Trajectory();

        // add each move to Trajectory vector
        void add(uint time, const geometry_msgs::TwistPtr &twist, int flag = 0);

        // calculate reverse vector for reverse time
        void calculate_reverse();

        // get next move in reverse time
        int get_next(TrajectoryNode &node);
    };

    void get_reverse(const geometry_msgs::TwistPtr &src, geometry_msgs::TwistPtr &dst);

    int get_reverse(int flag);
}

#endif
