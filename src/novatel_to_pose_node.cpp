#include "novatel_to_pose/novatel_to_pose.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "novatel_to_pose");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    venus::NovatelToPose nov_to_pose(node, private_nh);
    ros::spin();
    return 0;
}