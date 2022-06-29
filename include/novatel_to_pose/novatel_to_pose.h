#pragma once

#include <eigen3/Eigen/Eigen>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <novatel_oem7_msgs/INSPVA.h>
// #define ACCEPT_USE_OF_DEPRECATED_PROJ_API_H
#include <proj_api.h>
#include "novatel_to_pose/time_conversion.h"

namespace venus
{
    class NovatelToPose
    {
    public:
        NovatelToPose() = delete;
        NovatelToPose(ros::NodeHandle &node, ros::NodeHandle &private_nh);
        virtual ~NovatelToPose() = default;

    private:
        void novatel_inspva_callback(const novatel_oem7_msgs::INSPVA::Ptr &msg);

    private:
        struct Pose
        {
            double x;
            double y;
            double z;
            double roll;
            double pitch;
            double yaw;
        };

        const char *WGS84_TEXT_ = "+proj=latlong +ellps=WGS84";
        const char *UTM_TEXT_ = "+proj=utm +zone=50 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs";

        bool use_gps_time_ = false;

        bool is_init_pose_ = false;
        Pose init_pose_;

        Eigen::Affine3d init_pos_;

        projPJ wgs84pj_source_;
        projPJ utm_target_;

        geometry_msgs::PoseStamped pose_msg_;

        ros::Publisher pose_pub_;
        ros::Subscriber nov_sub_;
    };

} // namespace venus