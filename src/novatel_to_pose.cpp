#include "novatel_to_pose/novatel_to_pose.h"

namespace venus
{
    NovatelToPose::NovatelToPose(ros::NodeHandle &node, ros::NodeHandle &private_nh)
    {
        wgs84pj_source_ = pj_init_plus(WGS84_TEXT_);
        utm_target_ = pj_init_plus(UTM_TEXT_);

        private_nh.param("use_gps_time", use_gps_time_, false);

        std::string current_pose_topic;
        private_nh.param("current_pose_topic", current_pose_topic, std::string("current_pose"));
        pose_pub_ = node.advertise<geometry_msgs::PoseStamped>(current_pose_topic, 10, true);

        nov_sub_ = node.subscribe("novatel/oem7/inspva", 10, &NovatelToPose::novatel_inspva_callback,
                                  this, ros::TransportHints().tcpNoDelay(true));

        pose_msg_.header.frame_id = "base_link";
    }

    void NovatelToPose::novatel_inspva_callback(const novatel_oem7_msgs::INSPVA::Ptr &msg)
    {
        // Get gps time from inspva
        const double gps_secs =
            gps2unix(msg->nov_header.gps_week_number * 604800 + msg->nov_header.gps_week_milliseconds * 0.001);

        double x = msg->longitude * DEG_TO_RAD;
        double y = msg->latitude * DEG_TO_RAD;
        pj_transform(wgs84pj_source_, utm_target_, 1, 1, &x, &y, NULL);

        // Set current pose
        Pose current_pose;
        current_pose.x = x;
        current_pose.y = y;
        current_pose.z = msg->height;
        current_pose.roll = msg->roll * DEG_TO_RAD;
        current_pose.pitch = msg->pitch * DEG_TO_RAD;
        // current_pose.yaw = -(msg->azimuth) * DEG_TO_RAD;
        current_pose.yaw = (90 - msg->azimuth) * DEG_TO_RAD;
        // std::cout << "yaw: " << (90 - msg->azimuth) << std::endl;

        Eigen::Quaterniond current_quat = Eigen::AngleAxisd(current_pose.roll, Eigen::Vector3d::UnitY()) *
                                          Eigen::AngleAxisd(current_pose.pitch, Eigen::Vector3d::UnitX()) *
                                          Eigen::AngleAxisd(current_pose.yaw, Eigen::Vector3d::UnitZ());
        current_quat.normalize();

        if (!is_init_pose_ && msg->status.status == msg->status.INS_SOLUTION_GOOD)
        {
            init_pos_.translation() << current_pose.x, current_pose.y, current_pose.z;
            init_pos_.linear() = current_quat.toRotationMatrix();

            pose_msg_.pose.position.x = 0;
            pose_msg_.pose.position.y = 0;
            pose_msg_.pose.position.z = 0;
            pose_msg_.pose.orientation.w = 1;
            pose_msg_.pose.orientation.x = 0;
            pose_msg_.pose.orientation.y = 0;
            pose_msg_.pose.orientation.z = 0;


            // // Set init pose
            // init_pose_ = current_pose;

            // pose_msg_.pose.position.x = 0; //init_pose_.x;
            // pose_msg_.pose.position.y = 0; //init_pose_.y;
            // pose_msg_.pose.position.z = 0; //init_pose_.z;
            // pose_msg_.pose.orientation.w = current_quat.w();
            // pose_msg_.pose.orientation.x = current_quat.x();
            // pose_msg_.pose.orientation.y = current_quat.y();
            // pose_msg_.pose.orientation.z = current_quat.z();

            is_init_pose_ = true;
        }
        else if (is_init_pose_)
        {
            // pose_msg_.pose.position.x = current_pose.x - init_pose_.x;
            // pose_msg_.pose.position.y = current_pose.y - init_pose_.y;
            // pose_msg_.pose.position.z = current_pose.z - init_pose_.z;
            // pose_msg_.pose.orientation.w = current_quat.w();
            // pose_msg_.pose.orientation.x = current_quat.x();
            // pose_msg_.pose.orientation.y = current_quat.y();
            // pose_msg_.pose.orientation.z = current_quat.z();

            Eigen::Affine3d current_pos;
            current_pos.translation() << current_pose.x, current_pose.y, current_pose.z;
            current_pos.linear() = current_quat.toRotationMatrix();

            Eigen::Affine3d c2i_pos = init_pos_.inverse() * current_pos;
            Eigen::Quaterniond c2i_quat(c2i_pos.rotation());
            c2i_quat.normalize();
            Eigen::Vector3d euler = c2i_quat.toRotationMatrix().eulerAngles(0, 1, 2);

            // std::cout << "yaw: " << euler[2] * RAD_TO_DEG << std::endl;
            // pose_msg_.pose.position.x = c2i_pos.translation().y();
            // pose_msg_.pose.position.y = -c2i_pos.translation().x();
            pose_msg_.pose.position.x = c2i_pos.translation().x();
            pose_msg_.pose.position.y = c2i_pos.translation().y();
            pose_msg_.pose.position.z = c2i_pos.translation().z();
            pose_msg_.pose.orientation.w = c2i_quat.w();
            pose_msg_.pose.orientation.x = c2i_quat.x();
            pose_msg_.pose.orientation.y = c2i_quat.y();
            pose_msg_.pose.orientation.z = c2i_quat.z();
        }

        // pose_msg_.header.frame_id = "base_link";
        if (use_gps_time_)
            pose_msg_.header.stamp = ros::Time(gps_secs);
        else
            pose_msg_.header.stamp = msg->header.stamp;
        pose_msg_.header.seq = msg->header.seq;

        pose_pub_.publish(pose_msg_);
    }
} // namespace venus
