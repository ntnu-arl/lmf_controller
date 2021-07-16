#ifndef LMF_CONTROLLER_NODE_H
#define LMF_CONTROLLER_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>
#include <random>
#include <deque>
#include <time.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mav_msgs/RateThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <std_srvs/Empty.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include "pid.h"

namespace lmf_control
{

    template <typename T>
    inline void GetRosParameter(const ros::NodeHandle &nh,
                                const std::string &key,
                                const T &default_value,
                                T *value)
    {
        ROS_ASSERT(value != nullptr);
        bool have_parameter = nh.getParam(key, *value);
        if (!have_parameter)
        {
            ROS_WARN_STREAM("[rosparam]: could not find parameter " << nh.getNamespace()
                                                                    << "/" << key << ", setting to default: " << default_value);
            *value = default_value;
        }
    }

    inline double wrapYaw(double yaw_angle)
    {
        while (yaw_angle > M_PI)
        {
            yaw_angle = yaw_angle - 2.0 * M_PI;
        }

        while (yaw_angle < -M_PI)
        {
            yaw_angle = yaw_angle + 2.0 * M_PI;
        }
        return yaw_angle;
    }

    // Default values.
    static const std::string kDefaultNamespace = "";
    static const std::string kDefaultCommandMotorSpeedTopic =
        mav_msgs::default_topics::COMMAND_ACTUATORS; // "command/motor_speed";
    static const std::string kDefaultCommandMultiDofJointTrajectoryTopic =
        mav_msgs::default_topics::COMMAND_TRAJECTORY; // "command/trajectory"
    static const std::string kDefaultCommandRollPitchYawrateThrustTopic =
        mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST;
    // "command/roll_pitch_yawrate_thrust"
    static const std::string kDefaultImuTopic =
        mav_msgs::default_topics::IMU; // "imu
    static const std::string kDefaultOdometryTopic =
        mav_msgs::default_topics::ODOMETRY; // "odometry"

    class LMFControllerNode
    {
    public:
        LMFControllerNode();
        ~LMFControllerNode();

    private:
        static constexpr double kGravity = 9.8066;

        bool use_vehicle_frame;
        bool receive_first_odom;
        bool receive_thrust_cmd;
        bool receive_vel_cmd;
        bool receive_pos_cmd;
        bool receive_first_goal;
        bool use_yaw_stabilize;
        bool fixed_height;
        bool swap_yaw_rate;
        bool is_sim;
        mav_msgs::EigenOdometry odometry;
        mav_msgs::RateThrust rate_thrust_cmd;
        mav_msgs::EigenOdometry goal_odometry, goal_training_odometry;
        geometry_msgs::Twist cmd_vel_V;
        double goal_yaw;
        std::string frame_id, vehicle_frame_id;
        double K_yaw;
        double yaw_rate_limit;
        double Kp_x, Ki_x, Kd_x, acc_x_max, alpha_x;
        double Kp_y, Ki_y, Kd_y, acc_y_max, alpha_y;
        double Kp_z, Ki_z, Kd_z, acc_z_max, alpha_z;
        double Kp_vel_x, Ki_vel_x, Kd_vel_x, alpha_vel_x;
        double Kp_vel_y, Ki_vel_y, Kd_vel_y, alpha_vel_y;
        double Kp_vel_z, Ki_vel_z, Kd_vel_z, alpha_vel_z;
        double odom_dtime;
        double z_static;
        PID *pid_x;
        PID *pid_y;
        PID *pid_z;
        PID *pid_vel_x;
        PID *pid_vel_y;
        PID *pid_vel_z;

        double mass;

        // subscribers
        ros::Subscriber cmd_rate_thrust_sub_;
        ros::Subscriber odometry_sub_;
        ros::Subscriber goal_pose_sub_;
        ros::Subscriber goal_training_pose_sub_;
        ros::Subscriber cmd_velocity_sub_;

        ros::Publisher cmd_roll_pitch_yawrate_thrust_pub_;

        ros::ServiceServer reset_service_;

        void RateThrustCallback(const mav_msgs::RateThrustPtr &rate_thrust_msg);

        void OdometryCallback(const nav_msgs::OdometryConstPtr &odometry_msg);

        void CmdPositionCallback(const geometry_msgs::Pose &goal_msg);

        void CmdVelocityCallback(const geometry_msgs::Twist &cmd_vel);

        bool ResetCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

        void convertGoal2WorldFrame(const geometry_msgs::Pose &goal, const mav_msgs::EigenOdometry &robot_odom, mav_msgs::EigenOdometry *goal_in_world);

        void convertGoal2VehicleFrame(const mav_msgs::EigenOdometry &goal_odom, const mav_msgs::EigenOdometry &robot_odom,
                                      nav_msgs::Odometry *goal_in_vehicle_frame);

        double calculateYawCtrl(double setpoint_yaw, double current_yaw);
    };
} // namespace lmf_control

#endif // LMF_CONTROLLER_NODE_H