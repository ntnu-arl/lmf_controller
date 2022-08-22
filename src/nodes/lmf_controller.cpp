#include "lmf_controller.h"

// #include <tf/transform_broadcaster.h>

namespace lmf_control
{
    LMFControllerNode::LMFControllerNode()
        : receive_first_odom(false),
          receive_thrust_cmd(false),
          receive_vel_cmd(false),
          receive_pos_cmd(false),
          receive_first_goal(false),
          has_new_range(false)
    {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");
        GetRosParameter(pnh, "is_sim", false, &is_sim);
        GetRosParameter(pnh, "use_range_sensor", false, &use_range_sensor);
        GetRosParameter(pnh, "use_vehicle_frame", true, &use_vehicle_frame);
        GetRosParameter(pnh, "use_yaw_stabilize", false, &use_yaw_stabilize);
        GetRosParameter(pnh, "fixed_height", false, &fixed_height);   // get latest height cmd from goal topic
        GetRosParameter(pnh, "swap_yaw_rate", false, &swap_yaw_rate); // use reference.angular_rates.z to store reference yaw angle

        GetRosParameter(pnh, "Kp_x", 0.0, &Kp_x);
        GetRosParameter(pnh, "Ki_x", 0.0, &Ki_x);
        GetRosParameter(pnh, "Kd_x", 0.0, &Kd_x);
        GetRosParameter(pnh, "Kp_vel_x", 0.0, &Kp_vel_x);
        GetRosParameter(pnh, "Ki_vel_x", 0.0, &Ki_vel_x);
        GetRosParameter(pnh, "Kd_vel_x", 0.0, &Kd_vel_x);
        ROS_WARN_STREAM("Kp_vel_x " << Kp_vel_x << ", Ki_vel_x " << Ki_vel_x << ", Kd_vel_x " << Kd_vel_x);
        GetRosParameter(pnh, "acc_x_max", 0.0, &acc_x_max);
        GetRosParameter(pnh, "alpha_x", 0.0, &alpha_x);

        GetRosParameter(pnh, "Kp_y", 0.0, &Kp_y);
        GetRosParameter(pnh, "Ki_y", 0.0, &Ki_y);
        GetRosParameter(pnh, "Kd_y", 0.0, &Kd_y);
        GetRosParameter(pnh, "Kp_vel_y", 0.0, &Kp_vel_y);
        GetRosParameter(pnh, "Ki_vel_y", 0.0, &Ki_vel_y);
        GetRosParameter(pnh, "Kd_vel_y", 0.0, &Kd_vel_y);
        ROS_WARN_STREAM("Kp_vel_y " << Kp_vel_y << ", Ki_vel_y " << Ki_vel_y << ", Kd_vel_y " << Kd_vel_y);        
        GetRosParameter(pnh, "acc_y_max", 0.0, &acc_y_max);
        GetRosParameter(pnh, "alpha_y", 0.0, &alpha_y);

        GetRosParameter(pnh, "Kp_z", 0.0, &Kp_z);
        GetRosParameter(pnh, "Ki_z", 0.0, &Ki_z);
        GetRosParameter(pnh, "Kd_z", 0.0, &Kd_z);
        GetRosParameter(pnh, "Kp_vel_z", 0.0, &Kp_vel_z);
        GetRosParameter(pnh, "Ki_vel_z", 0.0, &Ki_vel_z);
        GetRosParameter(pnh, "Kd_vel_z", 0.0, &Kd_vel_z);
        ROS_WARN_STREAM("Kp_vel_z " << Kp_vel_z << ", Ki_vel_z " << Ki_vel_z << ", Kd_vel_z " << Kd_vel_z); 
        GetRosParameter(pnh, "acc_z_max", 0.0, &acc_z_max);
        GetRosParameter(pnh, "alpha_z", 0.0, &alpha_z);

         
        // GetRosParameter(pnh, "antiwindup_radius_pos", antiwindup_radius_pos_default, &antiwindup_radius_pos);
        // GetRosParameter(pnh, "antiwindup_radius_vel", antiwindup_radius_vel_default, &antiwindup_radius_vel);
        // GetRosParameter(pnh, "integrator_pos_max", integrator_pos_max_default, &integrator_pos_max);
        // GetRosParameter(pnh, "integrator_vel_max", integrator_vel_max_default, &integrator_vel_max);
        if (!pnh.getParam("antiwindup_radius_pos", antiwindup_radius_pos)) {
            ROS_ERROR("antiwindup_radius_pos in lmf_controller is not loaded from ros parameter server");
            abort();
        }
        ROS_WARN_STREAM("antiwindup_radius_pos:" << antiwindup_radius_pos.at(0) << " "
                                                << antiwindup_radius_pos.at(1) << " "
                                                << antiwindup_radius_pos.at(2));
        
        if (!pnh.getParam("antiwindup_radius_vel", antiwindup_radius_vel)) {
            ROS_ERROR("antiwindup_radius_vel in lmf_controller is not loaded from ros parameter server");
            abort();
        }
        ROS_WARN_STREAM("antiwindup_radius_vel:" << antiwindup_radius_vel.at(0) << " "
                                                << antiwindup_radius_vel.at(1) << " "
                                                << antiwindup_radius_vel.at(2));
        
        if (!pnh.getParam("integrator_pos_max", integrator_pos_max)) {
            ROS_ERROR("integrator_pos_max in lmf_controller is not loaded from ros parameter server");
            abort();
        }
        ROS_WARN_STREAM("integrator_pos_max:" << integrator_pos_max.at(0) << " "
                                                << integrator_pos_max.at(1) << " "
                                                << integrator_pos_max.at(2));
        
        if (!pnh.getParam("integrator_vel_max", integrator_vel_max)) {
            ROS_ERROR("integrator_vel_max in lmf_controller is not loaded from ros parameter server");
            abort();
        }
        ROS_WARN_STREAM("integrator_vel_max:" << integrator_vel_max.at(0) << " "
                                                << integrator_vel_max.at(1) << " "
                                                << integrator_vel_max.at(2));

        GetRosParameter(pnh, "K_yaw", 1.8, &K_yaw);
        GetRosParameter(pnh, "yaw_rate_limit", 45.0, &yaw_rate_limit);
        yaw_rate_limit = yaw_rate_limit * M_PI/180;
        GetRosParameter(pnh, "z_static", 1.0, &z_static);
        ROS_WARN_STREAM("z_static:" << z_static);
        GetRosParameter(pnh, "mass", 1.0, &mass);
        ROS_WARN_STREAM("Mass:" << mass);

        rate_thrust_cmd.thrust.x = 0.0;
        rate_thrust_cmd.thrust.y = 0.0;
        rate_thrust_cmd.thrust.z = 0.0; // keep it on the ground
        rate_thrust_cmd.angular_rates.x = 0.0;
        rate_thrust_cmd.angular_rates.y = 0.0;
        rate_thrust_cmd.angular_rates.z = 0.0;

        //need to know current yaw angle of the robot if acc vector is expressed in world frame
        // cmd_rate_thrust_sub_ = nh.subscribe("command/rate_thrust", 1,
        //                                     &LMFControllerNode::RateThrustCallback, this);
        odometry_sub_ = nh.subscribe(kDefaultOdometryTopic, 1, &LMFControllerNode::OdometryCallback, this);
        // range_sub_ = nh.subscribe("range_sensor", 1, &LMFControllerNode::RangeCallback, this);
        // goal_pose_sub_ = nh.subscribe("goal", 1, &LMFControllerNode::CmdPositionCallback, this);
        cmd_velocity_sub_ = nh.subscribe("cmd_velocity", 1, &LMFControllerNode::CmdVelocityCallback, this);

        // cmd_roll_pitch_yawrate_thrust_pub_ = nh.advertise<mav_msgs::RollPitchYawrateThrust>(
        //     kDefaultCommandRollPitchYawrateThrustTopic, 1);
        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 1);

        reset_service_ = nh.advertiseService("pid_reset", &LMFControllerNode::ResetCallback, this);

        vehicle_frame_id = ros::this_node::getNamespace();
        ROS_WARN_STREAM("vehicle_frame_id:" << vehicle_frame_id);
    }

    LMFControllerNode::~LMFControllerNode() {}

    bool LMFControllerNode::ResetCallback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
    {
        ROS_INFO("Reset serviced called");
        pid_x->reset();
        pid_y->reset();
        pid_z->reset();
        pid_vel_x->reset();
        pid_vel_y->reset();
        pid_vel_z->reset();
        rate_thrust_cmd.thrust.x = 0.0;
        rate_thrust_cmd.thrust.y = 0.0;
        rate_thrust_cmd.thrust.z = 0.0;
        rate_thrust_cmd.angular_rates.x = 0.0;
        rate_thrust_cmd.angular_rates.y = 0.0;
        rate_thrust_cmd.angular_rates.z = 0.0;
        //receive_first_odom = false;
        receive_thrust_cmd = false;
        receive_pos_cmd = false;
        receive_vel_cmd = false;
        receive_first_goal = false;
        return true;
    }

    void LMFControllerNode::RateThrustCallback(
        const mav_msgs::RateThrustPtr &rate_thrust_msg)
    {
        if (!receive_first_odom)
        {
            return;
        }
        rate_thrust_cmd = *rate_thrust_msg;
        receive_thrust_cmd = true;
        receive_pos_cmd = false;
        receive_vel_cmd = false;
    }

    // should be in a seperate node
    void LMFControllerNode::CmdPositionCallback(const geometry_msgs::Pose &goal)
    {
        geometry_msgs::Pose goal_msg(goal);
        if ((goal_msg.orientation.x == 0.0) && (goal_msg.orientation.y == 0.0) && (goal_msg.orientation.z == 0.0) && (goal_msg.orientation.w == 0.0))
        {
            goal_msg.orientation.w = 1.0;
        }
        convertGoal2WorldFrame(goal_msg, odometry, &goal_odometry);
        Eigen::Vector3d goal_euler_angles;
        goal_odometry.getEulerAngles(&goal_euler_angles);
        goal_yaw = wrapYaw(goal_euler_angles(2));
        receive_pos_cmd = true;
        receive_first_goal = true;
        receive_vel_cmd = false;
        // DEBUG
        ROS_INFO_STREAM("Received goal: pos x " << goal_msg.position.x << ",y " << goal_msg.position.y << ",z " << goal_msg.position.z
                                                << ", orientation x " << goal_msg.orientation.x << ",y " << goal_msg.orientation.y << ",z " << goal_msg.orientation.z << ",w " << goal_msg.orientation.w);
        ROS_INFO_STREAM("Odom in world: pos x " << odometry.position_W(0) << ",y " << odometry.position_W(1) << ",z " << odometry.position_W(2));
        ROS_INFO_STREAM("Goal in world: pos x " << goal_odometry.position_W(0) << ",y " << goal_odometry.position_W(1) << ",z " << goal_odometry.position_W(2));
        ROS_INFO_STREAM("Goal yaw:" << goal_yaw * 180 / M_PI << " deg");
        ROS_INFO("**********");
    }

    void LMFControllerNode::CmdVelocityCallback(const geometry_msgs::Twist &cmd_vel)
    {
        double yaw_rate_cmd;
        Eigen::Vector3d current_rpy;
        odometry.getEulerAngles(&current_rpy);
        // convertROS2ENU(cmd_vel, cmd_vel_received); // cmd_vel_received is in ENU frame
        cmd_vel_received = cmd_vel;
        if (swap_yaw_rate) // receive reference yaw angle
        {
            goal_yaw = wrapYaw(cmd_vel_received.angular.z);
            yaw_rate_cmd = calculateYawCtrl(goal_yaw, current_rpy(2));
        }
        else
        {
            yaw_rate_cmd = cmd_vel_received.angular.z;
        }
        
        // TODO: convert to body frame?
        cmd_vel_send.linear = cmd_vel_received.linear;
        cmd_vel_send.angular.x = 0.0;
        cmd_vel_send.angular.y = 0.0;
        cmd_vel_send.angular.z = yaw_rate_cmd;
        cmd_vel_pub_.publish(cmd_vel_send);

        receive_pos_cmd = false;
        receive_vel_cmd = true;
        receive_thrust_cmd = false;
        // DEBUG
        ROS_INFO_STREAM("Received cmd_vel, cmd_vel: linear x " << cmd_vel.linear.x << ",y " << cmd_vel.linear.y << ",z " << cmd_vel.linear.z
                                                               << ", twist x " << cmd_vel.angular.x << ",y " << cmd_vel.angular.y << ",z " << cmd_vel.angular.z);
        ROS_INFO("**********");
    }

    void LMFControllerNode::convertROS2ENU(const geometry_msgs::Twist &twist_ros_msg, geometry_msgs::Twist &twist_enu_msg)
    {
        twist_enu_msg.linear.x = -twist_ros_msg.linear.y;
        twist_enu_msg.linear.y = twist_ros_msg.linear.x;
        twist_enu_msg.linear.z = twist_ros_msg.linear.z;

        twist_enu_msg.angular.x = -twist_ros_msg.angular.y;
        twist_enu_msg.angular.y = twist_ros_msg.angular.x;
        twist_enu_msg.angular.z = twist_ros_msg.angular.z;        
    }

    void LMFControllerNode::convertGoal2WorldFrame(const geometry_msgs::Pose &goal, const mav_msgs::EigenOdometry &robot_odom,
                                                   mav_msgs::EigenOdometry *goal_in_world)
    {
        Eigen::Vector3d robot_euler_angles;
        robot_odom.getEulerAngles(&robot_euler_angles);
        Eigen::Quaterniond goal_quat_in_world;
        Eigen::Vector3d goal_pos_in_world;
        if (use_vehicle_frame)
        {
            Eigen::Quaterniond goal_quat_in_vehicle = Eigen::Quaterniond(goal.orientation.w, goal.orientation.x, goal.orientation.y, goal.orientation.z);
            goal_quat_in_vehicle = goal_quat_in_vehicle.normalized();
            Eigen::Vector3d goal_euler_angles;
            mav_msgs::getEulerAnglesFromQuaternion(goal_quat_in_vehicle, &goal_euler_angles);
            goal_quat_in_world = Eigen::AngleAxisd(goal_euler_angles(2) + robot_euler_angles(2), Eigen::Vector3d::UnitZ());
            Eigen::Vector3d goal_pos_in_vehicle;
            goal_pos_in_vehicle << goal.position.x, goal.position.y, goal.position.z;
            goal_pos_in_world = Eigen::AngleAxisd(robot_euler_angles(2), Eigen::Vector3d::UnitZ()) * goal_pos_in_vehicle + robot_odom.position_W;
        }
        else // goal is in world frame
        {
            goal_quat_in_world = Eigen::Quaterniond(goal.orientation.w, goal.orientation.x, goal.orientation.y, goal.orientation.z);
            goal_quat_in_world = goal_quat_in_world.normalized();
            goal_pos_in_world(0) = goal.position.x;
            goal_pos_in_world(1) = goal.position.y;
            goal_pos_in_world(2) = goal.position.z;
        }
        goal_in_world->position_W = goal_pos_in_world;
        goal_in_world->orientation_W_B = goal_quat_in_world;
    }

    void LMFControllerNode::convertGoal2VehicleFrame(const mav_msgs::EigenOdometry &goal_odom, const mav_msgs::EigenOdometry &robot_odom,
                                                     nav_msgs::Odometry *goal_in_vehicle_frame)
    {
        Eigen::Vector3d goal_euler_angles, robot_euler_angles;
        goal_odom.getEulerAngles(&goal_euler_angles);
        robot_odom.getEulerAngles(&robot_euler_angles);
        Eigen::Quaterniond quat_VG;
        quat_VG = Eigen::AngleAxisd(goal_euler_angles(2) - robot_euler_angles(2), Eigen::Vector3d::UnitZ());
        Eigen::Vector3d pos_VG = Eigen::AngleAxisd(-robot_euler_angles(2), Eigen::Vector3d::UnitZ()) * (goal_odom.position_W - robot_odom.position_W);

        goal_in_vehicle_frame->header.stamp = ros::Time::now();
        goal_in_vehicle_frame->pose.pose.position.x = pos_VG(0);
        goal_in_vehicle_frame->pose.pose.position.y = pos_VG(1);
        goal_in_vehicle_frame->pose.pose.position.z = pos_VG(2);
        goal_in_vehicle_frame->pose.pose.orientation.x = quat_VG.x();
        goal_in_vehicle_frame->pose.pose.orientation.y = quat_VG.y();
        goal_in_vehicle_frame->pose.pose.orientation.z = quat_VG.z();
        goal_in_vehicle_frame->pose.pose.orientation.w = quat_VG.w();

        goal_in_vehicle_frame->twist.twist.linear.x = -robot_odom.velocity_B(0);
        goal_in_vehicle_frame->twist.twist.linear.y = -robot_odom.velocity_B(1);
        goal_in_vehicle_frame->twist.twist.linear.z = -robot_odom.velocity_B(2);
        goal_in_vehicle_frame->twist.twist.angular.x = -robot_odom.angular_velocity_B(0);
        goal_in_vehicle_frame->twist.twist.angular.y = -robot_odom.angular_velocity_B(1);
        goal_in_vehicle_frame->twist.twist.angular.z = -robot_odom.angular_velocity_B(2);
    }

    double LMFControllerNode::calculateYawCtrl(double setpoint_yaw, double current_yaw)
    {
        double yaw_error = setpoint_yaw - current_yaw;

        if (std::abs(yaw_error) > M_PI)
        {
            // if (yaw_error > 0.0)
            // {
            //     while (yaw_error > M_PI)
            //     {
            //         yaw_error = yaw_error - 2.0 * M_PI;
            //     }
            // }
            // else
            // {
            //     while (yaw_error < -M_PI)
            //     {
            //         yaw_error = yaw_error + 2.0 * M_PI;
            //     }
            // }
            yaw_error = wrapYaw(yaw_error);
        }

        double yaw_rate_cmd = K_yaw * yaw_error;

        if (yaw_rate_cmd > yaw_rate_limit)
        {
            yaw_rate_cmd = yaw_rate_limit;
        }

        if (yaw_rate_cmd < -yaw_rate_limit)
        {
            yaw_rate_cmd = -yaw_rate_limit;
        }

        return yaw_rate_cmd;
    }

    void LMFControllerNode::RangeCallback(const sensor_msgs::Range &range_msg)
    {
        range_value = range_msg;
        has_new_range = true;
    }

    // TODO: what if odometry drifts?
    // we only need velocity and attitude feedback
    void LMFControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr &odometry_msg)
    {
        // modify the sign of position and quaternion to match ROS convention
        mav_msgs::eigenOdometryFromMsg(*odometry_msg, &odometry); // TODO: what if odometry drifts? get rpy angles from autopilot

        Eigen::Vector3d current_rpy;
        odometry.getEulerAngles(&current_rpy);

        if (receive_vel_cmd)
        {
            double yaw_rate_cmd = 0.0;
            if (swap_yaw_rate) // receive reference yaw angle
            {
                yaw_rate_cmd = calculateYawCtrl(goal_yaw, current_rpy(2));
            }
            else
            {
                yaw_rate_cmd = cmd_vel_received.angular.z;
            }
            
            // TODO: convert to body frame?
            cmd_vel_send.linear = cmd_vel_received.linear;
            cmd_vel_send.angular.x = 0.0;
            cmd_vel_send.angular.y = 0.0;
            cmd_vel_send.angular.z = yaw_rate_cmd;
            cmd_vel_pub_.publish(cmd_vel_send);
        }

    }

} // namespace lmf_control

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lmf_control_node");

    lmf_control::LMFControllerNode lmf_control_node;

    ros::spin();

    return 0;
}