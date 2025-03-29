#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Int32.h>
#include "getch.h"
#include <cmath>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Dense>
#include <queue>
#include <state_estimation/Mav.h>

// key_ctrl_for_2T node
// This node allows the user to control two targets using the keyboard.

void bound_yaw(double* yaw){
    if(*yaw > M_PI)
        *yaw = *yaw - 2*M_PI;
    else if(*yaw < -M_PI)
        *yaw = *yaw + 2*M_PI;
}

int main(int argc, char **argv)
{
    //  ROS_initialize  //
    ros::init(argc, argv, "key_ctrl_for_2T");
    ros::NodeHandle nh;

    // objects - create MAV objects for the two targets
    // 使用正確的命名空間，這需要與您系統中的發布主題匹配
    MAV target1(nh, "target_1", 0);
    MAV target2(nh, "target_2", 4); // 使用ID 4，與launch文件中相符

    // 創建儲存當前姿態的變數
    geometry_msgs::PoseStamped current_pose1, current_pose2;
    bool received_pose1 = false;
    bool received_pose2 = false;
    
    // 創建姿態回調函數
    ros::Subscriber pose_sub1 = nh.subscribe<geometry_msgs::PoseStamped>(
        "/target_1/mavros/local_position/pose", 10, 
        [&](const geometry_msgs::PoseStamped::ConstPtr& msg) {
            current_pose1 = *msg;
            received_pose1 = true;
        });
    
    ros::Subscriber pose_sub2 = nh.subscribe<geometry_msgs::PoseStamped>(
        "/target_2/mavros/local_position/pose", 10, 
        [&](const geometry_msgs::PoseStamped::ConstPtr& msg) {
            current_pose2 = *msg;
            received_pose2 = true;
        });

    // publishers - one for each target
    ros::Publisher desired_pose_pub1 = nh.advertise<geometry_msgs::PoseStamped>("/target_1/mavros/setpoint_position/local", 10);
    ros::Publisher desired_pose_pub2 = nh.advertise<geometry_msgs::PoseStamped>("/target_2/mavros/setpoint_position/local", 10);

    // services - one set for each target
    ros::ServiceClient arming_client1 = nh.serviceClient<mavros_msgs::CommandBool>("/target_1/mavros/cmd/arming");
    ros::ServiceClient set_mode_client1 = nh.serviceClient<mavros_msgs::SetMode>("/target_1/mavros/set_mode");
    ros::ServiceClient land_client1 = nh.serviceClient<mavros_msgs::CommandTOL>("/target_1/mavros/cmd/land");

    ros::ServiceClient arming_client2 = nh.serviceClient<mavros_msgs::CommandBool>("/target_2/mavros/cmd/arming");
    ros::ServiceClient set_mode_client2 = nh.serviceClient<mavros_msgs::SetMode>("/target_2/mavros/set_mode");
    ros::ServiceClient land_client2 = nh.serviceClient<mavros_msgs::CommandTOL>("/target_2/mavros/cmd/land");

    ros::Rate rate(100);
    
    // Wait for pose initialization
    ROS_INFO("Wait for pose initialization for both targets");
    while (ros::ok() && (!target1.pose_init || !target2.pose_init)) 
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Wait for pose init - Target 1: %d, Target 2: %d", target1.pose_init, target2.pose_init);
    rate.sleep();
    }
    ROS_INFO("Both targets' poses initialized");
    
    // 初始化之後立即跳過 MAV 類中的 FCU 連接檢查
    bool fcu_check_bypassed = true;
    ROS_INFO("Bypassing FCU connection check");
    
    // 設置控制命令
    mavros_msgs::SetMode offb_set_mode1, offb_set_mode2;
    mavros_msgs::CommandBool arm_cmd1, arm_cmd2;
    arm_cmd1.request.value = true;
    arm_cmd2.request.value = true;
    ros::Time last_request1 = ros::Time::now();
    ros::Time last_request2 = ros::Time::now();
    mavros_msgs::CommandTOL land_request1, land_request2;
    
    // Set OFFBOARD mode for both targets
    offb_set_mode1.request.custom_mode = "OFFBOARD";
    offb_set_mode2.request.custom_mode = "OFFBOARD";
    
    // Enable OFFBOARD and arm for target 1
    if(set_mode_client1.call(offb_set_mode1) && offb_set_mode1.response.mode_sent)
        ROS_INFO("Target 1: Offboard enabled");
    if(arming_client1.call(arm_cmd1) && arm_cmd1.response.success)
        ROS_INFO("Target 1: Vehicle armed");
        
    // Enable OFFBOARD and arm for target 2
    if(set_mode_client2.call(offb_set_mode2) && offb_set_mode2.response.mode_sent)
        ROS_INFO("Target 2: Offboard enabled");
    if(arming_client2.call(arm_cmd2) && arm_cmd2.response.success)
        ROS_INFO("Target 2: Vehicle armed");

    // Initialize desired poses for both targets
    geometry_msgs::PoseStamped desired_pose1, desired_pose2;
    desired_pose1.pose.position.x = 12; // Initial position from launch file
    desired_pose1.pose.position.y = 12;
    desired_pose1.pose.position.z = 0;
    
    desired_pose2.pose.position.x = 13; // Initial position from launch file
    desired_pose2.pose.position.y = 13;
    desired_pose2.pose.position.z = 0;

    // Send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        desired_pose_pub1.publish(desired_pose1);
        desired_pose_pub2.publish(desired_pose2);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Both targets are ready to start");
    
    // Control parameters
    double move_step = 0.1;
    double desired_yaw1 = 0;
    double desired_yaw2 = 0;
    bool trajectory1 = false;
    bool trajectory2 = false;
    double trajectory_time1 = 0;
    double trajectory_time2 = 0;
    double current_x1 = 0, current_y1 = 0;
    double current_x2 = 0, current_y2 = 0;
    
    // Target selection (1 or 2)
    int active_target = 1; // Default to controlling target 1
    
    while (ros::ok()) 
    {
        // 使用我們自己的姿態訂閱代替依賴 MAV 類
        if (received_pose1 && active_target == 1) {
            ROS_INFO("Target 1 current position: [%.2f, %.2f, %.2f]", 
                     current_pose1.pose.position.x, 
                     current_pose1.pose.position.y, 
                     current_pose1.pose.position.z);
        }
        
        if (received_pose2 && active_target == 2) {
            ROS_INFO("Target 2 current position: [%.2f, %.2f, %.2f]", 
                     current_pose2.pose.position.x, 
                     current_pose2.pose.position.y, 
                     current_pose2.pose.position.z);
        }
        
        // Keyboard control
        int c = getch();
        
        // Update desired pose based on keyboard input
        if (c != 0) 
        {
            switch (c) {
                case 49:    // key '1' - select target 1
                    active_target = 1;
                    ROS_INFO("Controlling Target 1");
                    break;
                case 50:    // key '2' - select target 2
                    active_target = 2;
                    ROS_INFO("Controlling Target 2");
                    break;
                case 65:    // key up
                    if (active_target == 1)
                        desired_pose1.pose.position.z += move_step;
                    else
                        desired_pose2.pose.position.z += move_step;
                    break;
                case 66:    // key down
                    if (active_target == 1)
                        desired_pose1.pose.position.z += -move_step;
                    else
                        desired_pose2.pose.position.z += -move_step;
                    break;
                case 67:    // key CW(->)
                    if (active_target == 1) {
                        desired_yaw1 -= 0.03;
                        bound_yaw(&desired_yaw1);
                    } else {
                        desired_yaw2 -= 0.03;
                        bound_yaw(&desired_yaw2);
                    }
                    break;
                case 68:    // key CCW(<-)
                    if (active_target == 1) {
                        desired_yaw1 += 0.03;
                        bound_yaw(&desired_yaw1);
                    } else {
                        desired_yaw2 += 0.03;
                        bound_yaw(&desired_yaw2);
                    }
                    break;
                case 119:    // key forward(w)
                    if (active_target == 1)
                        desired_pose1.pose.position.y += move_step;
                    else
                        desired_pose2.pose.position.y += move_step;
                    break;
                case 115:    // key back(s)
                    if (active_target == 1)
                        desired_pose1.pose.position.y -= move_step;
                    else
                        desired_pose2.pose.position.y -= move_step;
                    break;
                case 97:    // key left(a)
                    if (active_target == 1)
                        desired_pose1.pose.position.x -= move_step;
                    else
                        desired_pose2.pose.position.x -= move_step;
                    break;
                case 100:    // key right(d)
                    if (active_target == 1)
                        desired_pose1.pose.position.x += move_step;
                    else
                        desired_pose2.pose.position.x += move_step;
                    break;
                case 108:    // key land(l)
                    if (active_target == 1)
                        desired_pose1.pose.position.z = 0.5;
                    else
                        desired_pose2.pose.position.z = 0.5;
                    break;
                case 101:    // key trajectory_CCW(e)
                    if (active_target == 1) {
                        trajectory1 = true;
                        trajectory_time1 = 0;
                        if (received_pose1) {
                            current_x1 = current_pose1.pose.position.x;
                            current_y1 = current_pose1.pose.position.y;
                        } else {
                            current_x1 = desired_pose1.pose.position.x;
                            current_y1 = desired_pose1.pose.position.y;
                        }
                    } else {
                        trajectory2 = true;
                        trajectory_time2 = 0;
                        if (received_pose2) {
                            current_x2 = current_pose2.pose.position.x;
                            current_y2 = current_pose2.pose.position.y;
                        } else {
                            current_x2 = desired_pose2.pose.position.x;
                            current_y2 = desired_pose2.pose.position.y;
                        }
                    }
                    break;
                case 112:    // key stop trajectory(p)
                    if (active_target == 1)
                        trajectory1 = false;
                    else
                        trajectory2 = false;
                    break;  
                case 111:    // key (o) - return to initial position at altitude 4
                    if (active_target == 1) {
                        desired_pose1.pose.position.x = 12;
                        desired_pose1.pose.position.y = 12;
                        desired_pose1.pose.position.z = 4;
                    } else {
                        desired_pose2.pose.position.x = 13;
                        desired_pose2.pose.position.y = 13;
                        desired_pose2.pose.position.z = 4;
                    }
                    break;
                case 51:    // key '3' - control both targets simultaneously
                    active_target = 3;
                    ROS_INFO("Controlling Both Targets");
                    break;
                case 52:    // key '4' - synchronize trajectories for both targets
                    trajectory1 = true;
                    trajectory2 = true;
                    trajectory_time1 = 0;
                    trajectory_time2 = 0;
                    
                    if (received_pose1) {
                        current_x1 = current_pose1.pose.position.x;
                        current_y1 = current_pose1.pose.position.y;
                    } else {
                        current_x1 = desired_pose1.pose.position.x;
                        current_y1 = desired_pose1.pose.position.y;
                    }
                    
                    if (received_pose2) {
                        current_x2 = current_pose2.pose.position.x;
                        current_y2 = current_pose2.pose.position.y;
                    } else {
                        current_x2 = desired_pose2.pose.position.x;
                        current_y2 = desired_pose2.pose.position.y;
                    }
                    
                    ROS_INFO("Starting synchronized trajectories for both targets");
                    break;
                case 53:    // key '5' - stop all trajectories
                    trajectory1 = false;
                    trajectory2 = false;
                    ROS_INFO("Stopping all trajectories");
                    break;
                case 107:   // key kill(k)
                    return 0;
            }
            
            // Display setpoint information
            if (active_target == 1) {
                ROS_INFO("Target 1 setpoint: %.2f, %.2f, %.2f", 
                        desired_pose1.pose.position.x, 
                        desired_pose1.pose.position.y, 
                        desired_pose1.pose.position.z);
            } else if (active_target == 2) {
                ROS_INFO("Target 2 setpoint: %.2f, %.2f, %.2f", 
                        desired_pose2.pose.position.x, 
                        desired_pose2.pose.position.y, 
                        desired_pose2.pose.position.z);
            } else {
                ROS_INFO("Both targets setpoints - T1: %.2f, %.2f, %.2f, T2: %.2f, %.2f, %.2f", 
                        desired_pose1.pose.position.x, desired_pose1.pose.position.y, desired_pose1.pose.position.z,
                        desired_pose2.pose.position.x, desired_pose2.pose.position.y, desired_pose2.pose.position.z);
            }
        }
        
        // Update trajectory for target 1 if active
        if(trajectory1)
        {
            trajectory_time1 += 0.0018;
            desired_pose1.pose.position.x = current_x1 + 2*cos(trajectory_time1);
            desired_pose1.pose.position.y = current_y1 + 2*sin(trajectory_time1);
        }
        
        // Update trajectory for target 2 if active
        if(trajectory2)
        {
            trajectory_time2 += 0.0018;
            desired_pose2.pose.position.x = current_x2 + 2*cos(trajectory_time2);
            desired_pose2.pose.position.y = current_y2 + 2*sin(trajectory_time2);
        }

        // Set orientation (yaw) using quaternions
        tf::Quaternion q1 = tf::createQuaternionFromYaw(desired_yaw1);
        desired_pose1.pose.orientation.x = q1.x();
        desired_pose1.pose.orientation.y = q1.y();
        desired_pose1.pose.orientation.z = q1.z();
        desired_pose1.pose.orientation.w = q1.w();
        
        tf::Quaternion q2 = tf::createQuaternionFromYaw(desired_yaw2);
        desired_pose2.pose.orientation.x = q2.x();
        desired_pose2.pose.orientation.y = q2.y();
        desired_pose2.pose.orientation.z = q2.z();
        desired_pose2.pose.orientation.w = q2.w();

        // Publish desired poses to both targets
        desired_pose_pub1.publish(desired_pose1);
        desired_pose_pub2.publish(desired_pose2);
        
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}