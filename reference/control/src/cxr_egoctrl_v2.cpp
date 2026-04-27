// V3版本加入了EXPLORE的速度档位选择功能
/**
 * 基于状态机的 EGO-Planner 轨迹跟踪控制器
 * Logic:
 * 1. READY: 自动切 OFFBOARD 并解锁
 * 2. TAKEOFF: 起飞到指定高度
 * 3. EXPLORE: 接收 EGO 轨迹并执行速度跟踪
 * 4. AUTO_LAND: 触发降落
 * 5. MANUAL_LAND: 监测落地并上锁
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/RCIn.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "PID_Controller.h" 

// 状态机枚举
enum FSM_STATE {
    READY,
    TAKEOFF,
    EXPLORE,    // 接受到trigger后进入轨迹跟踪状态
    AUTO_LAND,
    MANUAL_LAND, // 监测落地并上锁
    END
};

// 全局变量
FSM_STATE state = READY;
mavros_msgs::State current_state;
nav_msgs::Odometry position_msg;
quadrotor_msgs::PositionCommand ego;
mavros_msgs::PositionTarget current_goal;
ros::Time last_traj_time; // 记录上一次收到 EGO 轨迹的时间
int rc_value = 0;

// 位置 PID 控制器
Eigen::Vector3d current_velocity = Eigen::Vector3d::Zero(); // 当前速度(用于 D 项阻尼)
Eigen::Vector3d ego_vel_feedforward = Eigen::Vector3d::Zero(); // EGO前馈速度

// 坐标相关
float position_x, position_y, position_z, current_yaw;
float position_x_begin, position_y_begin, position_z_begin, yaw_begin;
bool get_first_pos = false;

// EGO 数据
bool receive_traj = false;
float ego_pos_x, ego_pos_y, ego_pos_z, ego_yaw;

// 控制参数
// 掩码: 忽略位置和加速度，只控制速度(VX, VY, VZ) + YAW
// 0b101111000111 = 0xBC7
// Bitmask mapping:
// 1 = ignore
// 1: pos x, 1: pos y, 1: pos z
// 0: vel x, 0: vel y, 0: vel z
// 1: acc x, 1: acc y, 1: acc z
// 0: force (unused) ? - check mavros doc, usually 0 is enable. 
// Actually for mavros position_target:
// IGNORE_PX | IGNORE_PY | IGNORE_PZ | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | IGNORE_YAW_RATE
unsigned short velocity_mask = 0b101111000111; 

// 参数设置
double takeoff_height = 0.7; // 起飞高度
double reach_err = 0.1;      // 位置误差阈值

// EXPLORE的速度档位
struct SpeedLimit {
    double max_vel_xy; // 水平速度上限
    double max_vel_z;  // 垂直速度上限
};
SpeedLimit speed_gears[] = {
    {0.5, 0.25},  // [0] 慢速/调试模式：很慢，适合初次测试
    {1.5, 1.0},  // [1] 标准模式：适合正常飞行
    {2.5, 2.0},  // [2] 运动模式：速度较快
    {5.0, 3.0}   // [3] 狂暴模式：极速（慎用）
};
int USE_GEAR_INDEX = 0; // 速度默认使用标准模式

// 回调函数
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

//遥控读取
void rc_cb(const mavros_msgs::RCIn::ConstPtr& msg)
{
    // 读取第7通道 (下标为6)
    if (msg->channels.size() > 6) {
        rc_value = msg->channels[6]; 
    }
}

void position_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    position_msg = *msg;
    tf2::Quaternion quat;
    tf2::convert(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    if (!get_first_pos) {
        position_x_begin = position_msg.pose.pose.position.x;
        position_y_begin = position_msg.pose.pose.position.y;
        position_z_begin = position_msg.pose.pose.position.z;
        yaw_begin = yaw;
        get_first_pos = true;
    }
    // 计算相对起飞点的坐标
    position_x = position_msg.pose.pose.position.x - position_x_begin;
    position_y = position_msg.pose.pose.position.y - position_y_begin;
    position_z = position_msg.pose.pose.position.z - position_z_begin;
    current_yaw = yaw;
    // 读取当前速度，供 PID 控制使用
    current_velocity(0) = msg->twist.twist.linear.x;
    current_velocity(1) = msg->twist.twist.linear.y;
    current_velocity(2) = msg->twist.twist.linear.z;
}

// EGO 轨迹回调
void twist_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
    receive_traj = true;
    last_traj_time = ros::Time::now();
    ego = *msg;
    ego_pos_x = ego.position.x;
    ego_pos_y = ego.position.y;
    ego_pos_z = ego.position.z;
    //读取速度作为前馈  
    ego_vel_feedforward(0) = msg->velocity.x;
    ego_vel_feedforward(1) = msg->velocity.y;
    ego_vel_feedforward(2) = msg->velocity.z;
    ego_yaw = ego.yaw + yaw_begin; 
}

// 速度限幅函数
double limit_velocity(double expect_vel, double limit_val)
{
    if (fabs(expect_vel) > limit_val)
        return limit_val * expect_vel / fabs(expect_vel);
    else
        return expect_vel;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fsm_ego_control");
    ros::NodeHandle nh;
    PID::pos_controller_PID pos_controller_pid; // 位置 PID 控制器实例
    // 订阅与发布
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    // ros::Subscriber position_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, position_cb);
    // ros::Subscriber position_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/vision_pose/pose", 10, position_cb);
    ros::Subscriber position_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry_offset", 10, position_cb);
    ros::Subscriber twist_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd_1", 10, twist_cb);
    ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 10, rc_cb);
    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    
    // 服务客户端
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");// 解锁
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");// 切换飞控模式

    ros::Rate rate(50.0); // 50Hz 控制频率

    // 等待连接
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO_THROTTLE(1, "Waiting for FCU connection...");
    }

    // 初始化 Setpoint，防止切 Offboard 瞬间掉下来
    current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    current_goal.type_mask = velocity_mask;
    current_goal.velocity.x = 0;
    current_goal.velocity.y = 0;
    current_goal.velocity.z = 0;
    current_goal.yaw = current_yaw;

	//为了满足 PX4 飞控的安全机制，强行先发送一段时间的控制指令
	//否则飞控会拒绝接入offboard模式
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(current_goal);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    
    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    //
    ros::Time last_loop_time = ros::Time::now();
    double delta_time = 0;
    Eigen::Vector3d pos_error = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel_cmd = Eigen::Vector3d::Zero();
    ROS_INFO("FSM Initialized. State: READY");

    while(ros::ok())
    {   
        pos_error.setZero();
        vel_cmd.setZero();
        ros::Time now = ros::Time::now();
        delta_time = (now - last_loop_time).toSec();
        last_loop_time = now;

        // 防止 dt 过大或过小 (例如第一次循环)
        if (delta_time > 0.1) delta_time = 0.1;
        if (delta_time < 0.001) delta_time = 0.001;
        
        // 通用消息头填充
        current_goal.header.stamp = now;
        current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_goal.type_mask = velocity_mask; // 速度控制模式

        switch (state)
        {
        case READY:
            // 1. 自动切入OFFBOARD
            if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(0.5)))
            {
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            }
            // 2. 解锁
            else if (current_state.mode == "OFFBOARD" && !current_state.armed && (ros::Time::now() - last_request > ros::Duration(0.5)))
            {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed -> Switch to TAKEOFF");
                    state = TAKEOFF;
                    // 记录初始点已经在 position_cb 中完成
                }
                last_request = ros::Time::now();
            }
            
            // 保持当前位置（怠速）
            // 等待切模式、等待解锁的这几秒钟里，程序不能闲着。它必须不断地给飞控发“速度=0”的指令，保持心跳
            current_goal.velocity.x = 0;
            current_goal.velocity.y = 0;
            current_goal.velocity.z = 0;
            current_goal.yaw = current_yaw;
            break;

        case TAKEOFF:
            {
                // 使用 PID 进行起飞控制
                // 目标位置: (0, 0, takeoff_height)
                Eigen::Vector3d target_pos(0, 0, takeoff_height);
                Eigen::Vector3d current_pos(position_x, position_y, position_z); // 注意：这里是相对于起飞点的坐标
                
                pos_error = target_pos - current_pos;

                // 调用 PID 计算速度
                vel_cmd = pos_controller_pid.posController(pos_error, current_velocity, delta_time);

                // 赋值并限幅
                // 水平方向限速小一点，垂直方向限速 1.0
                current_goal.velocity.x = limit_velocity(vel_cmd(0), 0.5); 
                current_goal.velocity.y = limit_velocity(vel_cmd(1), 0.5);
                current_goal.velocity.z = limit_velocity(vel_cmd(2), 1.0);

                current_goal.yaw = yaw_begin;

                // 判定到达
                if (fabs(pos_error(2)) < 0.10)
                {
                    ROS_INFO("Takeoff Complete -> Switch to EXPLORE");
                    state = EXPLORE;
                }
            }
            break;

        case EXPLORE:
            if (rc_value > 1400) 
            {
                ROS_INFO("Manual Interrupt -> AUTO_LAND");
                state = AUTO_LAND;
                break; 
            }

            if (receive_traj && (ros::Time::now() - last_traj_time < ros::Duration(0.5)))
            {
                // 计算位置误差
                // EGO 目标点 - 当前位置 
                Eigen::Vector3d ego_target_pos(ego_pos_x, ego_pos_y, ego_pos_z);
                Eigen::Vector3d current_pos(position_x, position_y, position_z);
                pos_error = ego_target_pos - current_pos;

                //计算 PID 输出的速度
                vel_cmd = pos_controller_pid.posController(pos_error, current_velocity, delta_time);

                //叠加 EGO 的前馈速度 (Feedforward)
                //EGO 知道下一刻要加速还是减速，PID 只是负责修误差
                vel_cmd += ego_vel_feedforward;

                int max_gear_idx = sizeof(speed_gears)/sizeof(speed_gears[0]) - 1;
                if (USE_GEAR_INDEX > max_gear_idx) USE_GEAR_INDEX = max_gear_idx;

                double limit_xy = speed_gears[USE_GEAR_INDEX].max_vel_xy;
                double limit_z  = speed_gears[USE_GEAR_INDEX].max_vel_z;
                //赋值并限幅
                current_goal.velocity.x = limit_velocity(vel_cmd(0), limit_xy);
                current_goal.velocity.y = limit_velocity(vel_cmd(1), limit_xy);
                current_goal.velocity.z = limit_velocity(vel_cmd(2), limit_z);
                
                current_goal.yaw = ego_yaw;
                //与当前目标点距离,
                ROS_INFO_THROTTLE(2.0, "Tracking: Err=%.2f, Vel=%.2f | Current Gear: %d (Limit: %.1f)", 
                                  pos_error.norm(), vel_cmd.norm(), USE_GEAR_INDEX, limit_xy);
            }
            else
            {
                // 超时悬停
                current_goal.velocity.x = 0; current_goal.velocity.y = 0; current_goal.velocity.z = 0;
                current_goal.yaw = current_yaw;
                ROS_WARN_THROTTLE(2.0, "Waiting for Planner...");
            }
            break;

        case AUTO_LAND:
            if (current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(0.5)))
            {
                if (set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent)
                {
                    ROS_INFO("AUTO.LAND enabled -> Switch to MANUAL_LAND monitor");
                    state = MANUAL_LAND;
                }
                last_request = ros::Time::now();
            }
            break;

        case MANUAL_LAND:
            // 监测是否落地并上锁
            // 判断依据：高度极低 且 速度极小
            // 注意：PX4 在 AUTO.LAND 模式下落地后通常会自动上锁，
            // 但为了保险，加一个强制上锁检查
            
            if (position_z < 0.2) // 距离起飞点高度小于 20cm
            {
                if (current_state.armed && (ros::Time::now() - last_request > ros::Duration(1.0)))
                {
                    // 尝试强制上锁（Disarm）
                    arm_cmd.request.value = false;
                    if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                    {
                        ROS_INFO("Vehicle disarmed. Mission Complete.");
                        state = END;
                    }
                    last_request = ros::Time::now();
                }
                else if (!current_state.armed)
                {
                    state = END;
                }
            }
            break;

        case END:
            // return 0; 
            break;
            
        default:
            break;
        }

        // 只有在非 AUTO.LAND 且 非 END 且 已连接 状态下才发布 Setpoint
        // 如果进入 AUTO.LAND，Mavros 会接管控制，不应再发送 Offboard Setpoints
        if (state != AUTO_LAND && state != MANUAL_LAND && state != END && current_state.connected)
        {
            local_pos_pub.publish(current_goal);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

    