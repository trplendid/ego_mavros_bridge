#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

mavros_msgs::State current_state;     // 飞控当前状态
mavros_msgs::PositionTarget setpoint; // 发送给飞控的目标点
bool has_received_cmd = false;        // 是否接收到ego_planner的标志位

// 获取飞控状态的回调函数
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// 接收ego-planner数据并翻译为MAVROS数据
void ego_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg){
    has_received_cmd = true; // 接收到了ego_planner的轨迹

    setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; // 设定目标点的坐标系为MAVROS的坐标系
    setpoint.header.stamp = ros::Time::now(); //设定目标点的时间戳 

// MAVROS里的ROS接口mavros/setpoint_position/local接受位置和偏航角数据
// mavros/setpoint_velocity/cmd_vel接受速度和偏航速度，没有接受加速的接口
// PX4内部的PID参数基于位置。速度模型进行的，如果引入加速度效果适得其反
//使用偏航角比偏航角速度控制更加稳定、平滑
//因此舍弃掉ego_planner输出的加速度和偏航角速度信息
    setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                         mavros_msgs::PositionTarget::IGNORE_AFY |
                         mavros_msgs::PositionTarget::IGNORE_AFZ |
                         mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    // 设定目标位置
    setpoint.position.x = msg->position.x;
    setpoint.position.y = msg->position.y;
    setpoint.position.z = msg->position.z;

    // 设定目标速度
    setpoint.velocity.x = msg->velocity.x;
    setpoint.velocity.y = msg->velocity.y;
    setpoint.velocity.z = msg->velocity.z;

    // 设定目标偏航角
    setpoint.yaw = msg->yaw;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ego_mavros_bridge");
    ros::NodeHandle nh;

    // 订阅飞控状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    // 订阅ego_planner的输出
    ros::Subscriber ego_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10, ego_cmd_cb); 
    
    // 发布给MAVROS
    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    
    // 服务客户端，发送解锁和切换模式请求
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // MAVROS要求发送频率必须大于2Hz才能维持Offboard模式
    ros::Rate rate(50.0);

    // 等待飞控连接
    ROS_INFO("Waiting for FCU connection...");
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected!");

    // 设定初始点
    setpoint.position.x = 0;
    setpoint.position.y = 0;
    setpoint.position.z = 1.0;

    for(int i = 100; ros::ok() && i > 0; --i){ // 确保PX4飞控offboard前有稳定的外部指令数据流
        local_pos_pub.publish(setpoint);       // 否则PX4会拒绝切换模式
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD"; // 请求切换offboard模式
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;                   // 请求解锁（先请求切换模式后请求解锁确保在解锁一瞬间飞控明白以何种模式响应）
    
    ros::Time last_request = ros::Time::now(); // 记录当前时间

    while(ros::ok()){
        // 如果收到了ego_planner的规划，就请求offboard模式和解锁

        // 如果当前不是offboard模式且距上次请求时间大于5秒
        // 请求offboard模式
        if (has_received_cmd) {
            if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
        // 如果当前未解锁且距上次请求时间大于5秒
        // 请求解锁
            } else {
                if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }
        }

        // 以50Hz持续发布最新的期望轨迹点
        local_pos_pub.publish(setpoint);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}