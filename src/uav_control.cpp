#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <iostream>
using namespace std;

mavros_msgs::State current_state;     // 飞控当前状态
mavros_msgs::PositionTarget setpoint; // 发送给飞控的目标点

// 获取飞控状态的回调函数
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int get_keyboard(){

    int i;
    cout << "请选择飞行模式：";
    cout << "圆形:1，正方形:2\n";
    cin >> i;
    return i;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_control"); // 初始化节点
    ros::NodeHandle nh;
    int FlightState = get_keyboard(); // 获取飞行模式

    // 订阅飞控的状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    // 发布给MAVROS
    ros::Publisher pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    // 服务客户端，发送解锁和切换模式请求
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

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
    setpoint.position.z = 2.0;

    // 先让无人机定点悬停
    for(int i = 100; ros::ok() && i > 0; --i){ // 确保PX4飞控offboard前有稳定的外部指令数据流
        pos_pub.publish(setpoint);             // 否则PX4会拒绝切换模式
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD"; // 请求切换offboard模式
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;                   // 请求解锁（先请求切换模式后请求解锁确保在解锁一瞬间飞控明白以何种模式响应）

    double start_time = ros::Time::now().toSec(); // 获取当前时间
    ros::Time last_request = ros::Time::now();    // 获取当前时间戳

    while(ros::ok()){

        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
            last_request = ros::Time::now();
            }
        }

        // 设定目标点的坐标系为MAVROS的坐标系
        setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; 
        // 忽略加速度和偏航速度
        setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                            mavros_msgs::PositionTarget::IGNORE_AFY |
                            mavros_msgs::PositionTarget::IGNORE_AFZ |
                            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        // 如果为圆形飞行
        if(FlightState == 1){
            if(current_state.mode == "OFFBOARD" && current_state.armed){

                double t = ros::Time::now().toSec() - start_time;
                
                // 起飞至目标高度并悬停
                if(t < 10.0){

                    setpoint.position.x = 0;
                    setpoint.position.y = 0;
                    setpoint.position.z = 2.0;
                }
                else{

                double t1 = t - 10.0;
                
                // 设定圆轨迹
                setpoint.position.x = 2 * cos(0.2*t1);
                setpoint.position.y = 2 * sin(0.2*t1);
                setpoint.position.z = 2.0;
                }

            }
            else{
                // 一直悬停
                setpoint.position.x = 0.0;
                setpoint.position.y = 0.0;
                setpoint.position.z = 2.0;
                // 重置起飞时间
                start_time = ros::Time::now().toSec();
            }
        }
        // 正方形飞行
        else if(FlightState == 2){
            if(current_state.mode == "OFFBOARD" && current_state.armed){

            double t = ros::Time::now().toSec() - start_time; 

            if (t < 8.0) {
                // 起飞至目标高度并悬停
                setpoint.position.x = 0.0;
                setpoint.position.y = 0.0;
                setpoint.position.z = 2.0;

            } else if (t < 16.0) {
                // 飞向第一个点 (2, 0, 2) 并悬停
                setpoint.position.x = 2.0;
                setpoint.position.y = 0.0;
                setpoint.position.z = 2.0;

            } else if (t < 24.0) {
                // 飞向第二个点 (2, 4, 2) 并悬停
                setpoint.position.x = 2.0;
                setpoint.position.y = 4.0;
                setpoint.position.z = 2.0;

            } else if (t < 32.0) {
                // 飞向第三个点 (-2, 4, 2) 并悬停
                setpoint.position.x = -2.0;
                setpoint.position.y = 4.0;
                setpoint.position.z = 2.0;

            } else if (t < 40.0) {
                // 飞向第四个点 (-2, 0, 2) 并悬停
                setpoint.position.x = -2.0;
                setpoint.position.y = 0.0;
                setpoint.position.z = 2.0;

            } else {
                // 飞回起点并一直悬停
                setpoint.position.x = 0.0;
                setpoint.position.y = 0.0;
                setpoint.position.z = 2.0;
            }
        } 
        else {
            setpoint.position.x = 0;
            setpoint.position.y = 0;
            setpoint.position.z = 2.0;
            start_time = ros::Time::now().toSec(); 
            }
        }

        // 如果飞行模式都不是这些则悬停 
        else{
            setpoint.position.x = 0;
            setpoint.position.y = 0;
            setpoint.position.z = 2.0;
        }

        pos_pub.publish(setpoint);
        ros::spinOnce();
        rate.sleep();
    }
        return 0;
}