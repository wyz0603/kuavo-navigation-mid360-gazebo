#pragma once

#include "tcp_server.hpp"
#include "data_converter.hpp"
#include "json.hpp"
#include <ros/ros.h>
#include <kuavo_msgs/sensorsData.h>
#include <kuavo_msgs/jointCmd.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <deque>
#include <mutex>
#include <memory>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <std_srvs/SetBool.h>

// 声明虚拟控制器类
class KuavoVirtualController {
public:
    static KuavoVirtualController* getInstance();
    void publishSensorData(const nlohmann::json& isaac_data);
    bool isROSActive() const {
        return ros::ok() && nh_ != nullptr;
    }

    // 添加获取力矩数据的方法声明
    std::vector<double> getLatestTauArm();
    std::vector<double> getLatestTauLeg();

    // 添加获取位置数据的方法声明
    std::vector<double> getLatestPosArm();
    std::vector<double> getLatestPosLeg();

    // 添加获取速度数据的方法声明
    std::vector<double> getLatestVelArm();
    std::vector<double> getLatestVelLeg();

private:
    // 私有构造函数，确保单例模式
    KuavoVirtualController();
    ~KuavoVirtualController();

    // ROS相关
    ros::NodeHandle* nh_;
    ros::Publisher sensor_pub_;
    ros::Subscriber cmd_sub_;

    // 回调函数
    void jointCmdCallback(const kuavo_msgs::jointCmd::ConstPtr& msg);

    // 辅助函数：将vector转换为ROS消息
    geometry_msgs::Vector3 vectorToROS(const std::vector<double>& vec);
    geometry_msgs::Quaternion quaternionToROS(const std::vector<double>& quat);

    // 单例实例
    static KuavoVirtualController* instance_;
    static bool ros_initialized_;

    // 添加速度历史和时间戳记录
    struct VelocityRecord {
        std::vector<double> velocities;
        double timestamp;
    };
    std::deque<VelocityRecord> velocity_history;
    
    // 定时器相关
    ros::Timer publish_timer_;
    static constexpr double PUBLISH_RATE = 500.0;  // 500Hz
    static constexpr double DT = 0.002;           // 2ms for 500Hz
    
    // 数据缓存
    kuavo_msgs::sensorsData latest_sensor_msg_;
    std::mutex sensor_mutex_;
    
    // 辅助函数
    void timerCallback(const ros::TimerEvent&);
    std::vector<double> calculateAcceleration(const std::vector<double>& curr_vel, 
                                            const std::vector<double>& prev_vel);
    void updateSensorMessage(const nlohmann::json& isaac_data);

    // 添加四元数插值函数声明
    geometry_msgs::Quaternion interpolateQuaternion(
        const geometry_msgs::Quaternion& q1,
        const geometry_msgs::Quaternion& q2,
        double t);

    // 添加插值所需的数据结构
    struct SensorData {
        std::vector<double> joint_q;
        std::vector<double> joint_v;
        std::vector<double> joint_vd;
        std::vector<double> joint_current;
        geometry_msgs::Vector3 imu_acc;
        geometry_msgs::Vector3 imu_gyro;
        geometry_msgs::Quaternion imu_quat;
        ros::Time timestamp;
    };

    SensorData previous_data_;
    SensorData current_data_;
    bool has_new_data_ = false;  // 初始化标志
    ros::Time last_isaac_update_;

    // 添加一个标志来指示是否已经收到第一帧数据
    bool first_data_received_ = false;

    // 添加一个计数器用于调试
    uint64_t publish_counter_ = 0;

    // 添加异步spinner
    std::unique_ptr<ros::AsyncSpinner> spinner_;

    // 添加服务相关成员
    ros::ServiceServer sim_start_service_;
    bool is_running_ = false;

    // 添加服务回调函数
    bool handleSimStart(std_srvs::SetBool::Request& req, 
                       std_srvs::SetBool::Response& res);

    // 添加存储力矩数据的成员变量
    std::vector<double> latest_tau_arm;
    std::vector<double> latest_tau_leg;

    // 添加存储位置数据的成员变量
    std::vector<double> latest_pos_arm;
    std::vector<double> latest_pos_leg;

    // 添加存储速度数据的成员变量
    std::vector<double> latest_vel_arm;
    std::vector<double> latest_vel_leg;

    // 添加机器人版本相关成员
    int robot_version_;
    static const int LEG_JOINTS_PER_SIDE = 6;
    static const int ARM_JOINTS_PER_SIDE = 7;
    static const int HEAD_JOINTS = 2;  // 添加头部关节数量
    static const int TOTAL_JOINTS = (LEG_JOINTS_PER_SIDE + ARM_JOINTS_PER_SIDE) * 2 + HEAD_JOINTS;
    static const int HEAD_START_IDX = TOTAL_JOINTS - HEAD_JOINTS;
    // 添加关节索引映射结构
    struct JointIndices {
        // 腿部关节索引
        std::vector<int> leg_l_indices;  // 左腿关节索引
        std::vector<int> leg_r_indices;  // 右腿关节索引
        // 手臂关节索引
        std::vector<int> arm_l_indices;  // 左臂关节索引
        std::vector<int> arm_r_indices;  // 右臂关节索引
    };
    JointIndices joint_indices_;
    void setupJointIndices();
};

// 原有的函数声明
std::string processMsg_isaac(const char *msg);
