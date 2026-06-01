#ifndef TOUCH_DEXHAND_ROS1_H
#define TOUCH_DEXHAND_ROS1_H
#include <ros/ros.h>
#include <thread>
#include <string>
#include "joint_address.hpp"
#include "dexhand/dexhand_def.h"
#include "dexhand/dexhand_controller.h"

#include "kuavo_msgs/dexhandCommand.h"
#include "kuavo_msgs/robotHandPosition.h"
#include "kuavo_msgs/gestureExecute.h"
#include "kuavo_msgs/gestureList.h"
#include "kuavo_msgs/gestureExecuteState.h"

namespace mujoco_node {
using namespace eef_controller;

class DexHandMujocoRosNode {
public:
    DexHandMujocoRosNode() = default;
    ~DexHandMujocoRosNode();

    /**
     * @brief Initialize the TouchDexHand node
     * @return true if initialization is successful, false otherwise
     */
    bool init(ros::NodeHandle& nh,
        const mjModel* model,  
        const JointGroupAddress &r_hand_address,
        const JointGroupAddress &l_hand_address,
        double frequency = 500.0);

    /**
     * @brief Stop the TouchDexHand node and cleanup resources
     */
    void stop();

    /**
     * @brief Get the hand joints num
     * @return the hand joints num
     */
    int get_hand_joints_num();
    
    void readCallback(const mjData *d);
    void writeCallback(mjData *d);

private:
    void dualHandCommandCallback(const kuavo_msgs::dexhandCommand::ConstPtr& msg);
    void controlSingleHand(HandSide side, const kuavo_msgs::dexhandCommand::ConstPtr& msg);
    // 兼容原来的 control_robot_hand_position 接口
    void controlHandCallback(const kuavo_msgs::robotHandPosition::ConstPtr& msg);

    /* gesture execute service. */
    bool gestureExecuteCallback(kuavo_msgs::gestureExecuteRequest &req,
                          kuavo_msgs::gestureExecuteResponse &res);
    /* gesture list service. */
    bool gestureListCallback(kuavo_msgs::gestureListRequest &req,
                          kuavo_msgs::gestureListResponse &res);
    /* gesture execute state service. */
    bool gestureExecuteStateCallback(kuavo_msgs::gestureExecuteStateRequest &req,
                          kuavo_msgs::gestureExecuteStateResponse &res);
    void publish_loop();
    
    /* ROS */
    ros::NodeHandle nh_;
    ros::Subscriber command_sub_;
    ros::Subscriber r_hand_command_sub_;
    ros::Subscriber l_hand_command_sub_;
    ros::Publisher status_pub_;
    
    // 兼容原来的 control_robot_hand_position 接口
    ros::Subscriber hand_sub_;  
    
    // 兼容原来的手势接口
    ros::ServiceServer gesture_execute_srv_;
    ros::ServiceServer gesture_list_srv_;
    ros::ServiceServer gesture_exec_state_srv_;
    
    DexhandControllerPtr controller_ = nullptr;
    std::thread publish_thread_;
    bool running_{false};

    int finger_count_;
    int hand_count_;
    double frequency_;

    std::shared_ptr<mujoco_node::MujocoDexHand> r_dexhand_ = nullptr;
    std::shared_ptr<mujoco_node::MujocoDexHand> l_dexhand_ = nullptr;
};

} // namespace eef_controller
#endif