#include "isaac_sim_system.hpp"

using namespace nlohmann;

// 静态成员初始化
KuavoVirtualController* KuavoVirtualController::instance_ = nullptr;
bool KuavoVirtualController::ros_initialized_ = false;

//  /sensors_data_raw [kuavo_msgs/sensorsData]
/**
---
header: 
  seq: 7816
  stamp: 
    secs: 1741847560
    nsecs: 115940895
  frame_id: "world"
sensor_time: 
  secs: 1741847559
  nsecs: 196897964
joint_data: 
  joint_q: [-0.029754160608350243, -0.007889370656936828, -0.5649344090031634, 0.8410804713040305, -0.32815284663064725, 0.031416581360902675, 0.02309647653945707, 0.015109102114694321, -0.5404466119165091, 0.8375827984048153, -0.3490431435097262, -0.022546292562215235, 0.09968449364627245, -0.0006077494574490696, -5.3901481638555005e-05, -0.24958314230759407, -8.503041665113862e-05, -0.00013286485819477878, 0.00020845160590736504, 0.10010768464346279, -0.0005390047477526978, -1.2999797558354412e-05, -0.24942121089660324, 9.342888911990551e-05, -8.690393595085258e-05, -0.0002217974733635503, -7.256965153923969e-05, 0.023473624667687393]
  joint_v: [4.899041293741952e-05, -5.162642925824546e-05, 0.0004986705249662349, -0.0008756452512821212, 0.00034308936195375265, -3.546657936862684e-05, 6.008172767202526e-06, -0.00023636590149971767, 0.0005470643178479609, -0.0009150342825170927, 0.0003247897832801182, 1.502366626158951e-05, 1.956599051814695e-05, 8.182415763349342e-06, 1.005698364619898e-06, 3.950831705837553e-05, 1.3754727082195142e-06, 1.6117669457345945e-05, 3.48075031268362e-06, -3.7906992543054994e-05, 2.5467842481573164e-05, 1.4856525926986324e-05, 1.3505405086505757e-05, -8.551974910855925e-06, 1.6794802558798195e-05, -1.0034992825854354e-06, -2.543880471706497e-05, 0.0001058498811370504]
  joint_vd: [0.006176447791102962, 0.0006648091816380307, 0.14655031114466702, -0.2636671714832209, 0.10661141282317498, 0.006760096026801975, 0.0035720728904415217, 0.001191224106650372, 0.15507741120314975, -0.28041793255113723, 0.11472688344950321, 0.009175329879318721, 0.002033642149918107, 0.00026013311242804405, 3.84007763424384e-05, -0.0006697468694946185, 7.158437566808923e-05, 0.0015840308373948268, 0.0006435670605030167, 0.000784244809194213, 0.00040297919551057966, 0.0004547778239369339, -0.0005792597409276278, -9.191571559595633e-05, 0.0006969156824528644, -0.0003634829716138416, -0.0014253754419972856, 0.01492836214208901]
  joint_current: [-0.22481561938763872, 0.08255016068643403, 3.9170018346685027, -21.678959787255604, 5.532124135519373, -0.1844490100505912, 1.1412805677888131, 0.05321337298013349, 0.8809159331155251, -21.80653411036448, 5.235730815760241, -0.18376613920316354, 0.3210757798268206, 0.17442280955821848, 0.02031579986914966, -0.3737837518159271, -0.009000307191503494, -0.14935063275931856, 0.05802547340955932, 0.3376763633210188, -0.21008885703556363, -0.024126749884848184, -0.3718199396634921, 0.008823160145331585, -0.1486487537536245, -0.06049903652705149, 0.0007481119147177403, -0.23481324642638632]
imu_data: 
  gyro: 
    x: -1.2770683753950431e-05
    y: -1.4437867921514442e-05
    z: 0.0001373881254591896
  acc: 
    x: -0.513663198326166
    y: -0.015066085270252024
    z: 9.822208583314248
  free_acc: 
    x: -0.0025946268143376416
    y: -0.000397402226965762
    z: 0.05057521191974779
  quat: 
    x: -0.0007247775321559346
    y: 0.026124528279576346
    z: -0.0009577139713752837
    w: 0.9996579747611912
end_effector_data: 
  name: []
  position: []
  velocity: []
  effort: []
---
*/

// /joint_cmd [kuavo_msgs/jointCmd]
/**
---
header: 
  seq: 7289
  stamp: 
    secs: 1741847689
    nsecs:   4684428
  frame_id: ''
joint_q: [-0.017572363346869287, -0.0011923210003364803, -0.5574876172602524, 0.8411684011007464, -0.33605680782369257, 0.016620184256881863, 0.018475690635097044, 0.001244314918005746, -0.5579641760156921, 0.8419230755526478, -0.33633475589809775, -0.01825813794340924, 0.09978998510612704, -1.0758419003691431e-05, -3.021280127381393e-07, -0.24965612069238455, 4.1024723550328476e-08, 4.533756539773794e-08, -8.202778249850973e-08, 0.09978133038596255, 4.690300812848089e-05, 9.056706237647212e-06, -0.24964589152468578, -4.603003841385389e-08, 4.5214621218497105e-08, 8.82819298094801e-08, 0.0, 0.0]
joint_v: [0.005905425121809903, 0.00031830787871423624, 0.001557243735926916, -0.0027779393558162524, 0.002143272994234835, -0.036701917151008887, -0.004565063532897975, -0.00030015465036476683, 0.0019515376416078193, -0.0034076478271750296, 0.0023193045504655576, 0.036687213120293875, 0.0013862393429117145, 6.157520195155101e-05, -7.26770090549245e-06, -0.003462192605468736, 4.1024723550328477e-05, 4.533756539773794e-05, -8.202778249850973e-05, 0.0014853772852745704, -0.00041576447717409147, -7.047377345549514e-05, -0.0035337646165309023, -4.603003841385388e-05, 4.5214621218497105e-05, 8.82819298094801e-05, 0.0, 0.0]
tau: [-2.1895523459757182, -0.09069144065204612, 2.4327571898157627, -21.87690845545964, 5.043965616498486, -0.004476798782007698, 1.928471008042087, 0.07492790897108209, 2.42698615446555, -21.858765326345324, 5.035413577724162, -0.005102497170813379, 0.32189708402994466, 0.19529193081392285, 0.022528472697526506, -0.3725274563303895, -0.008890267828460379, -0.14891241306097128, 0.05935736909866127, 0.33414411714114195, -0.18821728982736247, -0.021785339520086357, -0.37255314575368825, 0.008916647224842731, -0.14891998441783091, -0.05888531003022541, 0.0010981230322745917, -0.23489580968301835]
tau_max: [110.0, 110.0, 110.0, 110.0, 18.0, 18.0, 110.0, 110.0, 110.0, 110.0, 18.0, 18.0, 110.0, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0, 110.0, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0, 10.0, 10.0]
tau_ratio: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
joint_kp: [15.9507, 10.3, 10.3, 15.9507, 0.0, 0.0, 15.9507, 10.3, 10.3, 15.9507, 0.0, 0.0]
joint_kd: [2.73014, 7.53, 7.53, 2.73014, 0.0, 0.0, 2.73014, 7.53, 7.53, 2.73014, 0.0, 0.0]
control_modes: [2, 2, 2, 0, 0, 0, 2, 2, 2, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
---
*/

// 实现虚拟控制器类
KuavoVirtualController::KuavoVirtualController() : nh_(nullptr) {
    // 初始化ROS
    if (!ros_initialized_) {
        int argc = 0;
        char** argv = nullptr;
        ros::init(argc, argv, "kuavo_virtual_controller", ros::init_options::NoSigintHandler);
        ros_initialized_ = true;
    }

    // 创建NodeHandle
    nh_ = new ros::NodeHandle();
    
    // 创建一个新的NodeHandle和回调队列用于所有回调
    ros::NodeHandle* callback_nh = new ros::NodeHandle();
    auto callback_queue = new ros::CallbackQueue();
    callback_nh->setCallbackQueue(callback_queue);

    // 获取机器人版本参数
    nh_->param<int>("robot_version", robot_version_, 40); // 默认为40版本
    ROS_INFO("Initializing robot version: %d", robot_version_);

    // 创建关节索引
    setupJointIndices();

    // 创建发布者和订阅者，使用callback_nh
    sensor_pub_ = callback_nh->advertise<kuavo_msgs::sensorsData>("/sensors_data_raw", 1000);
    cmd_sub_ = callback_nh->subscribe("/joint_cmd", 1, &KuavoVirtualController::jointCmdCallback, this);

    // 添加服务器，使用callback_nh
    sim_start_service_ = callback_nh->advertiseService("sim_start", 
        &KuavoVirtualController::handleSimStart, this);

    // 创建定时器，500Hz
    double timer_period = 1.0/PUBLISH_RATE;
    ROS_INFO("Creating timer with period: %f seconds", timer_period);
    
    publish_timer_ = callback_nh->createTimer(
        ros::Duration(timer_period),
        &KuavoVirtualController::timerCallback,
        this,
        false,  // oneshot
        true    // autostart
    );

    // 创建异步spinner并启动，处理所有回调
    spinner_ = std::make_unique<ros::AsyncSpinner>(1, callback_queue);
    spinner_->start();
    
    // 设置初始运行状态为true
    is_running_ = true;
    
    ROS_INFO("KuavoVirtualController initialized");

    // 初始化力矩数组
    latest_tau_arm.resize(14, 0.0);
    latest_tau_leg.resize(12, 0.0);

    // 初始化位置数组
    latest_pos_arm.resize(14, 0.0);
    latest_pos_leg.resize(12, 0.0);

    // 初始化速度数组
    latest_vel_arm.resize(14, 0.0);
    latest_vel_leg.resize(12, 0.0);
}

KuavoVirtualController::~KuavoVirtualController() {
    if (spinner_) {
        spinner_->stop();
    }
    if (publish_timer_.isValid()) {
        publish_timer_.stop();
    }
    if (nh_) {
        delete nh_;
        nh_ = nullptr;
    }
    ROS_INFO("KuavoVirtualController destroyed");
}

KuavoVirtualController* KuavoVirtualController::getInstance() {
    if (instance_ == nullptr) {
        instance_ = new KuavoVirtualController();
    }
    return instance_;
}

void KuavoVirtualController::jointCmdCallback(const kuavo_msgs::jointCmd::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    
    // 初始化力矩向量
    latest_tau_arm.resize(14);  // 7 joints * 2 arms
    latest_tau_leg.resize(12);  // 6 joints * 2 legs
    
    // 初始化位置向量
    latest_pos_arm.resize(14);  // 7 joints * 2 arms
    latest_pos_leg.resize(12);  // 6 joints * 2 legs

    // 初始化速度向量
    latest_vel_arm.resize(14);  // 7 joints * 2 arms
    latest_vel_leg.resize(12);  // 6 joints * 2 legs

    if (robot_version_ == 40) {  // 40 短手短脚
        // 重新组织手臂力矩数据
        // zarm_l1_joint, zarm_r1_joint, zarm_l2_joint, zarm_r2_joint, ...
        for (int i = 0; i < 7; i++) {
            latest_tau_arm[i * 2] = msg->tau[i + 12];      // 左臂 (从索引12开始)
            latest_tau_arm[i * 2 + 1] = msg->tau[i + 19];  // 右臂 (从索引19开始)
        }
    
        // 重新组织腿部力矩数据
        // leg_l1_joint, leg_r1_joint, leg_l2_joint, leg_r2_joint, ...
        for (int i = 0; i < 6; i++) {
            latest_tau_leg[i * 2] = msg->tau[i];       // 左腿 (从索引0开始)
            latest_tau_leg[i * 2 + 1] = msg->tau[i + 6];   // 右腿 (从索引6开始)
        }

        // 重新组织手臂位置数据
        // zarm_l1_joint, zarm_r1_joint, zarm_l2_joint, zarm_r2_joint, ...
        for (int i = 0; i < 7; i++) {
            latest_pos_arm[i * 2] = msg->joint_q[i + 12];      // 左臂 (从索引12开始)
            latest_pos_arm[i * 2 + 1] = msg->joint_q[i + 19];  // 右臂 (从索引19开始)
        }
    
        // 重新组织腿部位置数据
        // leg_l1_joint, leg_r1_joint, leg_l2_joint, leg_r2_joint, ...
        for (int i = 0; i < 6; i++) {
            latest_pos_leg[i * 2] = msg->joint_q[i];       // 左腿 (从索引0开始)
            latest_pos_leg[i * 2 + 1] = msg->joint_q[i + 6];   // 右腿 (从索引6开始)
        }

        // 重新组织手臂速度数据
        // zarm_l1_joint, zarm_r1_joint, zarm_l2_joint, zarm_r2_joint, ...
        for (int i = 0; i < 7; i++) {
            latest_vel_arm[i * 2] = msg->joint_v[i + 12];      // 左臂 (从索引12开始)   
            latest_vel_arm[i * 2 + 1] = msg->joint_v[i + 19];  // 右臂 (从索引19开始)
        }

        // 重新组织腿部速度数据
        // leg_l1_joint, leg_r1_joint, leg_l2_joint, leg_r2_joint, ...
        for (int i = 0; i < 6; i++) {
            latest_vel_leg[i * 2] = msg->joint_v[i];       // 左腿 (从索引0开始)
            latest_vel_leg[i * 2 + 1] = msg->joint_v[i + 6];   // 右腿 (从索引6开始)
        }
    }
    else if (robot_version_ == 45) {  // 45 长手长脚
        // 重新组织手臂力矩数据
        // zarm_l1_joint, zarm_r1_joint, zarm_l2_joint, zarm_r2_joint, ...
        for (int i = 0; i < 7; i++) {
            latest_tau_arm[i * 2] = msg->tau[i + 12];      // 左臂 (从索引12开始)
            latest_tau_arm[i * 2 + 1] = msg->tau[i + 19];  // 右臂 (从索引19开始)
        }
    
        // 重新组织腿部力矩数据
        // leg_l1_joint, leg_r1_joint, leg_l2_joint, leg_r2_joint, ...
        for (int i = 0; i < 6; i++) {
            latest_tau_leg[i * 2] = msg->tau[i];       // 左腿 (从索引0开始)
            latest_tau_leg[i * 2 + 1] = msg->tau[i + 6];   // 右腿 (从索引6开始)
        }

        // 重新组织手臂位置数据
        // zarm_l1_joint, zarm_r1_joint, zarm_l2_joint, zarm_r2_joint, ...
        for (int i = 0; i < 7; i++) {
            latest_pos_arm[i * 2] = msg->joint_q[i + 12];      // 左臂 (从索引12开始)
            latest_pos_arm[i * 2 + 1] = msg->joint_q[i + 19];  // 右臂 (从索引19开始)
        }
    
        // 重新组织腿部位置数据
        // leg_l1_joint, leg_r1_joint, leg_l2_joint, leg_r2_joint, ...
        for (int i = 0; i < 6; i++) {
            latest_pos_leg[i * 2] = msg->joint_q[i];       // 左腿 (从索引0开始)
            latest_pos_leg[i * 2 + 1] = msg->joint_q[i + 6];   // 右腿 (从索引6开始)
        }

        // 重新组织手臂速度数据
        // zarm_l1_joint, zarm_r1_joint, zarm_l2_joint, zarm_r2_joint, ...
        for (int i = 0; i < 7; i++) {
            latest_vel_arm[i * 2] = msg->joint_v[i + 12];      // 左臂 (从索引12开始)   
            latest_vel_arm[i * 2 + 1] = msg->joint_v[i + 19];  // 右臂 (从索引19开始)
        }

        // 重新组织腿部速度数据
        // leg_l1_joint, leg_r1_joint, leg_l2_joint, leg_r2_joint, ...
        for (int i = 0; i < 6; i++) {
            latest_vel_leg[i * 2] = msg->joint_v[i];       // 左腿 (从索引0开始)
            latest_vel_leg[i * 2 + 1] = msg->joint_v[i + 6];   // 右腿 (从索引6开始)
        }
    }   
}

// 添加获取力矩数据的方法
std::vector<double> KuavoVirtualController::getLatestTauArm() {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    return latest_tau_arm;
}

std::vector<double> KuavoVirtualController::getLatestTauLeg() {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    return latest_tau_leg;
}

// 添加获取位置数据的方法
std::vector<double> KuavoVirtualController::getLatestPosArm() {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    return latest_pos_arm;
}

std::vector<double> KuavoVirtualController::getLatestPosLeg() {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    return latest_pos_leg;
}

// 添加获取速度数据的方法
std::vector<double> KuavoVirtualController::getLatestVelArm() {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    return latest_vel_arm;
}

std::vector<double> KuavoVirtualController::getLatestVelLeg() {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    return latest_vel_leg;
}

geometry_msgs::Vector3 KuavoVirtualController::vectorToROS(const std::vector<double>& vec) {
    geometry_msgs::Vector3 ros_vec;
    if (vec.size() >= 3) {
        ros_vec.x = vec[0];
        ros_vec.y = vec[1];
        ros_vec.z = vec[2];
    }
    return ros_vec;
}

geometry_msgs::Quaternion KuavoVirtualController::quaternionToROS(const std::vector<double>& quat) {
    geometry_msgs::Quaternion ros_quat;
    if (quat.size() >= 4) {
        ros_quat.w = quat[0];
        ros_quat.x = quat[1];
        ros_quat.y = quat[2];
        ros_quat.z = quat[3];
    }
    return ros_quat;
}

void KuavoVirtualController::timerCallback(const ros::TimerEvent&) {
    std::unique_lock<std::mutex> lock(sensor_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
        return;  // 如果无法获取锁，跳过这一次发布
    }
    
    if (!first_data_received_) {
        return;
    }
    
    // 使用客户端的时间戳
    ros::Time current_time = current_data_.timestamp;
    
    // 创建新的传感器消息
    kuavo_msgs::sensorsData sensor_msg;
    sensor_msg.header.stamp = current_time;
    sensor_msg.header.frame_id = "world";
    sensor_msg.sensor_time = current_time;  // 使用相同的时间戳
    
    // 计算插值系数
    double dt_since_update = (current_time - last_isaac_update_).toSec();
    double alpha = fmod(dt_since_update * PUBLISH_RATE, 1.0);
    
    // 对关节数据进行插值
    sensor_msg.joint_data.joint_q.resize(current_data_.joint_q.size());
    sensor_msg.joint_data.joint_v.resize(current_data_.joint_v.size());
    sensor_msg.joint_data.joint_torque.resize(current_data_.joint_current.size());
    sensor_msg.joint_data.joint_vd.resize(current_data_.joint_v.size());
    
    // 使用数值微分方法计算加速度
    static std::vector<double> last_velocity;
    static ros::Time last_velocity_time;
    
    // 初始化上一次速度数据
    if (last_velocity.empty()) {
        last_velocity = current_data_.joint_v;
        last_velocity_time = current_time;
        // 第一次运行时，将加速度设置为0
        for (size_t i = 0; i < current_data_.joint_v.size(); ++i) {
            sensor_msg.joint_data.joint_vd[i] = 0.0;
        }
    } else {
        // 计算实际时间间隔
        double dt = (current_time - last_velocity_time).toSec();
        if (dt > 0) {  // 确保时间间隔大于0
            for (size_t i = 0; i < current_data_.joint_v.size(); ++i) {
                // 使用当前速度和上一次速度计算加速度
                double current_velocity = current_data_.joint_v[i];
                sensor_msg.joint_data.joint_vd[i] = (current_velocity - last_velocity[i]) / dt;
                
                // 应用低通滤波器来平滑加速度数据
                static const double alpha_filter = 0.2;  // 滤波系数
                static std::vector<double> filtered_acc(current_data_.joint_v.size(), 0.0);
                filtered_acc[i] = alpha_filter * sensor_msg.joint_data.joint_vd[i] + 
                                (1 - alpha_filter) * filtered_acc[i];
                sensor_msg.joint_data.joint_vd[i] = filtered_acc[i];
            }
        }
        
        // 更新上一次的速度和时间
        last_velocity = current_data_.joint_v;
        last_velocity_time = current_time;
    }
    
    // 更新位置和速度数据
    for (size_t i = 0; i < current_data_.joint_q.size(); ++i) {
        sensor_msg.joint_data.joint_q[i] = previous_data_.joint_q[i] + 
            alpha * (current_data_.joint_q[i] - previous_data_.joint_q[i]);
        
        sensor_msg.joint_data.joint_v[i] = previous_data_.joint_v[i] + 
            alpha * (current_data_.joint_v[i] - previous_data_.joint_v[i]);
        
        sensor_msg.joint_data.joint_torque[i] = previous_data_.joint_current[i] + 
            alpha * (current_data_.joint_current[i] - previous_data_.joint_current[i]);
    }
    
    // 对IMU数据进行插值
    sensor_msg.imu_data.acc.x = previous_data_.imu_acc.x + 
        alpha * (current_data_.imu_acc.x - previous_data_.imu_acc.x);
    sensor_msg.imu_data.acc.y = previous_data_.imu_acc.y + 
        alpha * (current_data_.imu_acc.y - previous_data_.imu_acc.y);
    sensor_msg.imu_data.acc.z = previous_data_.imu_acc.z + 
        alpha * (current_data_.imu_acc.z - previous_data_.imu_acc.z);
    
    sensor_msg.imu_data.gyro.x = previous_data_.imu_gyro.x + 
        alpha * (current_data_.imu_gyro.x - previous_data_.imu_gyro.x);
    sensor_msg.imu_data.gyro.y = previous_data_.imu_gyro.y + 
        alpha * (current_data_.imu_gyro.y - previous_data_.imu_gyro.y);
    sensor_msg.imu_data.gyro.z = previous_data_.imu_gyro.z + 
        alpha * (current_data_.imu_gyro.z - previous_data_.imu_gyro.z);
    
    // 四元数插值
    sensor_msg.imu_data.quat = interpolateQuaternion(
        previous_data_.imu_quat, current_data_.imu_quat, alpha);
    
    // 只在running状态下发布消息
    if (is_running_) {
        sensor_pub_.publish(sensor_msg);
    }
}

// 添加四元数插值辅助函数
geometry_msgs::Quaternion KuavoVirtualController::interpolateQuaternion(
    const geometry_msgs::Quaternion& q1,
    const geometry_msgs::Quaternion& q2,
    double t) {
    // 简单线性插值（实际应用中可能需要使用SLERP）
    geometry_msgs::Quaternion result;
    result.x = q1.x + t * (q2.x - q1.x);
    result.y = q1.y + t * (q2.y - q1.y);
    result.z = q1.z + t * (q2.z - q1.z);
    result.w = q1.w + t * (q2.w - q1.w);
    
    // 归一化
    double norm = sqrt(result.x * result.x + result.y * result.y + 
                      result.z * result.z + result.w * result.w);
    result.x /= norm;
    result.y /= norm;
    result.z /= norm;
    result.w /= norm;
    
    return result;
}

/**
    初始化关节索引
*/
void KuavoVirtualController::setupJointIndices() {
    if (robot_version_ == 40) {  // 4代短手短脚
        // 左腿关节索引 (l1-l6)
        joint_indices_.leg_l_indices = {0, 4, 8, 12, 16, 20};
        // 右腿关节索引 (r1-r6)
        joint_indices_.leg_r_indices = {1, 5, 9, 13, 17, 21};
        // 左臂关节索引 (l1-l7)
        joint_indices_.arm_l_indices = {2, 6, 10, 14, 18, 22, 24};
        // 右臂关节索引 (r1-r7)
        joint_indices_.arm_r_indices = {3, 7, 11, 15, 19, 23, 25};
    } 
    else if (robot_version_ == 45) {  // 4pro长手
        // 左腿关节索引 (l1-l6)
        joint_indices_.leg_l_indices = {0, 5, 10, 14, 18, 22};
        // 右腿关节索引 (r1-r6)
        joint_indices_.leg_r_indices = {1, 6, 11, 15, 19, 23};
        // 左臂关节索引 (l1-l7)
        joint_indices_.arm_l_indices = {2, 7, 12, 16, 20, 24, 26};
        // 右臂关节索引 (r1-r7)
        joint_indices_.arm_r_indices = {3, 8, 13, 17, 21, 25, 27};
    }
    ROS_INFO("Joint indices initialized for robot version %d", robot_version_);
}

/**
    更新传感器数据
*/
void KuavoVirtualController::updateSensorMessage(const json& isaac_data) {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    
    if (!first_data_received_) {
        first_data_received_ = true;
    } else {
        previous_data_ = current_data_;
    }
    
    // 获取客户端的时间戳
    auto imu_data = isaac_data["imu"];
    double sim_time = imu_data["time"].get<double>();
    
    // 将仿真时间转换为ROS时间
    current_data_.timestamp.sec = static_cast<uint32_t>(sim_time);
    current_data_.timestamp.nsec = static_cast<uint32_t>((sim_time - static_cast<double>(current_data_.timestamp.sec)) * 1e9);
    
    last_isaac_update_ = current_data_.timestamp;
    
    // 获取原始数据
    std::vector<double> q_leg = isaac_data["q_leg"].get<std::vector<double>>();
    std::vector<double> dq_leg = isaac_data["dq_leg"].get<std::vector<double>>();
    std::vector<double> q_arm = isaac_data["q_arm"].get<std::vector<double>>();
    std::vector<double> dq_arm = isaac_data["dq_arm"].get<std::vector<double>>();
    std::vector<double> current_leg = isaac_data["current_leg"].get<std::vector<double>>();
    std::vector<double> current_arm = isaac_data["current_arm"].get<std::vector<double>>();
        
    current_data_.joint_q.resize(TOTAL_JOINTS);
    current_data_.joint_v.resize(TOTAL_JOINTS);
    current_data_.joint_current.resize(TOTAL_JOINTS);
    
    // 组合q dq current 关节数据
    if (robot_version_ == 40) { // 40 短手短脚
        // 处理左腿关节 (0-5)
        for (int i = 0; i < LEG_JOINTS_PER_SIDE; i++) {
            current_data_.joint_q[i] = q_leg[i * 2];
            current_data_.joint_v[i] = dq_leg[i * 2];
        }
    
        // 处理右腿关节 (6-11)
        for (int i = 0; i < LEG_JOINTS_PER_SIDE; i++) {
            current_data_.joint_q[i + LEG_JOINTS_PER_SIDE] = q_leg[i * 2 + 1];
            current_data_.joint_v[i + LEG_JOINTS_PER_SIDE] = dq_leg[i * 2 + 1];
        }
    
        // 处理左臂关节 (12-18)
        for (int i = 0; i < ARM_JOINTS_PER_SIDE; i++) {
            current_data_.joint_q[i + LEG_JOINTS_PER_SIDE * 2] = q_arm[i * 2];
            current_data_.joint_v[i + LEG_JOINTS_PER_SIDE * 2] = dq_arm[i * 2];
        }
    
        // 处理右臂关节 (19-25)
        for (int i = 0; i < ARM_JOINTS_PER_SIDE; i++) {
            current_data_.joint_q[i + LEG_JOINTS_PER_SIDE * 2 + ARM_JOINTS_PER_SIDE] = q_arm[i * 2 + 1];
            current_data_.joint_v[i + LEG_JOINTS_PER_SIDE * 2 + ARM_JOINTS_PER_SIDE] = dq_arm[i * 2 + 1];
        }
    
        // 添加头部关节 (26-27)，设置为0.0
        const int HEAD_START_IDX = TOTAL_JOINTS - HEAD_JOINTS;
        for (int i = 0; i < HEAD_JOINTS; i++) {
            current_data_.joint_q[HEAD_START_IDX + i] = 0.0;
            current_data_.joint_v[HEAD_START_IDX + i] = 0.0;
            current_data_.joint_current[HEAD_START_IDX + i] = 0.0;
        }
    
        // 重新组织力矩数据
        for (int i = 0; i < LEG_JOINTS_PER_SIDE; i++) {
            current_data_.joint_current[i] = current_leg[i * 2];                    // 左腿
            current_data_.joint_current[i + LEG_JOINTS_PER_SIDE] = current_leg[i * 2 + 1];  // 右腿
        }
    
        for (int i = 0; i < ARM_JOINTS_PER_SIDE; i++) {
            current_data_.joint_current[i + LEG_JOINTS_PER_SIDE * 2] = current_arm[i * 2];  // 左臂
            current_data_.joint_current[i + LEG_JOINTS_PER_SIDE * 2 + ARM_JOINTS_PER_SIDE] = current_arm[i * 2 + 1];  // 右臂
        }
    }
    else if (robot_version_ == 45) { // 45 长手
        // 处理左腿关节 (0-5)
        for (int i = 0; i < LEG_JOINTS_PER_SIDE; i++) {
            current_data_.joint_q[i] = q_leg[i * 2];
            current_data_.joint_v[i] = dq_leg[i * 2];
        }
    
        // 处理右腿关节 (6-11)
        for (int i = 0; i < LEG_JOINTS_PER_SIDE; i++) {
            current_data_.joint_q[i + LEG_JOINTS_PER_SIDE] = q_leg[i * 2 + 1];
            current_data_.joint_v[i + LEG_JOINTS_PER_SIDE] = dq_leg[i * 2 + 1];
        }
    
        // 处理左臂关节 (12-18)
        for (int i = 0; i < ARM_JOINTS_PER_SIDE; i++) {
            current_data_.joint_q[i + LEG_JOINTS_PER_SIDE * 2] = q_arm[i * 2];
            current_data_.joint_v[i + LEG_JOINTS_PER_SIDE * 2] = dq_arm[i * 2];
        }
    
        // 处理右臂关节 (19-25)
        for (int i = 0; i < ARM_JOINTS_PER_SIDE; i++) {
            current_data_.joint_q[i + LEG_JOINTS_PER_SIDE * 2 + ARM_JOINTS_PER_SIDE] = q_arm[i * 2 + 1];
            current_data_.joint_v[i + LEG_JOINTS_PER_SIDE * 2 + ARM_JOINTS_PER_SIDE] = dq_arm[i * 2 + 1];
        }
    
        // 添加头部关节 (26-27)，设置为0.0
        const int HEAD_START_IDX = TOTAL_JOINTS - HEAD_JOINTS;
        for (int i = 0; i < HEAD_JOINTS; i++) {
            current_data_.joint_q[HEAD_START_IDX + i] = 0.0;
            current_data_.joint_v[HEAD_START_IDX + i] = 0.0;
            current_data_.joint_current[HEAD_START_IDX + i] = 0.0;
        }
    
        // 重新组织力矩数据
        for (int i = 0; i < LEG_JOINTS_PER_SIDE; i++) {
            current_data_.joint_current[i] = current_leg[i * 2];                    // 左腿
            current_data_.joint_current[i + LEG_JOINTS_PER_SIDE] = current_leg[i * 2 + 1];  // 右腿
        }
    
        for (int i = 0; i < ARM_JOINTS_PER_SIDE; i++) {
            current_data_.joint_current[i + LEG_JOINTS_PER_SIDE * 2] = current_arm[i * 2];  // 左臂
            current_data_.joint_current[i + LEG_JOINTS_PER_SIDE * 2 + ARM_JOINTS_PER_SIDE] = current_arm[i * 2 + 1];  // 右臂
        }
    }
    // 设置IMU数据 - IMU正常
    std::vector<double> linear_acc = imu_data["linear_acceleration"].get<std::vector<double>>();
    std::vector<double> angular_vel = imu_data["angular_velocity"].get<std::vector<double>>();
    std::vector<double> orientation = imu_data["orientation"].get<std::vector<double>>();
    
    current_data_.imu_acc = vectorToROS(linear_acc);
    current_data_.imu_gyro = vectorToROS(angular_vel);
    current_data_.imu_quat = quaternionToROS(orientation);
    
    // 如果是第一帧数据，也将其设置为previous_data_
    if (velocity_history.empty()) {
        previous_data_ = current_data_;
    }
}

void KuavoVirtualController::publishSensorData(const json& isaac_data) {
    if (!isROSActive()) {
        ROS_ERROR("ROS node is not active!");
        return;
    }

    // 移除这里的运行状态检查，让数据处理继续进行
    updateSensorMessage(isaac_data);
}

bool KuavoVirtualController::handleSimStart(std_srvs::SetBool::Request& req,
                                          std_srvs::SetBool::Response& res) {
    is_running_ = req.data;
    if (req.data) {
        ROS_INFO("Received sim_start request: true");
    } else {
        ROS_INFO("Received sim_start request: false");
    }
    res.success = true;
    res.message = "Received sim_start request";
    return true;
}

// 修改processMsg_isaac函数
std::string processMsg_isaac(const char *msg) {
    static int print_counter = 0;
    json jmsg = bin2json(msg);
    
    // 获取控制器实例并发布数据
    auto controller = KuavoVirtualController::getInstance();
    controller->publishSensorData(jmsg);
    
    // 获取最新的力矩数据
    std::vector<double> tau_arm = controller->getLatestTauArm();
    std::vector<double> tau_leg = controller->getLatestTauLeg();
    
    // 获取最新的位置数据
    std::vector<double> pos_arm = controller->getLatestPosArm();
    std::vector<double> pos_leg = controller->getLatestPosLeg();

    // 获取最新的速度数据
    std::vector<double> vel_arm = controller->getLatestVelArm();
    std::vector<double> vel_leg = controller->getLatestVelLeg();

    // 如果力矩数据为空，填充零值
    if (tau_arm.empty()) {
        tau_arm.resize(14, 0.0);  // 14个手臂关节
    }
    if (tau_leg.empty()) {
        tau_leg.resize(12, 0.0);  // 12个腿部关节
    }
    if (pos_arm.empty()) {
        pos_arm.resize(14, 0.0);  // 14个手臂关节
    }
    if (pos_leg.empty()) {
        pos_leg.resize(12, 0.0);  // 12个腿部关节
    }
    if (vel_arm.empty()) {
        vel_arm.resize(14, 0.0);  // 14个手臂关节
    }
    if (vel_leg.empty()) {
        vel_leg.resize(12, 0.0);  // 12个腿部关节
    }
    
    // 打印位置信息
    // for (int i = 0; i < pos_leg.size(); i++) {
    //     ROS_INFO("leg_joint %d: %f", i, pos_leg[i]);
    // }
    // for (int i = 0; i < pos_arm.size(); i++) {
    //     std::cout << " ------------------------------ " << std::endl;
    //     ROS_INFO("arm_joint %d: %f", i, pos_arm[i]);
    //     std::cout << "  " << std::endl;
    // }
    
    // 创建临时调试数据
    std::vector<double> fixed_tau_leg(12, 200.0);
    std::vector<double> fixed_tau_arm(14, 200.0);
    std::vector<double> fixed_pos_leg(12, 1.57);
    std::vector<double> fixed_pos_arm(14, -1.57);

    // 创建响应数据
    json resp;
    // resp["arms"] = {
    //     {"ctrl_mode", "position"},
    //     {"joint_values", pos_arm}
    // };

    // resp["legs"] = {
    //     {"ctrl_mode", "position"},
    //     {"joint_values", pos_leg}
    // };

    // resp["arms"] = {
    //     {"ctrl_mode", "effort"},
    //     {"joint_values", tau_arm}  // 使用放大后的力矩数据
    // };
    
    resp["legs"] = {
        {"ctrl_mode", "effort"},
        {"joint_values", tau_leg}  // 使用放大后的力矩数据
    };

    resp["arms"] = {
        {"ctrl_mode", "velocity"},
        {"joint_values", vel_arm}  // 使用放大后的力矩数据
    };

    // resp["legs"] = {
    //     {"ctrl_mode", "velocity"},
    //     {"joint_values", vel_leg}  // 使用放大后的力矩数据
    // };

    return json2binstr(resp);
}
