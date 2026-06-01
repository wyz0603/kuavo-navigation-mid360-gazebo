/**
 * @file hesai_to_velodyne_converter.cpp
 * @brief Hesai激光雷达点云格式转换为Velodyne兼容格式
 * @author Kuavo Team
 * @date 2025
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

// 定义Hesai点云格式
namespace hesai_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        double timestamp;
        uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

// 定义Velodyne点云格式
namespace velodyne_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        float time;
        uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

// 注册点云类型
POINT_CLOUD_REGISTER_POINT_STRUCT(hesai_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, timestamp, timestamp)
    (uint16_t, ring, ring)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)

class HesaiToVelodyneConverter {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    ros::Subscriber pointcloud_sub_;
    ros::Publisher velodyne_pub_;

    ros::Subscriber imu_sub_;
    ros::Publisher imu_converted_pub_;
    
    // 参数
    std::string input_topic_;
    std::string output_topic_;
    std::string input_frame_;
    std::string output_frame_;
    std::string imu_topic_;
    std::string imu_output_topic_;
    double time_scale_factor_;
    bool use_intensity_filter_;
    float min_intensity_;
    float max_intensity_;
    bool use_range_filter_;
    float min_range_;
    float max_range_;
    
    // 统计信息
    int total_points_received_;
    int total_points_published_;
    int conversion_count_;
    
public:
    HesaiToVelodyneConverter() : 
        private_nh_("~"),
        total_points_received_(0),
        total_points_published_(0),
        conversion_count_(0) {
        
        // 读取参数
        private_nh_.param<std::string>("input_topic", input_topic_, "/lidar_points");
        private_nh_.param<std::string>("output_topic", output_topic_, "/lidar_points_converted");
        private_nh_.param<std::string>("imu_topic", imu_topic_, "/lidar_imu");
        private_nh_.param<std::string>("imu_output_topic", imu_output_topic_, "lidar_imu_converted");
        private_nh_.param<std::string>("input_frame", input_frame_, "hesai_lidar");
        private_nh_.param<std::string>("output_frame", output_frame_, "livox_frame");
        private_nh_.param<double>("time_scale_factor", time_scale_factor_, 1000.0); // 将秒转换为毫秒
        
        // 滤波参数
        private_nh_.param<bool>("use_intensity_filter", use_intensity_filter_, false);
        private_nh_.param<float>("min_intensity", min_intensity_, 0.0);
        private_nh_.param<float>("max_intensity", max_intensity_, 255.0);
        private_nh_.param<bool>("use_range_filter", use_range_filter_, true);
        private_nh_.param<float>("min_range", min_range_, 0.5);
        private_nh_.param<float>("max_range", max_range_, 100.0);
        
        // 初始化发布者和订阅者
        pointcloud_sub_ = nh_.subscribe(input_topic_, 10, &HesaiToVelodyneConverter::pointcloudCallback, this);
        imu_sub_ = nh_.subscribe(imu_topic_, 10, &HesaiToVelodyneConverter::imuCallback, this);
        velodyne_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 10);
        imu_converted_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_output_topic_, 10);
        ROS_INFO("Hesai to Velodyne Converter initialized");
        ROS_INFO("Input topic: %s", input_topic_.c_str());
        ROS_INFO("Output topic: %s", output_topic_.c_str());
        ROS_INFO("IMU topic: %s", imu_topic_.c_str());
        ROS_INFO("IMU output topic: %s", imu_output_topic_.c_str());
        ROS_INFO("Input frame: %s", input_frame_.c_str());
        ROS_INFO("Output frame: %s", output_frame_.c_str());
        ROS_INFO("Time scale factor: %.1f", time_scale_factor_);
        ROS_INFO("Range filter: [%.2f, %.2f] meters", min_range_, max_range_);
        
        // 启动统计定时器
        ros::Timer stats_timer = nh_.createTimer(ros::Duration(10.0), &HesaiToVelodyneConverter::printStats, this);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& input_msg) {
        imu_converted_pub_.publish(*input_msg);
    }
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input_msg) {
        try {
            // 检查输入消息是否包含必要字段
            bool has_timestamp = false, has_ring = false;
            for (const auto& field : input_msg->fields) {
                if (field.name == "timestamp") has_timestamp = true;
                if (field.name == "ring") has_ring = true;
            }
            
            if (!has_timestamp || !has_ring) {
                ROS_WARN_THROTTLE(5.0, "Input point cloud missing required fields (timestamp/ring). "
                                      "Available fields: %s", getFieldNames(input_msg).c_str());
                return;
            }
            
            // 转换点云格式
            pcl::PointCloud<hesai_ros::Point> hesai_cloud;
            pcl::PointCloud<velodyne_ros::Point> velodyne_cloud;
            
            // 从ROS消息转换为PCL
            pcl::fromROSMsg(*input_msg, hesai_cloud);
            total_points_received_ += hesai_cloud.size();
            
            if (hesai_cloud.empty()) {
                ROS_WARN_THROTTLE(5.0, "Received empty point cloud");
                return;
            }
            
            // 预分配空间
            velodyne_cloud.reserve(hesai_cloud.size());
            
            // 计算时间基准（使用第一个点的时间戳）
            double base_time = hesai_cloud.points[0].timestamp;
            
            // 逐点转换
            for (const auto& hesai_point : hesai_cloud.points) {
                // 距离滤波
                if (use_range_filter_) {
                    float range = sqrt(hesai_point.x * hesai_point.x + 
                                     hesai_point.y * hesai_point.y + 
                                     hesai_point.z * hesai_point.z);
                    if (range < min_range_ || range > max_range_) {
                        continue;
                    }
                }
                
                // 强度滤波
                if (use_intensity_filter_) {
                    if (hesai_point.intensity < min_intensity_ || 
                        hesai_point.intensity > max_intensity_) {
                        continue;
                    }
                }
                
                // 创建Velodyne格式的点
                velodyne_ros::Point velodyne_point;
                velodyne_point.x = hesai_point.x;
                velodyne_point.y = hesai_point.y;
                velodyne_point.z = hesai_point.z;
                velodyne_point.intensity = hesai_point.intensity;
                velodyne_point.ring = hesai_point.ring;
                
                // 时间戳转换：从双精度秒转换为单精度相对时间（毫秒）
                velodyne_point.time = static_cast<float>((hesai_point.timestamp - base_time) * time_scale_factor_);
                
                velodyne_cloud.push_back(velodyne_point);
            }
            
            if (velodyne_cloud.empty()) {
                ROS_WARN_THROTTLE(5.0, "All points filtered out, publishing empty cloud");
            }
            
            total_points_published_ += velodyne_cloud.size();
            conversion_count_++;
            
            // 转换为ROS消息并发布
            sensor_msgs::PointCloud2 output_msg;
            pcl::toROSMsg(velodyne_cloud, output_msg);
            
            // 设置消息头
            output_msg.header.stamp = input_msg->header.stamp;
            output_msg.header.frame_id = output_frame_.empty() ? input_msg->header.frame_id : output_frame_;
            
            velodyne_pub_.publish(output_msg);
            
            ROS_DEBUG("Converted %zu points to %zu points", hesai_cloud.size(), velodyne_cloud.size());
            
        } catch (const std::exception& e) {
            ROS_ERROR("Error in point cloud conversion: %s", e.what());
        }
    }
    
    std::string getFieldNames(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        std::string field_names;
        for (size_t i = 0; i < msg->fields.size(); ++i) {
            if (i > 0) field_names += ", ";
            field_names += msg->fields[i].name;
        }
        return field_names;
    }
    
    void printStats(const ros::TimerEvent&) {
        if (conversion_count_ > 0) {
            double avg_input = static_cast<double>(total_points_received_) / conversion_count_;
            double avg_output = static_cast<double>(total_points_published_) / conversion_count_;
            double retention_rate = total_points_received_ > 0 ? 
                (static_cast<double>(total_points_published_) / total_points_received_ * 100.0) : 0.0;
            
            ROS_INFO("Conversion Stats - Frames: %d, Avg Input: %.1f pts, Avg Output: %.1f pts, Retention: %.1f%%",
                     conversion_count_, avg_input, avg_output, retention_rate);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "hesai_to_velodyne_converter");
    
    try {
        HesaiToVelodyneConverter converter;
        ROS_INFO("Hesai to Velodyne converter started");
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("Failed to start converter: %s", e.what());
        return 1;
    }
    
    return 0;
}
