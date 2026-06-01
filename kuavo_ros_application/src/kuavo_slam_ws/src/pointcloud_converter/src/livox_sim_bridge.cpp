// Bridge node: Gazebo Livox simulation plugin -> FAST-LIO inputs.
//
// Subscribes:  sensor_msgs/PointCloud   on /scan          (from liblivox_laser_simulation.so)
//              sensor_msgs/Imu          on /livox/imu     (from libgazebo_ros_imu_sensor.so)
// Publishes:   livox_ros_driver2/CustomMsg on /livox/lidar (for FAST-LIO, lidar_type=1)
//              sensor_msgs/Imu             on /livox/imu_corrected (for FAST-LIO)
//
// We do NOT publish /livox/cloud here (FAST-LIO's /cloud_registered_body is
// remapped to /livox/cloud and is what octomap consumes) nor a static TF for
// livox_frame (FAST-LIO publishes odom -> livox_frame dynamically).
//
// COORDINATE CORRECTION
// ---------------------
// The kuavo URDF mounts the MID360 in a chain that is NOT a pure 180-deg flip:
//   biped_s45_gazebo.urdf:  zhead_2_link -> radar    rpy=(0,    0.22969, 0)
//   livox_lidar.xacro:      radar        -> livox_link rpy=(pi, 0,       0)
// so the raw IMU at rest measures specific force ~(+2.2, 0, -9.55), not
// (0, 0, -9.81). A hard-coded 180-deg-X correction would still leave a ~13-deg
// pitch tilt, which is exactly what shows up in the map.
//
// Two modes are supported:
//   1) explicit rpy (default, robust): correction_rpy = [roll, pitch, yaw] is
//      the URDF rotation that takes a vector FROM lidar_frame TO an upright
//      robot frame. For kuavo s45 the right values are (pi, 0.22969, 0):
//        v_upright = R_z(yaw) * R_y(pitch) * R_x(roll) * v_lidar
//   2) TF lookup (optional): set upright_frame non-empty and the node will
//      lookupTransform(upright_frame, lidar_frame) at startup. Falls back to
//      the explicit rpy if the lookup times out.

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <livox_ros_driver2/CustomPoint.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <vector>

class LivoxSimBridge {
 public:
  LivoxSimBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : tf_buffer_(), tf_listener_(tf_buffer_), R_(tf2::Matrix3x3::getIdentity()),
        correction_active_(false) {
    pnh.param<std::string>("scan_topic", scan_topic_, "/scan");
    pnh.param<std::string>("lidar_topic", lidar_topic_, "/livox/lidar");
    pnh.param<std::string>("imu_in_topic",  imu_in_topic_,  "/livox/imu");
    pnh.param<std::string>("imu_out_topic", imu_out_topic_, "/livox/imu_corrected");
    pnh.param<std::string>("output_frame", output_frame_, "livox_frame");

    // --- Mode 2: optional TF lookup -----------------------------------------
    pnh.param<std::string>("lidar_frame",  lidar_frame_,  "livox_link");
    pnh.param<std::string>("upright_frame", upright_frame_, "");
    pnh.param<double>("tf_lookup_timeout", tf_lookup_timeout_, 10.0);

    // --- Mode 1: explicit rpy (default, used if TF lookup is disabled or fails)
    // Defaults are the empirically-tuned kuavo s45 values:
    //   roll=pi          (livox_lidar.xacro inversion)
    //   pitch=0.11 rad   (radar joint nominal 0.22969 minus settling)
    std::vector<double> rpy_default = {3.14159, 0.11, 0.0};
    std::vector<double> rpy;
    pnh.param<std::vector<double>>("correction_rpy", rpy, rpy_default);
    if (rpy.size() != 3) {
      ROS_WARN("livox_sim_bridge: correction_rpy must have 3 elements; using default.");
      rpy = rpy_default;
    }

    pnh.param<int>("scan_line", scan_line_, 4);
    pnh.param<int>("default_reflectivity", default_reflectivity_, 100);
    pnh.param<double>("frame_duration_ms", frame_duration_ms_, 100.0);
    // Hard Z shift added to every lidar point AFTER the rpy correction.
    // Use to lift the displayed cloud when you don't want to switch RViz's
    // Fixed Frame. Note: this physically translates the data so vertical motion
    // estimation in SLAM gets a constant bias; harmless for walking, not
    // recommended if the robot jumps/falls.
    pnh.param<double>("cloud_z_offset", cloud_z_offset_, 1.5);

    // Z-band filter applied AFTER rpy correction but BEFORE cloud_z_offset.
    // Coordinates are in the "virtual upright" sensor frame: 0 = lidar level,
    // negative = below the lidar, positive = above. Set min_z just above the
    // floor (e.g. -1.3 for a 1.5 m tall robot) to drop the ground ring scan.
    pnh.param<double>("min_z", min_z_, -1e9);
    pnh.param<double>("max_z", max_z_,  1e9);

    bool got_from_tf = false;
    if (!upright_frame_.empty()) {
      got_from_tf = lookupCorrectionFromTf();
    }
    if (!got_from_tf) {
      setCorrectionFromRpy(rpy[0], rpy[1], rpy[2]);
    }

    sub_scan_  = nh.subscribe(scan_topic_, 10, &LivoxSimBridge::scanCb, this);
    sub_imu_   = nh.subscribe(imu_in_topic_, 200, &LivoxSimBridge::imuCb, this);
    pub_lidar_ = nh.advertise<livox_ros_driver2::CustomMsg>(lidar_topic_, 10);
    pub_imu_   = nh.advertise<sensor_msgs::Imu>(imu_out_topic_, 200);

    double r, p, y;
    R_.getRPY(r, p, y);
    ROS_INFO_STREAM("livox_sim_bridge ready:\n"
                    "  " << scan_topic_   << " -> " << lidar_topic_ << "\n"
                    "  " << imu_in_topic_ << " -> " << imu_out_topic_ << "\n"
                    "  output_frame=" << output_frame_
                    << "  correction=" << (correction_active_ ? "ACTIVE" : "DISABLED")
                    << "  source=" << (got_from_tf ? "TF" : "explicit rpy")
                    << "  rpy(rad)=[" << r << ", " << p << ", " << y << "]"
                    << "  cloud_z_offset=" << cloud_z_offset_
                    << "  z_filter=[" << min_z_ << ", " << max_z_ << "]");
  }

  bool lookupCorrectionFromTf() {
    const ros::Time deadline = ros::Time::now() + ros::Duration(tf_lookup_timeout_);
    while (ros::ok() && ros::Time::now() < deadline) {
      try {
        geometry_msgs::TransformStamped tf =
            tf_buffer_.lookupTransform(upright_frame_, lidar_frame_, ros::Time(0));
        tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                          tf.transform.rotation.z, tf.transform.rotation.w);
        R_ = tf2::Matrix3x3(q);
        correction_active_ = true;
        return true;
      } catch (const tf2::TransformException& e) {
        ROS_WARN_THROTTLE(5.0, "livox_sim_bridge: waiting for TF %s -> %s: %s",
                          lidar_frame_.c_str(), upright_frame_.c_str(), e.what());
        ros::Duration(0.5).sleep();
      }
    }
    ROS_WARN_STREAM("livox_sim_bridge: TF lookup " << lidar_frame_ << " -> "
                    << upright_frame_ << " timed out after " << tf_lookup_timeout_
                    << "s; falling back to explicit correction_rpy.");
    return false;
  }

  void setCorrectionFromRpy(double roll, double pitch, double yaw) {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    R_ = tf2::Matrix3x3(q);
    correction_active_ = (std::abs(roll) > 1e-9 || std::abs(pitch) > 1e-9 || std::abs(yaw) > 1e-9);
  }

  inline void applyR(double& x, double& y, double& z) const {
    if (!correction_active_) return;
    const tf2::Vector3 v(x, y, z);
    const tf2::Vector3 vo = R_ * v;
    x = vo.x(); y = vo.y(); z = vo.z();
  }
  inline void applyR(float& x, float& y, float& z) const {
    if (!correction_active_) return;
    const tf2::Vector3 v(x, y, z);
    const tf2::Vector3 vo = R_ * v;
    x = static_cast<float>(vo.x());
    y = static_cast<float>(vo.y());
    z = static_cast<float>(vo.z());
  }

  void scanCb(const sensor_msgs::PointCloud::ConstPtr& msg) {
    if (msg->points.empty()) return;
    const size_t n = msg->points.size();
    const ros::Time stamp = msg->header.stamp;

    livox_ros_driver2::CustomMsg out;
    out.header.stamp = stamp;
    out.header.frame_id = output_frame_;
    out.timebase = static_cast<uint64_t>(stamp.toNSec());
    out.lidar_id = 0;
    const double frame_ns = frame_duration_ms_ * 1e6;
    const float z_off = static_cast<float>(cloud_z_offset_);
    const float min_z = static_cast<float>(min_z_);
    const float max_z = static_cast<float>(max_z_);
    out.points.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      float x = msg->points[i].x;
      float y = msg->points[i].y;
      float z = msg->points[i].z;
      applyR(x, y, z);
      if (z < min_z || z > max_z) continue;     // drop floor/ceiling/etc.
      livox_ros_driver2::CustomPoint p;
      p.x = x; p.y = y; p.z = z + z_off;
      p.reflectivity = static_cast<uint8_t>(default_reflectivity_);
      p.tag = 0;
      p.line = static_cast<uint8_t>(out.points.size() % scan_line_);
      p.offset_time = static_cast<uint32_t>((frame_ns * out.points.size()) / n);
      out.points.push_back(p);
    }
    out.point_num = out.points.size();
    pub_lidar_.publish(out);
  }

  void imuCb(const sensor_msgs::Imu::ConstPtr& msg) {
    sensor_msgs::Imu out = *msg;
    out.header.frame_id = output_frame_;
    applyR(out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z);
    applyR(out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z);
    pub_imu_.publish(out);
  }

 private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2::Matrix3x3 R_;
  bool correction_active_;

  ros::Subscriber sub_scan_, sub_imu_;
  ros::Publisher pub_lidar_, pub_imu_;
  std::string scan_topic_, lidar_topic_;
  std::string imu_in_topic_, imu_out_topic_;
  std::string output_frame_, lidar_frame_, upright_frame_;
  double tf_lookup_timeout_;
  int scan_line_;
  int default_reflectivity_;
  double frame_duration_ms_;
  double cloud_z_offset_;
  double min_z_, max_z_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_sim_bridge");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  LivoxSimBridge bridge(nh, pnh);
  ros::spin();
  return 0;
}
