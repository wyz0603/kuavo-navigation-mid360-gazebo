#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <thread>
#include <iostream>
#include "tcp_server.hpp"

class TcpNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit()
  {
    NODELET_INFO("Initializing TCP Nodelet nodelet...");
    control_thread = std::thread(&TcpNodelet::simloop, this);
    if (!control_thread.joinable())
    {
      std::cerr << "Failed to start control thread." << std::endl;
      exit(1);
    }
    NODELET_INFO("TCP Nodelet nodelet initialized.");
  }

private:
  ros::NodeHandle nh;
  std::thread control_thread;
  void simloop()
  {
    TcpServer server("0.0.0.0", 8800);
    server.launch();
  }
};

PLUGINLIB_EXPORT_CLASS(TcpNodelet, nodelet::Nodelet)
