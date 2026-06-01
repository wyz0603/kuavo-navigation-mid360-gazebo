# !/bin/bash


kuavo_ros_application_dir=$(realpath "$(dirname "$(dirname "$(dirname "$0")")")")
echo "kuavo_ros_application_dir: $kuavo_ros_application_dir"
abs_script_dir=$(realpath "$(dirname "$0")")

echo "Building the project in $kuavo_ros_application_dir"

# Build Livox-SDK
if [ ! -f "/usr/local/lib/liblivox_sdk_static.a" ] ; then
    echo "Building Livox-SDK..."
    cd $abs_script_dir/Livox-SDK
    mkdir -p build
    cd build
    rm -rf * || true
    cmake .. && make -j$(nproc)
    sudo make install
else
    echo "Livox-SDK already installed, skipping..."
fi

# --- . 强制重新构建逻辑 ---
# 定义库文件路径变量（使用之前定义的 INSTALL_PREFIX）
INSTALL_PREFIX=/usr/local
LIVOX_STATIC_LIB="${INSTALL_PREFIX}/lib/liblivox_lidar_sdk_static.a"
LIVOX_SHARED_LIB="${INSTALL_PREFIX}/lib/liblivox_lidar_sdk_shared.so"

# 执行清理操作（这里直接复用你之前的清理逻辑）
sudo rm -f "$LIVOX_STATIC_LIB"
sudo rm -f "$LIVOX_SHARED_LIB"
sudo rm -rf "${INSTALL_PREFIX}/include/livox"
sudo rm -rf "${INSTALL_PREFIX}/lib/cmake/Livox-SDK2"
sudo rm -rf "${INSTALL_PREFIX}/share/Livox-SDK2"

# 判断：如果任意一个库文件存在
echo "开始构建 Livox-SDK2..."
cd "$abs_script_dir/Livox-SDK2" || exit 1
mkdir -p build
cd build || exit 1

# 强制重新配置和编译
rm -rf * 
cmake .. -DCMAKE_INSTALL_PREFIX=${INSTALL_PREFIX}
make -j$(nproc) && sudo make install


# install ros dependencies
cd $kuavo_ros_application_dir
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
rosdep update --rosdistro noetic
rosdep install --from-paths "$abs_script_dir/src" --ignore-src -r -y

# Build livox_ros_driver2
cd $kuavo_ros_application_dir
catkin build -DROS_EDITION=ROS1 livox_ros_driver2

# Build fast_lio and livox_fast_lio
cd $abs_script_dir/src/FAST_LIO
git submodule update --init
cd $kuavo_ros_application_dir
catkin build fast_lio

# Build octomap_mapping
cd $kuavo_ros_application_dir
sudo apt-get install -y octovis
catkin build octomap_mapping octomap_rviz_plugins

# Build pointcloud_to_laserscan
sudo apt-get install -y ros-noetic-tf2-sensor-msgs
cd $kuavo_ros_application_dir
catkin build pointcloud_to_laserscan

# Build rs_to_velodyne
cd $kuavo_ros_application_dir
catkin build rs_to_velodyne

# Build rslidar_sdk
cd $kuavo_ros_application_dir
sudo apt-get install -y libyaml-cpp-dev
sudo apt-get install -y  libpcap-dev
catkin build rslidar_sdk

# Build kuavo_mapping
cd $kuavo_ros_application_dir
catkin build kuavo_mapping

# Build foxglove_bridge
cd $kuavo_ros_application_dir
cp -r $abs_script_dir/src/ros-foxglove-bridge/tls/* ~/.tls
sudo apt-get install -y ros-noetic-ros-babel-fish
catkin build foxglove_bridge

cd $kuavo_ros_application_dir
sudo apt-get install -y python3-pip
pip3 install open3d -i https://pypi.tuna.tsinghua.edu.cn/simple
pip3 install --upgrade setuptools importlib_metadata -i https://pypi.tuna.tsinghua.edu.cn/simple
catkin build navigation kuavo_navigation mpc_local_planner

mkdir -p ~/maps || echo "~/maps already exists"

pip install -U numpy
