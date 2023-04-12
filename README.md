# 麦克纳姆轮小车开发记录
```
开发人员：杨工

工作记录：
————2023年4月9号
1、麦克纳姆轮小车模型建立，添加雷达、摄像头、陀螺仪数据
2、自定义地图数据
3、添加键盘遥控节点，WASD控制，QE为转向，Shift加速，其他键盘任意键暂停
4、gmapping建图，保存地图
5、amcl定位，move_base导航
6、cartographer建图、保存地图

————2023年4月12号
1、添加cartographer编译运行步骤
2、新建两个额外的仓库存放依赖文件和cartographer_ros文件

```

**系统环境**

```
ubuntu 18.04
ros melodic
```

**整体文件目录**

分为三个仓库保存，分别是：
```
McKnum_car_sim 小车的导航仿真代码
carto_file 运行cartographer的依赖文件
carto_ws cartographer的工作空间
```

下载protobuf和cartographer包：
```
git clone git@github.com:haicheng12/carto_file.git //下载完自行解压里面两个压缩包
```

下载ceres包：
```
cd carto_file
git clone https://github.com/ceres-solver/ceres-solver.git
```

安装ceres包：
```
# CMake
sudo apt-get install cmake
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev libgflags-dev
# BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# Eigen3
sudo apt-get install libeigen3-dev
# SuiteSparse and CXSparse (optional)
sudo apt-get install libsuitesparse-dev
```

进入ceres执行下面的命令：
```
mkdir build
cmake ..
make//这个过程非常的漫长
sudo make install
```

安装protobuf3：
```
sudo apt-get install autoconf automake libtool curl make g++ unzip//安装依赖
cd protobuf
cd protobuf
./autogen.sh
./configure
make
make check
sudo make install
sudo ldconfig// 输出protobuf版本信息则表示安装成功
protoc --version
```

安装cartographer包：
```
sudo apt-get install -y \
    g++ \
    git \
    google-mock \
    libboost-all-dev \
    libcairo2-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    liblua5.2-dev \
    libsuitesparse-dev \
    ninja-build \
    python-sphinx
```

```
cd cartographer
mkdir build
cmake .. -G Ninja//可能报错误一，解决办法见错误一。还有可能错误二，解决办法见错误二
ninja//需要很长时间
ninja test
sudo ninja install
```

如果有错误：
错误一：provided by "absl"
```
sudo apt-get install stow
sudo chmod +x ~/cartographer/src/cartographer/scripts/install_abseil.sh//相对路径
cd ~/cartographer/src/cartographer/scripts
./install_abseil.sh
```
错误二：Did not find Lua >= 5.2.
```
sudo apt-get install liblua5.2-dev
```

编译cartographer_ros：
```
mkdir -p ~/carto_ws/src
cd carto_ws/src
catkin_init_workspace
git clone git@github.com:haicheng12/carto_ws.git
cd ~/carto_ws
catkin_make_isolated --install --use-ninja//需要很长时间
source install_isolated/setup.bash
```

**仿真测试**

编译代码：
```
mkdir -p ~/catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
git clone git@github.com:haicheng12/McKnum_car_sim.git
cd ~/catkin_ws
catkin_make
```

仿真环境启动：
```
$ roslaunch atom atom_world.launch
```
键盘遥控：
```
$ rosrun atom teleop_cmd_vel
```

安装navigation依赖环境：
```
sudo apt install ros-melodic-navigation
```

**gmapping建图**

启动建图：
```
$ roslaunch atom gmapping.launch
```

遥控小车走完建图区域
```
$ rosrun atom teleop_cmd_vel
```

保存地图：
```
$ rosrun map_server map_saver -f map //保存的地图map.pgm和map.yaml放到atom/maps/
```

**amcl定位和move_base导航**

启动定位和导航：
```
$ roslaunch atom navigation.launch
```


**cartographer建图**

仿真环境启动：
```
$ roslaunch atom atom_world.launch
```

启动cartographer：
```
$ roslaunch cartographer_ros demo_revo_lds.launch
```

修改的东西：
cartographer_ros/cartographer_ros/launch/demo_revo_lds.launch
```
<launch>
  <param name="/use_sim_time" value="true" />

  <node name="cartographer_node" pkg="cartographer_ros"
      type="cartographer_node" args="
          -configuration_directory $(find cartographer_ros)/configuration_files
          -configuration_basename revo_lds.lua"
      output="screen">
    <remap from="scan" to="scan" />
  </node>

  <node name="rviz" pkg="rviz" type="rviz" required="true"
      args="-d $(find cartographer_ros)/configuration_files/demo_2d.rviz" />
</launch>
```
cartographer_ros/cartographer_ros/configuration_files/revo_lds.lua
```
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "hokuyo_link",
  published_frame = "hokuyo_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.min_range = 0.2
TRAJECTORY_BUILDER_2D.max_range = 20.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.65

return options
```

保存地图：
```
$ rosservice call /finish_trajectory 0
status: 
  code: 0
  message: "Finished trajectory 0."
```

```
$ rosservice call /write_state  "filename: '/home/ubuntu/map.pbstream'
include_unfinished_submaps: false" 
status: 
  code: 0
  message: "State written to '/home/ubuntu/map.pbstream'." //ubuntu改为自己电脑的名字

```
转化地图：
```
$ rosrun cartographer_ros cartographer_pbstream_to_ros_map  -map_filestem=/home/ubuntu/map -pbstream_filename=/home/ubuntu/map.pbstream -resolution=0.05 //ubuntu改为自己电脑的名字
```








