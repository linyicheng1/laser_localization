# laser_localization 

laser_localization 是一个应用于小范围场景的3D激光雷达定位算法，典型应用场景为工业园区、小区或者变电站等。融合3D激光点云，轮式里程计以及IMU角度信息，实现高精实时定位。采用分支定界搜索算法进行全局定位，而局部点云和全局地图采用NDT匹配方法。轮式里程计和IMU角度信息作为匹配算法的运动先验参数，加速激光匹配，避免陷入局部最优。


[<img src="https://user-images.githubusercontent.com/50650063/199487864-d3f48906-44dc-4baf-8523-500bca800770.png" width = "600" height = "200" alt="效果展示" align=center />](https://www.bilibili.com/video/BV12P4y1m7nH/?spm_id_from=333.999.0.0&vd_source=4dd69fa6d40221a0fa0733def5c4708a)

[演示视频](https://www.bilibili.com/video/BV12P4y1m7nH/?spm_id_from=333.999.0.0&vd_source=4dd69fa6d40221a0fa0733def5c4708a)


|            |                    |                          |
|------------|--------------------|--------------------------|
| **适用定位范围** | 500m x 500m (无GPS) | \ (有GPS 情况下未测试，理论在1km以上) |  
| **运行速度**   | 小于 0.8m/s (单激光)    | 1-2m/s (雷达 + 轮式里程计)      |   
| **定位精度**   | 典型值 2-3cm          | 典型值 2-3cm                |  


## 1. 准备环境

1.1 **Ubuntu** 和 **ROS**，Ubuntu 18.04. ROS Dashing && Foxy

```
sudo apt install ros-YOUR_DISTRO-desktop ros-YOUR_DISTRO-pcl
```
[ROS2 安装 WIKI](https://docs.ros.org/en/dashing/Installation/Ubuntu-Development-Setup.html)

## 2. ROS2 中编译 laser_localization

克隆本项目然后编译：

```shell
cd ~/catkin_ws/src
git clone https://github.com/linyicheng1/laser_localization.git
cd ../
colcon build 
source ~/catkin_ws/install/setup.bash
```

## 3. KITTI 数据集上运行

下载KITTI数据集：
[百度网盘]()|[谷歌网盘]()

修改代码 `./launch/kitti_test.py` 中的路径：

```python 
61| parameters=[
62|     {"kitti_path": "${YOUR_PATH}"}, 
63| ]

40| {"global_map": "${YOUR_PATH}"},
```

运行代码：

```shell 
ros2 launch ~/catkin_ws/src/laser_localization/launch/kitti_test.py
```

## 4. 在你的设备上运行

#### 4.0  点云地图构建

可采用目前主流的3D激光SLAM算法构建点云地图并保存在`./map/map.pcd`处。
推荐的3D点云算法：[hdl_graph_slam](https://github.com/koide3/hdl_graph_slam)、[ALOAM](https://github.com/tops666/Aloam)、[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)

#### 4.1  单激光模式

单激光模式仅需要一个3D雷达即可，可用于快速部署在低速小车中运行。需要提供 `tf` 变换树中 `base_link->laser` 的相对变换关系，并提供3d点云数据话题`pointcloud2`。

运行代码：

```shell
ros2 launch ~/catkin_ws/src/laser_localization/launch/single_laser.py
```

#### 4.2  多传感器融合定位

多传感器融合定位模式下精度和运行速度均能得以提升，但需要提供更多信息。
1. 里程计数据，`tf`变换树中 `odom->base_link`, 为轮式里程计计算得到。里程计的计算需要对机器人轮子半径进行标定，直走10m以上标定轮距，旋转多圈并标定轴距。

运行代码：

```shell
ros2 launch ~/catkin_ws/src/laser_localization/launch/localization.py
```

## 5. 定位算法优势


- 激光里程计 + 激光点云与地图匹配，只采用3d雷达的激光定位方案，可以达到10-15hz左右的输出，在低速时直接使用部署非常简单。

- 激光定位与轮式里程计分离，可选择性的采用轮式里程计信息为激光匹配提供先验，有效避免车轮打滑等扰动影响。

- 提供匹配置信度信息，实时反馈当前定位状态。

- 提供全局定位获得初始值方法，并包含程序掉电保存当前位置的功能。

## 6. 许可证

本源代码在[GPLv3](http://www.gnu.org/licenses/)许可证下发布
