# laser_localization | [CN](https://github.com/linyicheng1/laser_localization/blob/main/README_CN.md)

laser_localization is a 3D LiDAR localization algorithm applied to small area scenes, typical application scenarios are industrial parks, neighborhoods or substations, etc. It combines 3D laser point cloud, wheeled odometer and IMU angle information to achieve high precision real-time positioning. A branch-and-bound search algorithm is used for global positioning, while NDT matching is used for local point clouds and global maps. The wheeled odometer and IMU angle information are used as the motion a priori parameters of the matching algorithm to accelerate laser matching and avoid falling into local optimum.


[<img src="https://user-images.githubusercontent.com/50650063/199487864-d3f48906-44dc-4baf-8523-500bca800770.png" width = "600" height = "200" alt="效果展示" align=center />](https://www.bilibili.com/video/BV12P4y1m7nH/?spm_id_from=333.999.0.0&vd_source=4dd69fa6d40221a0fa0733def5c4708a)

[Demo Video](https://www.bilibili.com/video/BV12P4y1m7nH/?spm_id_from=333.999.0.0&vd_source=4dd69fa6d40221a0fa0733def5c4708a)


|            |                    |                          |
|------------|--------------------|--------------------------|
| **Scene Range** | 500m x 500m (without GPS) | \ (with GPS case not tested, theoretical in 1km above.) |  
| **Speed**   | less than 0.8m/s (only 3d laser)    | 1-2m/s (3d laser + Wheeled odometer)      |   
| **Accuracy**   | Typical 2-3cm          | Typical 2-3cm                |  


## 1. Prerequisites

#### 1.1 **Ubuntu** 和 **ROS**，Ubuntu 18.04. ROS Dashing && Foxy

```
sudo apt install ros-YOUR_DISTRO-desktop ros-YOUR_DISTRO-pcl
```
[ROS2 Install WIKI](https://docs.ros.org/en/dashing/Installation/Ubuntu-Development-Setup.html)

## 2. build laser_localization on ROS2

Clone the repository and build:

```shell
cd ~/catkin_ws/src
git clone https://github.com/linyicheng1/laser_localization.git
cd ../
colcon build 
source ~/catkin_ws/install/setup.bash
```

## 3. Run with KITTI dataset 

Download KITTI dataset：
[BaiDu Driver](https://pan.baidu.com/s/1BaVZKkQu8WT2Yo4k4Omjug?pwd=6hrm)|[Google Driver](https://drive.google.com/file/d/1_gsbxX-M7xfUJSB0JO8qOGX5Adfaw0q_/view?usp=sharing)

Modify the path code in `./launch/localization.py`：

```python 
23| 
24| 
25| 
26| 
```

run cmd in shell：

```shell 
ros2 launch ~/catkin_ws/src/laser_localization/launch/single_laser.py
```

## 4. Run with your device

#### 4.0 Build point cloud map 

Point cloud maps can be constructed using the current mainstream 3D laser SLAM algorithm and saved at `. /map/map.pcd`.

Recommended 3D point cloud algorithm: [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam)、[ALOAM](https://github.com/tops666/Aloam)、[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)

#### 4.1  Laser only mode 

The single laser mode requires only one 3d radar and can be used for rapid deployment to operate in low speed carts. The relative transformation relationship of `base_link->laser` in the `tf` transformation tree needs to be provided, and the 3d point cloud data topic `pointcloud2` needs to be provided.

run cmd:

```shell
ros2 launch ~/catkin_ws/src/laser_localization/launch/single_laser.py
```

#### 4.2 Multi-sensor fusion localization

Multi-sensor fusion positioning mode improves accuracy and speed, but requires more information.

1. The odometer data, `odom->base_link` in the `tf` transformation tree, is calculated for the wheel odometer. The calculation of the odometer requires calibration of the robot's wheel radius, calibration of the wheel distance over 10m straight ahead, and multiple rotations and calibration of the axis distance.

Run in shell:

```shell
ros2 launch ~/catkin_ws/src/laser_localization/launch/localization.py
```

## 5. Advantages

- Laser odometer + laser point cloud with map matching, laser positioning solution using only 3d radar, can reach about 10-15hz output, very easy to deploy at low speed using directly.

- Laser positioning is separated from wheel odometer, which can selectively use wheel odometer information to provide a priori for laser matching, effectively avoiding the influence of wheel slip and other disturbances.

- Provide matching confidence information and real-time feedback on current positioning status.

- Provide global positioning to obtain the initial value method, and include the function of program power down to save the current position.

## 6. Licence

The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.
