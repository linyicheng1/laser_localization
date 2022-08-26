# laser_localization
laser localization base global map for robotics 

针对固定区域，小范围内运行的机器人的定位算法，如小区、工业园区，变电站等场景，提出的基于NDT匹配的全局定位算法。

支持单激光雷达定位，可选择性的添加轮式里程计。

主要特点：

1. 激光里程计 + 激光点云与地图匹配，单独独立工作的激光定位方案，可以达到10-15hz左右的输出。
2. 激光定位与轮式里程计分离，部分算法采用轮式里程计信息预测定位。但是轮式里程计在户外场景中出现打滑等不确定因素太多。而NDT算法对初始值敏感，需要耗费大量的时间才能将误差纠正回来，或者直接计算结果被预测值带偏，导致定位精度下降。
3. 提供匹配置信度信息，实时反馈当前定位状态。
4. 提供全局定位获得初始值方法（不稳定，依旧建议手动设置初始位置，每次从初始位置启动）

## 激光里程计

采用NDT算法对相邻激光点云进行匹配得到相对位姿，累计得到激光里程计信息。

![image.png](http://www.static.linyicheng.com.cn/articles/801686ad3353336c27228b273f1c7778.png)

**逻辑图**
![laser_odometer.svg](http://www.static.linyicheng.com.cn/articles/cf02b00d9bd136a591474fbe3b780701.svg)

## 基于地图的定位

TODO

## 轮式里程计

TODO

## 全局定位初始化

TODO
