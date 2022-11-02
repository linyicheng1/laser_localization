# laser_localization

针对固定区域，小范围内运行的机器人的定位算法，如小区、工业园区，变电站等场景，提出的基于3d激光NDT匹配的全局定位算法。


 [<img src="https://user-images.githubusercontent.com/50650063/199487864-d3f48906-44dc-4baf-8523-500bca800770.png" width = "600" height = "200" alt="效果展示" align=center />](https://www.bilibili.com/video/BV12P4y1m7nH/?spm_id_from=333.999.0.0&vd_source=4dd69fa6d40221a0fa0733def5c4708a)


## 算法基本参数 

|            |                    |                          |
|------------|--------------------|--------------------------|
| **适用定位范围** | 500m x 500m (无GPS) | \ (有GPS 情况下未测试，理论在1km以上) |  
| **运行速度**   | 小于 0.8m/s (单激光)    | 1-2m/s (雷达 + 轮式里程计)      |   
| **定位精度**   | 典型值 2-3cm          | 典型值 2-3cm                |  


## 定位算法优势与特点

1. 激光里程计 + 激光点云与地图匹配，只采用3d雷达的激光定位方案，可以达到10-15hz左右的输出，在低速时直接使用部署非常简单。
2. 激光定位与轮式里程计分离，可选择性的采用轮式里程计信息为激光匹配提供先验，有效避免车轮打滑等扰动影响。
3. 提供匹配置信度信息，实时反馈当前定位状态。
4. 提供全局定位获得初始值方法，并包含程序掉电保存当前位置的功能。

## 激光里程计

采用NDT算法对相邻激光点云进行匹配得到相对位姿，累计得到激光里程计信息。

![image.png](http://www.static.linyicheng.com.cn/articles/801686ad3353336c27228b273f1c7778.png)

**逻辑图**
![laser_odometer.svg](http://www.static.linyicheng.com.cn/articles/cf02b00d9bd136a591474fbe3b780701.svg)

## 基于地图的定位


## 轮式里程计

TODO

## 全局定位初始化

TODO
