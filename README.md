# 阿杰的 ROS 工具箱

## 视频介绍

Bilibili: [《一种简单易用的激光雷达定位方法》](https://www.bilibili.com/video/BV1fB29YzEgP/)  
Youtube: [《一种简单易用的激光雷达定位方法》](https://www.youtube.com/watch?v=0JqGX8lKRu0)  
Bilibili: [《代价地图清除》](https://www.bilibili.com/video/BV1kwzqYyEe7/)  
Youtube: [《代价地图清除》](https://www.youtube.com/watch?v=giHf_PY4EmY)  
基础：[《ROS 快速入门教材》](https://www.bilibili.com/video/BV1BP4y1o7pw/)  
扩展：[《ROS 导航，除了 DWA 和 TEB 还有没有其他选择？》](https://www.bilibili.com/video/BV1E7qKYjEjY/)

## 下载源代码

1. 获取源码:
```
cd ~/catkin_ws/src/
git clone https://github.com/6-robot/jie_ware.git
```
2. 编译
```
cd ~/catkin_ws
catkin_make
```
## 激光定位
1. 修改导航Launch文件，用如下内容替换AMCL节点
```
<node pkg="jie_ware" type="lidar_loc" name="lidar_loc" >
    <param name="base_frame" value="base_footprint" />
    <param name="odom_frame" value="odom" />
    <param name="laser_frame" value="laser" />
    <param name="laser_topic" value="scan" />
</node>
```
2. 运行修改后的Launch文件
```
roslaunch jie_ware lidar_loc_test.launch 
```
## 代价地图清除
1. 修改导航Launch文件，添加如下内容
```
<node pkg="jie_ware" type="costmap_cleaner" name="costmap_cleaner" />
```
2. 运行修改后的Launch文件，正常设置机器人的估计位置即可。