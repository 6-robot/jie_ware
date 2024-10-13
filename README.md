# 阿杰的 ROS 工具箱

## 视频介绍

Bilibili: [《一种简单易用的激光雷达定位方法》](https://www.bilibili.com/video/BV1fB29YzEgP/)  
Youtube: [《一种简单易用的激光雷达定位方法》](https://www.youtube.com/watch?v=0JqGX8lKRu0)

## 使用步骤

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
3. 修改Launch文件，用如下内容替换AMCL节点
```
<node pkg="jie_ware" type="lidar_loc" name="lidar_loc" >
    <param name="base_frame" value="base_footprint" />
    <param name="laser_frame" value="laser" />
    <param name="laser_topic" value="scan" />
</node>
```
4. 运行修改后的Launch文件
```
roslaunch jie_ware lidar_loc_test.launch 
```