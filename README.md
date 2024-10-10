# JIE Ware ROS 工具集

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
3. 修改Launch文件，替换AMCL

~~<node pkg="amcl" type="amcl" name="amcl">~~
~~    ......~~
~~</node>~~
```
<node pkg="jie_ware" type="lidar_loc" name="lidar_loc" >
    <param name="base_frame" value="base_footprint" />
    <param name="laser_frame" value="laser" />
</node>
```
4. 运行修改后的Launch文件
```
roslaunch jie_ware lidar_loc_test.launch 
```