/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2020-2025, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/* @author Zhang Wanjie                                             */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <limits> 
#include <cmath> 

class CLidarFilter
{
public:
    CLidarFilter(ros::NodeHandle& nh, ros::NodeHandle& pnh); 
private:
     ros::NodeHandle n_; 
     ros::Publisher scan_pub_;
     ros::Subscriber scan_sub_;
     std::string source_topic_name_;
     std::string pub_topic_name_;
     double outlier_threshold_;

     void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};

CLidarFilter::CLidarFilter(ros::NodeHandle& nh, ros::NodeHandle& pnh) : n_(nh)
{
    // 从launch文件获取参数
    pnh.param<std::string>("source_topic", source_topic_name_, "/scan");        // 原始数据话题
    pnh.param<std::string>("pub_topic", pub_topic_name_, "/scan_filtered");     // 滤波后的数据话题
    pnh.param<double>("outlier_threshold", outlier_threshold_, 0.1);            // 用于离群点判断的阈值

    scan_pub_ = n_.advertise<sensor_msgs::LaserScan>(pub_topic_name_, 10);
    scan_sub_ = n_.subscribe<sensor_msgs::LaserScan>(source_topic_name_, 10, &CLidarFilter::lidarCallback, this);
}

void CLidarFilter::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int nRanges = scan->ranges.size();

    // 如果点数太少，无法进行近邻比较，直接发布原始数据
    if (nRanges < 3) 
    {
        scan_pub_.publish(scan);
        return;
    }

    sensor_msgs::LaserScan new_scan;

    // 拷贝源数据
    new_scan.header = scan->header; 
    new_scan.angle_min = scan->angle_min;
    new_scan.angle_max = scan->angle_max;
    new_scan.angle_increment = scan->angle_increment;
    new_scan.time_increment = scan->time_increment;
    new_scan.scan_time = scan->scan_time; 
    new_scan.range_min = scan->range_min; 
    new_scan.range_max = scan->range_max;
    new_scan.ranges = scan->ranges; 
    if (!scan->intensities.empty()) 
    {
        new_scan.intensities = scan->intensities; 
    }
	
    // 对new_scan中的离群点进行剔除
    // 遍历备份数据 new_scan.ranges，从第二个点到倒数第二个点
    // 因为第一个点没有前一个点，最后一个点没有后一个点
    for (int i = 1; i < nRanges - 1; ++i)
    {
        float prev_range = new_scan.ranges[i-1];
        float current_range = new_scan.ranges[i];
        float next_range = new_scan.ranges[i+1];

        // 检查当前点是否有效 (在min_range和max_range之间，且不是inf/nan)
        // 使用 new_scan 的 range_min 和 range_max 进行有效性判断
        bool current_valid = std::isfinite(current_range) && 
                             current_range >= new_scan.range_min && 
                             current_range <= new_scan.range_max;

        if (!current_valid) 
        {
            continue; // 当前点本身无效，跳过
        }

        // 离群点判定
        if (std::abs(current_range - prev_range) > outlier_threshold_ &&
            std::abs(current_range - next_range) > outlier_threshold_)
        {
            // 判定为离群点，将其无效化
            new_scan.ranges[i] = std::numeric_limits<float>::infinity();
            // 如果有强度信息，将其对应强度设为0
            if (!new_scan.intensities.empty() && i < new_scan.intensities.size()) 
            {
                new_scan.intensities[i] = 0.0f; 
            }
        }
    }

    scan_pub_.publish(new_scan);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"lidar_filter_node");

    ros::NodeHandle nh; 
    ros::NodeHandle pnh("~"); 

    CLidarFilter lidar_filter(nh, pnh);
    
    ros::spin();
    return 0; 
}