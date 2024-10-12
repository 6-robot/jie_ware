#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <vector>
#include <cmath>

nav_msgs::OccupancyGrid map_msg;
cv::Mat map_cropped;
cv::Mat map_temp;
cv::Mat map_match;
sensor_msgs::RegionOfInterest map_roi_info;
std::vector<cv::Point2f> scan_points;
std::vector<cv::Point2f> transform_points;
ros::ServiceClient clear_costmaps_client;
std::string base_frame;
std::string laser_frame;

float lidar_x = 250, lidar_y = 250, lidar_yaw = 0;
int cur_sum = 0;
int clear_countdown = -1;

// 初始姿态回调函数
void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    try {
        static tf2_ros::Buffer tfBuffer;
        static tf2_ros::TransformListener tfListener(tfBuffer);

        // 1. 查询从 base_frame 到 laser_frame 的转换
        geometry_msgs::TransformStamped transformStamped = 
            tfBuffer.lookupTransform(base_frame, laser_frame, ros::Time(0), ros::Duration(1.0));

        // 2. 创建一个 stamped pose 用于转换
        geometry_msgs::PoseStamped base_pose, laser_pose;
        base_pose.header = msg->header;
        base_pose.pose = msg->pose.pose;

        // 3. 将 base_footprint 的位姿转换为 laser 的位姿
        tf2::doTransform(base_pose, laser_pose, transformStamped);

        // 4. 从转换后的消息中提取位置和方向信息
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        tf2::Quaternion q(
            laser_pose.pose.orientation.x,
            laser_pose.pose.orientation.y,
            laser_pose.pose.orientation.z,
            laser_pose.pose.orientation.w);
        
        // 5. 将四元数转换为yaw角度
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // 6. 将地图坐标转换为OpenCV坐标（OpenCV的y轴是向下的，而ROS的y轴是向上的）
        lidar_x = (x - map_msg.info.origin.position.x) / map_msg.info.resolution - map_roi_info.x_offset;
        lidar_y = (y - map_msg.info.origin.position.y) / map_msg.info.resolution - map_roi_info.y_offset;
        
        // 7. 设置yaw角度
        lidar_yaw = -yaw;
        clear_countdown = 30;
    }
    catch (tf2::TransformException &ex) 
    {
        ROS_WARN("无法获取从 %s 到 %s 的转换: %s", base_frame.c_str(), laser_frame.c_str(), ex.what());
    }
}

void crop_map();
void processMap();

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_msg = *msg;
    crop_map();
    processMap();        
}

void crop_map()
{
    // 显示地图信息
    std_msgs::Header header = map_msg.header;
    nav_msgs::MapMetaData info = map_msg.info;
    
    //用来统计地图有效区域的变量
    int xMax,xMin,yMax,yMin ;
    xMax=xMin= info.width/2;
    yMax=yMin= info.height/2;
    bool bFirstPoint = true;

    // 把地图数据转换成图片
    cv::Mat map_raw(info.height, info.width, CV_8UC1, cv::Scalar(128)); // 灰色背景

    for(int y = 0; y < info.height; y++)
    {
        for(int x = 0; x < info.width; x++)
        {
            int index = y * info.width + x;
            
            // 直接使用map_msg.data中的值
            map_raw.at<uchar>(y, x) = static_cast<uchar>(map_msg.data[index]);

            // 统计有效区域
            if(map_msg.data[index] != -1)
            {
                if(bFirstPoint)
                {
                    xMax = xMin = x;
                    yMax = yMin = y;
                    bFirstPoint = false;
                    continue;
                }
                xMin = std::min(xMin, x);
                xMax = std::max(xMax, x);
                yMin = std::min(yMin, y);
                yMax = std::max(yMax, y);
            }
        }
    }
    // 计算有效区域的中心点坐标
    int cen_x = (xMin + xMax)/2;
    int cen_y = (yMin + yMax)/2;

    // 按照有效区域对地图进行裁剪
    int new_half_width = abs(xMax - xMin)/2 + 50;
    int new_half_height = abs(yMax - yMin)/2 + 50;
    int new_origin_x = cen_x - new_half_width;
    int new_origin_y = cen_y - new_half_height;

    int new_width = new_half_width*2;
    int new_height = new_half_height*2;
    cv::Rect roi(new_origin_x, new_origin_y, new_width, new_height);
    cv::Mat roi_map = map_raw(roi).clone();

    map_cropped = roi_map;

    // 地图的裁减信息 
    map_roi_info.x_offset = new_origin_x;
    map_roi_info.y_offset = new_origin_y;
    map_roi_info.width = new_width;
    map_roi_info.height = new_height;

    geometry_msgs::PoseWithCovarianceStamped init_pose;
    init_pose.pose.pose.position.x = 0.0;
    init_pose.pose.pose.position.y = 0.0;
    init_pose.pose.pose.position.y = 0.0;
    init_pose.pose.pose.orientation.x = 0.0;
    init_pose.pose.pose.orientation.y = 0.0;
    init_pose.pose.pose.orientation.z = 0.0;
    init_pose.pose.pose.orientation.w = 1.0;

    geometry_msgs::PoseWithCovarianceStamped::ConstPtr init_pose_ptr(new geometry_msgs::PoseWithCovarianceStamped(init_pose));
    initialPoseCallback(init_pose_ptr);
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::vector<cv::Point2f> scan_recv;
    // scan_points.clear();
    double angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
        if (msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max)
        {
            float x = msg->ranges[i] * cos(angle) / map_msg.info.resolution;
            float y = -msg->ranges[i] * sin(angle) / map_msg.info.resolution;
            scan_recv.push_back(cv::Point2f(x, y));
        }
        angle += msg->angle_increment;
    }
    scan_points = scan_recv;
    if(clear_countdown > -1)
        clear_countdown --;
    if(clear_countdown == 0)
    {
        std_srvs::Empty srv;
        clear_costmaps_client.call(srv);
    }
}

cv::Mat createGradientMask(int size)
{
    cv::Mat mask(size, size, CV_8UC1);
    int center = size / 2;
    for (int y = 0; y < size; y++)
    {
        for (int x = 0; x < size; x++)
        {
            double distance = std::hypot(x - center, y - center);
            int value = cv::saturate_cast<uchar>(255 * std::max(0.0, 1.0 - distance / center));
            mask.at<uchar>(y, x) = value;
        }
    }
    return mask;
}

void processMap()
{
    if (map_cropped.empty()) return;

    map_temp = cv::Mat::zeros(map_cropped.size(), CV_8UC1);
    cv::Mat gradient_mask = createGradientMask(101);  // 创建一个101x101的渐变掩模

    for (int y = 0; y < map_cropped.rows; y++)
    {
        for (int x = 0; x < map_cropped.cols; x++)
        {
            if (map_cropped.at<uchar>(y, x) == 100)  // 障碍物
            {
                int left = std::max(0, x - 50);
                int top = std::max(0, y - 50);
                int right = std::min(map_cropped.cols - 1, x + 50);
                int bottom = std::min(map_cropped.rows - 1, y + 50);

                cv::Rect roi(left, top, right - left + 1, bottom - top + 1);
                cv::Mat region = map_temp(roi);

                int mask_left = 50 - (x - left);
                int mask_top = 50 - (y - top);
                cv::Rect mask_roi(mask_left, mask_top, roi.width, roi.height);
                cv::Mat mask = gradient_mask(mask_roi);

                cv::max(region, mask, region);
            }
        }
    }

}

void pose_tf()
{
    if (map_cropped.empty() || map_msg.data.empty()) return;

    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);

    // 1. 计算在裁剪地图中的实际米制坐标
    double x_meters = (lidar_x + map_roi_info.x_offset) * map_msg.info.resolution;
    double y_meters = (map_cropped.rows - lidar_y + map_roi_info.y_offset) * map_msg.info.resolution;

    // 2. 考虑原始地图的原点偏移
    x_meters += map_msg.info.origin.position.x;
    y_meters += map_msg.info.origin.position.y;

    // 3. 处理yaw角度
    double yaw_ros = -lidar_yaw;

    // 4. 将弧度转换为四元数
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_ros);

    // 5. 计算 base_footprint 在 map 中的位置
    double base_x = x_meters;
    double base_y = -y_meters;

    // 6. 查询 odom 到 base_frame 的变换
    geometry_msgs::TransformStamped odom_to_base;
    try {
        odom_to_base = tfBuffer.lookupTransform("odom", laser_frame, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
    }

    // 7. 计算 map 到 odom 的变换
    tf2::Transform map_to_base, odom_to_base_tf2;
    map_to_base.setOrigin(tf2::Vector3(base_x, base_y, 0));
    map_to_base.setRotation(q);

    tf2::fromMsg(odom_to_base.transform, odom_to_base_tf2);
    tf2::Transform map_to_odom = map_to_base * odom_to_base_tf2.inverse();

    // 8. 发布 map 到 odom 的变换
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped map_to_odom_msg;

    map_to_odom_msg.header.stamp = ros::Time::now();
    map_to_odom_msg.header.frame_id = "map";
    map_to_odom_msg.child_frame_id = "odom";
    map_to_odom_msg.transform = tf2::toMsg(map_to_odom);

    // 计算 yaw 角度
    tf2::Quaternion q_tf2(
        map_to_odom_msg.transform.rotation.x,
        map_to_odom_msg.transform.rotation.y,
        map_to_odom_msg.transform.rotation.z,
        map_to_odom_msg.transform.rotation.w);
    tf2::Matrix3x3 m(q_tf2);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    br.sendTransform(map_to_odom_msg);
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "lidar_loc");

    // 读取参数
    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("base_frame", base_frame, "base_footprint");
    private_nh.param<std::string>("laser_frame", laser_frame, "laser");

    ros::NodeHandle nh;
    ros::Subscriber map_sub = nh.subscribe("/map", 1, mapCallback);
    ros::Subscriber scan_sub = nh.subscribe("/scan", 1, scanCallback);
    ros::Subscriber initial_pose_sub = nh.subscribe("/initialpose", 1, initialPoseCallback);
    clear_costmaps_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

    ros::Rate rate(100);  // 匹配频率

    while (ros::ok())
    {
        if (!map_cropped.empty())
        {
            map_match = map_temp.clone();
            // cv::cvtColor(map_match, map_match, cv::COLOR_GRAY2BGR);

            // 遍历 map_cropped，将 map_match 中对应的障碍物像素设置为紫色
            // for (int y = 0; y < map_cropped.rows; ++y)
            // {
            //     for (int x = 0; x < map_cropped.cols; ++x)
            //     {
            //         if (map_cropped.at<uchar>(y, x) == 100)  //  100 表示障碍物
            //         {
            //             map_match.at<cv::Vec3b>(y, x) = cv::Vec3b(128, 0, 128);  // 紫色 (BGR格式)
            //         }
            //     }
            // }

            // 计算三种情况下的雷达点坐标数组
            std::vector<cv::Point2f> transform_points, clockwise_points, counter_points;
            float deg_to_rad = M_PI / 180.0;
            int max_sum = 0;
            float best_dx = 0, best_dy = 0, best_dyaw = 0;

            for (const auto& point : scan_points)
            {
                // 情况一：原始角度
                float rotated_x = point.x * cos(lidar_yaw) - point.y * sin(lidar_yaw);
                float rotated_y = point.x * sin(lidar_yaw) + point.y * cos(lidar_yaw);
                transform_points.push_back(cv::Point2f(rotated_x + lidar_x, lidar_y - rotated_y));

                // 情况二：顺时针旋转1度
                float clockwise_yaw = lidar_yaw + deg_to_rad;
                rotated_x = point.x * cos(clockwise_yaw) - point.y * sin(clockwise_yaw);
                rotated_y = point.x * sin(clockwise_yaw) + point.y * cos(clockwise_yaw);
                clockwise_points.push_back(cv::Point2f(rotated_x + lidar_x, lidar_y - rotated_y));

                // 情况三：逆时针旋转1度
                float counter_yaw = lidar_yaw - deg_to_rad;
                rotated_x = point.x * cos(counter_yaw) - point.y * sin(counter_yaw);
                rotated_y = point.x * sin(counter_yaw) + point.y * cos(counter_yaw);
                counter_points.push_back(cv::Point2f(rotated_x + lidar_x, lidar_y - rotated_y));
            }

            // 计算15种变换方式的匹配值
            std::vector<cv::Point2f> offsets = {{0,0}, {1,0}, {-1,0}, {0,1}, {0,-1}};
            std::vector<std::vector<cv::Point2f>> point_sets = {transform_points, clockwise_points, counter_points};
            std::vector<float> yaw_offsets = {0, deg_to_rad, -deg_to_rad};

            for (int i = 0; i < offsets.size(); ++i)
            {
                for (int j = 0; j < point_sets.size(); ++j)
                {
                    int sum = 0;
                    for (const auto& point : point_sets[j])
                    {
                        float px = point.x + offsets[i].x;
                        float py = point.y + offsets[i].y;
                        if (px >= 0 && px < map_temp.cols && py >= 0 && py < map_temp.rows)
                        {
                            sum += map_temp.at<uchar>(py, px);
                        }
                    }
                    if (sum > max_sum)
                    {
                        max_sum = sum;
                        best_dx = offsets[i].x;
                        best_dy = offsets[i].y;
                        best_dyaw = yaw_offsets[j];
                    }
                }
            }

            // 更新雷达位置和角度
            lidar_x += best_dx;
            lidar_y += best_dy;
            lidar_yaw += best_dyaw;

            // 绘制最佳匹配的测距点
            // for (const auto& point : transform_points)
            // {
            //     float px = point.x + best_dx;
            //     float py = point.y + best_dy;
            //     if (px >= 0 && px < map_match.cols && py >= 0 && py < map_match.rows)
            //     {
            //         map_match.at<cv::Vec3b>(py, px) = cv::Vec3b(0, 0, 255);  // BGR 格式：红色
            //     }
            // }
        }

        pose_tf();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}