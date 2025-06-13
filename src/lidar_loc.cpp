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
sensor_msgs::RegionOfInterest map_roi_info;
std::vector<cv::Point2f> scan_points;
std::vector<cv::Point2f> best_transform;
ros::ServiceClient clear_costmaps_client;
std::string base_frame;
std::string odom_frame;
std::string laser_frame;
std::string laser_topic;

float lidar_x = 250, lidar_y = 250, lidar_yaw = 0;
float deg_to_rad = M_PI / 180.0;
int cur_sum = 0;
int clear_countdown = -1;
int scan_count = 0;


// 初始姿态回调函数
void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    // 1. 直接从输入消息中提取 map 坐标系下的位置和方向信息
    double map_x = msg->pose.pose.position.x;
    double map_y = msg->pose.pose.position.y;
    tf2::Quaternion q;
    // 使用 tf2::fromMsg 将 geometry_msgs::Quaternion 转换为 tf2::Quaternion
    tf2::fromMsg(msg->pose.pose.orientation, q);

    // 2. 将四元数转换为 yaw 角度 (相对于 map 坐标系)
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // 3. 检查 map_msg 是否有效
    if (map_msg.info.resolution <= 0) {
        ROS_ERROR("地图信息无效或未接收");
        return;
    }

    // 4. 将地图坐标转换为裁切后的地图栅格坐标
    lidar_x = (map_x - map_msg.info.origin.position.x) / map_msg.info.resolution - map_roi_info.x_offset;
    lidar_y = (map_y - map_msg.info.origin.position.y) / map_msg.info.resolution - map_roi_info.y_offset;

    // 5. 设置 yaw 角度
    lidar_yaw = -yaw;

    // 6. 设置倒计时
    clear_countdown = 30;
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
            if(map_msg.data[index] == 100)
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

    if(new_origin_x < 0) new_origin_x = 0;
    if((new_origin_x + new_width) > info.width) new_width = info.width - new_origin_x;
    if(new_origin_y < 0) new_origin_y = 0;
    if((new_origin_y + new_height) > info.height) new_height = info.height - new_origin_y;

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

static bool lidar_is_inverted = false;
bool check(float x, float y, float yaw);
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    scan_points.clear();
    double angle = msg->angle_min;

    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);

    // 从激光雷达到底盘的转换
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform(base_frame, laser_frame,ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
    }

    // 检测雷达是否倒装
    tf2::Quaternion q_lidar;
    tf2::fromMsg(transformStamped.transform.rotation, q_lidar);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_lidar).getRPY(roll, pitch, yaw);
    
    const double tolerance = 0.1;
    bool lidar_is_inverted = std::abs(std::abs(roll) - M_PI) < tolerance;
    lidar_is_inverted *= !(std::abs(std::abs(pitch) - M_PI) < tolerance);

    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
        if (msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max)
        {
            // 1. 首先在激光雷达坐标系下计算点的坐标
            float x_laser = msg->ranges[i] * cos(angle);
            float y_laser = -msg->ranges[i] * sin(angle);

            // 2. 创建要转换的点
            geometry_msgs::PointStamped point_laser;
            point_laser.header.frame_id = laser_frame;
            point_laser.header.stamp = msg->header.stamp;
            point_laser.point.x = x_laser;
            point_laser.point.y = y_laser;
            point_laser.point.z = 0.0;

            // 3. 转换到底盘坐标系
            geometry_msgs::PointStamped point_base;
            tf2::doTransform(point_laser, point_base, transformStamped);

            // 4. 转换为栅格地图坐标并存储
            float x = point_base.point.x / map_msg.info.resolution;
            float y = point_base.point.y / map_msg.info.resolution;
            if (lidar_is_inverted)
            {
                // 如果雷达倒装，x和y需要取反
                x = -x;
                y = -y;
            }

            scan_points.push_back(cv::Point2f(x, y));
        }
        angle += msg->angle_increment;
    }
    if(scan_count == 0)
        scan_count ++;

    while (ros::ok())
    {
        if (!map_cropped.empty())
        {
            // 计算三种情况下的雷达点坐标数组
            std::vector<cv::Point2f> transform_points, clockwise_points, counter_points;
            
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

            // 判断匹配循环是否可以终止
            if(check(lidar_x , lidar_y , lidar_yaw))
            {
                break;
            }
        }
        else
        {
            break;
        }
    }

    if(clear_countdown > -1)
        clear_countdown --;
    if(clear_countdown == 0)
    {
        std_srvs::Empty srv;
        clear_costmaps_client.call(srv);
    }
}

std::deque<std::tuple<float, float, float>> data_queue;
const size_t max_size = 10;
bool check(float x, float y, float yaw)
{
    if(x == 0 && y == 0 && yaw == 0)
    {
        data_queue.clear();
        return true;
    }

    // 添加新数据
    data_queue.push_back(std::make_tuple(x, y, yaw));

    // 如果队列超过最大大小，移除最旧的数据
    if (data_queue.size() > max_size) 
    {
        data_queue.pop_front();
    }

    // 如果队列已满，检查第一个和最后一个元素
    if (data_queue.size() == max_size) 
    {
        auto& first = data_queue.front();
        auto& last = data_queue.back();

        float dx = std::abs(std::get<0>(last) - std::get<0>(first));
        float dy = std::abs(std::get<1>(last) - std::get<1>(first));
        float dyaw = std::abs(std::get<2>(last) - std::get<2>(first));

        // 如果所有差值的绝对值都小于5，清空队列退出循环
        if (dx < 5 && dy < 5 && dyaw < 5*deg_to_rad)
        {
            data_queue.clear();
            return true;
        }
    }
    return false;
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
    // 如果没有收到过雷达数据或地图，则不进行任何操作
    if (scan_count == 0) return;
    if (map_cropped.empty() || map_msg.data.empty() || map_msg.info.resolution <= 0) return;

    // TF listener只需要一个静态实例
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);

    // ---------------------------------------------------------------------------------
    // 步骤 1: 将机器人位姿从【裁切地图的像素坐标】转换为【完整地图的米制坐标】
    // ---------------------------------------------------------------------------------

    // 1.1 首先，将裁切地图中的像素坐标(lidar_x, lidar_y)转换成完整地图中的像素坐标
    double full_map_pixel_x = lidar_x + map_roi_info.x_offset;
    double full_map_pixel_y = lidar_y + map_roi_info.y_offset;

    // 1.2 然后，将完整地图的像素坐标转换为 'map' 坐标系下的米制坐标
    double x_in_map_frame = full_map_pixel_x * map_msg.info.resolution + map_msg.info.origin.position.x;
    double y_in_map_frame = full_map_pixel_y * map_msg.info.resolution + map_msg.info.origin.position.y;

    // 1.3 处理yaw角度。这里的-号是为了补偿匹配过程中的坐标系定义
    double yaw_in_map_frame = -lidar_yaw;

    // ---------------------------------------------------------------------------------
    // 步骤 2: 构建从 'map' 到 'base_frame' 的变换
    // 这个变换描述了 base_frame 在 map 坐标系中的位置和姿态
    // ---------------------------------------------------------------------------------
    tf2::Transform map_to_base;
    map_to_base.setOrigin(tf2::Vector3(x_in_map_frame, y_in_map_frame, 0.0));
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_in_map_frame);
    map_to_base.setRotation(q);

    // ---------------------------------------------------------------------------------
    // 步骤 3: 查询从 'odom' 到 'base_frame' 的变换
    // 这是由轮式里程计或其它里程计源发布的
    // ---------------------------------------------------------------------------------
    geometry_msgs::TransformStamped odom_to_base_msg;
    try {
        // 使用ros::Time(0)来获取最新的可用变换
        odom_to_base_msg = tfBuffer.lookupTransform(odom_frame, base_frame, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("无法获取从 '%s' 到 '%s' 的变换: %s", odom_frame.c_str(), base_frame.c_str(), ex.what());
        return;
    }

    // ---------------------------------------------------------------------------------
    // 步骤 4: 计算从 'map' 到 'odom' 的变换
    // 这是定位节点的核心任务，它提供了对里程计漂移的修正
    // TF关系: T_map_odom * T_odom_base = T_map_base
    // 因此: T_map_odom = T_map_base * (T_odom_base)^-1
    // ---------------------------------------------------------------------------------
    tf2::Transform odom_to_base_tf2;
    tf2::fromMsg(odom_to_base_msg.transform, odom_to_base_tf2);
    tf2::Transform map_to_odom = map_to_base * odom_to_base_tf2.inverse();

    // ---------------------------------------------------------------------------------
    // 步骤 5: 发布 'map' -> 'odom' 的变换
    // ---------------------------------------------------------------------------------
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped map_to_odom_msg;

    map_to_odom_msg.header.stamp = ros::Time::now(); // 使用当前时间
    map_to_odom_msg.header.frame_id = "map";
    map_to_odom_msg.child_frame_id = odom_frame;
    map_to_odom_msg.transform = tf2::toMsg(map_to_odom);

    br.sendTransform(map_to_odom_msg);
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "lidar_loc");

    // 读取参数
    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("base_frame", base_frame, "base_footprint");
    private_nh.param<std::string>("odom_frame", odom_frame, "odom");
    private_nh.param<std::string>("laser_frame", laser_frame, "laser");
    private_nh.param<std::string>("laser_topic", laser_topic, "scan");

    ros::NodeHandle nh;
    ros::Subscriber map_sub = nh.subscribe("map", 1, mapCallback);
    ros::Subscriber scan_sub = nh.subscribe(laser_topic, 1, scanCallback);
    ros::Subscriber initial_pose_sub = nh.subscribe("initialpose", 1, initialPoseCallback);
    clear_costmaps_client = nh.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");

    ros::Rate rate(30);  // tf发送频率

    while (ros::ok())
    {
        pose_tf();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}