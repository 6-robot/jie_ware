#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>

class CostmapClearer
{
public:
    CostmapClearer() : nh_("~")
    {
        setlocale(LC_ALL,"");
        initial_pose_sub_ = nh_.subscribe("/initialpose", 1, &CostmapClearer::initialPoseCallback, this);
        clear_costmaps_client_ = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

    }

    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        std_srvs::Empty srv;
        if (clear_costmaps_client_.call(srv))
        {
            ROS_INFO("成功清除代价地图！");
        }
        else
        {
            ROS_ERROR("代价地图清除失败.");
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber initial_pose_sub_;
    ros::ServiceClient clear_costmaps_client_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "costmap_cleaner");
    CostmapClearer clearer;
    ros::spin();
    return 0;
}