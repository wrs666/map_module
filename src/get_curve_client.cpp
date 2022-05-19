#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <std_srvs/Trigger.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <map_module/curve.h>
#include <map_module/curvepoint.h>
#include <map_module/path_planning.h>
#include <map_module/get_curve.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "get_curve_client_node");
    ros::NodeHandle nh;
    ros::ServiceClient get_curve_client = nh.serviceClient<map_module::get_curve>("/get_curve");
    map_module::get_curve get_curve_srv;
    if(get_curve_client.call(get_curve_srv))
    {
        std::cout<<"points : "<<get_curve_srv.response.center_lane.geometry.points.front().x<<std::endl;
    }
    else
    {
        ROS_ERROR("service /get_curve called failed!") ;
        return 0;
    }
    
    ros::spin();

}




