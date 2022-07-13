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
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>
#include <fstream>
#include <sstream> 
#include <cmath>

using namespace std;

#define path "/home/wrs/map/src/map_module/data/mission_curve.csv"

class navigation_test_node
{
  public:
    void setinitialposecallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
    {
      ROS_INFO("Set initial pose.");
      get_curve_srv.request.current_pose.header = msg.header;
      get_curve_srv.request.current_pose.pose = msg.pose.pose;
      get_curve_srv.request.request_length = 100;
      get_curve_srv.request.point_margin = 0.1;
      if(get_curve_client.call(get_curve_srv))
      {
        std::cout<<"points : "<<get_curve_srv.response.center_lane.geometry.points.front().x<<std::endl;
        if(get_curve_srv.response.goal_exist == true)
          std::cout<<"goal_exist : true"<<std::endl;
        else
          std::cout<<"goal_exist : false"<<std::endl;
        }
      else
      {
        ROS_ERROR("service /get_curve called failed!");
      }
    }
    
    navigation_test_node(ros::NodeHandle nh) 
    {
      nh_ = nh;
      initial_pose_sub = nh_.subscribe("/initialpose", 1, &navigation_test_node::setinitialposecallback, this);
     
      get_curve_client = nh.serviceClient<map_module::get_curve>("/get_curve");
    }

  private:

    ros::NodeHandle nh_;
    ros::Subscriber initial_pose_sub;
    geometry_msgs::PoseStamped initial_pose;
    ros::ServiceClient get_curve_client;
    map_module::get_curve get_curve_srv;
 
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_test_node");
    ros::NodeHandle nh;

    navigation_test_node test(nh);
    ros::spin();
}




