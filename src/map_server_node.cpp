#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Header.h>
#include <std_srvs/Trigger.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>
#include <map_module/curve.h>
#include <map_module/curvepoint.h>
#include <map_module/path_planning.h>
#include <map_module/get_curve.h>
#include <fstream>
#include <unistd.h>

#define path "/home/wrs/map/src/map_module/recordingpath_filtered.csv"

class MapServerNode {

 public:
  MapServerNode(ros::NodeHandle nh, ros::NodeHandle pnh) {
    ROS_INFO("map server node init");
    nh_ = nh;

    get_curve_server_ = this->nh_.advertiseService("/get_curve", &MapServerNode::getCurveCallback, this);
    visual_path_pub = nh.advertise<nav_msgs::Path>("/visual_path",1);
    ROS_INFO("map_server_node preparation finished");
  }

  map_module::curve csv_to_curve(){

    map_module::curve curve;

    std::ifstream inFile(path, std::ios::in);
    if (!inFile)
    {
      std::cout << "打开文件失败！" << std::endl;
      exit(1);
    }

    int number=0;
    std::string line;
    map_module::curvepoint curve_point;
    //std::string field;
    while (getline(inFile, line))//getline(inFile, line)表示按行读取CSV文件中的数据
    {      
      number++;
      if(number == 1)
        continue;
      std::string field;
      std::istringstream sin(line); //将整行字符串line读入到字符串流sin中
    
      //TrackPoint element;

      getline(sin, field, ','); //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
      curve_point.x = (double)atof(field.c_str());//将刚刚读取的字符串转换成int

      getline(sin, field, ',');
      curve_point.y = (double)atof(field.c_str());

      getline(sin, field, ','); 
      curve_point.theta = (double)atof(field.c_str());

      getline(sin, field, ',');
      curve_point.kappa = (double)atof(field.c_str());

      curve.points.push_back(curve_point);
    }
    inFile.close();
    std::cout << "read csv : " << (number - 1) << "  lines." << std::endl;
    std::cout << "Finished csv reading." << std::endl;
    return curve;
  }


  geometry_msgs::Pose to_pose(map_module::curvepoint point){
    geometry_msgs::Pose pose;
    pose.position.x = point.x;
    pose.position.y = point.y;
    pose.position.z = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = sin(point.theta/2);
    pose.orientation.w = cos(point.theta/2);
    return pose;
  }

 private:

  bool getCurveCallback(map_module::get_curve::Request& request, map_module::get_curve::Response& response) {

    map_module::curve curve;
    curve = csv_to_curve();
    curve.header.seq = 1;
    curve.header.stamp = ros::Time::now();
    curve.header.frame_id = "world";
    
    map_module::locallane lane;
    lane.geometry = curve;
    response.center_lane = lane;
    response.left_lane_exist = false;
    response.right_lane_exist = false;
    
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "world";
    goal_pose.header.stamp = ros::Time::now();
    goal_pose.header.seq = 1;
    goal_pose.pose.position.x = curve.points.back().x;
    goal_pose.pose.position.y = curve.points.back().y;
    goal_pose.pose.position.z = 0;
    goal_pose.pose.orientation.x = 0;
    goal_pose.pose.orientation.y = 0;
    goal_pose.pose.orientation.z = sin(curve.points.back().theta/2);
    goal_pose.pose.orientation.w = cos(curve.points.back().theta/2);
    response.goal_pose = goal_pose;

    response.current_pose_state = map_module::get_curveResponse::NORMAL;
    response.status = map_module::get_curveResponse::SUCCEED;

    std_msgs::Header header;
    geometry_msgs::PoseStamped posestamped;
    header.frame_id = "world";
      //visual_path.header.stamp = ;
    visual_path.header = header;
    //static_curve_srv.request.mode = map_module::path_planningRequest::STATICCURVE;
    for (auto point : curve.points) {
        //static_curve_srv.request.static_curve.points.push_back(to_curvepoint(point));
      posestamped.header = header;
      posestamped.pose = to_pose(point);
      visual_path.poses.push_back(posestamped);
        //std::cout<<to_curvepoint(point)<<std::endl;
    }
    visual_path_pub.publish(visual_path);
    return true;
  }

  nav_msgs::Path visual_path;
  //std::vector<TrackPoint> curve_data;
  ros::NodeHandle nh_;
  ros::ServiceServer get_curve_server_;
  ros::Publisher visual_path_pub;
};



int main(int argc, char** argv) {
  ros::init(argc, argv, "map_server_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  MapServerNode map_server_node(nh, pnh);
  ros::spin();

  return 0;
}