#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Header.h>
#include <std_srvs/Trigger.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Trigger.h>
#include <map_module/curve.h>
#include <map_module/curvepoint.h>
#include <map_module/path_planning.h>
#include <map_module/get_curve.h>
#include <fstream>
#include <unistd.h>
#include <cmath>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

#define path "/home/wrs/map/src/map_module/recordingpath_final.csv"
#define PI 3.1415926

using namespace std;
using namespace Eigen;

class MapServerNode {

 public:
  MapServerNode(ros::NodeHandle nh, ros::NodeHandle pnh) {
    ROS_INFO("map server node init");
    nh_ = nh;

    get_curve_server_ = this->nh_.advertiseService("/get_curve", &MapServerNode::getCurveCallback, this);
    visual_path_pub = nh.advertise<nav_msgs::Path>("/visual_path",1);
    ROS_INFO("map_server_node preparation finished");
    marker_pub = nh.advertise<visualization_msgs::Marker>("/path_kappa_information", 10);
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

  map_module::curve Section_Interploration(vector<map_module::curvepoint> points)
  {
    map_module::curve processed_section;
    processed_section.points.push_back(points[0]);
    double x1, y1, theta1, kappa1, x2, y2, theta2, kappa2;
    x1 = points[0].x;
    y1 = points[0].y;
    theta1 = points[0].theta;
    kappa1 = points[0].kappa;
    x2 = points[1].x;
    y2 = points[1].y;
    theta2 = points[1].theta;
    kappa2 = points[1].kappa;

    //solve AX=B
    MatrixXd A(6,6);
    VectorXd X(6);
    VectorXd B(6);

    A << 1, x1, pow(x1, 2), pow(x1, 3), pow(x1, 4), pow(x1, 5),
         1, x2, pow(x2, 2), pow(x2, 3), pow(x2, 4), pow(x2, 5),
         0, 1, 2 * x1, 3 * pow(x1, 2), 4 * pow(x1, 3), 5 * pow(x1, 4),
         0, 1, 2 * x2, 3 * pow(x2, 2), 4 * pow(x2, 3), 5 * pow(x2, 4),
         0, 0, 2, 6 * x1, 12 * pow(x1, 2), 20 * pow(x1, 3),
         0, 0, 2, 6 * x2, 12 * pow(x2, 2), 20 * pow(x2, 3);
    B << y1, y2, tan(theta1), tan(theta2), kappa1 * pow((1 + pow(tan(theta1), 2)), 1.5), kappa2 * pow((1 + pow(tan(theta2), 2)), 1.5);
    X = A.colPivHouseholderQr().solve(B);
  
    double x = x1;
    map_module:: curvepoint p;
    VectorXd x_power(6);
    VectorXd coef_0d(6);
    VectorXd coef_1d(6);
    VectorXd coef_2d(6);
    while(x < x2)
    {
      x_power << 1, x, pow(x, 2), pow(x, 3), pow(x, 4), pow(x, 5);
      coef_0d << X(0), X(1), X(2), X(3), X(4), X(5);
      coef_1d << X(1), 2 * X(2), 3 * X(3), 4 * X(4), 5 * X(5), 0;
      coef_2d << 2 * X(2), 6 * X(3), 12 * X(4), 20 * X(5), 0, 0;
      p.x = x;
      p.y = x_power.dot(coef_0d);
      p.theta = atan(x_power.dot(coef_1d));
      p.kappa = x_power.dot(coef_2d)/pow((1+pow(x_power.dot(coef_1d), 2)), 1.5);
      processed_section.points.push_back(p);
      x = x + 0.1;
    }

    return processed_section;
  } 

  map_module::curve Interploration()
  {
    map_module::curve full_path_data;
    map_module::curve path_data;
    path_data = csv_to_curve();
    int section_number = sizeof(path_data.points);
    vector<map_module::curvepoint> section_point(2);
    int n1;
    //int n2;
    for (int intervel_order = 1; intervel_order < section_number; intervel_order++)
    {
      section_point[0] = path_data.points[intervel_order - 1];
      section_point[1] = path_data.points[intervel_order];
      map_module::curve section_path_data = Section_Interploration(section_point);
      n1 = section_path_data.points.size();
      //n2 = int(n1);
      for(int i = 0; i < n1; i++)
        full_path_data.points.push_back(section_path_data.points.at(i));
    
    }

    
    return full_path_data;
  
  }

 private:

  bool getCurveCallback(map_module::get_curve::Request& request, map_module::get_curve::Response& response) {

    map_module::curve curve;
    //curve = csv_to_curve();
    curve = Interploration();
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

    //在rviz中进行路径的可视化
    double curv_comb_scale = 10;
    std_msgs::Header header;
    geometry_msgs::PoseStamped posestamped;
    geometry_msgs::Point p;
    header.frame_id = "world";
    visual_path.header = header;

    visualization_msgs::Marker line_strip, line_list;
    line_strip.header.frame_id = line_list.header.frame_id = "world";
    line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    line_strip.ns = line_list.ns = "points_and_lines";
    line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

    line_strip.id = 0;
    line_list.id = 1;

    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    line_strip.scale.x = 0.03;
    line_list.scale.x = 0.03;

        // Line strip 是蓝色
    line_strip.color.b = 1.0f;
    line_strip.color.a = 1.0;

    // Line list 为红色
    line_list.color.r = 1.0f;
    line_list.color.a = 1.0;

    double last_heading = 0;
    double radius_theta = 0;

    for (auto point : curve.points) {
        //static_curve_srv.request.static_curve.points.push_back(to_curvepoint(point));
      posestamped.header = header;
      posestamped.pose = to_pose(point);
      visual_path.poses.push_back(posestamped);
        //std::cout<<to_curvepoint(point)<<std::endl;

      p.x = point.x;
      p.y = point.y;
      p.z = 0;
      line_list.points.push_back(p);
      
      //计算曲率疏的端点（另一端是路径点）
      if(point.theta < last_heading)
        radius_theta = PI/2 + point.theta;
      else 
        radius_theta = point.theta - PI/2;
      p.x = point.x + curv_comb_scale * fabs(point.kappa) * cos(radius_theta);
      p.y = point.y + curv_comb_scale * fabs(point.kappa) * sin(radius_theta);
      p.z = 0; 
      line_strip.points.push_back(p);
      line_list.points.push_back(p);
      last_heading = point.theta;
    }
    visual_path_pub.publish(visual_path);
    marker_pub.publish(line_strip);
    marker_pub.publish(line_list);
    
    return true;
  }

  nav_msgs::Path visual_path;
  //std::vector<TrackPoint> curve_data;
  ros::NodeHandle nh_;
  ros::ServiceServer get_curve_server_;
  ros::Publisher visual_path_pub;
  ros::Publisher marker_pub;//路径曲率信息可视化的publisher
};



int main(int argc, char** argv) {
  ros::init(argc, argv, "map_server_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  MapServerNode map_server_node(nh, pnh);
  ros::spin();

  return 0;
}