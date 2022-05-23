#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Header.h>
#include <std_srvs/Trigger.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Trigger.h>
#include <map_module/curve.h>
#include <map_module/curvepoint.h>
#include <map_module/path_planning.h>
#include <fstream>
#include <unistd.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sstream>
#include <cmath>

#define path "/home/wrs/map/src/map_module/path3.csv"

class TrackPoint {
  private:
    float x_;
    float y_;
    float theta_;
    float curvature_;
  
  public:
    TrackPoint() {}
    TrackPoint(float x, float y, float t, float c) {
      x_ = x;
      y_ = y;
      theta_ = t;
      curvature_ = c;
    }

    void setStation(float x)
    {
      x_ = x;
    }
    void setOffset(float y)
    {
      y_ = y;
    }
    void setHeading(float t)
    {
      theta_ = t;
    }
    void setCurvature(float c)
    {
       curvature_ = c;
    }

    float getStation()
    {
      return x_;
    }
    float getOffset()
    {
      return y_;
    }
    float getHeading()
    {
      return theta_;
    }
    float getCurvature()
    {
      return curvature_;
    }
};

class StaticCurveNode {

 public:
  StaticCurveNode(ros::NodeHandle nh, ros::NodeHandle pnh) {
    ROS_INFO("init");

    char buffer[256];
    char *val = getcwd(buffer, sizeof(buffer));
    if (val) {
        std::cout << buffer << std::endl;
    }

    nh_ = nh;

    start_static_curve_server_ = this->nh_.advertiseService("/start_static_curve", &StaticCurveNode::startStaticCurveCallback, this);
    path_planning_client_ = nh.serviceClient<map_module::path_planning>("/PLANNING_SERVICE_name");
    visual_path_pub = nh.advertise<nav_msgs::Path>("/visual_path",1);
    markerArrayPub = nh_.advertise<visualization_msgs::MarkerArray>("/markerArray", 10);
    ROS_INFO("Static_Curve_Node preparation finished");
  }

  std::vector<TrackPoint> csv_to_curve()
  {
    std::vector<TrackPoint> path_data;
    path_data.resize(0);
    std::ifstream inFile(path, std::ios::in);
    if (!inFile)
    {
      std::cout << "打开文件失败！" << std::endl;
      exit(1);
    }
    int number=0;
    std::string line;
    //std::string field;
    while (getline(inFile, line))//getline(inFile, line)表示按行读取CSV文件中的数据
    {
      std::string field;
      std::istringstream sin(line); //将整行字符串line读入到字符串流sin中
    
      TrackPoint element;

      getline(sin, field, ','); //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
      element.setStation((double)atof(field.c_str()));//将刚刚读取的字符串转换成int

      getline(sin, field, ',');
      element.setOffset((double)atof(field.c_str()));

      getline(sin, field, ','); 
      element.setHeading((double)atof(field.c_str()));

      getline(sin, field, ','); 
      element.setCurvature((double)atof(field.c_str()));
      number++;
      if(number == 1)
        continue;

      path_data.push_back(element);
      
    }
    inFile.close();
    std::cout << "共读取了：" << number << "行" << std::endl;
    std::cout << "读取数据完成" << std::endl;
    return path_data;
  }

  map_module::curvepoint to_curvepoint(TrackPoint& anchor_point) {
    map_module::curvepoint point;
    point.x = anchor_point.getStation();
    point.y = anchor_point.getOffset();
    point.theta = anchor_point.getHeading();
    point.kappa = anchor_point.getCurvature();
    return point;
  }

  geometry_msgs::Pose to_pose(TrackPoint& anchor_point){
    geometry_msgs::Pose pose;
    pose.position.x = anchor_point.getStation();
    pose.position.y = anchor_point.getOffset();
    pose.position.z = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = sin(anchor_point.getHeading()/2);
    pose.orientation.w = cos(anchor_point.getHeading()/2);
    return pose;
  }

  // void publish()
  // {
  //   ros::Rate rate(10);
  //   while(ros::ok())
  //   {
  //     visual_path_pub.publish(visual_path);
  //   }
  // }

 private:

  bool startStaticCurveCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
    
    // 找到对应的路径
    const std::vector<TrackPoint>* selected_curve = nullptr;
    curve_data = csv_to_curve();
    selected_curve = &curve_data;

    if (selected_curve == nullptr || selected_curve->size() == 0) {
      ROS_INFO("Invalid curve data.");
    } 
    else {
      ROS_INFO("run static curve service");
      map_module::path_planning static_curve_srv;
      std_msgs::Header header;
      geometry_msgs::PoseStamped posestamped;
      header.frame_id = "world";
      //visual_path.header.stamp = ;
      visual_path.header = header;
      static_curve_srv.request.mode = map_module::path_planningRequest::STATICCURVE;
      int k=0;
      for (auto point : *selected_curve) {
        static_curve_srv.request.static_curve.points.push_back(to_curvepoint(point));
        posestamped.header = header;
        posestamped.pose = to_pose(point);
        visual_path.poses.push_back(posestamped);
        //std::cout<<to_curvepoint(point)<<std::endl;
        visualization_msgs::Marker marker;
        marker.header.frame_id="world";
        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id =k;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.scale.z = 0.1;
        marker.color.b = 1.0f;
        marker.color.g = 1.0f;
        marker.color.r = 1.0f;
        marker.color.a = 1;
        geometry_msgs::Pose pose;
        pose.position.x = point.getStation();
        pose.position.y = point.getOffset();
        pose.position.z =0;
        std::ostringstream str;
        str<<round(point.getCurvature()*10000)/10000;
        marker.text=str.str();
        marker.pose=pose;
        markerArray.markers.push_back(marker);
        markerArrayPub.publish(markerArray);
        k++;
      }
      visual_path_pub.publish(visual_path);
      if (!path_planning_client_.call(static_curve_srv)) {
        ROS_ERROR("PATH_PLANNING_service called failed");
      } else {
        ROS_INFO("PATH_PLANNING_service succeed");
      }
    }
    response.success = true;
    return true;
  }

  nav_msgs::Path visual_path;
  visualization_msgs::MarkerArray markerArray;
  std::vector<TrackPoint> curve_data;
  ros::NodeHandle nh_;
  ros::ServiceServer start_static_curve_server_;
  ros::ServiceClient path_planning_client_;
  ros::Publisher visual_path_pub;
  ros::Publisher markerArrayPub;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "static_curve_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  StaticCurveNode static_curve_node(nh, pnh);
  ros::spin();

  return 0;
}
