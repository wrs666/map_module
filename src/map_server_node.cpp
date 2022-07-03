#include <ros/time.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Header.h>
#include <std_srvs/Trigger.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>
#include <map_module/get_curve.h>
#include <unistd.h>

#include <vec_map.h>

#define path "/home/wrs/map/src/map_module/generated_path.csv"
#define osm_map_url "package://map_module/data/square_backward2.osm"


using namespace std;
using namespace Eigen;

enum path_source
{
  csv_file,
  osm_mission
};

class MapServerNode {

 public:
  MapServerNode(ros::NodeHandle nh, ros::NodeHandle pnh) {
    ROS_INFO("map server node init");
    nh_ = nh;
    map = vec_map(osm_map_url, nh_);

    get_curve_server_ = this->nh_.advertiseService("/get_curve", &MapServerNode::getCurveCallback, this);
    visual_path_pub = nh.advertise<nav_msgs::Path>("/visual_path",1);//csv路径可视化
    ROS_INFO("map_server_node preparation finished");
    marker_pub = nh.advertise<visualization_msgs::Marker>("/path_kappa_information", 10);
    visual_path_number = 0;
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
  


  geometry_msgs::Pose curve_to_pose(map_module::curvepoint point){
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

  map_module::curvepoint to_curvepoint(TrackPoint trackpoint)
  {
    map_module::curvepoint curvepoint;
    curvepoint.x = trackpoint.x_;
    curvepoint.y = trackpoint.y_;
    curvepoint.theta = trackpoint.theta_;
    curvepoint.kappa = trackpoint.curvature_;
    return curvepoint;
  }

  geometry_msgs::Pose track_to_pose(TrackPoint point){
    geometry_msgs::Pose pose;
    pose.position.x = point.x_;
    pose.position.y = point.y_;
    pose.position.z = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = sin(point.theta_/2);
    pose.orientation.w = cos(point.theta_/2);
    return pose;
  }

  map_module::curve to_curve(vector<TrackPoint> &track)
  {
    map_module::curve curve;
    map_module::curvepoint curvepoint;
    for(lint i = 0; i < track.size(); i++)
    {
      curvepoint = to_curvepoint(track[i]);
      curve.points.push_back(curvepoint);
    }
    return curve;
  }


  void vec_map_vis()
  {
    map.map_visualization_pub();
  }

  int find_curve_destination(double total_length)
  {
    int curve_size = map.mission_curve_with_length.size();
    cout<<"num of curve points : "<<curve_size<<endl;
    if(total_length >= map.mission_curve_with_length.back().length_)
      return curve_size - 1;
    else
    {
      double index = total_length / map.mission_curve_with_length.back().length_ * (curve_size - 1);
      cout<<"index of curve section destination : "<<index<<endl;
      if(ceil(index) - index < index - floor(index))
        return ceil(index);
      else
        return floor(index);
    }

  }

  map_module::curve refresh_curve(double required_length, geometry_msgs::PoseStamped current_pose, map_module::curve curve)
  {
    map_module::curve current_curve; 
    int start_index = 0;
    int n = 0;
    //todo:判断车的位置是否在地图中
    //todo：判断当前车的朝向是否可达终点
    double min_distance = 100;//random assginment
    double current_distance;
    for(auto point : curve.points)
    {
      current_distance = sqrt(pow((point.x - current_pose.pose.position.x), 2) + pow((point.y - current_pose.pose.position.y), 2)); 
      if(current_distance < min_distance)
      {
        min_distance = current_distance;
        start_index = n;
      }
      n++;
    }

    cout<<"min_distance : "<<min_distance<<endl;

    double past_length = map.mission_curve_with_length[start_index].length_;
    cout<<"start_index : "<<start_index<<endl;
    cout<<"start length : "<< past_length <<endl;
    double total_length = required_length + past_length;
    int end_index = find_curve_destination(total_length);

    cout<<"end_index : "<<end_index<<endl;


    vector<map_module::curvepoint>::iterator iter;
    auto begin_iter = curve.points.begin() + start_index;
    auto end_iter = curve.points.begin() +  end_index + 1;
    
    current_curve.points.assign(begin_iter, end_iter);
    return current_curve;
  }

 private:

  bool getCurveCallback(map_module::get_curve::Request& request, map_module::get_curve::Response& response) {

    nav_msgs::Path visual_path;
    visual_path.header.seq = 1;
    visual_path.header.stamp = ros::Time::now();
    visual_path.header.frame_id = "world";

    curve_origin = osm_mission;
    visual_path.poses.resize(0);

    map_module::curve global_curve;
    map_module::curve curve;
    if (curve_origin == csv_file)
    {
      curve = csv_to_curve();

      //填充goal_pose
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
      response.status = map_module::get_curveResponse::SUCCEED;
    }
    else if(curve_origin == osm_mission)
    {
      global_curve = to_curve(map.mission_curve);
      curve = refresh_curve(request.request_length, request.current_pose, global_curve);

      geometry_msgs::PoseStamped goal_pose;
      goal_pose.header.frame_id = "world";
      goal_pose.header.stamp = ros::Time::now();
      goal_pose.header.seq = 1;

      goal_pose.pose = track_to_pose(map.goal_pose);
      response.goal_pose = goal_pose;
      response.status = map_module::get_curveResponse::SUCCEED;

    }
    else
    {
      ROS_WARN("curve_origin was set wrong");
      response.status = map_module::get_curveResponse::GOAL_UNREACHABLE;
      return false;
    }
    
    curve.points.pop_back();// ????????????bug
    
    curve.header.seq = 1;
    curve.header.stamp = ros::Time::now();
    curve.header.frame_id = "world";
    
    map_module::locallane lane;
    lane.geometry = curve;
    response.center_lane = lane;
    response.left_lane_exist = false;
    response.right_lane_exist = false;
    
    //to extend
    response.current_pose_state = map_module::get_curveResponse::NORMAL;

    //在rviz中进行路径的可视化
    double curv_comb_scale = 10;//曲率梳半径放大倍数
    std_msgs::Header header;
    geometry_msgs::PoseStamped posestamped;
    geometry_msgs::Point p;
    header.frame_id = "world";


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
    
    int curve_size = curve.points.size();
    //bool first_flag = true;
    for (int i = 1; i < curve_size + 1; i++) {
      //cout<<"curve.points size : "<<curve_size<<endl;
        //static_curve_srv.request.static_curve.points.push_back(to_curvepoint(point));
      map_module::curvepoint point_precursor = curve.points[i - 1];
      map_module::curvepoint point = curve.points[i];
      map_module::curvepoint point_successor = curve.points[i + 1];
      
      posestamped.header = header;
      posestamped.pose = curve_to_pose(point);
      visual_path.poses.push_back(posestamped);      
      
      double road_theta;
      if(point.theta > PI/2)
        road_theta = point.theta - PI;
      else if(point.theta < -PI/2)
        road_theta = point.theta + PI;
      else
        road_theta = point.theta;

      int k1 = 1;
      if(road_theta < 0)
        k1 = -1;
      
      int k2 = 0;
      Point p1(point_precursor.x, point_precursor.y);
      Point p2(point_successor.x, point_successor.y);
      Line chord_line(p1, p2);
      double ref_y = chord_line.a * point.x + chord_line.b;
      if(ref_y > point.y)
        k2 = -1;
      else if(ref_y < point.y)
        k2 = 1;

        //std::cout<<to_curvepoint(point)<<std::endl;

      p.x = point.x;
      p.y = point.y;
      p.z = 0;
      line_list.points.push_back(p);
      
      //计算曲率疏的端点（另一端是路径点）
      radius_theta = k1 * abs(road_theta) + k2 * PI / 2;
      p.x = point.x + curv_comb_scale * fabs(point.kappa) * cos(radius_theta);
      p.y = point.y + curv_comb_scale * fabs(point.kappa) * sin(radius_theta);
      p.z = 0; 
      line_strip.points.push_back(p);
      line_list.points.push_back(p);
      last_heading = point.theta;
    }
    //visual_path.poses.pop_back();//????????????????????最后一个神奇元素哪来的？？？？？？？？？？？？？？？？？
    visual_path_pub.publish(visual_path);
    marker_pub.publish(line_strip);
    marker_pub.publish(line_list);
    
    return true;
  }

  path_source curve_origin;
  int visual_path_number;
  //std::vector<TrackPoint> curve_data;
  ros::NodeHandle nh_;
  ros::ServiceServer get_curve_server_;
  ros::Publisher visual_path_pub;
  ros::Publisher marker_pub;//路径曲率信息可视化的publisher
  vec_map map;
};



int main(int argc, char** argv) {
  ros::init(argc, argv, "map_server_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  MapServerNode map_server_node(nh, pnh);
  
  ros::Rate loop_rate(1);
    
  while(ros::ok())
  {
    map_server_node.vec_map_vis();
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}