#ifndef __VEC_MAP_H__
#define __VEC_MAP_H__

#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <geographic_msgs/GetGeographicMap.h>
#include <cmath>
#include <geodesy/utm.h>
#include <sstream>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <nav_msgs/Path.h>
#include "./geometry.h"
#include <fstream>
#include <sstream> 

using namespace Eigen;

#define PI 3.1415926

typedef long unsigned int lint;

//重载运算符，以拼接vector
template<typename T>
vector<T> operator+ (vector<T> a, vector<T> b)
{
  a.insert(a.end(), b.begin(), b.end());
  return a;
}

// enum lane_type
// {
//   line,
//   spline
// };

// struct curve_segment
// {
//   lane_type lanetype;
//   double length;
//   int start_index;
//   int end_index;
// };

template <typename T>
bool in_range(T value, T border1, T border2)
{
  if(border1  > border2)
    return (border1 >= value && border2 <= value);
  else
    return (border2 >= value && border1 <= value);
}

template <typename T>
int sign(T value)
{
  if(value > 0)
    return 1;
  else if(value < 0)
    return -1;
  else
    return 0;
}


class props_find_key
{
  public:
    props_find_key(string s): key(s) {}
    bool operator () (const prop &keyvalue) const
    {
      return(keyvalue.key == key);
    }

    string key;

};


class Road
{
  public:    
    UUID id_;
    string divider_;
    string highway_;
    int lanes_backward_;
    int lanes_forward_;
    vector<double> width_backward_;
    vector<double> width_forward_;
    //x升序排列
    vector<Line> geometry_list;
    vector<osm_point> junction_points;
    vector<osm_point> points;
    bool junction_path ;
    osm_point junction;
    static int marker_id;
    ros::NodeHandle nh_;
    ros::Publisher road_pub;

    Road() {}
    Road(vector<osm_point> point, keyvalue_arr props, UUID id);
    void road_add_vis_info(visualization_msgs::MarkerArray &map);

    bool operator== (const Road &b)
    {
      //Road类添加ID；以ID相同为判断标准
      return (this -> id_ == b.id_);
    }
    double get_mission_point_offset(int lane_sequence)
    {
      double offset;
      if(lane_sequence > 0)
      {
        offset = 0.075;
        for(int i = 0; i < lane_sequence; i++)
        {
          offset = offset + width_forward_[i];
        }
        offset = offset - width_forward_[lane_sequence - 1] / 2;
      }

      else if(lane_sequence < 0)
      {
        offset = -0.075;
        for(int i = 0; i > lane_sequence; i--)
        {
          offset = offset - width_backward_[-i];
        }
        offset = offset + width_backward_[-lane_sequence - 1] / 2;
      }

      else
      {
        ROS_ERROR("Wrong lane_sequence of mission_point: it can't be zero.");
        offset = 0;
      }

      return offset;
    }
};


class Junction
{    
  public:  
    osm_point junction;
    vector<osm_point> junction_points;
    std::map<osm_point, Road> connection;
    static int junction_sequence;

    //navigation
    double astar_G;
    double astar_H;
    double astar_F;
    //还真可以这样
    vector<Junction>::iterator parent;
    //尚未填充
    std::map<double, vector<Junction>::iterator> edge;

    Junction() {}

    Junction(osm_point junction)
    {
      this -> junction = junction;
      this->edge.clear();
      this->connection.clear();
    }
    void add_points(osm_point p)
    {
      auto iter = find_if(junction_points.begin(), junction_points.end(), [p](osm_point &jp){return (jp.id == p.id);});
      if(iter == junction_points.end())
        junction_points.push_back(p);
    }

    void add_connection(osm_point point, Road road)
    {
      connection.emplace(point, road);
    }

    void junctions_add_vis_info(visualization_msgs::MarkerArray &junctions_map);

    bool operator== (const Junction &j)
    {
      //Road类添加ID；以ID相同为判断标准
      return (this -> junction.id == j.junction.id);
    }
};


class roads_find_id
{
  public:
    roads_find_id(UUID arr): id(arr) {}
    bool operator () (Road &road)
    {
      auto iter = find_if(road.points.begin(), road.points.end(), [this](osm_point &p) {return (p.id == this -> id);});
      return (iter != road.points.end());
    }
    UUID id;
};

class junction_find_point
{
  public:
    junction_find_point(osm_point junction_point): point(junction_point) {}
    
    bool operator () (Junction &junction)
    {
      auto iter = find(junction.junction_points.begin(), junction.junction_points.end(), point);//避免在程序逻辑出错时，反复push同一个元素，导致占用内存很大
      return (iter != junction.junction_points.end());
    }
    
    osm_point point;
};

class junction_find_link_road
{
  public:
    junction_find_link_road(vector<Road>::iterator iter): iter_road(iter) {}
    
    bool operator () (Junction &junction)
    {
      auto iter = find_if(junction.connection.begin(), junction.connection.end(), [this](pair<const osm_point, Road> &p) {return (p.second.id_ == this->iter_road->id_);});
      return (iter != junction.connection.end());
    }
    
    vector<Road>::iterator iter_road;
};

class points_find_keyvalue
{
  public:
    points_find_keyvalue(string k, string v): key(k), value(v) {}
    bool operator () (osm_point &point)
    {
      auto iter = find_if(point.props.begin(), point.props.end(), [this](prop &p) {return (p.key == this -> key && p.value == this -> value);});
      return (iter != point.props.end());
    }
    string key;
    string value;
};

struct road_pos_info
{
  vector<Road>::iterator road;
  int segment;
  Point center_projection;
  int lane_sequence;
  int direction;
  vector<osm_point>::iterator junction_point;
  bool is_goal;
};


class vec_map
{
  public:
    vector<double> border_junction;
    visualization_msgs::MarkerArray road_map;//element of map_io
    visualization_msgs::MarkerArray junctions_map;
    geographic_msgs::GeographicMap raw_map;
    string map_path;
    vector<osm_point> global_points;
    vector<Road> roads;
    vector<Junction> junctions;
    TrackPoint goal_pose; 
    bool mission_exist;
    std::map<int, mission_point> mission_points;
    nav_msgs::Path visual_mission_curve;
    nav_msgs::Path visual_navigation_curve;
    //todo: delete mission_curve
    vector<map_module::curvepoint> mission_curve;
    geometry_msgs::PoseStamped nav_goal;
    vector<map_module::curvepoint> navigation_curve;
    vector<mission_curve_point> mission_curve_with_length;
    //vector<int> lane_turn;
    double curve_length;
    ros::Publisher roads_vis_pub;
    ros::Publisher junctions_vis_pub;
    ros::Publisher mission_curve_vis_pub;
    ros::Publisher navigation_curve_vis_pub;
    double point_margin;

    vec_map() {}
    vec_map(string osm_url, ros::NodeHandle nh): map_path(osm_url)
    {
      point_margin = 0.1;
      curve_length = 0;
      border_junction.push_back(3.5);
      border_junction.push_back(3.5);
      navigation_curve.resize(0);
      resolve_map(nh);
    }

    //assign values to global points and mission points,
    //construct junctions;
    void resolve_points();

    //assign values to roads
    void resolve_roads();
    
    void resolve_junctions();
    void add_edges_in_junctions();
    void junction_edge_test();

    //double get_mission_point_offset(int lane_sequence, Road road);

    //由任务点(key == mission)得到路径
    //在任务点列表中，依次两点之间生成路径
    
    vector<map_module::curvepoint> get_curve_line(Point p1, Point p2, double offset , vector<double> border, double &layback);
    //vector<map_module::curvepoint> Section_Interploration(TrackPoint p1, TrackPoint p2, int kdxdy[], double &layback);
    vector<map_module::curvepoint> spline_interploration(TrackPoint p1, TrackPoint p2, vector<double> border, double &layback);
    vector<map_module::curvepoint> get_curve_spline(Point p1, Point p2, double offset1, double offset2, int kdxdy[], double a1, double a2, vector<double> border, double &layback);
    vector<map_module::curvepoint> get_curve_between_mission(mission_point p1, mission_point p2, double &layback);

    vector<map_module::curvepoint> generate_mission_curve();

    void mission_curve_add_vis_info();

    void store_curve_in_csv();
    void store_navigation_curve();

    void resolve_map(ros::NodeHandle nh);

    void map_visualization_pub();

    void correct_mission_curve();

    //navigation part
    geometry_msgs::Pose info_to_pose(road_pos_info pos);
    road_pos_info get_guided_junction_point(vector<Junction>::iterator iter1, vector<Junction>::iterator iter2);
    road_pos_info get_last_junction_point(vector<Junction>::iterator iter_j, road_pos_info goal);
    road_pos_info get_road_pos_info(geometry_msgs::PoseStamped pose);
    vector<Junction>::iterator nav_get_first_junction(geometry_msgs::PoseStamped pose);
    vector<Junction>::iterator nav_get_last_junction(geometry_msgs::PoseStamped pose);
    bool astar_navigation(vector<Junction>::iterator &start, vector<Junction>::iterator &goal);
    vector<vector<Junction>::iterator>::iterator get_min_F_node(vector<vector<Junction>::iterator> &open_list);
    vector<vector<Junction>::iterator> build_junctions_queue(vector<Junction>::iterator goal);

    vector<int> build_lane_turn(vector<Point> points);    
    vector<map_module::curvepoint> get_guided_curve(road_pos_info start, double &layback, int turn);
    //vector<map_module::curvepoint> get_junction_curve();
    bool get_navigation_curve(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped goal_pose);
    vector<map_module::curvepoint> just_go(vector<SegmentCenterPoint> center_points, road_pos_info start, Point end, double &layback, int turn);

    void navigation_curve_add_vis_info();


    geometry_msgs::Pose track_to_pose(map_module::curvepoint point){
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
};

#endif
