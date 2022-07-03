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
    Line geometry_;
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
};

class Junction
{    
  public:  
    osm_point junction;
    vector<osm_point> junction_points;
    map<osm_point, Road> connection;
    static int junction_sequence;

    Junction() {}

    Junction(osm_point junction)
    {
      this -> junction = junction;
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


class vec_map
{
  public:

    visualization_msgs::MarkerArray road_map;//element of map_io
    visualization_msgs::MarkerArray junctions_map;
    geographic_msgs::GeographicMap raw_map;
    string map_path;
    vector<osm_point> global_points;
    vector<Road> roads;
    vector<Junction> junctions;
    TrackPoint goal_pose; 
    map<int, mission_point> mission_points;
    nav_msgs::Path visual_mission_curve;
    //todo: delete mission_curve
    vector<TrackPoint> mission_curve;
    vector<mission_curve_point> mission_curve_with_length;
    double curve_length;
    ros::Publisher roads_vis_pub;
    ros::Publisher junctions_vis_pub;
    ros::Publisher mission_curve_vis_pub;
    double point_margin;

    vec_map() {}
    vec_map(string osm_url, ros::NodeHandle nh): map_path(osm_url)
    {
      point_margin = 0.1;
      resolve_map(nh);
    }

    //assign values to global points and mission points,
    //construct junctions;
    void resolve_points();

    //assign values to roads
    void resolve_roads();
    
    void resolve_junctions();

    double get_mission_point_offset(mission_point p, Road road);

    //由任务点(key == mission)得到路径
    //在任务点列表中，依次两点之间生成路径
    
    vector<TrackPoint> get_curve_line(osm_point p1, osm_point p2, double offset , double &layback);
    vector<TrackPoint> Section_Interploration(TrackPoint p1, TrackPoint p2, int kdxdy[], double &layback);
    vector<TrackPoint> spline_interploration(TrackPoint p1, TrackPoint p2, double &layback);
    vector<TrackPoint> get_curve_spline(osm_point p1, osm_point p2, double offset1, double offset2, int kdxdy[], double a1, double a2, double &layback);
    vector<TrackPoint> get_curve_between_mission(mission_point p1, mission_point p2, double &layback);

    vector<TrackPoint> generate_mission_curve();

    void mission_curve_add_vis_info();

    void store_curve_in_csv();

    void resolve_map(ros::NodeHandle nh);

    void map_visualization_pub();

    void correct_mission_curve();

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
};

#endif
