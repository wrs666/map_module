#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <geographic_msgs/GeographicMap.h>
#include <geographic_msgs/GetGeographicMap.h>
#include <vector>
#include <cmath>
#include <geodesy/utm.h>
#include <sstream>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <map_module/curve.h>
#include <map_module/curvepoint.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unordered_map>

using namespace std;
using namespace Eigen;

typedef long unsigned int lint;
typedef geographic_msgs::KeyValue prop;
typedef vector<prop> keyvalue_arr;
using UUID = boost::array<uint8_t, 16>;


class TrackPoint {

  public:
    TrackPoint() {}
    TrackPoint(double x, double y, double t, double c) {
      x_ = x;
      y_ = y;
      theta_ = t;
      curvature_ = c;
    }
    
    double x_;
    double y_;
    double theta_;
    double curvature_;
};

class Point{
  public:
    Point() {}
    Point(double x, double y): x_(x), y_(y) {}

    double x_;
    double y_;
};

class Line
{
  public:
    Point p1_;
    Point p2_;
    //y=ax+b
    double a;
    double b;

    Line() {}
    Line(Point p1, Point p2): p1_(p1), p2_(p2)
    {
      a = (p2.y_ - p1.y_)/(p2.x_ - p1.x_);
      b = (p2.x_ * p1.y_ - p1.x_ * p2.y_)/(p2.x_ - p1.x_);
    }
};

typedef struct osm_point
{
  UUID id;
  Point p;
  keyvalue_arr props;

  osm_point() {}
  osm_point(UUID id_p, Point p_p, keyvalue_arr props_p): id(id_p), p(p_p), props(props_p) {}

}osm_point;

vector<string> split(const std::string& s, char delimiter) {
  vector<string> tokens;
  string token;
  istringstream tokenStream(s);
  while (getline(tokenStream, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
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
    Road() {}
    Road(vector<osm_point> point, keyvalue_arr props)
    {
      points = point;
      Line geometry(point.front().p, point.back().p);
      geometry_ = geometry;
      
      junction_points.resize(0);


      //如果此道路是路口道路
      int points_size = points.size();
      vector<prop>::iterator iter;
      bool junction_path = false;
      for(int i = 0; i < points_size; i++)
      {
        if(junction_path == false)
        {
          iter = find_if(points[i].props.begin(), points[i].props.end(), props_find_key("junction"));
          if(iter != points[i].props.end())
          {
            junction_path = true;
            junction = points[i];
            junction_path = true;
          }
        }
        iter = find_if(points[i].props.begin(), points[i].props.end(), props_find_key("highway"));
        if(iter != points[i].props.end() && (*iter).value == "junction_point")
          junction_points.push_back(points[i]);
      }
        

      string arr_cache;
      vector<string> string_arr;

      vector<prop>::iterator it;
      it = find_if(props.begin(), props.end(), &Road::find_divider);
      divider_ = (*it).value;
      it = find_if(props.begin(), props.end(), &Road::find_highway);
      highway_ = (*it).value;
      it = find_if(props.begin(), props.end(), &Road::find_lanesbackward);
      lanes_backward_ = atoi((*it).value.c_str());
      it = find_if(props.begin(), props.end(), &Road::find_lanesforward);
      lanes_forward_ = atoi((*it).value.c_str());

      width_backward_.resize(0);
      width_forward_.resize(0);

      it = find_if(props.begin(), props.end(), find_widthlanesbackward);
      arr_cache = (*it).value.c_str();
      string_arr = split(arr_cache, '|');
      for(lint i = 0; i < string_arr.size(); i++)
      {
        width_backward_.push_back(atof(string_arr[i].c_str()));
      }

      it = find_if(props.begin(), props.end(), find_widthlanesforward);
      arr_cache = (*it).value.c_str();
      string_arr = split(arr_cache, '|');
      for(lint i = 0; i < string_arr.size(); i++)
      {
        width_forward_.push_back(atof(string_arr[i].c_str()));
      }

    }

    static bool find_divider(prop& keyvalue)
    {
      if(keyvalue.key == "divider")
        return true;
      else
        return false;

    }

    static bool find_highway(prop& keyvalue)
    {
      if(keyvalue.key == "highway")
        return true;
      else
        return false;

    }

    static bool find_lanesbackward(prop& keyvalue)
    {
      if(keyvalue.key == "lanes:backward")
        return true;
      else
        return false;

    }
    
    static bool find_lanesforward(prop& keyvalue)
    {
      if(keyvalue.key == "lanes:forward")
        return true;
      else
        return false;

    }

    static bool find_widthlanesforward(prop& keyvalue)
    {
      if(keyvalue.key == "width:lanes:forward")
        return true;
      else
        return false;

    }

    static bool find_widthlanesbackward(prop& keyvalue)
    {
      if(keyvalue.key == "width:lanes:backward")
        return true;
      else
        return false;

    }

    void road_compile(visualization_msgs::MarkerArray &map, int& id)
    {
      geometry_msgs::Point p1, p2;

      //生成道路中心线
      visualization_msgs::Marker line_list;
      line_list.header.frame_id = "world";
      //line_list.header.stamp = ros::Time::now();
      //line_list.ns = "points_and_lines";
      line_list.action = visualization_msgs::Marker::ADD;
      line_list.pose.orientation.w = 1.0;
      //line_list.id = id;
      line_list.type = visualization_msgs::Marker::LINE_LIST;
      line_list.scale.x = 0.1;

      visualization_msgs::Marker center = line_list;
      center.id = id;
      center.color.r = 1.0f;
      center.color.g = 1.0f;
      center.color.a = 1.0;

      Line center_left = generate_line(geometry_, -0.075);
      Line center_right = generate_line(geometry_, 0.075);
      p1.x = center_left.p1_.x_;
      p1.y = center_left.p1_.y_;
      p1.z = 0;
      p2.x = center_left.p2_.x_;
      p2.y = center_left.p2_.y_;
      p2.z = 0;
      center.points.push_back(p1);
      center.points.push_back(p2);

      p1.x = center_right.p1_.x_;
      p1.y = center_right.p1_.y_;
      p1.z = 0;
      p2.x = center_right.p2_.x_;
      p2.y = center_right.p2_.y_;
      p2.z = 0;
      center.points.push_back(p1);
      center.points.push_back(p2);

      map.markers.push_back(center);
      id++;

      //添加车道线
      visualization_msgs::Marker lane_line;
      lane_line = line_list;
      lane_line.id = id;
      
      lane_line.color.r = 1.0f;
      lane_line.color.g = 1.0f;
      lane_line.color.b = 1.0f;
      lane_line.color.a = 1.0;

     //中心线以右
      double offset = 0.075;

      for(int i = 0; i < lanes_forward_; i++)
      {
        Line laneline = generate_line(center_right, offset + width_forward_[i]);
        offset = offset + width_forward_[i];
        p1.x = laneline.p1_.x_;
        p1.y = laneline.p1_.y_;
        p1.z = 0;
        p2.x = laneline.p2_.x_;
        p2.y = laneline.p2_.y_;
        p2.z = 0;
        lane_line.points.push_back(p1);
        lane_line.points.push_back(p2);

      }

      //中心线以左
      offset = -0.075;

      for(int i = 0; i < lanes_backward_; i++)
      {

        Line laneline = generate_line(center_left, offset - width_backward_[i]);
        offset = offset - width_backward_[i];
        p1.x = laneline.p1_.x_;
        p1.y = laneline.p1_.y_;
        p1.z = 0;
        p2.x = laneline.p2_.x_;
        p2.y = laneline.p2_.y_;
        p2.z = 0;
        lane_line.points.push_back(p1);
        lane_line.points.push_back(p2);

      }

      map.markers.push_back(lane_line);
      id++;
    }

    Line generate_line(Line center, double offset)
    //offset 左偏为负 右偏为正
    {
      Point p1;
      Point p2;
      double a = -1 / center.a;
      // double b1 = center.p1_.y_ + center.p1_.x_ / center.a;
      // double b2 = center.p2_.y_ + center.p2_.x_ / center.a;
      p1.x_ = center.p1_.x_ + offset / sqrt(1 + a * a);
      p1.y_ = center.p1_.y_ + offset * a / sqrt(1 + a * a);
      p2.x_ = center.p2_.x_ + offset / sqrt(1 + a * a);
      p2.y_ = center.p2_.y_ + offset * a / sqrt(1 + a * a);

      Line line(p1, p2);
      return line;
    }



    string divider_;
    string highway_;
    int lanes_backward_;
    int lanes_forward_;
    vector<double> width_backward_;
    vector<double> width_forward_;
    Line geometry_;
    vector<osm_point> junction_points;
    vector<osm_point> points;
    bool junction_path = false;
    osm_point junction;

    ros::NodeHandle nh_;
    ros::Publisher road_pub;

};

class Junction
{
  public:
    osm_point junction;
    vector<osm_point> junction_points;

    Junction(osm_point junction)
    {
      this -> junction = junction;
      junction_points.resize(0);
    }
    void add_points(osm_point p)
    {
      junction_points.push_back(p);
    }

    vector<TrackPoint> get_curve(osm_point former, osm_point latter, double point_margin)
    {
      map_module::curve processed_section;
      double x1, y1, theta1, kappa1, x2, y2, theta2, kappa2;
      x1 = former.p.x_;
      y1 = former.p.y_;
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

   unodered_map
};

// map_module::curve generate_mission_curve(vector<osm_point> global_points, map<int, osm_point> mission_points, vector<Road> &roads)
// {
//   map_module::curve mission_curve;
//   vector<TrackPoint> raw_curve;
//   vector<TrackPoint> raw_curve_section;
//   raw_curve.resize(0);

//   //按顺序依次将每两个任务点之间的路径点填充到路径中
//   for(auto iter = mission_points.begin(); next(iter) != mission_points.end(); iter++)
//   {
//     raw_curve_section.resize(0);
//     raw_curve_section = get_curve_between_mission((*iter).second, (*next(iter)).second, roads);
//     raw_curve.insert(raw_curve.end(), raw_curve_section.begin(), raw_curve_section.end());
//   }

//   //转换成消息类型中的标准格式
//   int length = raw_curve.size();
//   mission_curve.points.reserve(length);
//   map_module::curvepoint p;
//   for(int i = 0; i < length; i++)
//   {
//     p.x = raw_curve[i].x_;
//     p.y = raw_curve[i].y_;
//     p.theta = raw_curve[i].theta_;
//     p.kappa = raw_curve[i].curvature_;
//     mission_curve.points.push_back(p);
//   }
//   return mission_curve;
// }

class roads_find_id
{
  public:
    roads_find_id(UUID arr): id(arr) {}
    bool operator () (Road &road)
    {
      auto iter = find(road.points.begin(), road.points.end(), [this](osm_point &p) {return (p.id == this -> id);});
      return (iter != road.points.end());
    }
    UUID id;
};

vector<TrackPoint> get_curve_between_mission(osm_point p1, osm_point p2, const vector<Road> &roads, const vector<Junction> &junctions)
{
  vector<TrackPoint> raw_curve_section;
  //找到任务点所在的路
  auto iter_former = find_if(roads.begin(), roads.end(), roads_find_id(p1.id));
  auto iter_latter = find_if(roads.begin(), roads.end(), roads_find_id(p2.id));
  //



  return raw_curve_section;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_io_node");
    ros::NodeHandle nh;

    visualization_msgs::MarkerArray road_map;
    ros::Publisher map_pub = nh.advertise<visualization_msgs::MarkerArray>("/map", 1);
    int marker_id = 0;

    geographic_msgs::GeographicMap raw_map;
    ros::service::waitForService("get_geographic_map");
    ros::ServiceClient osm_client;
    osm_client = nh.serviceClient<geographic_msgs::GetGeographicMap>("get_geographic_map");
    string map_path = "package://map_module/data/road.osm";
    geographic_msgs::GetGeographicMap osm_srv;
    osm_srv.request.url = map_path;

    if (!osm_client.call(osm_srv)) {
      ROS_ERROR_STREAM("error in map file " << map_path << " reading.");
    }

    raw_map = osm_srv.response.map;

    //获取全局点列表
    int points_num = raw_map.points.size();
    vector<osm_point> global_points;
    vector<prop>::iterator iter;
    vector<Road> roads;
    vector<Junction> junctions;
    roads.resize(0); 
    map<int, osm_point> mission_points;
    global_points.reserve(points_num);
    global_points.resize(0);

    
    for(int i = 0; i < points_num; i++)
    {
      geographic_msgs::WayPoint way_point = raw_map.points[i];
      geodesy::UTMPoint utm_point;
      geodesy::fromMsg(way_point.position, utm_point);
      // cout<<"utm_point.easting: "<<utm_point.easting<<endl;
      // cout<<"utm_point.northing: "<<utm_point.northing<<endl;
      // cout<<"******************************"<<endl;
      Point p(utm_point.easting - 284000, utm_point.northing - 3793000);

      geodesy::fromMsg(way_point.position, utm_point);
      //osm_point op(way_point.id.uuid, p, way_point.props);
      global_points.emplace_back(way_point.id.uuid, p, way_point.props);
      iter = find_if(way_point.props.begin(), way_point.props.end(), props_find_key("mission"));
      if(iter != way_point.props.end())
      {
        mission_points.emplace(atoi((*iter).value.c_str()), osm_point(way_point.id.uuid, p, way_point.props));
        ROS_INFO("Got mission point. ");
      }
      
      iter = find_if(way_point.props.begin(), way_point.props.end(), props_find_key("junction"));
      if(iter != way_point.props.end())
      {
        Junction junction(osm_point(way_point.id.uuid, p, way_point.props));
        junctions.push_back(junction);
      }
        
      

    }

    //根据feature构建道路
    //todo 将任务点，路口点读进Road
    int feature_number = raw_map.features.size();
    for(int j = 0; j < feature_number; j++)
    {
      //找到相关的点
      vector<osm_point> road_point;
      road_point.reserve(raw_map.features[j].components.size());
      road_point.resize(0);

      for(int k = 0; k < points_num; k++)
      {
        for(lint l = 0; l < raw_map.features[j].components.size(); l++)
        {
          if(global_points[k].id == raw_map.features[j].components[l].uuid)
          {
            road_point.push_back(global_points[k]);
          }
            
        }
        
      }

      Road road(road_point, raw_map.features[j].props);
      road.road_compile(road_map, marker_id);
    } 

    //对每个junction，添加与其相连的junction_point
    vector<Junction>::iterator iter_junction;
    for(iter_junction = junctions.begin(); iter_junction != junctions.end(); iter_junction = next(iter_junction))
    {
      UUID id = (*iter_junction).junction.id;
      
      vector<Road>::iterator iter_road = find_if(roads.begin(), roads.end(), roads_find_id(id));
      while(iter_road != roads.end())
      {
        if((*iter_road).junction_points.size() == 1)
          (*iter_junction).add_points((*iter_road).junction_points[0]);
        else if((*iter_road).junction_points.size() > 1)
          ROS_WARN("Wrong_junction drawn, junction is connected to more than a juntion point in a junction path.");
        else
          ROS_WARN("Here is a isolated junction. Please add junction_point to it.");
        if(next(iter_road) != roads.end())
          iter_road = find_if(next(iter_road), roads.end(), roads_find_id(id));
      } 
    }

    //由任务点(key == mission)得到路径
    //在任务点列表中，依次两点之间生成路径



    //可视化
    ros::Rate loop_rate(1);
    while(ros::ok())
    {
      map_pub.publish(road_map);
      loop_rate.sleep();
      ros::spinOnce();
    }

 
    

}