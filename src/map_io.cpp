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
#include <nav_msgs/Path.h>

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

    //这样写真的可以嘛
    //以道路中心线为基准，生成车道线的几何形状，中心线以右为正，中心线以左为负。
    Line generate_line(Line center, double offset)
    {
    //道路是有方向的，以中心线为基准的左右和空间绝对左右不一定一致
    int k = 1;
    if(center.a < 0)
      k = -1;

    Point p1;
    Point p2;
    double a = -1 / center.a;
    // double b1 = center.p1_.y_ + center.p1_.x_ / center.a;
    // double b2 = center.p2_.y_ + center.p2_.x_ / center.a;
    p1.x_ = center.p1_.x_ + k * offset / sqrt(1 + a * a);
    p1.y_ = center.p1_.y_ + k * offset * a / sqrt(1 + a * a);
    p2.x_ = center.p2_.x_ + k * offset / sqrt(1 + a * a);
    p2.y_ = center.p2_.y_ + k * offset * a / sqrt(1 + a * a);

    Line line(p1, p2);
    return line;
  }
};

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

template <typename T>
bool in_range(T value, T border1, T border2)
{
  if(border1  > border2)
    return (border1 >= value && border2 <= value);
  else
    return (border2 >= value && border1 <= value);
}

typedef struct osm_point
{
  UUID id;
  Point p;
  keyvalue_arr props;

  osm_point() {}
  osm_point(UUID id_p, Point p_p, keyvalue_arr props_p): id(id_p), p(p_p), props(props_p) {}
  
  //自定义的数据类型作键值，需要重载 <
  bool operator < (const osm_point &p) const
  {
    return((this -> p.x_) < p.p.x_);
  }

  //利用std::find，需要重载 ==
  bool operator== (const osm_point &p)
  {
    return (this -> id == p.id);
  }

}osm_point;

struct mission_point : public osm_point
{
  mission_point() {}
  mission_point(UUID id_p, Point p_p, keyvalue_arr props_p, int index): osm_point(id_p, p_p, props_p), lane_sequence(index) {}

  int lane_sequence;
};

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
    Road(vector<osm_point> point, keyvalue_arr props, UUID id)
    {
      this -> id_ = id;
      points = point;
      Line geometry(point.front().p, point.back().p);
      geometry_ = geometry;
      
      junction_points.resize(0);


      //将路口点解析并添加到juntion_points成员中
      junction_path = false;
      int points_size = points.size();
      vector<prop>::iterator iter;
      for(int i = 0; i < points_size; i++)
      {
        //如果是路口道路，标记
        if(junction_path == false)
        {
          iter = find_if(points[i].props.begin(), points[i].props.end(), props_find_key("junction"));
          if(iter != points[i].props.end())
          {
            junction_path = true;
            junction = points[i];
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
      if(this -> junction_path == true)
        return;
      else{
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
    }
     
    //以道路中心线为基准，生成车道线的几何形状，中心线以右为正，中心线以左为负。
    Line generate_line(Line center, double offset)
    {
      //道路是有方向的，以中心线为基准的左右和空间绝对左右不一定一致
      int k = 1;
      if(center.a < 0)
        k = -1;

      Point p1;
      Point p2;
      double a = -1 / center.a;
      // double b1 = center.p1_.y_ + center.p1_.x_ / center.a;
      // double b2 = center.p2_.y_ + center.p2_.x_ / center.a;
      p1.x_ = center.p1_.x_ + k * offset / sqrt(1 + a * a);
      p1.y_ = center.p1_.y_ + k * offset * a / sqrt(1 + a * a);
      p2.x_ = center.p2_.x_ + k * offset / sqrt(1 + a * a);
      p2.y_ = center.p2_.y_ + k * offset * a / sqrt(1 + a * a);

      Line line(p1, p2);
      return line;
    }


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

    ros::NodeHandle nh_;
    ros::Publisher road_pub;

};

bool operator== (const Road &a, const Road &b)
{
  //Road类添加ID；以ID相同为判断标准
  return (a.id_ == b.id_);
}

class Junction
{
  public:

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

    void junctions_vis()
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id="world";
      marker.header.stamp = ros::Time::now();
      //marker.action = visualization_msgs::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.id = junction_sequence;
      junction_sequence++;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.scale.z = 7;
      marker.color.b = 1.0f;
      marker.color.g = 1.0f;
      marker.color.r = 1.0f;
      marker.color.a = 1;
      // std::ostringstream str;
      //   str<<round(point.getCurvature()*10000)/10000;
      marker.text = "JUNCTION";
      geometry_msgs::Pose pose;
      pose.position.x = junction.p.x_;
      pose.position.y = junction.p.y_;
      pose.position.z =0;
      marker.pose=pose;
      junctions_map.markers.push_back(marker);
    }

    osm_point junction;
    vector<osm_point> junction_points;
    map<osm_point, Road> connection;
    static int junction_sequence;
    static visualization_msgs::MarkerArray junctions_map;
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

//重载运算符，以拼接vector
template<typename T>
vector<T> operator+ (vector<T> a, vector<T> b)
{
  a.insert(a.end(), b.begin(), b.end());
  return a;
}

double get_mission_point_offset(mission_point p, Road road)
{
  cout<<"in function get_mission_point_offset"<<endl;
  double offset;
  if(p.lane_sequence > 0)
  {
    offset = 0.075;
    for(int i = 0; i < p.lane_sequence; i++)
    {
      offset = offset + road.width_forward_[i];
    }
    offset = offset - road.width_forward_[p.lane_sequence - 1] / 2;
  }

  else if(p.lane_sequence < 0)
  {
    offset = -0.075;
    for(int i = 0; i > p.lane_sequence; i--)
    {
      offset = offset - road.width_forward_[i];
    }
    offset = offset + road.width_forward_[p.lane_sequence - 1] / 2;
  }

  else
  {
    ROS_INFO("Wrong lane_sequence of mission_point: it can't be zero.");
    offset = 0;
  }

  cout<<"out function get_mission_point_offset"<<endl;
  return offset;
}

vector<TrackPoint> get_curve_line(osm_point p1, osm_point p2, double offset)
{
  cout<<"in function get_curve_line"<<endl;
  //生成路径几何形状
  Line center_line(p1.p, p2.p);
  Line curve_line = center_line.generate_line(center_line, offset);
  vector<TrackPoint> curve_points;
  curve_points.resize(0);
  
  //采样
  int k =sign(p2.p.x_ - p1.p.x_);
  //double x, y;
  TrackPoint p;
  double theta = atan(curve_line.a);
  for(double x = p1.p.x_; in_range(x, p1.p.x_, p2.p.x_); x = x + 0.1 * k)
  {
    curve_points.emplace_back(x, curve_line.a * x + curve_line.b, theta, 0);
  }
  cout<<"out function get_curve_line"<<endl;
  return curve_points;  
}

vector<TrackPoint> Section_Interploration(TrackPoint p1, TrackPoint p2)
{ 
  cout<<"in function Section_Interploration"<<endl;
  vector<TrackPoint> curve_points;
  curve_points.resize(0);
  curve_points.push_back(p1);

  double x1, y1, theta1, kappa1, x2, y2, theta2, kappa2;
    x1 = p1.x_;
    y1 = p1.y_;
    theta1 = p1.theta_;
    kappa1 = p1.curvature_;
    x2 = p2.x_;
    y2 = p2.y_;
    theta2 = p2.theta_;
    kappa2 = p2.curvature_;

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
    TrackPoint p;
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
      p.x_ = x;
      p.y_ = x_power.dot(coef_0d);
      p.theta_ = atan(x_power.dot(coef_1d));
      p.curvature_ = x_power.dot(coef_2d)/pow((1+pow(x_power.dot(coef_1d), 2)), 1.5);
      curve_points.push_back(p);
      x = x + 0.1;
    }
    curve_points.push_back(p2);
    cout<<"out function Section_Interploration"<<endl;
    return curve_points;
}

vector<TrackPoint> get_curve_spline(osm_point p1, osm_point p2, double offset1, double offset2, double a1, double a2)
{
  cout<<"in function get_curve_spline"<<endl;
  double a1_ = -1 / a1;
  double a2_ = -1 / a2;
  int k1 = 1;
  int k2 = 1;
  if(a1 < 0)
    k1 = -1;
  if(a2 < 0)
    k2 = -1;

  double x1 = p1.p.x_ + k1 * offset1 / sqrt(1 + a1_ * a1_);
  double x2 = p2.p.x_ + k2 * offset2 / sqrt(1 + a2_ * a2_);
  double y1 = p1.p.y_ + k1 * offset1 * a1_ / sqrt(1 + a1_ * a1_);
  double y2 = p2.p.y_ + k2 * offset2 * a2_ / sqrt(1 + a2_ * a2_);
 
  cout<<"out function get_curve_spline"<<endl;
  //还真可以这么写？
  return Section_Interploration(TrackPoint(x1, y1, atan(a1), 0), TrackPoint(x2, y2, atan(a2), 0));

}

vector<TrackPoint> get_curve_between_mission(mission_point p1, mission_point p2, vector<Road> &roads, vector<Junction> &junctions)
{
  cout<<"in function get_curve_between_mission"<<endl;
  vector<TrackPoint> raw_curve_section;
  raw_curve_section.resize(0);
  //找到任务点所在的路
  vector<Road>:: iterator iter_former, iter_latter;
  iter_former = find_if(roads.begin(), roads.end(), roads_find_id(p1.id));
  iter_latter = find_if(roads.begin(), roads.end(), roads_find_id(p2.id));
  //路径基于道路中心线的offset
  double offset1, offset2;
  offset1 = get_mission_point_offset(p1, (*iter_former));
  offset2 = get_mission_point_offset(p2, (*iter_latter));

  double a1 = (*iter_former).geometry_.a;
  double a2 = (*iter_latter).geometry_.a;


  //确定连接两条道路的路口
  Junction junction;
  //路口与道路连接的路口点
  osm_point junction_point1, junction_point2;
  vector<Junction>::iterator iter_junction;
  //iter1 第一段路路口点的迭代器
  for(vector<osm_point>::iterator iter1 = (*iter_former).junction_points.begin(); iter1 != (*iter_former).junction_points.end(); iter1 = next(iter1))
  {
    iter_junction = find_if(junctions.begin(), junctions.end(), junction_find_point(*iter1));
    if(iter_junction != junctions.end())
    {
      junction = *iter_junction;
      for(auto iter_map = junction.connection.begin(); iter_map != junction.connection.end(); iter_map = next(iter_map))
      {
        if((*iter_latter) == iter_map -> second)
        {
          junction_point1 = *iter1;
          junction_point2 = iter_map -> first;
          raw_curve_section = get_curve_line(p1, junction_point1, offset1) + get_curve_spline(junction_point1, junction_point2, offset1, offset2, a1, a2) + get_curve_line(junction_point2, p2, offset2);
          break;
        }
      }
      if(raw_curve_section.size() != 0)
        break;
    }
    else
      ROS_WARN("Naked juntion_points.");

  }
  if(raw_curve_section.size() == 0)
    ROS_WARN("No curve generate between this two mission_points.");

  cout<<"out function get_curve_between_mission"<<endl;
  return raw_curve_section;
}

vector<TrackPoint> generate_mission_curve(map<int, mission_point> mission_points, vector<Road> &roads, vector<Junction> &junctions)
{
  cout<<"in function generate_mission_curve"<<endl;
  vector<TrackPoint> raw_curve;
  vector<TrackPoint> raw_curve_section;
  raw_curve.resize(0);
  int n = 0;
  //按顺序依次将每两个任务点之间的路径点填充到路径中
  for(auto iter = mission_points.begin(); next(iter) != mission_points.end(); iter++)
  {
    n++;
    cout<<"add path "<<n<<" th times."<<endl;
    raw_curve_section.resize(0);
    raw_curve_section = get_curve_between_mission((*iter).second, (*next(iter)).second, roads, junctions);
    cout<<"finished one path generation."<<endl;
    raw_curve.insert(raw_curve.end(), raw_curve_section.begin(), raw_curve_section.end());
    cout<<"finished insert this path."<<endl;
  }

  cout<<"out function generate_mission_curve"<<endl;
  return raw_curve;
}




geometry_msgs::Pose to_pose(TrackPoint point){
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

//ros::Publisher 
// Junction::junction_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/junction_text", 1);

visualization_msgs::MarkerArray Junction::junctions_map;//讲道理这个应该是Junction上层的类包含的元素，不应该是Junction的类内静态变量
int Junction::junction_sequence = 0;

int main(int argc, char** argv)
{
    ROS_INFO("INTO main point now");
    ros::init(argc, argv, "map_io_node");
    ros::NodeHandle nh;

    visualization_msgs::MarkerArray road_map;
    ros::Publisher map_pub = nh.advertise<visualization_msgs::MarkerArray>("/map", 1);
    int marker_id = 0;//todo: 写成类内静态变量

    Junction::junctions_map.markers.resize(0);
    ros::Publisher junction_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/junction_text", 1);

    geographic_msgs::GeographicMap raw_map;
    ros::service::waitForService("get_geographic_map");
    ros::ServiceClient osm_client;
    osm_client = nh.serviceClient<geographic_msgs::GetGeographicMap>("get_geographic_map");
    string map_path = "package://map_module/data/road.osm";
    geographic_msgs::GetGeographicMap osm_srv;
    osm_srv.request.url = map_path;
    ros::Publisher mission_path_pub = nh.advertise<nav_msgs::Path>("/mission_path", 1);

    if (!osm_client.call(osm_srv)) {
      ROS_ERROR_STREAM("error in map file " << map_path << " reading.");
    }

    raw_map = osm_srv.response.map;

    //地图数据元素
    //获取全局点列表
    int points_num = raw_map.points.size();
    vector<osm_point> global_points;
    vector<prop>::iterator iter;
    vector<Road> roads;
    vector<Junction> junctions;
    junctions.resize(0);
    roads.resize(0); 
    map<int, mission_point> mission_points;
    global_points.reserve(points_num);
    global_points.resize(0);
    
    // visualization_msgs::MarkerArray Juntion::junctions_map.markers.resize(0);

    
    for(int i = 0; i < points_num; i++)
    {
      geographic_msgs::WayPoint way_point = raw_map.points[i];
      geodesy::UTMPoint utm_point;
      geodesy::fromMsg(way_point.position, utm_point);
      // cout<<"utm_point.easting: "<<utm_point.easting<<endl;
      // cout<<"utm_point.northing: "<<utm_point.northing<<endl;
      // cout<<"******************************"<<endl;
      Point p(utm_point.easting - 284000, utm_point.northing - 3793000);
      cout<<"x: "<<p.x_<<"   "<<"y: "<<p.y_<<endl;
      geodesy::fromMsg(way_point.position, utm_point);
      //osm_point op(way_point.id.uuid, p, way_point.props);
      global_points.emplace_back(way_point.id.uuid, p, way_point.props);
      iter = find_if(way_point.props.begin(), way_point.props.end(), props_find_key("mission"));
      if(iter != way_point.props.end())
      {
        ROS_INFO("Find mission point. ");
        auto iter_lane = find_if(way_point.props.begin(), way_point.props.end(), props_find_key("lane_sequence"));
        if(iter_lane != way_point.props.end())
        {
          int index = atoi((iter_lane -> value).c_str());        
          mission_points.emplace(atoi((*iter).value.c_str()), mission_point(way_point.id.uuid, p, way_point.props, index));
          ROS_INFO("Got mission point. ");
        }

      }
      
      iter = find_if(way_point.props.begin(), way_point.props.end(), props_find_key("junction"));
      if(iter != way_point.props.end())
      {
        Junction junction(osm_point(way_point.id.uuid, p, way_point.props));
        junction.junctions_vis();//填充路口可视化的消息类型
        junctions.push_back(junction);
      }
        
      

    }

    ROS_INFO("FINISH point resolved.");

    //根据feature构建道路
    //todo 将任务点，路口点读进Road
    int feature_number = raw_map.features.size();
    for(int j = 0; j < feature_number; j++)
    {
      //找到相关的点
      vector<osm_point> road_point;
      road_point.reserve(raw_map.features[j].components.size());
      road_point.resize(0);

      // for(int k = 0; k < points_num; k++)
      // {
      //   for(lint l = 0; l < raw_map.features[j].components.size(); l++)
      //   {
      //     if(global_points[k].id == raw_map.features[j].components[l].uuid)
      //     {
      //       road_point.push_back(global_points[k]);
      //     }
      //   }
      // }
      for(lint l = 0; l < raw_map.features[j].components.size(); l++)
      {
        UUID id = raw_map.features[j].components[l].uuid;
        auto iter = find_if(global_points.begin(), global_points.end(), [id](osm_point &p) { return (p.id == id);});
        if(iter != global_points.end())
          road_point.push_back(*iter);
        else
          ROS_INFO("WRONG! Found point in road but not in global map");
        
      }
      

      Road road(road_point, raw_map.features[j].props, raw_map.features[j].id.uuid);
      road.road_compile(road_map, marker_id);
      roads.push_back(road);
    } 

    ROS_INFO("FINISH roads resolved.");

    //构建junctuon
    //对每个junction，添加与其相连的junction_point
    vector<Junction>::iterator iter_junction;//查找路口的迭代器
    vector<Road> roads_contains_junctions;
    osm_point junction_point;
    vector<Road>::iterator iter_road;//查找道路的迭代器
    int junction_number = 0;
    int road_number = 0;
    for(iter_junction = junctions.begin(); iter_junction != junctions.end(); iter_junction = next(iter_junction))
    {
      junction_number++;     
      cout<<"resolving "<<junction_number<<" th junction."<<endl;
      roads_contains_junctions.resize(0);
      UUID id = (*iter_junction).junction.id;
      
      iter_road = find_if(roads.begin(), roads.end(), roads_find_id(id));
      while(iter_road != roads.end())
      {
        road_number++;
        cout<<"resolving"<<road_number<<" th road of the "<<junction_number<<" th junction"<<endl;
        if((*iter_road).junction_points.size() == 1)
        {
          junction_point = (*iter_road).junction_points[0];
          (*iter_junction).add_points(junction_point);
          //添加与此路口连接的道路信息
          //查找包含此路口点的道路
          auto iter1 = find_if(roads.begin(), iter_road, roads_find_id(junction_point.id));
          auto iter2 = find_if(next(iter_road), roads.end(), roads_find_id(junction_point.id));
          if(iter1 == iter_road)
          {
            if(iter2 == roads.end())
              ROS_WARN("Here a juntion_point is not connected to a road.");
            else
              (*iter_junction).add_connection(junction_point, (*iter2));
          }
          else if(iter2 != roads.end())
            ROS_WARN("ERROR: more than two roads connected to one junction_point!");
          else
            (*iter_junction).add_connection(junction_point, (*iter1));
          
        }
        else if((*iter_road).junction_points.size() > 1)
          ROS_WARN("Wrong_junction drawn, junction is connected to more than a juntion point in a junction path.");
        else
          ROS_WARN("Here is a isolated junction. Please add junction_point to it.");

        if(next(iter_road) != roads.end())
          iter_road = find_if(next(iter_road), roads.end(), roads_find_id(id));
        else
          break;

      } 
    }

    ROS_INFO("FINISH junctions resolved.");

    //由任务点(key == mission)得到路径
    //在任务点列表中，依次两点之间生成路径
    vector<TrackPoint> mission_path = generate_mission_curve(mission_points, roads, junctions);
    ROS_INFO("FINISH mission_path resolved.");
    //将路径信息转换成可被rviz可视化的消息类型
    geometry_msgs::PoseStamped posestamped;
    nav_msgs::Path visual_mission_path;
    
    visual_mission_path.header.frame_id = "world";
    for (auto point : mission_path) {
        //static_curve_srv.request.static_curve.points.push_back(to_curvepoint(point));
      posestamped.header.frame_id = "world";
      posestamped.pose = to_pose(point);
      visual_mission_path.poses.push_back(posestamped);

    }

    

    //可视化
    ros::Rate loop_rate(1);
    while(ros::ok())
    {
      map_pub.publish(road_map);
      mission_path_pub.publish(visual_mission_path);
      junction_vis_pub.publish(Junction::junctions_map);
      loop_rate.sleep();
      ros::spinOnce();
    }

 
    

}

/*
TODO:
对于osm数据的检查
路口的可视化用文字代替
统一仿函数：
    仿函数整理（抽象出最少最适用的）
    能用仿函数的地方用仿函数
*/