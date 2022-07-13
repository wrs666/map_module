#ifndef __GEOMETRY_H__
#define __GEOMETRY_H__

#include <ros/ros.h>
#include <geographic_msgs/GeographicMap.h>
#include <vector>
#include <map_module/curve.h>
#include <map_module/curvepoint.h>
#include <geometry_msgs/Pose.h>

using namespace std;

typedef geographic_msgs::KeyValue prop;
typedef vector<prop> keyvalue_arr;
using UUID = boost::array<uint8_t, 16>;

class Point{
  public:
    Point() {}
    Point(double x, double y): x_(x), y_(y) {}

    double x_;
    double y_;

    Point generate_Point(double a, double offset)
    {
      int k = 1;
      if(a < 0)
          k = -1;
      double a_v = -1 / a;

      double x = this->x_ + k * offset / sqrt(1 + a_v * a_v);
      double y = this->y_ + k * offset * a_v / sqrt(1 + a_v * a_v);
      return Point(x, y);
    }
};

// double get_distance(Point p1, Point p2)
// {
//     double l;
//     l = sqrt(pow(p2.x_ - p1.x_, 2) + pow(p2.y_ - p1.y_, 2));
//     return l;
// }


class TrackPoint : public Point
{

  public:
    TrackPoint() {}
    //可以这么用吗
    TrackPoint(Point p, double t, double c): Point(p), theta_(t), curvature_(c) {}
    TrackPoint(double x, double y, double t, double c) {
      x_ = x;
      y_ = y;
      theta_ = t;
      curvature_ = c;
    }
    
    double theta_;
    double curvature_;
};

class SegmentCenterPoint: public TrackPoint
{  public:
    double segment_length;
    double d_theta;
    int segment;
};

// struct mission_curve_with_length
// {
//   double length;
//   vector<TrackPoint> curve;
// };

class mission_curve_point: public TrackPoint
{
  public:
    mission_curve_point() {}
    //子类调用父类构造函数的用法
    mission_curve_point(double x, double y, double t, double c, double length): TrackPoint(x, y, t, c), length_(length) {}
    mission_curve_point(TrackPoint p, double length): TrackPoint(p), length_(length) {}
    double length_;
};

class Line
{
  public:
    Point p1_;//shorter x
    Point p2_;//higher x
    //y=ax+b
    double a;
    double b;
    double length;

    Line() {}
    Line(Point p1, Point p2)
    {
      if(p2.x_ > p1.x_)
      {
        p1_ = p1;
        p2_ = p2;
      }
      else
      {
        p1_ = p2;
        p2_ = p1;
      }
      a = (p2.y_ - p1.y_)/(p2.x_ - p1.x_);
      b = (p2.x_ * p1.y_ - p1.x_ * p2.y_)/(p2.x_ - p1.x_);
      length = sqrt(pow(p1_.x_ - p2_.x_, 2) + pow(p1_.y_ - p2_.y_, 2));
    }

    Point get_center()
    {
      return Point((p1_.x_ + p2_.x_) / 2, (p1_.y_ + p2_.y_) / 2);
    }
    

    //这样写真的可以嘛
    //以道路中心线为基准，生成车道线的几何形状，中心线以右为正，中心线以左为负。
    Line generate_line(double offset)
    {
        //道路是有方向的，以中心线为基准的左右和空间绝对左右不一定一致
        int k = 1;
        if(this -> a < 0)
            k = -1;

        Point p1;
        Point p2;
        double a = -1 / this -> a;
        // double b1 = center.p1_.y_ + center.p1_.x_ / center.a;
        // double b2 = center.p2_.y_ + center.p2_.x_ / center.a;
        p1.x_ = this -> p1_.x_ + k * offset / sqrt(1 + a * a);
        p1.y_ = this -> p1_.y_ + k * offset * a / sqrt(1 + a * a);
        p2.x_ = this -> p2_.x_ + k * offset / sqrt(1 + a * a);
        p2.y_ = this -> p2_.y_ + k * offset * a / sqrt(1 + a * a);

        Line line(p1, p2);
        return line;
    }
};

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

// geometry_msgs::Pose track_to_pose(TrackPoint point){
//   geometry_msgs::Pose pose;
//   pose.position.x = point.x_;
//   pose.position.y = point.y_;
//   pose.position.z = 0;
//   pose.orientation.x = 0;
//   pose.orientation.y = 0;
//   pose.orientation.z = sin(point.theta_/2);
//   pose.orientation.w = cos(point.theta_/2);
//   return pose;
// }

#endif