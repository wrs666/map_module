#include <vec_map.h>
#include <algorithm>
int Junction::junction_sequence = 0;
int Road::marker_id = 0;

//重载运算符，以拼接vector
template<typename T>
vector<T> operator+ (vector<T> a, vector<T> b)
{
  a.insert(a.end(), b.begin(), b.end());
  return a;
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

template <typename T>
bool in_range(T value, T border1, T border2)
{
  if(border1  > border2)
    return (border1 >= value && border2 <= value);
  else
    return (border2 >= value && border1 <= value);
}

// template <typename T>
// void swap(T &value1, T &value2)
// {
//   T store;
//   store = value1;
//   value1 = value2;
//   value2 = store;
// }


vector<string> split(const std::string& s, char delimiter) {
  vector<string> tokens;
  string token;
  istringstream tokenStream(s);
  while (getline(tokenStream, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}

Road::Road(vector<osm_point> point, keyvalue_arr props, UUID id)
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
  it = find_if(props.begin(), props.end(), props_find_key("divider"));
  divider_ = (*it).value;
  it = find_if(props.begin(), props.end(), props_find_key("highway"));
  highway_ = (*it).value;
  it = find_if(props.begin(), props.end(), props_find_key("lanes:backward"));
  lanes_backward_ = atoi((*it).value.c_str());
  it = find_if(props.begin(), props.end(), props_find_key("lanes:forward"));
  lanes_forward_ = atoi((*it).value.c_str());

  width_backward_.resize(0);
  width_forward_.resize(0);

  it = find_if(props.begin(), props.end(), props_find_key("width:lanes:backward"));
  arr_cache = (*it).value.c_str();
  string_arr = split(arr_cache, '|');
  for(lint i = 0; i < string_arr.size(); i++)
  {
    width_backward_.push_back(atof(string_arr[i].c_str()));
  }

  it = find_if(props.begin(), props.end(), props_find_key("width:lanes:forward"));
  arr_cache = (*it).value.c_str();
  string_arr = split(arr_cache, '|');
  for(lint i = 0; i < string_arr.size(); i++)
  {
    width_forward_.push_back(atof(string_arr[i].c_str()));
  }

}

void Road::road_add_vis_info(visualization_msgs::MarkerArray &map)
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
    line_list.scale.x = 0.15;

    visualization_msgs::Marker center = line_list;
    center.id = marker_id;
    center.color.r = 1.0f;
    center.color.g = 1.0f;
    center.color.a = 1.0;

    Line center_left = geometry_.generate_line(-0.15);
    Line center_right = geometry_.generate_line(0.15);
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
    marker_id++;

    //添加车道线
    visualization_msgs::Marker lane_line;
    lane_line = line_list;
    lane_line.id = marker_id;
  
    lane_line.color.r = 1.0f;
    lane_line.color.g = 1.0f;
    lane_line.color.b = 1.0f;
    lane_line.color.a = 1.0;

    //中心线以右
    double offset = 0.075;

    for(int i = 0; i < lanes_forward_; i++)
    {
      Line laneline = center_right.generate_line(offset + width_forward_[i]);
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

      Line laneline = center_left.generate_line(offset - width_backward_[i]);
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
    marker_id++;
  }
}

void Junction::junctions_add_vis_info(visualization_msgs::MarkerArray &junctions_map)
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

//得到某个任务点相对于道路中心线的offset
double vec_map::get_mission_point_offset(mission_point p, Road road)
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
      offset = offset - road.width_backward_[-i];
    }
    offset = offset + road.width_backward_[-p.lane_sequence - 1] / 2;
  }

  else
  {
    ROS_INFO("Wrong lane_sequence of mission_point: it can't be zero.");
    offset = 0;
  }

  cout<<"out function get_mission_point_offset"<<endl;
  return offset;
}

//地图上两点之间生成直线路径
vector<TrackPoint> vec_map::get_curve_line(osm_point p1, osm_point p2, double offset , double &layback)//
{
  cout << "this layback is : "<< layback << endl;
  //cout<<"in function get_curve_line"<<endl;
  //生成路径几何形状
  // cout<<"line p1 -- x1 :"<<p1.p.x_<<" y1 : "<<p1.p.y_<<endl;
  // cout<<"line p2 -- x2 :"<<p2.p.x_<<" y2 : "<<p2.p.y_<<endl;
  Line center_line(p1.p, p2.p);
  Line curve_line = center_line.generate_line(offset);
  vector<TrackPoint> curve_points;
  curve_points.resize(0);

  cout<<"line p1 -- x1 :"<<curve_line.p1_.x_<<" y1 : "<<curve_line.p1_.y_<<endl;
  cout<<"line p2 -- x2 :"<<curve_line.p2_.x_<<" y2 : "<<curve_line.p2_.y_<<endl;
  
  //采样
  int k =sign(p2.p.x_ - p1.p.x_);
  cout<<"kkkkkk : "<<k<<endl;
  //double x, y;
  TrackPoint p;
  double theta = atan(curve_line.a);
  if(k < 0)
    if(theta > 0)
      theta = theta - PI;
    else
      theta = theta + PI;

  double x_margin = point_margin * cos(atan(curve_line.a));
  double x_layback = layback * cos(atan(curve_line.a));
  cout <<"x_margin = " << x_margin << endl;
  for(double x = curve_line.p1_.x_ + k * (x_margin - x_layback); in_range(x, curve_line.p1_.x_, curve_line.p2_.x_); x = x + x_margin * k)
  {
    curve_points.emplace_back(x, curve_line.a * x + curve_line.b, theta, 0);
    mission_curve_with_length.emplace_back(x, curve_line.a * x + curve_line.b, theta, 0, curve_length);
    curve_length = curve_length + point_margin;
  }
  cout<<"Now the curve_length is accumulated to : "<<curve_length<<endl;
  double x_gap = curve_line.p2_.x_ - curve_points.back().x_;
  double y_gap = curve_line.p2_.y_ - curve_points.back().y_;
  layback = sqrt(pow(x_gap,2) + pow(y_gap, 2));
  cout << "next layback is : "<< layback << endl;
  //cout<<"out function get_curve_line"<<endl;
  return curve_points;  
}

//几何上两点之间五次样条插值出一段路径
vector<TrackPoint> vec_map::Section_Interploration(TrackPoint p1, TrackPoint p2, int kdxdy[], double &layback)
{ 
  //cout<<"in function Section_Interploration"<<endl;
  cout << "this layback is : "<< layback << endl;
  //double length = 0;
  vector<TrackPoint> curve_points;
  curve_points.resize(0);
  //curve_points.push_back(p1);

  double x1, y1, theta1, kappa1, x2, y2, theta2, kappa2;
  x1 = p1.x_;
  y1 = p1.y_;
  theta1 = p1.theta_;
  kappa1 = p1.curvature_;
  x2 = p2.x_;
  y2 = p2.y_;
  theta2 = p2.theta_;
  kappa2 = p2.curvature_;

  // if(kdxdy[1] * kdxdy[3] > 0 && kdxdy[0] * kdxdy[2] < 0)
  // {
  //   swap(x1, y1);
  //   swap(x2, y2);
  //   theta1 = atan(1/tan(theta1));
  //   theta2 = atan(1/tan(theta2));
  //   //todo: deal with curvature
  // }

  cout<<"x1, y1, theta1 : "<< x1 << " " << y1 << " " << theta1 <<endl;
  cout<<"x2, y2, theta2 : "<< x2 << " " << y2 << " " << theta2 <<endl;

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
  // B << y1, y2, tan(theta1), tan(theta2), kappa1 * pow((1 + pow(tan(theta1), 2)), 1.5), kappa2 * pow((1 + pow(tan(theta2), 2)), 1.5);
  B << y1, y2, tan(theta1), tan(theta2), 0, 0;
  X = A.lu().solve(B);
  cout<<"X : "<<X(0)<<" "<<X(1)<<" "<<X(2)<<" "<<X(3)<<" "<<X(4)<<" "<<X(5)<<endl;

  
  TrackPoint p;
  VectorXd x_power(6);
  VectorXd coef_0d(6);
  VectorXd coef_1d(6);
  VectorXd coef_2d(6);      
  
  coef_0d << X(0), X(1), X(2), X(3), X(4), X(5);
  coef_1d << X(1), 2 * X(2), 3 * X(3), 4 * X(4), 5 * X(5), 0;
  coef_2d << 2 * X(2), 6 * X(3), 12 * X(4), 20 * X(5), 0, 0;

  //验证连接处曲率是否连续
  VectorXd x1_power(6);
  VectorXd x2_power(6);
  VectorXd x1_series(6);
  VectorXd x2_series(6);
  x1_power << 1, x1, pow(x1, 2), pow(x1, 3), pow(x1, 4), pow(x1, 5);
  x2_power << 1, x2, pow(x2, 2), pow(x2, 3), pow(x2, 4), pow(x2, 5);
  x1_series << A(4,0), A(4,1), A(4,2), A(4,3), A(4,4), A(4,5);
  x2_series << A(5,0), A(5,1), A(5,2), A(5,3), A(5,4), A(5,5);
  // x1_series << A(0,0), A(0,1), A(0,2), A(0,3), A(0,4), A(0,5);
  // x2_series << A(1,0), A(1,1), A(1,2), A(1,3), A(1,4), A(1,5);
  // x1_series << A(2,0), A(2,1), A(2,2), A(2,3), A(2,4), A(2,5);
  // x2_series << A(3,0), A(3,1), A(3,2), A(3,3), A(3,4), A(3,5);


  double kappa_start = x1_power.dot(coef_2d)/pow((1+pow(x1_power.dot(coef_1d), 2)), 1.5); 
  double kappa_end = x2_power.dot(coef_2d)/pow((1+pow(x2_power.dot(coef_1d), 2)), 1.5);

  // double y1dd_gap = x1_series.dot(X);
  // double y2dd_gap = x2_series.dot(X);

  // cout<<"y1dd gap : "<< y1dd_gap<<endl;
  // cout<<"y2dd_gap : "<< y2dd_gap<<endl;

  // double kappa_start = x1_power.dot(coef_2d); 
  // double kappa_end = x2_power.dot(coef_2d);

  cout<<"kappa_start : "<< kappa_start<<endl;
  cout<<"kappa_end : "<<kappa_end<<endl;
  
  double x = x1;  

  int k =sign(x2 - x1);

  x = x + k * (point_margin - layback);

  double dlength = 0.01;
  double dx;

  
  while(in_range(x, x1, x2))
  {
    x_power << 1, x, pow(x, 2), pow(x, 3), pow(x, 4), pow(x, 5);
    p.x_ = x;
    p.y_ = x_power.dot(coef_0d);
    p.theta_ = atan(x_power.dot(coef_1d));
    p.curvature_ = x_power.dot(coef_2d)/pow((1+pow(x_power.dot(coef_1d), 2)), 1.5);

    if(kdxdy[1] * kdxdy[3] > 0 && kdxdy[0] * kdxdy[2] < 0)
    {
      swap(p.x_, p.y_);
      p.theta_ = atan(1/x_power.dot(coef_1d));
      //p.curvature_ = (-p.curvature_);
      //todo: deal with curvature
    }

    curve_points.push_back(p);
    mission_curve_with_length.emplace_back(p, curve_length);
    curve_length = curve_length + point_margin;

    for(int i = 0; i < 10; i++)
    {
      x_power << 1, x, pow(x, 2), pow(x, 3), pow(x, 4), pow(x, 5);
      dx = dlength / sqrt(1+pow(x_power.dot(coef_1d), 2));
      x = x + k * dx;
      if(!in_range(x, x1, x2))
      {
        break;
      }
        
    }

  }
 
  cout<<"Now the curve_length is accumulated to : "<<curve_length<<endl;

  double x_gap = p2.x_ - curve_points.back().x_;
  double y_gap = p2.y_ - curve_points.back().y_;
  layback = sqrt(pow(x_gap,2) + pow(y_gap, 2));
  cout << "next layback is : "<< layback << endl;
  //cout<<"out function Section_Interploration"<<endl;
  return curve_points;
}

vector<TrackPoint> vec_map::spline_interploration(TrackPoint p1, TrackPoint p2, double &layback)
{
  vector<TrackPoint> curve_points;
  curve_points.resize(0);
  //curve_points.push_back(p1);

  double x1, y1, theta1, kappa1, x2, y2, theta2, kappa2;
  x1 = p1.x_;
  y1 = p1.y_;
  theta1 = p1.theta_;
  kappa1 = p1.curvature_;
  x2 = p2.x_;
  y2 = p2.y_;
  theta2 = p2.theta_;
  kappa2 = p2.curvature_;

  cout<<"x1, y1, theta1 : "<< x1 << " " << y1 << " " << theta1 <<endl;
  cout<<"x2, y2, theta2 : "<< x2 << " " << y2 << " " << theta2 <<endl;
  
  double s_distance = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));

  cout<<"SSSSSSS : "<<s_distance<<endl;

  double vx1, vx2, vy1, vy2, ax1, ax2, ay1,ay2;
  vx1 = cos(theta1);
  vx2 = cos(theta2);
  vy1 = sin(theta1);
  vy2 = sin(theta2);
  ax1 = -sin(theta1) * kappa1;
  ax2 = -sin(theta2) * kappa2;
  ay1 = cos(theta1) * kappa1;
  ay2 = cos(theta2) * kappa2;

  double a[6];
  a[0] = x1;
  a[1] = vx1;
  a[2] = ax1 / 2;
  a[3] = (-3 * ax1 + ax2) / (2 * s_distance) + (-6 * vx1 - 4 * vx2) / pow(s_distance, 2) - 10 * (x1 -x2) / pow(s_distance, 3);
  a[4] = (1.5 * ax1 - ax2) / pow(s_distance, 2) + (8 * vx1 + 7 * vx2) / pow(s_distance, 3) + 15 * (x1 - x2) / pow(s_distance, 4);
  a[5] = (-ax1 + ax2) / (2 * pow(s_distance, 3)) - 3 * (vx1 + vx2) / pow(s_distance, 4) + (-6 * x1 + 6 * x2) / pow(s_distance, 5);

  double b[6];
  b[0] = y1;
  b[1] = vy1;
  b[2] = ay1 / 2;
  b[3] = (-3 * ay1 + ay2) / (2 * s_distance) + (-6 * vy1 - 4 * vy2) / pow(s_distance, 2) - 10 * (y1 -y2) / pow(s_distance, 3);
  b[4] = (1.5 * ay1 - ay2) / pow(s_distance, 2) + (8 * vy1 + 7 * vy2) / pow(s_distance, 3) + 15 * (y1 - y2) / pow(s_distance, 4);
  b[5] = (-ay1 + ay2) / (2 * pow(s_distance, 3)) - 3 * (vy1 + vy2) / pow(s_distance, 4) + (-6 * y1 + 6 * y2) / pow(s_distance, 5); 

  double kappa_start = (vx1 * ay1 - ax1 * vy1) / pow( (pow(vx1, 2) + pow(vy1, 2) ), 1.5);
  double kappa_end = (vx2 * ay2 - ax2 * vy2) / pow( (pow(vx2, 2) + pow(vy2, 2) ), 1.5);

  cout<<"kappa_start : "<< kappa_start<<endl;
  cout<<"kappa_end : "<<kappa_end<<endl;

  double s = point_margin - layback;
  TrackPoint p;
  double vx, vy, ax, ay, theta;

  while(s < s_distance)
  {
    VectorXd s_power(6);
    VectorXd coef_x(6);
    VectorXd coef_y(6);
    VectorXd coef_vx(6);
    VectorXd coef_vy(6);
    VectorXd coef_ax(6);
    VectorXd coef_ay(6);
      
    coef_x << a[0], a[1], a[2], a[3], a[4], a[5];
    coef_y << b[0], b[1], b[2], b[3], b[4], b[5];
    coef_vx << a[1], 2 * a[2], 3 * a[3], 4 * a[4], 5 * a[5], 0;
    coef_vy << b[1], 2 * b[2], 3 * b[3], 4 * b[4], 5 * b[5], 0;
    coef_ax << 2 * a[2], 6 * a[3], 12 * a[4], 20 * a[5], 0, 0;
    coef_ay << 2 * b[2], 6 * b[3], 12 * b[4], 20 * b[5], 0, 0;


    s_power << 1, s, pow(s, 2), pow(s, 3), pow(s, 4), pow(s, 5);
    
    p.x_ = coef_x.dot(s_power);
    p.y_ = coef_y.dot(s_power);
    vx = coef_vx.dot(s_power);
    vy = coef_vy.dot(s_power);
    ax = coef_ax.dot(s_power);
    ay = coef_ay.dot(s_power);

    theta = atan(vy / vx);

    if(vx < 0)
      if(vy < 0)
        p.theta_ = theta - PI;
      else
        p.theta_ = theta + PI;
    else
      p.theta_ = theta; 

    p.curvature_ = (vx * ay - ax * vy) / pow( (pow(vx, 2) + pow(vy, 2) ), 1.5);

    curve_points.push_back(p);
    curve_length = curve_length + point_margin;
    mission_curve_with_length.emplace_back(p, curve_length);

    s = s + point_margin;
  }
  
 
  cout<<"Now the curve_length is accumulated to : "<<curve_length<<endl;

  double x_gap = p2.x_ - curve_points.back().x_;
  double y_gap = p2.y_ - curve_points.back().y_;
  layback = sqrt(pow(x_gap,2) + pow(y_gap, 2));
  cout << "next layback is : "<< layback << endl;
  //cout<<"out function Section_Interploration"<<endl;
  return curve_points;

}

//vector<double> get

//地图上两点之间生成五次样条曲线
vector<TrackPoint> vec_map::get_curve_spline(osm_point p1, osm_point p2, double offset1, double offset2, int kdxdy[], double a1, double a2, double &layback)
{
  //cout<<"in function get_curve_spline"<<endl;
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

  //cout<<"out function get_curve_spline"<<endl;
  //还真可以这么写？
  double theta1, theta2;
  if(kdxdy[0] < 0)
    if(a1 < 0)
      theta1 = PI + atan(a1);
    else
      theta1 = atan(a1) - PI;
  else
    theta1 = atan(a1);

  if(kdxdy[2] < 0)
    if(a2 < 0)
      theta2 = PI + atan(a2);
    else
      theta2 = atan(a2) - PI;
  else
    theta2 = atan(a2);
  //return Section_Interploration(TrackPoint(x1, y1, theta1, 0), TrackPoint(x2, y2, theta2, 0), kdxdy, layback);
  return spline_interploration(TrackPoint(x1, y1, theta1, 0), TrackPoint(x2, y2, theta2, 0), layback);

}



//两个任务点之间生成一段路径
vector<TrackPoint> vec_map::get_curve_between_mission(mission_point p1, mission_point p2, double &layback)
{

  int k_dx_dy[4];
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
          k_dx_dy[0] = sign(junction_point1.p.x_ - p1.p.x_);
          k_dx_dy[1] = sign(junction_point1.p.y_ - p1.p.y_);
          k_dx_dy[2] = sign(p2.p.x_ - junction_point2.p.x_);
          k_dx_dy[3] = sign(p2.p.y_ - junction_point2.p.y_);
          raw_curve_section = get_curve_line(p1, junction_point1, offset1, layback);
          raw_curve_section = raw_curve_section + get_curve_spline(junction_point1, junction_point2, offset1, offset2, k_dx_dy, a1, a2, layback);
          raw_curve_section = raw_curve_section + get_curve_line(junction_point2, p2, offset2, layback);
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



//assign values to global points and mission points,
//construct junctions;
void vec_map::resolve_points()
{
  int points_num = raw_map.points.size();
  for(int i = 0; i < points_num; i++)
  {
    geographic_msgs::WayPoint way_point = raw_map.points[i];
    geodesy::UTMPoint utm_point;
    geodesy::fromMsg(way_point.position, utm_point);
    // cout<<"utm_point.easting: "<<utm_point.easting<<endl;
    // cout<<"utm_point.northing: "<<utm_point.northing<<endl;
    // cout<<"******************************"<<endl;
    Point p(utm_point.easting - 284000, utm_point.northing - 3793000);
    cout<<"information of points: x: "<<p.x_<<"   "<<"y: "<<p.y_<<endl;
    geodesy::fromMsg(way_point.position, utm_point);
    //osm_point op(way_point.id.uuid, p, way_point.props);
    global_points.emplace_back(way_point.id.uuid, p, way_point.props);
    auto iter = find_if(way_point.props.begin(), way_point.props.end(), props_find_key("mission"));
    if(iter != way_point.props.end())
    {
      ROS_INFO("Find mission point. ");
      auto iter_lane = find_if(way_point.props.begin(), way_point.props.end(), props_find_key("lane_sequence"));
      //没有指定lane_sequence的mission点不被解析
      if(iter_lane != way_point.props.end())
      {
        int index = atoi((iter_lane -> value).c_str());        
        mission_points.emplace(atoi((*iter).value.c_str()), mission_point(way_point.id.uuid, p, way_point.props, index));
        cout<<"**mission point info mission: "<< atoi((*iter).value.c_str())<< " lane : "<<index<<endl;
        ROS_INFO("Got mission point. ");
      }

    }
  
    iter = find_if(way_point.props.begin(), way_point.props.end(), props_find_key("junction"));
    if(iter != way_point.props.end())
    {
      Junction junction(osm_point(way_point.id.uuid, p, way_point.props));
      junction.junctions_add_vis_info(junctions_map);//填充路口可视化的消息类型
      junctions.push_back(junction);
    }
    
  

  }

  for(auto iter = mission_points.begin(); iter != mission_points.end(); iter++)
  {
    cout<<"mission: "<<(*iter).first<<"  x: "<<(*iter).second.p.x_<<endl;
  }

  ROS_INFO("FINISH point resolved.");
}

//assign values to roads
void vec_map::resolve_roads()
{
  //根据feature构建道路
  //todo 将任务点，路口点读进Road
  int feature_number = raw_map.features.size();
  for(int j = 0; j < feature_number; j++)
  {
    //找到相关的点
    vector<osm_point> road_point;
    road_point.reserve(raw_map.features[j].components.size());
    road_point.resize(0);

    for(lint l = 0; l < raw_map.features[j].components.size(); l++)
    {
      UUID id = raw_map.features[j].components[l].uuid;
      auto iter = find_if(global_points.begin(), global_points.end(), [id](osm_point &p) { return (p.id == id);});
      if(iter != global_points.end())
        road_point.push_back(*iter);
      else
        ROS_INFO("WRONG! Found point in road but not in global map");
    
    }
  
    if(raw_map.features[j].props.size() > 0)
    {
      Road road(road_point, raw_map.features[j].props, raw_map.features[j].id.uuid);
      road.road_add_vis_info(road_map);
      roads.push_back(road);
    }
  } 

  ROS_INFO("FINISH roads resolved.");
}
    
void vec_map::resolve_junctions()
{
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
}

    

//由任务点(key == mission)得到路径
//在任务点列表中，依次两点之间生成路径
vector<TrackPoint> vec_map::generate_mission_curve()
{
  mission_curve_with_length.resize(0);
  curve_length = 0;
  cout<<"in function generate_mission_curve"<<endl;
  vector<TrackPoint> raw_curve;
  vector<TrackPoint> raw_curve_section;
  raw_curve.resize(0);
  int n = 0;
  double lay_back = 0;
  //按顺序依次将每两个任务点之间的路径点填充到路径中
  for(auto iter = mission_points.begin(); next(iter) != mission_points.end(); iter++)
  {
    n++;
    cout<<"add path "<<n<<" th times."<<endl;
    raw_curve_section.resize(0);
    raw_curve_section = get_curve_between_mission((*iter).second, (*next(iter)).second, lay_back);
    cout<<"finished one path generation."<<endl;
    raw_curve.insert(raw_curve.end(), raw_curve_section.begin(), raw_curve_section.end());
    cout<<"finished insert this path."<<endl;
  }
  
  osm_point goal_point = mission_points.at(mission_points.size() - 1);

  vector<Road>::iterator iter_goal_road = find_if(roads.begin(), roads.end(), roads_find_id(goal_point.id));

  goal_pose.x_ = goal_point.p.x_;
  goal_pose.y_ = goal_point.p.y_;
  goal_pose.theta_ = atan((*iter_goal_road).geometry_.a);
  goal_pose.curvature_ = 0;

  curve_length = curve_length - point_margin;

  cout<<"out function generate_mission_curve"<<endl;
  ROS_INFO("FINISH mission_curve resolved");
  return raw_curve;
}

void vec_map::mission_curve_add_vis_info()
{
  //将路径信息转换成可被rviz可视化的消息类型
  geometry_msgs::PoseStamped posestamped;
  visual_mission_curve.poses.resize(0);
  visual_mission_curve.header.frame_id = "world";
  visual_mission_curve.header.seq = 0;
  visual_mission_curve.header.stamp = ros::Time::now();
  for (auto point : mission_curve) {
      //static_curve_srv.request.static_curve.points.push_back(to_curvepoint(point));
    posestamped.header.frame_id = "world";
    posestamped.pose = track_to_pose(point);
    visual_mission_curve.poses.push_back(posestamped);
  }
}

void vec_map::store_curve_in_csv()
{
  ofstream outFile;  
  outFile.open("/home/wrs/map/src/map_module/data/mission_curve.csv", ios::out);
  for(auto point : mission_curve)
  {
    outFile << point.x_ << ',' << point.y_ << ',' << point.theta_ << ',' << point.curvature_ << endl;  
  }
  outFile.close(); 
}


void vec_map::map_visualization_pub()
{
  roads_vis_pub.publish(road_map);
  mission_curve_vis_pub.publish(visual_mission_curve);
  junctions_vis_pub.publish(junctions_map);
}

void vec_map::resolve_map(ros::NodeHandle nh)
{
  roads_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("roads_vis", 1);
  junctions_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("junction_vis", 1);
  mission_curve_vis_pub = nh.advertise<nav_msgs::Path>("mission_curve_vis", 1);

  road_map.markers.resize(0);
  junctions_map.markers.resize(0);
  ros::service::waitForService("get_geographic_map");
  ros::ServiceClient osm_client;
  osm_client = nh.serviceClient<geographic_msgs::GetGeographicMap>("get_geographic_map");
  geographic_msgs::GetGeographicMap osm_srv;
  osm_srv.request.url = map_path;
  if (!osm_client.call(osm_srv)) {
    ROS_ERROR_STREAM("error in map file " << map_path << " reading.");
  }

  raw_map = osm_srv.response.map;

  //地图数据元素
  //获取全局点列表
  resolve_points();

  //根据feature构建道路
  //todo 将任务点，路口点读进Road
  resolve_roads();
  
  //构建junctuon
  //对每个junction，添加与其相连的junction_point
  resolve_junctions();

  //由任务点(key == mission)得到路径
  //在任务点列表中，依次两点之间生成路径
  mission_curve = generate_mission_curve();
  //correct_mission_curve();
  mission_curve_add_vis_info();
  ROS_INFO("FINISH vec_map resolved");
  store_curve_in_csv();
  ROS_INFO("curve SAVED!");
}

// void vec_map::correct_mission_curve()
// {
//   int n1 = mission_curve.size();
//   cout<<"mission_curve size: "<<n1<<endl;
//   for(int n = 0; n < n1 - 1; n++)
//   {
//     if(mission_curve[n].x_ > mission_curve[n+1].x_)
//       if(mission_curve[n].theta_ > 0)
//         mission_curve[n].theta_ = mission_curve[n].theta_ - PI;
//       else if(mission_curve[n].theta_ < 0)
//         mission_curve[n].theta_ = mission_curve[n].theta_ + PI;
//   }
//   mission_curve.pop_back();

//   int n2 = mission_curve_with_length.size();
//   cout<<"mission_curve_with_length size: "<<n2<<endl;
//   for(int n = 0; n < n2 - 1; n++)
//   {
//     if(mission_curve_with_length[n].x_ > mission_curve_with_length[n+1].x_)
//       if(mission_curve_with_length[n].theta_ > 0)
//         mission_curve_with_length[n].theta_ = PI - abs(mission_curve_with_length[n].theta_);
//       else if(mission_curve_with_length[n].theta_ < 0)
//         mission_curve_with_length[n].theta_ = mission_curve_with_length[n].theta_ + PI;
//   }
//   mission_curve_with_length.pop_back();
//}


// int main(int argc, char** argv)
// {
//     ROS_INFO("INTO main point now");
//     ros::init(argc, argv, "map_io_node");
//     ros::NodeHandle nh;
//     vec_map map(osm_map_url, nh);
//     //可视化
//     map.store_curve_in_csv();
//     ros::Rate loop_rate(1);
    
//     while(ros::ok())
//     {
//       map.map_visualization_pub();
//       loop_rate.sleep();
//       ros::spinOnce();
//     }

// }

/*
TODO:
对于osm数据的检查
路口的可视化用文字代替
统一仿函数：
    仿函数整理（抽象出最少最适用的）
    能用仿函数的地方用仿函数
vec_map类返回元素原始数据的方法
fix Road:: props_find_key, generate_line
函数直接访问类成员，不用返回值来回赋值

vec_map 应该是原始数据类型，在map_server中填充可视化信息
如果vec_map中的信息是可视化消息类型的，不方便去进行扩展
可以在vec_map中添加可视化的方法，减少与server的耦合，但实际的操作肯定还是要给map_server；
*/