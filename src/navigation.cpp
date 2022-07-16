#include <vec_map.h>
#include <float.h>

typedef vector<Junction>::iterator junc_iter;
typedef vector<Road>::iterator road_iter;

double length_of_lane_change = 15;
double fixed_turn_length = 30;
double max_road_width = 20;
double overflowing_road_range = 10;

//vector<double> border_junction = {2.5, 2.5};

double check_theta(double t)
{
  while(abs(t) > PI)
  {
    if(t > PI)
      t = t - 2 * PI;
    else if(t < -PI)
      t = t + 2 * PI;
  }
  return t;
}

double get_distance(Point p1, Point p2)
{
  double l;
  l = sqrt(pow(p2.x_ - p1.x_, 2) + pow(p2.y_ - p1.y_, 2));
  return l;
}

//由道路斜率和横向方向得到车的世界坐标系下朝向
double adjust_theta(int direction, double a)
{
  if(direction == -1)
    if(atan(a) < 0)
      return (PI + atan(a));
    else
      return (atan(a) - PI);
  else
    return atan(a); 
}

road_pos_info vec_map::get_last_junction_point(vector<Junction>::iterator iter_j, road_pos_info goal)
{
  road_pos_info instance;
  instance.road = roads.end();
  if(iter_j->junction_points.size() == 0)
  {
    ROS_ERROR("A start last junction has no junction point");
    return instance;
  }
  for(auto iter_jp = iter_j->junction_points.begin(); iter_jp != iter_j->junction_points.end(); iter_jp++)
  {
    for(auto iter = goal.road->junction_points.begin(); iter !=goal.road->junction_points.end(); iter++)
    {
      if((*iter_jp) == (*iter))
      {
        instance = goal;
        instance.center_projection = iter_jp->p;
        int n = goal.road->geometry_list.size();
        if(goal.direction == 1)
          instance.segment = 0;
        else
          instance.segment = n - 1;
        return instance;
      }
    }
  }
  return instance;
}

//根据下一个路口，找到对应的路口点
road_pos_info vec_map::get_guided_junction_point(vector<Junction>::iterator iter1, vector<Junction>::iterator iter2)
{
  road_pos_info instance;
  instance.road = roads.end();
  vector<osm_point>::iterator iter_rjp;
  if(iter1->junction_points.size() == 0)
  {
    ROS_ERROR("get_guided_junction_point failed, no junction point");
    return instance;
  }
    
  for(auto iter = iter1->junction_points.begin(); iter != iter1->junction_points.end(); iter++)
  {
    auto iter_road = roads.begin();
    while(find_if(iter_road, roads.end(), roads_find_id(iter->id)) != roads.end())
    {
      iter_road = find_if(iter_road, roads.end(), roads_find_id(iter->id));
      if(iter_road != roads.end())
      {
      //   if(iter_road->junction_points.size() == 0)
      //     return instance;
        for(auto iter_jp = iter_road->junction_points.begin(); iter_jp != iter_road->junction_points.end(); iter_jp++)
        {
          auto iter_j2 = find_if(junctions.begin(), junctions.end(), junction_find_point((*iter_jp)));
          if(iter_j2 == iter2)
          {
            instance.road = iter_road;
            instance.center_projection = iter->p;
            instance.junction_point = iter_jp;
            instance.direction = sign(iter->p.x_ - iter1->junction.p.x_);
            if(instance.direction > 0)
            {
              instance.segment = 0;
              instance.lane_sequence = ceil((double)iter_road->lanes_forward_ / 2);
            }
              
            else
            {
              instance.segment = iter_road->geometry_list.size() - 1;
              instance.lane_sequence = -ceil((double)iter_road->lanes_backward_ / 2);
            }
          
            return instance;
          }
        }
      }
      iter_road++;
    }
  }
  ROS_ERROR("No junction point found guided to next junction");
  return instance;
}

geometry_msgs::Pose vec_map::info_to_pose(road_pos_info pos)
{
  geometry_msgs::Pose pose;
  double offset = pos.road->get_mission_point_offset(pos.lane_sequence); 
  Point p = pos.center_projection.generate_Point(pos.road->geometry_list[pos.segment].a, offset);
  pose.position.x = p.x_;
  pose.position.y = p.y_;
  pose.position.z = 0;
  double theta = adjust_theta(pos.direction, pos.road->geometry_list[pos.segment].a);
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = sin(theta/2);
  pose.orientation.w = cos(theta/2);
  return pose;
}

road_pos_info vec_map::get_road_pos_info(geometry_msgs::PoseStamped pose)
{
  double x = pose.pose.position.x;
  double y = pose.pose.position.y;
  road_pos_info instance;
  instance.road = roads.end();
  instance.segment = -1; 
  instance.lane_sequence = 0;
  double min_distance = max_road_width;
  double current_distance;

  //填充road; segment
  road_iter iter;
  iter = roads.begin();
  while(iter != roads.end())
  {
    if(iter->junction_path == true)
    {
      iter++;
      continue;
    }
    for(int i = 0; i < iter -> geometry_list.size(); i++)
    {
      current_distance = abs(iter -> geometry_list[i].a * x - y + iter -> geometry_list[i].b) / sqrt(1 + pow(iter -> geometry_list[i].a, 2));
      if(current_distance < min_distance)
      {
        if(in_range(x, iter -> geometry_list[i].p1_.x_, iter -> geometry_list[i].p2_.x_))
        {
          min_distance = current_distance;
          instance.road = iter;
          instance.segment = i;
        }  
      }
    }
    iter++;
  }  

  if(instance.road == roads.end())
  {
    while(iter != roads.end())
    {
      if(iter->junction_path == false)
      {
        for(int i = 0; i < iter -> geometry_list.size(); i++)
        {
          current_distance = abs(iter -> geometry_list[i].a * x - y + iter -> geometry_list[i].b) / sqrt(1 + pow(iter -> geometry_list[i].a, 2));
          if(current_distance < min_distance)
          {
            if(in_range(x, iter -> geometry_list[0].p1_.x_, iter -> geometry_list[0].p1_.x_ - overflowing_road_range))
            {
              instance.road = iter;
              instance.segment = 0;
            }
            else if(in_range(x, iter -> geometry_list.back().p2_.x_, iter -> geometry_list.back().p2_.x_ + overflowing_road_range))
            {
              instance.road = iter;
              instance.segment = iter -> geometry_list.size() - 1;
            }

          }
        }
      }
      iter++;
    }
  }


  
  //没在路上直接返回
  if(instance.road == roads.end())
  {
    ROS_ERROR("position is not on the road network");
    return instance;
  }

  //填充direction
  double cos_pos_theta = pow(pose.pose.orientation.w, 2) - pow(pose.pose.orientation.z, 2);
  instance.direction = sign(cos_pos_theta);

  //判断方向是否正确
  Line road_line = instance.road->geometry_list[instance.segment];
  if(instance.direction * (road_line.a * x + road_line.b - y) < 0)
  {
    ROS_ERROR("direction of pose set is wrong.");
    instance.segment = -1;
    return instance;
  }
 
  //填充junction_point
  instance.junction_point = instance.road -> junction_points.end();
  if(instance.road -> junction_points.size() == 0)
  {
    ROS_ERROR("pose is on an road with no junction point");
    return instance;
  }
     
  for(vector<osm_point>::iterator iter = instance.road -> junction_points.begin(); iter != instance.road -> junction_points.end(); iter++)
  {
    if(cos_pos_theta * (iter -> p.x_ - pose.pose.position.x) > 0)
    {
      instance.junction_point = iter;
      break;
    }   
  }
  
  //填充lane_sequence
  int k = sign(instance.road -> geometry_list[instance.segment].a * x + instance.road -> geometry_list[instance.segment].b - y);
  int n = 0;
  double dist = 0;
  while(dist < min_distance)
  {
    n++;
    if(k > 0)
      if(n <= instance.road -> width_forward_.size())
        dist = dist + instance.road -> width_forward_[n - 1];
      else
      { 
        ROS_WARN("position is out of range of the road width.");
        n = instance.road -> width_forward_.size();
        break;
      }

    else
    {
      if(n <= instance.road -> width_backward_.size())
        dist = dist + instance.road -> width_backward_[n - 1];
      else
      { 
        ROS_WARN("position is out of range of the road width.");
        n = instance.road -> width_backward_.size();
        break;
      }
    } 
  }
  instance.lane_sequence = k * n;

  //填充center_projection 
  instance.center_projection.x_ = (instance.road -> geometry_list[instance.segment].a * (y - instance.road -> geometry_list[instance.segment].b) + x) / (pow(instance.road -> geometry_list[instance.segment].a, 2) + 1);
  instance.center_projection.y_ = (instance.road -> geometry_list[instance.segment].a * instance.center_projection.x_ + instance.road -> geometry_list[instance.segment].b);
  std::cout<<"Center projection  X:"<<instance.center_projection.x_<<"  Y:"<<instance.center_projection.y_<<endl;
  return instance;

}

vector<Junction>::iterator vec_map::nav_get_first_junction(geometry_msgs::PoseStamped pose)
{
  road_pos_info pos;
  pos = get_road_pos_info(pose);
  if(pos.road == roads.end())
  {
    ROS_ERROR("Current position is not on the road network, could not find guide junction");
    return junctions.end();
  }
  else if(pos.junction_point == pos .road ->junction_points.end())
  {
    ROS_ERROR("Get Wrong guided junction point, could not generated navigation path");
    return(junctions.end());
  }
  else
  {
    vector<Junction>::iterator iter = find_if(junctions.begin(), junctions.end(), junction_find_point(*(pos.junction_point)));
    if(iter != junctions.end())
    {
      ROS_INFO("Found guided junction");
      return iter;
    }
    else
    {
      ROS_ERROR("NO guided junction FOUND! The guided junction point links to nowhere");
      return(junctions.end());
    }
  }
}

vector<Junction>::iterator vec_map::nav_get_last_junction(geometry_msgs::PoseStamped pose)
{
  road_pos_info pos;
  pos = get_road_pos_info(pose);
  if(pos.road == roads.end())
  {
    ROS_ERROR("Goal position is not on the road network, navigation failed");
    return junctions.end();
  }
  else 
  {
    vector<osm_point>::iterator junction_point = (pos.road) -> junction_points.end();
    //寻找连接终点的路口点
    if(pos.road -> junction_points.size() == 0)
    {
      ROS_ERROR("Goal is on a road with no junction point, navigation failed");
      return junctions.end();
    }
    for(auto iter =  pos.road->junction_points.begin(); iter != pos.road -> junction_points.end(); iter++)
    {
      if(iter != pos.junction_point)
        junction_point = iter;
    }
    if(junction_point == (pos.road) -> junction_points.end())
    {
      ROS_ERROR("No junction point linked to pose, navigation failed");
      return junctions.end();
    }
    else
    {
      vector<Junction>::iterator iter_junc = find_if(junctions.begin(), junctions.end(), junction_find_point(*junction_point));//////////????????
      if(iter_junc != junctions.end())
      {
        ROS_INFO("Found last junction");
        return iter_junc;
      }
      else
      {
        ROS_ERROR("NO junction linked to goal FOUND! navigation failed");
        return(junctions.end());
      }
    }
  }
}

vector<vector<Junction>::iterator>::iterator vec_map::get_min_F_node(vector<vector<Junction>::iterator> &open_list)
{
  vector<junc_iter>::iterator instance = open_list.end();
  double min_F = DBL_MAX;
  int n = open_list.size();
  if(n == 0)
  {
    ROS_INFO("openlist is empty");
    return open_list.end();
  }
  else
  {
    for(auto iter = open_list.begin(); iter != open_list.end(); iter++)
    {
      (*iter) -> astar_F = (*iter) -> astar_G + (*iter) -> astar_H;
      if((*iter) -> astar_F < min_F)
      {
        min_F = (*iter) -> astar_F;
        instance =iter;
      }
      
    }
  }
  return instance;
}


bool vec_map::astar_navigation(vector<Junction>::iterator &start, vector<Junction>::iterator &goal)
{
  if(start == junctions.end() || goal == junctions.end())
  {
    ROS_ERROR("Invalid current pose or goal pose");
    return false;
  }
  vector<junc_iter> openlist;
  vector<junc_iter> closelist;
  openlist.resize(0);
  closelist.resize(0);  
  vector<junc_iter>::iterator A;
  //初始化
  for(auto iter = junctions.begin(); iter != junctions.end(); iter++)
  {
    iter -> astar_H = sqrt(pow((iter -> junction.p.x_ - goal -> junction.p.x_), 2) + pow(iter -> junction.p.y_ - goal -> junction.p.y_, 2));
    iter -> parent = junctions.end();
    iter -> astar_G = DBL_MAX;
    //iter -> astar_F = iter -> astar_G + iter -> astar_H;
  }
  start -> astar_G = 0;
  //start -> astar_F = start -> astar_G + start -> astar_H;
  openlist.push_back(start);
  ROS_INFO("A* start");
  junc_iter it;
  while(openlist.size() != 0)
  {
    A = get_min_F_node(openlist);    
    
    if(A == openlist.end())
    {
      std::cout<<"Can not get min F node from openlist."<<endl;
      break;
    }

    it = (*A);

    openlist.erase(A);
    
    if(it == goal)
    {
      ROS_INFO("FINISH sorting junctions in Astar.");
      return true;
    }

    if(it->edge.size() == 0)
    {
      ROS_ERROR("a node with no edge in A*, Wrong");
      return false;
    }

    if(it->edge.size() > 10)
    {
      std::cout<<"numbers of neighbor junction: "<<it->edge.size()<<endl;
      std::cout<<"junction position x : "<<it->junction.p.x_ <<endl;
      ROS_ERROR("edge resolved failed, to much edges");
      return false;
    }
    
    
    junc_iter neighbor;
    for(auto iter = it -> edge.begin(); iter != it -> edge.end(); iter++)
    {
      neighbor = iter -> second;
      if(find(closelist.begin(), closelist.end(), neighbor) == closelist.end())
      {  
        if(neighbor -> astar_G > it -> astar_G + iter -> first)
        {
          neighbor -> parent = it;
          neighbor -> astar_G = it -> astar_G + iter -> first;
        }
        openlist.push_back(neighbor);
      }
    }
    closelist.push_back(it);
  }
  ROS_ERROR("Astar failed, please check whether the goal is reachable.");
  return false;
}

//vector从last junction反向排序到first junction
vector<vector<Junction>::iterator> vec_map::build_junctions_queue(vector<Junction>::iterator goal)
{
  vector<junc_iter> astar_junc;
  astar_junc.resize(0);
  junc_iter iter = goal;
  while(iter != junctions.end())
  {
    astar_junc.push_back(iter);
    iter = iter -> parent;
  } 
  return astar_junc;
}

vector<int> vec_map::build_lane_turn(vector<Point> points)
{
  vector<int> lane_turn;
  lane_turn.resize(0);
  int size = points.size();
  Point p0, p1, p2;
  int k;
  double dx1, dx2, dy1, dy2, cos_theta;
  for(int i = 1; i < (size - 1); i++)
  {
    p0 = points[i - 1];
    p1 = points[i];
    Line current_line(p0, p1);
    p2 = points[i + 1];
    dx1 = p1.x_ - p0.x_;
    k = sign(dx1);
    dx2 = p2.x_ - p1.x_;
    dy1 = p1.y_ - p0.y_;
    dy2 = p2.y_ - p1.y_;
    cos_theta = ((dx1 * dx2) + (dy1 * dy2)) / sqrt((pow(dx1, 2) + pow(dy1, 2)) * (pow(dx2, 2) + pow(dy2, 2))); 
    if(cos_theta > 0.5)
    {
      //直行
      lane_turn.push_back(0);
    }
    else if(k * (p2.y_ - current_line.a * p2.x_ - current_line.b) > 0)
      lane_turn.push_back(1);//左转
    else
      lane_turn.push_back(-1);//右转
  }
  return lane_turn;
}


double distance_to_junction_point(road_pos_info pos, osm_point junc_p)
{
  double distance = 0;
  int geo_size = pos.road->geometry_list.size();
  int segment  = pos.segment;
  int direction = sign(junc_p.p.x_ - pos.center_projection.x_);    

  if(direction > 0)
    distance = get_distance(pos.center_projection, pos.road->geometry_list[segment].p2_);
  else
    distance = get_distance(pos.center_projection, pos.road->geometry_list[segment].p1_);
  
  
  while (in_range(segment, 0, geo_size - 1))
  {
    distance = distance + pos.road->geometry_list[segment].length;
    segment = segment + direction;
  }
  return distance;
}

//index: 当前车所在的车道序号
//返回值是从当前到路口点的行驶顺序
vector<SegmentCenterPoint> get_center_points(road_pos_info start, Point end, int index)
{
  vector<SegmentCenterPoint> instance;
  instance.resize(0);
  int direction = sign(end.x_ - start.center_projection.x_);
  int segment  = start.segment;
  int geo_size = start.road->geometry_list.size();
  SegmentCenterPoint scp;
  double offset = start.road->get_mission_point_offset(index);

  Line line;

  while (in_range(segment, 0, geo_size - 1))
  { 
    line = start.road->geometry_list[segment].generate_line(offset);
    scp.x_ = line.get_center().x_;
    scp.y_ = line.get_center().y_;
    scp.theta_ = adjust_theta(direction, start.road->geometry_list[segment].a);
    scp.segment = segment;

    scp.segment_length = start.road->geometry_list[segment].length;
    instance.push_back(scp);

    segment = segment + direction;
  }
  int p_size = instance.size();
  instance[0].curvature_ = 0;
  instance[0].d_theta = 0;
  instance[p_size - 1].curvature_ = 0;
  if(p_size > 1)
  {
    for(int i = 1; i < p_size; i++)
    {
      instance[i].d_theta = instance[i].theta_ - instance[i - 1].theta_;
      instance[i].curvature_ = instance[i].d_theta / get_distance(instance[i], instance[i - 1]); 
    }   
    instance[p_size - 1].curvature_ = 0; 
  }
  
  return instance;
}

//center_poits就是按车到路口点驾驶顺序的
vector<map_module::curvepoint> vec_map::just_go(vector<SegmentCenterPoint> center_points, road_pos_info start, Point end, double &layback, int turn)
{
  vector<map_module::curvepoint> curve;
  curve.resize(0);
  vector<double> border;
  double width;
  if(start.lane_sequence > 0)
    width = start.road ->width_forward_[start.lane_sequence - 1];
  else
    width = start.road ->width_backward_[-start.lane_sequence - 1]; 
  border.push_back(width / 2);
  border.push_back(width / 2);
  Point p = start.center_projection.generate_Point(start.road ->geometry_list[start.segment].a, start.road->get_mission_point_offset(start.lane_sequence));
  if((end.x_ - p.x_) * (center_points.front().x_ - p.x_) > 0) 
    curve = get_curve_line(p, center_points.front(), 0, border, layback);
  
  
  int n = center_points.size();
  if(n > 1)
    for(int i = 0; i < n - 1; i++)
      curve = curve + spline_interploration(center_points[i], center_points[i + 1], border, layback);
  //Point end_in_lane;
  int end_segment;
  for(int i = 0; i < start.road->geometry_list.size(); i++)
  {
    if(in_range(end.x_, start.road->geometry_list[i].p1_.x_, start.road->geometry_list[i].p2_.x_))
    {
      end_segment = i;
      break;
    }
      
  }
 
  Point end_in_lane;
  if(start.junction_point->p.x_ == end.x_)
  {  
    int direction = sign(end.x_ - start.center_projection.x_);
    int lane_seq;
    if(turn == 0)
      if(direction > 0)
        lane_seq = ceil((double)start.road->lanes_forward_ / 2);
      else
        lane_seq = -ceil((double)start.road->lanes_backward_ / 2); 
    else if(turn == 1)
      lane_seq = direction;
    else if(turn == -1)
      if(direction > 0)
        lane_seq = start.road->lanes_forward_;
      else
        lane_seq = -start.road->lanes_backward_; 
    else
      lane_seq = start.lane_sequence;
    
    end_in_lane = end.generate_Point(start.road->geometry_list[end_segment].a, start.road->get_mission_point_offset(lane_seq));
  }
  else
    end_in_lane = end.generate_Point(start.road->geometry_list[end_segment].a, start.road->get_mission_point_offset(start.lane_sequence));
  
  curve = curve + get_curve_line(center_points.back(), end_in_lane, 0, border, layback);
  return curve;
}

//center_poits就是按车到路口点驾驶顺序的
//反向用不到adjust_points
void adjust_points(vector<SegmentCenterPoint> &center_points, road_pos_info start, Point end, int turn, int current_lane)
{
  //center_points顺序方向
  int direction;
  if(center_points.size() == 1)
    direction = start.direction;
  else if (center_points.size() < 1)
  {
    ROS_ERROR("could not adjust center points when it is empty");
    return;    
  }
  else
    direction = sign(center_points.back().x_ - center_points.front().x_);
  int n = center_points.size();
  double length_sum = 0;
  double last_length_sum = 0;
  int index1 = -1, index2 = -1;
  SegmentCenterPoint adjust_point1, adjust_point2;
  for(int i = n - 1; i >=0; i--)
  {
    length_sum = last_length_sum + center_points[i].segment_length;
    if(length_sum > fixed_turn_length && last_length_sum < fixed_turn_length)
    {
      double gap = fixed_turn_length - last_length_sum;
      Line line, adjust_line;
      line = start.road->geometry_list[center_points[i].segment];
      double offset = 0;
      if(turn == 1)
        offset = start.road->get_mission_point_offset(direction);
      else if(turn == -1)
        if(direction > 0) 
          offset = start.road->get_mission_point_offset(start.road->lanes_forward_);
        else
          offset = start.road->get_mission_point_offset(-start.road->lanes_backward_);
      else
        if(direction > 0) 
          offset = start.road->get_mission_point_offset(ceil((double)start.road->lanes_forward_ / 2));
        else
          offset = start.road->get_mission_point_offset(-ceil((double)start.road->lanes_backward_ / 2));

      
      adjust_line = line.generate_line(offset);
      if(direction == 1)
        adjust_point1.x_ = line.p2_.x_ - gap * cos(atan(line.a));
      else
        adjust_point1.x_ = line.p1_.x_ + gap * cos(atan(line.a));
      adjust_point1.y_ = adjust_line.a * adjust_point1.x_ + adjust_line.b;
      adjust_point1.theta_ = adjust_theta(direction, line.a);
      adjust_point1.segment = center_points[i].segment;
      
      index1 = i;
    }

    if(length_sum > fixed_turn_length + length_of_lane_change && last_length_sum < fixed_turn_length + length_of_lane_change)
    {
      double gap = fixed_turn_length + length_of_lane_change - last_length_sum;
      Line line2, adjust_line2;
      line2 = start.road->geometry_list[center_points[i].segment];
      double offset2 = 0;
      
   
      offset2 = start.road->get_mission_point_offset(current_lane);

      adjust_line2 = line2.generate_line(offset2);
      if(direction == 1)
        adjust_point2.x_ = line2.p2_.x_ - gap * cos(atan(line2.a));
      else
        adjust_point2.x_ = line2.p1_.x_ + gap * cos(atan(line2.a));
      adjust_point2.y_ = adjust_line2.a * adjust_point2.x_ + adjust_line2.b;
      adjust_point2.theta_ = adjust_theta(direction, line2.a);
      adjust_point2.segment = center_points[i].segment;

      index2 = i;
    }
    last_length_sum = length_sum;
  }

  if(index1 >= 0 && index2 >= 0)
  {
    vector<SegmentCenterPoint>::iterator cut_iter;
    for(auto iter =center_points.begin(); iter != center_points.end(); iter++)
    {
      if(iter -> segment == adjust_point2.segment)
      {
        ROS_INFO("Begin to adjust center points ");
        center_points.erase(iter, center_points.end());
        break;
      }      
    }
    
    adjust_point2.curvature_ = 0;
    
    adjust_point1.curvature_ = 0;
    center_points.push_back(adjust_point2);
    center_points.push_back(adjust_point1);
  }
}


vector<map_module::curvepoint> vec_map::get_guided_curve(road_pos_info start, double &layback, int turn)
{
  vector<SegmentCenterPoint> center_points;
  vector<map_module::curvepoint> curve;
  center_points = get_center_points(start, (*start.junction_point).p, start.lane_sequence);
  adjust_points(center_points, start, (*start.junction_point).p, turn, start.lane_sequence);
  curve = just_go(center_points, start, (*start.junction_point).p, layback, turn);
  return curve;
}





bool vec_map::get_navigation_curve(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped goal_pose)
{
  navigation_curve.resize(0);
  curve_length = 0;
  road_pos_info start = get_road_pos_info(start_pose);
  if(start.segment == -1)
  {
    ROS_ERROR("Invalid start position");
    return false;
  }
    
  road_pos_info goal = get_road_pos_info(goal_pose);
  if(goal.segment == -1)
  {
    ROS_ERROR("Invalid goal position");
    return false;
  }

  //调整goal pose到路网的规范位置
  this->nav_goal.pose = info_to_pose(goal);

  //当前位置和终点在同一条车道
  if(start.road == goal.road)
  {
    ROS_INFO("Current position is on the same road where the goal was set.");
    //分别在实黄线的两端，终点不可达
    if(start.lane_sequence * goal.lane_sequence < 0)
    {
      ROS_ERROR("Current pose and goal pose are on the same road, but in different direction");
      return false;
    }
    else if((goal.center_projection.x_ - start.center_projection.x_) * start.direction < 0)
    {
      ROS_ERROR("Current pose is beyong goal pose on the same road, goal is unreachable");
      return false;
    }
    else
    {
      vector<SegmentCenterPoint> center_points = get_center_points(start, start.junction_point->p, goal.lane_sequence);
      double layback = 0.1;
      start.lane_sequence = goal.lane_sequence;
      navigation_curve = just_go(center_points, start, start.junction_point->p, layback, 2);
      return true;
    } 
  }
  //不在同一条车道
  else
  {
    vector<Junction>::iterator junc_start, junc_end;
    junc_start = nav_get_first_junction(start_pose);
    junc_end = nav_get_last_junction(goal_pose);
    if(astar_navigation(junc_start, junc_end) == false)
    {
      ROS_ERROR("Astar failed");
      return false;
    }
    else
    {
      vector<junc_iter> junc_r = build_junctions_queue(junc_end);
      vector<junc_iter> junc_queue;
      vector<junc_iter>::reverse_iterator iter;
      //build junc_queue
      for(iter = junc_r.rbegin(); iter != junc_r.rend(); iter++)
        junc_queue.push_back((*iter));
      vector<Point> juncs;
      juncs.push_back(start.center_projection);
      //build lane turn
      for(auto iter_j = junc_queue.begin(); iter_j != junc_queue.end(); iter_j++)  
        juncs.push_back((*iter_j)->junction.p);
      juncs.push_back(goal.center_projection);
      vector<int> laneturn = build_lane_turn(juncs);
      if(laneturn.size() != junc_queue.size())
      {
        std::cout<<"lane turn vector size: "<<laneturn.size()<<endl;
        std::cout<<"junction queue vector size: "<<junc_queue.size()<<endl;
        ROS_ERROR("Wrong");
        return false;
      }
      double layback = 0.1;
      road_pos_info err_start;
      err_start = start;
      int dx1, dx2, dy1, dy2;
      Point p1, p2;
      double a1, a2, offset1, offset2;
      //int dxdy[4];        
      int lane_seq;
      int direction;
      for(int junc_sequence = 0; junc_sequence < junc_queue.size(); junc_sequence++)//????
      {
        //从当前位置去到下一个路口点的路径
        navigation_curve = navigation_curve + get_guided_curve(err_start, layback, laneturn[junc_sequence]);
        Point p1 = err_start.junction_point->p;

        //////////////////////////////////////////
        direction = sign(err_start.lane_sequence);

        if(laneturn[junc_sequence] == 0)
          if(direction > 0)
            lane_seq = ceil((double)err_start.road->lanes_forward_ / 2);
          else
            lane_seq = -ceil((double)err_start.road->lanes_backward_ / 2); 
        else if(laneturn[junc_sequence] == 1)
          lane_seq = direction;
        else if(laneturn[junc_sequence] == -1)
          if(direction > 0)
            lane_seq = start.road->lanes_forward_;
          else
            lane_seq = -start.road->lanes_backward_; 


        offset1 = err_start.road->get_mission_point_offset(lane_seq);
        a1 = err_start.road->geometry_list[err_start.segment].a;
        dx1 = sign(juncs[junc_sequence + 1].x_ - juncs[junc_sequence].x_);
        dy1 = sign(juncs[junc_sequence + 1].y_ - juncs[junc_sequence].y_);
        
        road_pos_info jp_info;
        if(junc_sequence < junc_queue.size() - 1)
          jp_info = get_guided_junction_point(junc_queue[junc_sequence], junc_queue[junc_sequence + 1]);
        else
          jp_info = get_last_junction_point(junc_queue[junc_sequence], goal);

        if(jp_info.road == roads.end())
        {
          std::cout<<"junction number: "<<junc_sequence<<endl;
          ROS_ERROR("guided junction point failed to find ");
          return false;
        }
        p2 = jp_info.center_projection;
        offset2 = jp_info.road->get_mission_point_offset(jp_info.lane_sequence);
        a2 = jp_info.road->geometry_list[jp_info.segment].a;
        dx2 = sign(juncs[junc_sequence + 2].x_ - juncs[junc_sequence + 1].x_);
        dy2 = sign(juncs[junc_sequence + 2].y_ - juncs[junc_sequence + 1].y_);
        int dxdy[4] = {dx1, dy1, dx2, dy2};
        //border need to be assigned;
        navigation_curve = navigation_curve + get_curve_spline(p1, p2, offset1, offset2, dxdy, a1, a2, border_junction, layback);

        err_start = jp_info;
      }
      //最后通过路口的一段路

      //最后道路路口点到达终点 的轨迹 
      //layback取平均值
      layback = 0.05;
      vector<SegmentCenterPoint> last_curve_points = get_center_points(goal, err_start.center_projection, goal.lane_sequence);
      vector<map_module::curvepoint> last_curve_r = just_go(last_curve_points, goal, err_start.center_projection, layback, 2);
      vector<map_module::curvepoint> last_curve;
      last_curve.resize(0);
      vector<map_module::curvepoint>::reverse_iterator iter_r;
      for(iter_r = last_curve_r.rbegin(); iter_r != last_curve_r.rend(); iter_r++)
      {
        iter_r->theta = check_theta(iter_r->theta + PI);
        iter_r->kappa = -(iter_r->kappa);
        last_curve.push_back((*iter_r));
      }
        
      navigation_curve = navigation_curve + last_curve;

      store_navigation_curve();

      return true;
    }
    return false;
  }
  return false;
}