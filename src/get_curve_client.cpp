#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <std_srvs/Trigger.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <map_module/curve.h>
#include <map_module/curvepoint.h>
#include <map_module/path_planning.h>
#include <map_module/get_curve.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <fstream>
#include <sstream> 
#include <cmath>

using namespace std;

#define path "/home/wrs/map/src/map_module/data/mission_curve.csv"

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

vector<geometry_msgs::PoseStamped> csv_to_posetrack(){

    vector<geometry_msgs::PoseStamped> posetrack;
    posetrack.resize(0);

    std_msgs::Header header;
    header.frame_id = "world";

    std::ifstream inFile(path, std::ios::in);
    if (!inFile)
    {
        std::cout << "打开文件失败！" << std::endl;
        exit(1);
    }

    double theta;

    int number=0;
    std::string line;
    geometry_msgs::PoseStamped p;
    //std::string field;
    while (getline(inFile, line))//getline(inFile, line)表示按行读取CSV文件中的数据
    {      
        number++;
        if(number == 1)
          continue;
        std::string field;
        std::istringstream sinn(line); //将整行字符串line读入到字符串流sinn中

        //TrackPoint element;
        p.header = header;

        getline(sinn, field, ','); //将字符串流sinn中的字符读入到field字符串中，以逗号为分隔符
        p.pose.position.x = (double)atof(field.c_str());//将刚刚读取的字符串转换成int

        getline(sinn, field, ',');
        p.pose.position.y = (double)atof(field.c_str());

        p.pose.position.z = 0;

        getline(sinn, field, ','); 
        theta = (double)atof(field.c_str());
        p.pose.orientation.x = 0;
        p.pose.orientation.y = 0;
        p.pose.orientation.z = sin(theta/2);
        p.pose.orientation.w = cos(theta/2);

        getline(sinn, field, ',');

        posetrack.push_back(p);
    }
    inFile.close();
    std::cout << "read csv : " << (number - 1) << "  lines." << std::endl;
    std::cout << "Finished csv reading." << std::endl;
    return posetrack;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "get_curve_client_node");
    ros::NodeHandle nh;

    geometry_msgs::PoseStamped p;
    std_msgs::Header header;
    header.frame_id = "world";

    visualization_msgs::Marker marker;    
    marker.header.frame_id = "world";
	marker.header.stamp = ros::Time::now();
    marker.ns = "current_pose";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.scale.x = 2;
    marker.scale.y = 2;
    marker.scale.z = 2;
	marker.color.r = 1.0f;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	marker.color.a = 1.0;

    ros::Rate loop_rate(10);
    
    ros::Publisher current_pose_pub = nh.advertise<visualization_msgs::Marker>("/fake_current_pose", 1);
    ros::ServiceClient get_curve_client = nh.serviceClient<map_module::get_curve>("/get_curve");

    vector<geometry_msgs::PoseStamped> pose_track = csv_to_posetrack();
    for(auto i = 0; i < pose_track.size(); i=i+50)
    {
        marker.pose = pose_track[i].pose;
        
        map_module::get_curve get_curve_srv;
        get_curve_srv.request.request_length = 100;
        get_curve_srv.request.current_pose = pose_track[i];
        if(get_curve_client.call(get_curve_srv))
        {
            std::cout<<"points : "<<get_curve_srv.response.center_lane.geometry.points.front().x<<std::endl;
        }
        else
        {
            ROS_ERROR("service /get_curve called failed!") ;
            return 0;
        }

        current_pose_pub.publish(marker);
        loop_rate.sleep();
    }

    ros::spin();

}




