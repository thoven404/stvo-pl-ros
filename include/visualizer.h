
#include "ros/ros.h"
#include <auxiliar.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Core>
#include <cmath>

namespace StVO {

class Visualizer{
public:
    Visualizer(ros::NodeHandle &n);
    void publish(Eigen::Matrix3d Rwc, Eigen::Vector3d twc, std::vector<geometry_msgs::Point> &points_, std::vector<geometry_msgs::Point> &lines_);
private:
    ros::Publisher path_pub;
    ros::Publisher points_pub;
    ros::Publisher lines_pub;
    tf::TransformBroadcaster br;

    visualization_msgs::Marker path;
    visualization_msgs::Marker points;
    visualization_msgs::Marker lines;
    tf::Transform transform;
    tf::Quaternion q;
    tf::Quaternion pq;
    Eigen::Vector3d ptwc;
    
};

} // namespace StVO