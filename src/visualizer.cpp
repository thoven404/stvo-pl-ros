#include "visualizer.h"

namespace StVO{

Visualizer::Visualizer(ros::NodeHandle &n){
    path_pub = n.advertise<visualization_msgs::Marker>("caml_path", 1000);
    points_pub = n.advertise<visualization_msgs::Marker>("points_cloud", 1000);
    lines_pub = n.advertise<visualization_msgs::Marker>("lineSenments_cloud", 1000);

    path.header.frame_id = "map";
    path.ns =  "path";
    path.id = 0;
    path.action = visualization_msgs::Marker::ADD;
    path.pose.orientation.w = 1.0;
    path.type = visualization_msgs::Marker::LINE_STRIP;
    path.scale.x = 0.01;
    path.scale.y = 0.01;
    path.color.b = 1.0f;    // path is blue
    path.color.a = 1.0f;
    path.lifetime = ros::Duration();

    points.header.frame_id = "map";
    points.ns =  "points_and_lines";
    points.id = 0;
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.01;  // 点的大小
    points.scale.y = 0.01;
    points.color.r = 1.0f;  // points are red
    points.color.a = 1.0f;
    points.lifetime = ros::Duration();

    lines.header.frame_id = "map";
    lines.ns =  "points_and_lines";
    lines.id = 0;
    lines.action = visualization_msgs::Marker::ADD;
    lines.pose.orientation.w = 1.0;
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.scale.x = 0.01;  // 点的大小
    lines.scale.y = 0.01;
    lines.color.g = 1.0f;  // liness are green
    lines.color.a = 1.0f;
    lines.lifetime = ros::Duration();

    ptwc.setZero();
    pq.setValue(0.0, 0.0, 0.0, 1.0);

}

void Visualizer::publish(Eigen::Matrix3d Rwc, Eigen::Vector3d twc, std::vector<geometry_msgs::Point> &points_, std::vector<geometry_msgs::Point> &lines_){
    
    ros::Time t = ros::Time::now();

    Eigen::Vector3d p_wc = twc;
    geometry_msgs::Point point;
    point.x = p_wc(0);  point.y = p_wc(1);  point.z = p_wc(2);
    path.points.push_back(point);
    path.header.stamp =  t;
    path_pub.publish(path);

    transform.setOrigin( tf::Vector3(twc(0), twc(1), twc(2)) );
    Eigen::Quaterniond qfR(Rwc);
    q.setValue(qfR.x(),qfR.y(),qfR.z(),qfR.w());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, t, "map", "Twc"));

    points.points.clear();
    lines.points.clear();

    // if(q.angle(pq) > M_PI_4 || (twc - ptwc).norm() > 1.0){
        if(!points_.empty()){
            points.points.insert(points.points.end(), points_.begin(), points_.end());
            // points.points = points_;
            points.header.stamp =  t;
            points_pub.publish(points);
        }

        if(!lines_.empty()){
            lines.points.insert(lines.points.end(), lines_.begin(), lines_.end());
            // lines.points = lines_;
            lines.header.stamp =  t;
        }

        lines_pub.publish(lines);
        pq = q;
        ptwc = twc;
    // }

}


}