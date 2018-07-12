#include <iostream>
#include <cmath>
#include <mutex>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "natnet_ros/RigidBody.h"
#include "natnet_ros/Marker.h"

class MarkersVisualization 
{
    public:
        MarkersVisualization();
        ~MarkersVisualization();
        void bodyCallback(const natnet_ros::RigidBody::ConstPtr &msg);
        void markerCallback(const natnet_ros::Marker::ConstPtr &msg);
        void pseudoColor(std_msgs::ColorRGBA &marker, double phase);
        void mainLoop();
    private:
        ros::NodeHandle nh;
        ros::Publisher viz_pub;
        ros::Subscriber body_sub;
        ros::Subscriber marker_sub;
        std::mutex body_mtx;
        std::mutex marker_mtx;
        std::vector<geometry_msgs::Pose> poses_msg;
        std::vector<geometry_msgs::Point> points_msg;
        ros::Rate loop_rate;
        int count;
};

MarkersVisualization::MarkersVisualization() :
    count(0), 
    loop_rate(100)
{
    viz_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
    body_sub = nh.subscribe("NatNet/RigidBody", 100, &MarkersVisualization::bodyCallback, this);
    marker_sub = nh.subscribe("NatNet/Marker", 100, &MarkersVisualization::markerCallback, this);
}

MarkersVisualization::~MarkersVisualization() 
{
}

void MarkersVisualization::bodyCallback(const natnet_ros::RigidBody::ConstPtr &msg) 
{
    if (this->body_mtx.try_lock()) 
    {
        this->poses_msg.clear();
        for (auto &pose : msg->pose_array) 
        {
            this->poses_msg.push_back(pose);
        }
        /*
        for (std::size_t i = 0; i < msg->id_array.size(); ++i) 
        {
            this->indices.push_back(i);
            this->poses_msg.push_back(msg->pose_array[i]);
        }
        */
        this->body_mtx.unlock();
    }
}

void MarkersVisualization::markerCallback(const natnet_ros::Marker::ConstPtr &msg) 
{
    if (this->marker_mtx.try_lock()) 
    {
        this->points_msg.clear();
        for (auto &point : msg->point_array) 
        {
            this->points_msg.push_back(point);
        }
        this->marker_mtx.unlock();
    }
}

void MarkersVisualization::pseudoColor(std_msgs::ColorRGBA &color, 
                                       const double phase) 
{
    color.r = (std::sin(1.5 * M_PI * phase + M_PI + M_PI / 4) + 1) / 2.0;
    color.g = (std::sin(1.5 * M_PI * phase + M_PI + M_PI / 4 + M_PI / 2) + 1) / 2.0;
    color.b = (std::sin(1.5 * M_PI * phase + M_PI + M_PI / 4 + M_PI) + 1) / 2.0;
    color.a = 1.0;
}

void MarkersVisualization::mainLoop() 
{
    visualization_msgs::MarkerArray markers;
    while (ros::ok()) 
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "optitrack";
        marker.header.stamp = ros::Time::now();
        marker.ns = "marker" + std::to_string(count);
        marker.id = this->count++;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.lifetime = ros::Duration(5.0);

        this->marker_mtx.lock();
        for (auto &point_msg : this->points_msg) 
        {
            marker.points.push_back(point_msg);
        }
        this->marker_mtx.unlock();

        markers.markers.push_back(marker);

        if (count >= 100) 
        {
            markers.markers.erase(markers.markers.begin());
        }

        for (std::size_t i = 0; i < markers.markers.size(); ++i) 
        {
            this->pseudoColor(markers.markers[i].color, i * 0.01);
        }

        viz_pub.publish(markers);

        ros::spinOnce();
        this->loop_rate.sleep();
    }
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "natnet_viz_node");
    
    MarkersVisualization viz;

    viz.mainLoop();

    return 0;
}