#include <iostream>
#include <NatNetLinux/NatNet.h>
#include <NatNetLinux/CommandListener.h>
#include <NatNetLinux/FrameListener.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/UInt16.h>
#include "natnet_ros/RigidBody.h"
#include "natnet_ros/Marker.h"

class NatNetROS 
{
    public:
        NatNetROS();
        ~NatNetROS();
        void mainLoop();

    private:
        void publishData(FrameListener &frameListener);
        ros::NodeHandle nh, private_nh;
        ros::Publisher body_pub;
        ros::Publisher marker_pub;
        tf2_ros::TransformBroadcaster tf_broadcaster;
        std::string parent_frame_id;
        uint32_t local_address;
        uint32_t server_address;
        bool y_up;
};

NatNetROS::NatNetROS() :
    private_nh("~"), 
    parent_frame_id("optitrack"), 
    y_up(false)
{
    body_pub = this->nh.advertise<natnet_ros::RigidBody>("NatNet/RigidBody", 100);
    marker_pub = this->nh.advertise<natnet_ros::Marker>("NatNet/Marker", 100);

    this->private_nh.getParam("parent_frame_id", this->parent_frame_id);
    this->private_nh.getParam("y_up", this->y_up);

    std::string local_ip, server_ip;
    this->private_nh.getParam("local_address", local_ip);
    this->private_nh.getParam("server_address", server_ip);
    this->local_address = inet_addr(local_ip.c_str());
    this->server_address = inet_addr(server_ip.c_str());
}

NatNetROS::~NatNetROS() 
{
}

void NatNetROS::publishData(FrameListener &frameListener) 
{
    bool valid;
    MocapFrame frame;
    natnet_ros::RigidBody body_msg;
    natnet_ros::Marker marker_msg;
    std_msgs::UInt32 id;
    geometry_msgs::Pose pose_msg;
    geometry_msgs::Point point_msg;
    geometry_msgs::TransformStamped transform_stamped;

    while (ros::ok()) 
    {
        // Try to get a new frame from the listener
        MocapFrame frame(frameListener.pop(&valid).first);
        
        if (valid) 
        {
            const std::vector<RigidBody> &bodies_pose = frame.rigidBodies();
            const std::vector<Point3f> &markers_point = frame.unIdMarkers();
            const ros::Time now_time = ros::Time::now();

            for (auto &body_pose : bodies_pose) 
            {
                const int &body_id = body_pose.id();
                const Point3f &body_location = body_pose.location();
                const Quaternion4f &body_orientation = body_pose.orientation();

                id.data = body_id;
                transform_stamped.header.stamp = now_time;
                transform_stamped.header.frame_id = this->parent_frame_id;
                transform_stamped.child_frame_id = "body" + std::to_string(body_id);

                pose_msg.position.x = transform_stamped.transform.translation.x = body_location.x;
                pose_msg.position.y = transform_stamped.transform.translation.y = body_location.y;
                pose_msg.position.z = transform_stamped.transform.translation.z = body_location.z;
                pose_msg.orientation.x = transform_stamped.transform.rotation.x = body_orientation.qx;
                pose_msg.orientation.y = transform_stamped.transform.rotation.y = body_orientation.qy;
                pose_msg.orientation.z = transform_stamped.transform.rotation.z = body_orientation.qz;
                pose_msg.orientation.w = transform_stamped.transform.rotation.w = body_orientation.qw;

                if (this->y_up) 
                {
                    pose_msg.position.y = transform_stamped.transform.translation.y = -body_location.z;
                    pose_msg.position.z = transform_stamped.transform.translation.z = body_location.y;
                    pose_msg.orientation.y = transform_stamped.transform.rotation.y = -body_orientation.qz;
                    pose_msg.orientation.z = transform_stamped.transform.rotation.z = body_orientation.qy;
                }

                body_msg.id_array.push_back(id);
                body_msg.pose_array.push_back(pose_msg);
                this->tf_broadcaster.sendTransform(transform_stamped);
            }
            for (auto &marker_point : markers_point) 
            {
                point_msg.x = marker_point.x;
                point_msg.y = marker_point.z;
                point_msg.z = marker_point.y;
                if (this->y_up) 
                {
                    point_msg.y = -marker_point.z;
                    point_msg.z = marker_point.y;
                }
                marker_msg.point_array.push_back(point_msg);
            }

            this->body_pub.publish(body_msg);
            this->marker_pub.publish(marker_msg);

            body_msg.id_array.clear();
            body_msg.pose_array.clear();
            marker_msg.point_array.clear();
        }
    }
}

void NatNetROS::mainLoop() 
{
    struct sockaddr_in command_server = NatNet::createAddress(this->server_address, NatNet::commandPort);
    int command_socket, data_socket;
    command_socket = NatNet::createCommandSocket(this->local_address);
    data_socket = NatNet::createDataSocket(this->local_address);
    CommandListener command_listener(command_socket);
    command_listener.start();

    // Send a ping packet & Wait ping response
    NatNetPacket ping = NatNetPacket::pingPacket();
    ping.send(command_socket, command_server);
    unsigned char natnet_major, natnet_minor;
    command_listener.getNatNetVersion(natnet_major, natnet_minor);
    
    FrameListener frame_listener(data_socket, natnet_major, natnet_minor);
    frame_listener.start();
    
    this->publishData(frame_listener);

    frame_listener.stop();
    command_listener.stop();
    frame_listener.join();
    command_listener.join();
    
    close(data_socket);
    close(command_socket);
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "natnet_node");
    
    NatNetROS natnet;

    natnet.mainLoop();

    return 0;
}