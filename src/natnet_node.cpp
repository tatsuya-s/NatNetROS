#include <iostream>
#include <NatNetLinux/NatNet.h>
#include <NatNetLinux/CommandListener.h>
#include <NatNetLinux/FrameListener.h>
#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include "natnet_ros/RigidBody.h"
#include "natnet_ros/Marker.h"

class NatNetROS {
    public:
        NatNetROS();
        ~NatNetROS();
        void MainLoop();

    private:
        void PublishData(FrameListener& frameListener);
        ros::NodeHandle nh;
        ros::Publisher body_pub;
        ros::Publisher marker_pub;
        ros::Rate loop_rate;

        uint32_t local_address;
        uint32_t server_address;
};

NatNetROS::NatNetROS() : loop_rate(10) {
    body_pub = nh.advertise<natnet_ros::RigidBody>("NatNet/RigidBody", 100);
    marker_pub = nh.advertise<natnet_ros::Marker>("NatNet/Marker", 100);
    
    std::string local_ip, server_ip;
    nh.getParam("/natnet_node/local_address", local_ip);
    nh.getParam("/natnet_node/server_address", server_ip);
    this->local_address = inet_addr(local_ip.c_str());
    this->server_address = inet_addr(server_ip.c_str());
}

NatNetROS::~NatNetROS() {
}

void NatNetROS::PublishData(FrameListener& frameListener) {
    bool valid;
    MocapFrame frame;
    natnet_ros::RigidBody body_msg;
    natnet_ros::Marker marker_msg;
    std_msgs::UInt32 id;
    geometry_msgs::Pose pose_msg;
    geometry_msgs::Point point_msg;

    while (ros::ok()) {
        // Try to get a new frame from the listener.
        MocapFrame frame(frameListener.pop(&valid).first);
        
        if (valid) {
            const std::vector<RigidBody>& bodies_pose = frame.rigidBodies();
            const std::vector<Point3f>& markers_point = frame.unIdMarkers();

            for (int i = 0; i < bodies_pose.size(); ++i) {
                id.data = bodies_pose[i].id();
                pose_msg.position.x = bodies_pose[i].location().x;
                pose_msg.position.y = bodies_pose[i].location().y;
                pose_msg.position.z = bodies_pose[i].location().z;
                pose_msg.orientation.x = bodies_pose[i].orientation().qx;
                pose_msg.orientation.y = bodies_pose[i].orientation().qy;
                pose_msg.orientation.z = bodies_pose[i].orientation().qz;
                pose_msg.orientation.w = bodies_pose[i].orientation().qw;
                body_msg.id_array.push_back(id);
                body_msg.pose_array.push_back(pose_msg);
            }
            for (int i = 0; i < markers_point.size(); ++i) {
                point_msg.x = markers_point[i].x;
                point_msg.y = markers_point[i].y;
                point_msg.z = markers_point[i].z;
                marker_msg.point_array.push_back(point_msg);
                //marker_msg.stamp = ros::Time::now();
            }

            this->body_pub.publish(body_msg);
            this->marker_pub.publish(marker_msg);

            body_msg.id_array.clear();
            body_msg.pose_array.clear();
            marker_msg.point_array.clear();
        }
    }
}

void NatNetROS::MainLoop() {
    // Use this socket address to send commands to the server.
    struct sockaddr_in command_server = NatNet::createAddress(this->server_address, NatNet::commandPort);
    
    // Create sockets
    int command_socket, data_socket;
    command_socket = NatNet::createCommandSocket(this->local_address);
    data_socket = NatNet::createDataSocket(this->local_address);

    // Start the CommandListener in a new thread.
    CommandListener command_listener(command_socket);
    command_listener.start();

    // Send a ping packet to the server so that it sends us the NatNet version
    // in its response to commandListener.
    NatNetPacket ping = NatNetPacket::pingPacket();
    ping.send(command_socket, command_server);

    // Wait here for ping response to give us the NatNet version.
    unsigned char natnet_major, natnet_minor;
    command_listener.getNatNetVersion(natnet_major, natnet_minor);
    
    // Start up a FrameListener in a new thread.
    FrameListener frame_listener(data_socket, natnet_major, natnet_minor);
    frame_listener.start();
    
    PublishData(frame_listener);

    // Wait for threads to finish.
    frame_listener.stop();
    command_listener.stop();
    frame_listener.join();
    command_listener.join();
    
    // Epilogue
    close(data_socket);
    close(command_socket);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "NatNetROS_node");
    
    NatNetROS natnet;

    natnet.MainLoop();

    return 0;
}