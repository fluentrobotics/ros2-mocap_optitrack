#ifndef MOCAPPUBLISHER_H
#define MOCAPPUBLISHER_H

#include <unordered_map>
#include <vector>

#include <NatNetTypes.h>

#include "mocap_optitrack_interfaces/msg/rigid_body_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"


class MoCapPublisher: public rclcpp::Node
{
private:
    //Attributes
    rclcpp::Publisher<mocap_optitrack_interfaces::msg::RigidBodyArray>::SharedPtr publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string ros_global_frame_id_;
    std::unordered_map<int, std::string> map_mocap_id_ros_frame_id_;

    //Methods
    void sendFakeMessage();

public:
    // Definition of the construtors
    MoCapPublisher();

    // Send methods
    void sendRigidBodyMessage(double cameraMidExposureSecsSinceEpoch, sRigidBodyData* bodies_ptr, int nRigidBodies);
    void updateTfTree(const mocap_optitrack_interfaces::msg::RigidBodyArray& msg);

    // Getters
    std::string getServerAddress();
    int getConnectionType();
    std::string getMulticastAddress();
    uint16_t getServerCommandPort();
    uint16_t getServerDataPort();
    bool isRecordingRequested();
    std::string getTakeName();

    // Setters
};

#endif  // MOCAPPUBLISHER_H
