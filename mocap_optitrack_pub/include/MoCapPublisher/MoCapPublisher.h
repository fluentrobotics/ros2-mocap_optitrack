#ifndef MOCAPPUBLISHER_H
#define MOCAPPUBLISHER_H

#include <vector>
#include <NatNetTypes.h>

#include "rclcpp/rclcpp.hpp"
#include "mocap_optitrack_interfaces/msg/rigid_body_array.hpp"


using namespace std;
using namespace std::chrono;
class MoCapPublisher: public rclcpp::Node
{
private:
    //Attributes
    rclcpp::Publisher<mocap_optitrack_interfaces::msg::RigidBodyArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    high_resolution_clock::time_point t_start;

    //Methods
    void sendFakeMessage();
    

public:
    // Definition of the construtors
    MoCapPublisher();

    // Send methods
    void sendRigidBodyMessage(sRigidBodyData* bodies, int nRigidBodies);
 
    // Getters
    
    // Setters
    
};
 
#endif