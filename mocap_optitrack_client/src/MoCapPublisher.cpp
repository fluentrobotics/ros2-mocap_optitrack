// RoS2 Node that handles the connection with the NatNet server (Motive)
#include <MoCapPublisher.h>

// Include standard libraries
#include <stdio.h>
#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
// Include the MoCap NatNet client
#include <MoCapNatNetClient.h>

#include <Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

bool cmpRigidBodyId(sRigidBodyData body_a, sRigidBodyData body_b) {
  return body_a.ID < body_b.ID;
}

MoCapPublisher::MoCapPublisher() : Node("natnet_client") {
  // Declare the ROS2 parameters used by the NatNet Client
  this->declare_parameter<std::string>("server_address", "10.125.37.2");
  this->declare_parameter<int>("connection_type", 0);
  this->declare_parameter<std::string>("multi_cast_address", "239.255.42.99");
  this->declare_parameter<uint16_t>("server_command_port", 1510);
  this->declare_parameter<uint16_t>("server_data_port", 1511);
  this->declare_parameter<std::string>("pub_topic", "rigid_body_topic");
  this->declare_parameter<bool>("record", true);
  this->declare_parameter<std::string>("take_name", "");

  this->declare_parameter<std::string>("ros_global_frame_id", "map");
  this->declare_parameter("object_mocap_num_ids", rclcpp::PARAMETER_INTEGER_ARRAY);
  this->declare_parameter("object_ros_frame_ids", rclcpp::PARAMETER_STRING_ARRAY);

  // Read ROS frames and tracked objects
  this->get_parameter("ros_global_frame_id", this->ros_global_frame_id_);
  std::vector<long int> object_mocap_num_ids;
  std::vector<std::string> object_ros_frame_ids;
  this->get_parameter("object_mocap_num_ids", object_mocap_num_ids);
  this->get_parameter("object_ros_frame_ids", object_ros_frame_ids);

  if (object_mocap_num_ids.size() != object_ros_frame_ids.size()) {
    RCLCPP_WARN(this->get_logger(), "Found %ld num ids but %ld frame ids in natnetclient config.",
                object_mocap_num_ids.size(), object_ros_frame_ids.size());
  }
  const size_t object_count = std::min(object_mocap_num_ids.size(), object_ros_frame_ids.size());
  for (size_t i = 0; i < object_count; ++i) {
    const int mocap_num_id = object_mocap_num_ids[i];
    const std::string& ros_frame_id = object_ros_frame_ids[i];
    this->map_mocap_id_ros_frame_id_[mocap_num_id] = ros_frame_id;

    RCLCPP_WARN_STREAM(this->get_logger(), "Tracking object " << mocap_num_id << ": " << ros_frame_id);
  }

  // Create the publishers
  std::string topic_;
  this->get_parameter("pub_topic", topic_);
  this->publisher_ =
      this->create_publisher<mocap_optitrack_interfaces::msg::RigidBodyArray>(topic_.c_str(), 10);
  this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  //
  // Just for testing purposes send make messages every 500ms
  // this->timer_ = this->create_wall_timer(500ms, std::bind(&MoCapPublisher::sendFakeMessage,
  // this));
  //
  // Log info about creation
  RCLCPP_INFO(this->get_logger(), "Created MoCap publisher node.\n");

  // TO REMOVE
  std::string address_;
  this->get_parameter("server_address", address_);
  RCLCPP_INFO(this->get_logger(), address_.c_str());
}

// Method that send over the ROS network the data of a rigid body
void MoCapPublisher::sendRigidBodyMessage(double cameraMidExposureSecsSinceEpoch,
                                          sRigidBodyData* bodies_ptr,
                                          int nRigidBodies) {
  std::vector<sRigidBodyData> bodies;
  for (int i = 0; i < nRigidBodies; i++) {
    bodies.push_back(bodies_ptr[i]);
  }

  // sort rigid bodies by their id
  std::sort(bodies.begin(), bodies.end(), cmpRigidBodyId);

  // Convert seconds since epoch to ROS time
  // int64_t cameraMidExposureNanoSecsSinceEpoch = int64_t(cameraMidExposureSecsSinceEpoch * 1e9)

  // TODO: BUG: cameraMidExposureSecsSinceEpoch is infinity.
  // Temporarily fix: use the ROS clock.
  // rclcpp::Time cameraMidExposureTime = rclcpp::Time(cameraMidExposureNanoSecsSinceEpoch);
  const rclcpp::Time cameraMidExposureTime = this->get_clock()->now();

  // Instanciate variables
  mocap_optitrack_interfaces::msg::RigidBodyArray msg;
  msg.header.stamp = cameraMidExposureTime;

  // Log
  RCLCPP_INFO(get_logger(), "Sending message containing %d Rigid Bodies.\n\n", nRigidBodies);
  // Loop over all the rigid bodies
  for (int i = 0; i < nRigidBodies; i++) {
    RCLCPP_INFO(this->get_logger(), "Rigid Body [ID=%d  Error=%3.2f  Valid=%d]\n", bodies[i].ID,
                bodies[i].MeanError, bodies[i].params & 0x01);
    RCLCPP_INFO(this->get_logger(), "\tx\ty\tz\tqx\tqy\tqz\tqw\n");
    RCLCPP_INFO(this->get_logger(), "\t%3.5f\t%3.5f\t%3.5f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
                bodies[i].x, bodies[i].y, bodies[i].z, bodies[i].qx, bodies[i].qy, bodies[i].qz,
                bodies[i].qw);
    //
    // Create the rigid body message
    mocap_optitrack_interfaces::msg::RigidBody rb;
    rb.header.stamp = cameraMidExposureTime;
    rb.id = bodies[i].ID;
    rb.valid = bodies[i].params & 0x01;
    rb.mean_error = bodies[i].MeanError;
    rb.pose_stamped.pose.position.x = bodies[i].x;
    rb.pose_stamped.pose.position.y = bodies[i].y;
    rb.pose_stamped.pose.position.z = bodies[i].z;
    rb.pose_stamped.pose.orientation.x = bodies[i].qx;
    rb.pose_stamped.pose.orientation.y = bodies[i].qy;
    rb.pose_stamped.pose.orientation.z = bodies[i].qz;
    rb.pose_stamped.pose.orientation.w = bodies[i].qw;
    //
    rb.pose_stamped.header.stamp = cameraMidExposureTime;
    //
    // Add the current rigid body to the array of rigid bodies
    msg.rigid_bodies.push_back(rb);
  }
  // Publish the message.
  publisher_->publish(msg);
  this->updateTfTree(msg);
}

// Update the ROS Tf tree with the rigid body poses.
void MoCapPublisher::updateTfTree(
    const mocap_optitrack_interfaces::msg::RigidBodyArray& pose_array) {
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = pose_array.header.stamp;
  t.header.frame_id = this->ros_global_frame_id_;

  for (const mocap_optitrack_interfaces::msg::RigidBody& pose : pose_array.rigid_bodies) {
    if (this->map_mocap_id_ros_frame_id_.count(pose.id) == 0 || !pose.valid) {
      continue;
    }

    t.child_frame_id = this->map_mocap_id_ros_frame_id_[pose.id];

    t.transform.translation.x = pose.pose_stamped.pose.position.x;
    t.transform.translation.y = pose.pose_stamped.pose.position.y;
    t.transform.translation.z = pose.pose_stamped.pose.position.z;
    t.transform.rotation.x = pose.pose_stamped.pose.orientation.x;
    t.transform.rotation.y = pose.pose_stamped.pose.orientation.y;
    t.transform.rotation.z = pose.pose_stamped.pose.orientation.z;
    t.transform.rotation.w = pose.pose_stamped.pose.orientation.w;

    // The motion capture tracks the centroid of the markers placed on a rigid
    // body. In general, this will not be the same point as the base link of a
    // robot, so a robot and marker-arrangement-specific correction needs to be
    // applied to the pose obtained from the motion capture.
    if (t.child_frame_id == "base_link") {
      const Eigen::Isometry3d T_map_centroid = tf2::transformToEigen(t);

      // Last updated for Stretch on 2024-04-18
      const Eigen::Isometry3d T_centroid__base_link(Eigen::Translation3d(0.0954, 0.00652, -0.1339));

      const Eigen::Isometry3d T_map__base_link = T_map_centroid * T_centroid__base_link;

      // Copy the transform field to the original TransformStamped because the
      // header in the new TransformStamped might not be set properly.
      geometry_msgs::msg::TransformStamped t2 = tf2::eigenToTransform(T_map__base_link);
      t.transform = t2.transform;
    }

    this->tf_broadcaster_->sendTransform(t);
  }
}

// Method used to send fake messages to the client
void MoCapPublisher::sendFakeMessage() {
  int nRigidBodies = 2;
  sRigidBodyData* bodies = (sRigidBodyData*)malloc(nRigidBodies * sizeof(sRigidBodyData));

  for (int i = 0; i < nRigidBodies; i++) {
    bodies[i].x = 1 * i;
    bodies[i].y = 2 * i;
    bodies[i].z = 3 * i;
    bodies[i].qx = 4 * i;
    bodies[i].qy = 5 * i;
    bodies[i].qz = 6 * i;
    bodies[i].qw = 7 * i;
    bodies[i].MeanError = 0.0;
    bodies[i].ID = i;
    bodies[i].params = 1;
  }
  // Send the message
  this->sendRigidBodyMessage(this->get_clock()->now().seconds(), bodies, nRigidBodies);

  // Free the rigid bodies
  free(bodies);
}

std::string MoCapPublisher::getServerAddress() {
  std::string addr_;
  this->get_parameter("server_address", addr_);
  return addr_;
}

int MoCapPublisher::getConnectionType() {
  int type_ = 0;
  this->get_parameter("connection_type", type_);
  return type_;
}

std::string MoCapPublisher::getMulticastAddress() {
  std::string addr_;
  this->get_parameter("multi_cast_address", addr_);
  return addr_;
}

uint16_t MoCapPublisher::getServerCommandPort() {
  uint16_t port_;
  this->get_parameter("server_command_port", port_);
  return port_;
}

uint16_t MoCapPublisher::getServerDataPort() {
  uint16_t port_;
  this->get_parameter("server_data_port", port_);
  return port_;
}

bool MoCapPublisher::isRecordingRequested() {
  bool record_ = true;
  this->get_parameter("record", record_);
  return record_;
}

std::string MoCapPublisher::getTakeName() {
  std::string takeName_;
  this->get_parameter("take_name", takeName_);

  if (takeName_.empty()) {
    // set take name to the current date and time in the format
    time_t curr_time;
    tm* curr_tm;
    char datetime_string[100];

    time(&curr_time);
    curr_tm = localtime(&curr_time);

    // "take_20230101_235959"
    strftime(datetime_string, 50, "take_%Y%m%d_%H%M%S", curr_tm);
    takeName_ = std::string(datetime_string);
  }

  return takeName_;
}

// Main
int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create the ROS2 publisher
  auto mocapPub = std::make_shared<MoCapPublisher>();
  // Create the MoCapNatNetClient
  MoCapNatNetClient* c = new MoCapNatNetClient(mocapPub.get());
  // Try to connect the client
  int retCode = c->connect();
  if (retCode != 0) {
    return retCode;
  }
  // Ready to receive marker stream
  rclcpp::spin(mocapPub);
  // disconnect the clinet
  c->disconnect();
  // Delete all the objects created
  delete c;
  rclcpp::shutdown();  // delete the ROS2 nodes
  return 0;
}
