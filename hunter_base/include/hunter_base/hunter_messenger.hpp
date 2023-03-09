/*
 * hunter_messenger.hpp
 *
 * Created on: Jun 14, 2019 10:24
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef HUNTER_MESSENGER_HPP
#define HUNTER_MESSENGER_HPP

#include <string>
#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include "ascent/Ascent.h"
#include "ascent/Utility.h"
#include "hunter_base/bicycle_model.hpp"
#include "hunter_base/hunter_params.hpp"

#include "ugv_sdk/mobile_robot/hunter_robot.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"
#include "interfaces/msg/hunter_status.hpp"
#include "interfaces/msg/hunter_light_cmd.hpp"

namespace westonrobot {

template <typename SystemModel>
class SystemPropagator {
 public:
  asc::state_t Propagate(asc::state_t init_state,
                         typename SystemModel::control_t u, double t0,
                         double tf, double dt) {
    double t = t0;
    asc::state_t x = init_state;

    while (t <= tf) {
      integrator_(SystemModel(u), x, t, dt);
      // Note: you may need to add additional constraints to [x]
    }
    return x;
  }

 private:
  asc::RK4 integrator_;
};

template <typename HunterType>
class HunterMessenger {
 public:
  HunterMessenger(std::shared_ptr<HunterType> hunter, rclcpp::Node *node)
      : hunter_(hunter), node_(node) {}
  // SystemPropagator<BicycleKinematics> model_;
  void SetOdometryFrame(std::string frame) { odom_frame_ = frame; }
  void SetBaseFrame(std::string frame) { base_frame_ = frame; }
  void SetOdometryTopicName(std::string name) { odom_topic_name_ = name; }
  void SetWeelbase(float Weelbase){
    l = Weelbase;
  }
  void SetTrack(float Track){
    w = Track;
  }
  void SetMaxSteerAngleCentral(float Angle){
    max_steer_angle_central = Angle;
  }

  void SetSimulationMode(int loop_rate) {
    simulated_robot_ = true;
    sim_control_rate_ = loop_rate;
  }

  void SetupSubscription() {
    // odometry publisher
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    odom_pub_ =
        node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_, 50);
    status_pub_ = node_->create_publisher<interfaces::msg::HunterStatus>(
        "/hunter_status", 10);

    // cmd subscriber
    motion_cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&HunterMessenger::TwistCmdCallback, this,
                  std::placeholders::_1));
    // battery status publisher
    battery_status_pub_ = node_->create_publisher<sensor_msgs::msg::BatteryState>(
        "/base/battery", 10);
    
  }
  void PublishBatteryStatus(){
    current_time_ = node_->get_clock()->now();

    static bool init_run = true;
    if (init_run) {
      last_time_ = current_time_;
      init_run = false;
      return;
    }

    auto state = hunter_->GetRobotState();
    auto actuator = hunter_->GetActuatorState();
    sensor_msgs::msg::BatteryState battery_msg;
    battery_msg.header.stamp = current_time_;
    battery_msg.header.frame_id = "battery";
    battery_msg.current = (actuator.actuator_hs_state[0].current + actuator.actuator_hs_state[1].current)/2;
    // if battery voltage is below 35 V, the max voltage is 29.4
    // if battery voltage is above 35 V, the max voltage is 54.6V
    // Min battery voltage is 24V
    // Min battery voltage is 40V if battery voltage is above 30 V
    battery_msg.voltage = state.system_state.battery_voltage; 
    if (battery_msg.voltage < 35) {
      double tmp_percentage = 0.65085267771558497 * pow((battery_msg.voltage), 4) - 
        68.168813766265387 * pow((battery_msg.voltage), 3) + 
        2669.0893112435269 * pow((battery_msg.voltage), 2) -
        46282.892652307622 * battery_msg.voltage + 
        299832.86802959372; //created from https://arachnoid.com/polysolve/
      // Ensure percentage is between 0 and 100
      if (tmp_percentage < 0) {
        tmp_percentage = 0;
      } else if (tmp_percentage > 100) {
        tmp_percentage = 1;
      } else {
        battery_msg.percentage = tmp_percentage / 100;
      }
      battery_msg.design_capacity = 30; // 30Ah
    } else {

      double tmp_percentage = 0.0035363083052703092 * pow(battery_msg.voltage, 4) 
        -0.60604460460754128 * pow(battery_msg.voltage, 3) +  
        37.954435576128162 * pow(battery_msg.voltage, 2) -
        1016.7870628390317 * battery_msg.voltage + 
        9692.6828595033203;
      // battery_msg.design_capacity = 60; // 60Ah
      // Ensure percentage is between 0 and 100
      if (tmp_percentage < 0) {
        tmp_percentage = 0;
      } else if (tmp_percentage > 100) {
        tmp_percentage = 1;
      } else {
        battery_msg.percentage = tmp_percentage / 100;
      }
    }

    battery_status_pub_->publish(battery_msg);
  }

  void PublishStateToROS() {
    current_time_ = node_->get_clock()->now();

    static bool init_run = true;
    if (init_run) {
      last_time_ = current_time_;
      init_run = false;
      return;
    }
    double dt = (current_time_ - last_time_).seconds();

    auto state = hunter_->GetRobotState();

    // publish hunter state message
    interfaces::msg::HunterStatus status_msg;

    status_msg.header.stamp = current_time_;

    status_msg.linear_velocity = state.motion_state.linear_velocity;
    double phi =ConvertInnerAngleToCentral(state.motion_state.angular_velocity);
    status_msg.steering_angle = phi;
    // status_msg.angular_velocity = state.motion_state.angular_velocity;

    status_msg.vehicle_state = state.system_state.vehicle_state;
    status_msg.control_mode = state.system_state.control_mode;
    status_msg.error_code = state.system_state.error_code;
    status_msg.battery_voltage = state.system_state.battery_voltage;

    auto actuator = hunter_->GetActuatorState();

    for (int i = 0; i < 3; ++i) {
      // actuator_hs_state
      uint8_t motor_id = actuator.actuator_hs_state[i].motor_id;

      status_msg.actuator_states[motor_id].rpm =
          actuator.actuator_hs_state[i].rpm;
      status_msg.actuator_states[motor_id].current =
          actuator.actuator_hs_state[i].current;
      status_msg.actuator_states[motor_id].pulse_count =
          actuator.actuator_hs_state[i].pulse_count;

      // actuator_ls_state
      motor_id = actuator.actuator_ls_state[i].motor_id;

      status_msg.actuator_states[motor_id].driver_voltage =
          actuator.actuator_ls_state[i].driver_voltage;
      status_msg.actuator_states[motor_id].driver_temperature =
          actuator.actuator_ls_state[i].driver_temp;
      status_msg.actuator_states[motor_id].motor_temperature =
          actuator.actuator_ls_state[i].motor_temp;
      status_msg.actuator_states[motor_id].driver_state =
          actuator.actuator_ls_state[i].driver_state;
    }

    status_pub_->publish(status_msg);

    // publish odometry and tf
    PublishOdometryToROS(state.motion_state, dt);

    // record time for next integration
    last_time_ = current_time_;
  }

 private:
  std::shared_ptr<HunterType> hunter_;
  rclcpp::Node *node_;
  

  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;

  bool simulated_robot_ = false;
  int sim_control_rate_ = 50;
  bool battery_status_ = false;

  westonrobot::SystemPropagator<BicycleKinematics> model_;

  std::mutex twist_mutex_;
  geometry_msgs::msg::Twist current_twist_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<interfaces::msg::HunterStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_status_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_sub_;
  

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // speed variables
  double position_x_ = 0.0;
  double position_y_ = 0.0;
  double theta_ = 0.0;

  static constexpr double steer_angle_tolerance = 0.005;  // ~+-0.287 degrees
  double l = 0.0;
  double w = 0.0;
  double max_steer_angle_central = 0.0;

  double previous_position_x_ = 0.0;
  double previous_position_y_ = 0.0;
  double d_x_ = 0.0;
  double d_y_ = 0.0;

  double covar_pose_position = 0.0;
  double covar_pose_twist    = 1.0; // Start with a bit of uncertainty in the
                                    // twist
  double covar_twist_linear  = 0.0;
  double covar_twist_angular = 0.0;

  rclcpp::Time last_time_;
  rclcpp::Time current_time_;

  void TwistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    
    if (!simulated_robot_) {
      SetHunterMotionCommand(msg);
    } else {
      std::lock_guard<std::mutex> guard(twist_mutex_);
      current_twist_ = *msg.get();
    }
    // ROS_INFO("Cmd received:%f, %f", msg->linear.x, msg->angular.z);
  }

  // template <typename T,std::enable_if_t<!std::is_base_of<HunterRobot, T>::value,bool> = true>
  void SetHunterMotionCommand(const geometry_msgs::msg::Twist::SharedPtr &msg) {

    std::shared_ptr<HunterRobot> base;
    // base->SetMotionCommand
    
    double steer_cmd = msg->angular.z;
    if(steer_cmd > max_steer_angle_central)
      steer_cmd = max_steer_angle_central;
    if(steer_cmd < - max_steer_angle_central)
        steer_cmd = - max_steer_angle_central;
    
    double phi_i = ConvertCentralAngleToInner(steer_cmd); 

   std::cout << "set steering angle: " << phi_i << std::endl;
    hunter_->SetMotionCommand(msg->linear.x, phi_i);
    // hunter_
 
  }

  double ConvertCentralAngleToInner(double angle)
  {
    double phi = angle;
    double phi_i = 0;
    if (phi > steer_angle_tolerance) {
      // left turn
      phi_i = std::atan(2 * l * std::sin(phi) /
                        (2 * l * std::cos(phi) - w * std::sin(phi)));
    } else if (phi < -steer_angle_tolerance) {
      // right turn
      phi = -phi;
      phi_i = std::atan(2 * l * std::sin(phi) /
                        (2 * l * std::cos(phi) - w * std::sin(phi)));
      phi_i = -phi_i;
    }
    return phi_i;
  }
  double ConvertInnerAngleToCentral(double angle)
  {
    double phi = 0;
    double phi_i = angle;
    if (phi_i > steer_angle_tolerance) {
      // left turn
      double r = l / std::tan(phi_i) + w / 2;
      phi = std::atan(l / r);
    } else if (phi_i < -steer_angle_tolerance) {
      // right turn
      double r = l / std::tan(-phi_i) + w / 2;
      phi = std::atan(l / r);
      phi = -phi;
    }
    return phi;

  }

  geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
  }

  void PublishOdometryToROS(const MotionStateMessage &msg, double dt) {
    // perform numerical integration to get an estimation of pose
    double linear_speed_ = msg.linear_velocity;
    // double angular_speed = msg.angular_velocity;
    double steering_angle_ = msg.steering_angle;
    // double lateral_speed = 0;

    asc::state_t state =
      model_.Propagate({position_x_, position_y_, theta_},
                       {linear_speed_, steering_angle_}, 0, dt, dt / 100);
    position_x_ = state[0];
    position_y_ = state[1];
    theta_ = state[2];

    d_x_ = position_x_ - previous_position_x_;
    d_y_ = position_y_ - previous_position_y_;

    // Covariance
    covar_pose_position += 0.01 * std::sqrt(d_x_*d_x_ + d_y_*d_y_);
    covar_pose_twist    += 0.01 * std::fabs(theta_);
    covar_twist_linear  = 0.1 * std::fabs(linear_speed_);
    covar_twist_angular = 0.1 * std::fabs(steering_angle_);

    geometry_msgs::msg::Quaternion odom_quat =
        createQuaternionMsgFromYaw(theta_);

    // publish tf transformation
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time_;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    tf_msg.transform.translation.x = position_x_;
    tf_msg.transform.translation.y = position_y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat;

    tf_broadcaster_->sendTransform(tf_msg);

    // publish odometry and tf messages
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = position_x_;
    odom_msg.pose.pose.position.y = position_y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    odom_msg.twist.twist.linear.x = linear_speed_;
    odom_msg.twist.twist.linear.y = 0;
    odom_msg.twist.twist.angular.z = linear_speed_ / l * std::tan(steering_angle_);

    odom_msg.pose.covariance = {covar_pose_position, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, covar_pose_position, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, covar_pose_position, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, covar_pose_twist, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, covar_pose_twist, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, covar_pose_twist};
    odom_msg.twist.covariance = {covar_twist_linear, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, covar_twist_linear, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, covar_twist_linear, 0.0, 0.0, 0.0,   
                                0.0, 0.0, 0.0, covar_twist_angular, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, covar_twist_angular, 0.0,   
                                0.0, 0.0, 0.0, 0.0, 0.0, covar_twist_angular}; 

    odom_pub_->publish(odom_msg);
  }
};
}  // namespace westonrobot

#endif /* Hunter_MESSENGER_HPP */
