
#include <algorithm>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/Led.h>
#include <kobuki_msgs/WheelDropEvent.h>

//
// Publishers
//
rclcpp::publisher::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
rclcpp::publisher::Publisher<kobuki_msgs::Led>::SharedPtr led1_pub;
rclcpp::publisher::Publisher<kobuki_msgs::Led>::SharedPtr led2_pub;

//
// State
//
/// Flag for stopping
bool stop_;
/// Flag for left bumper's state
bool bumper_left_pressed_;
/// Flag for center bumper's state
bool bumper_center_pressed_;
/// Flag for right bumper's state
bool bumper_right_pressed_;
/// Flag for left cliff sensor's state
bool cliff_left_detected_;
/// Flag for center cliff sensor's state
bool cliff_center_detected_;
/// Flag for right cliff sensor's state
bool cliff_right_detected_;
/// Flag for left wheel drop sensor's state
bool wheel_drop_left_detected_;
/// Flag for right wheel drop sensor's state
bool wheel_drop_right_detected_;
/// Flag for bumper LED's state
bool led_bumper_on_;
/// Flag for cliff sensor LED's state
bool led_cliff_on_;
/// Flag for wheel drop sensor LED's state
bool led_wheel_drop_on_;
/// Linear velocity for moving straight
double vel_lin_;
/// Angular velocity for rotating
double vel_ang_;
/// Randomly chosen turning duration
ros::Duration turning_duration_;
/// Randomly chosen turning direction
int turning_direction_;
/// Start time of turning
ros::Time turning_start_;
/// Flag for turning state
bool turning_;

static void bumperCallback(const kobuki_msgs::BumperEventConstPtr msg)
{
    if (msg->state == kobuki_msgs::BumperEvent::PRESSED)
    {
      switch (msg->bumper)
      {
        case kobuki_msgs::BumperEvent::LEFT:
          if (!bumper_left_pressed_)
          {
            bumper_left_pressed_ = true;
            change_direction_ = true;
          }
          break;
        case kobuki_msgs::BumperEvent::CENTER:
          if (!bumper_center_pressed_)
          {
            bumper_center_pressed_ = true;
            change_direction_ = true;
          }
          break;
        case kobuki_msgs::BumperEvent::RIGHT:
          if (!bumper_right_pressed_)
          {
            bumper_right_pressed_ = true;
            change_direction_ = true;
          }
          break;
      }
    }
    else // kobuki_msgs::BumperEvent::RELEASED
    {
      switch (msg->bumper)
      {
        case kobuki_msgs::BumperEvent::LEFT:    bumper_left_pressed_   = false; break;
        case kobuki_msgs::BumperEvent::CENTER:  bumper_center_pressed_ = false; break;
        case kobuki_msgs::BumperEvent::RIGHT:   bumper_right_pressed_  = false; break;
      }
    }
    if (!led_bumper_on_ && (bumper_left_pressed_ || bumper_center_pressed_ || bumper_right_pressed_))
    {
      kobuki_msgs::LedPtr led_msg_ptr;
      led_msg_ptr.reset(new kobuki_msgs::Led());
      led_msg_ptr->value = kobuki_msgs::Led::ORANGE;
      led1_publisher_.publish(led_msg_ptr);
      led_bumper_on_ = true;
    }
    else if (led_bumper_on_ && (!bumper_left_pressed_ && !bumper_center_pressed_ && !bumper_right_pressed_))
    {
      kobuki_msgs::LedPtr led_msg_ptr;
      led_msg_ptr.reset(new kobuki_msgs::Led());
      led_msg_ptr->value = kobuki_msgs::Led::BLACK;
      led1_publisher_.publish(led_msg_ptr);
      led_bumper_on_ = false;
    }
    if (change_direction_)
    {
      ROS_INFO_STREAM("Bumper pressed. Changing direction. [" << name_ << "]");
    }
}

static void cliffCallback(const kobuki_msgs::CliffEventConstPtr msg)
{
  if (msg->state == kobuki_msgs::CliffEvent::CLIFF)
  {
    switch (msg->sensor)
    {
      case kobuki_msgs::CliffEvent::LEFT:
        if (!cliff_left_detected_)
        {
          cliff_left_detected_ = true;
          change_direction_ = true;
        }
        break;
      case kobuki_msgs::CliffEvent::CENTER:
        if (!cliff_center_detected_)
        {
          cliff_center_detected_ = true;
          change_direction_ = true;
        }
        break;
      case kobuki_msgs::CliffEvent::RIGHT:
        if (!cliff_right_detected_)
        {
          cliff_right_detected_ = true;
          change_direction_ = true;
        }
        break;
    }
  }
  else // kobuki_msgs::BumperEvent::FLOOR
  {
    switch (msg->sensor)
    {
      case kobuki_msgs::CliffEvent::LEFT:    cliff_left_detected_   = false; break;
      case kobuki_msgs::CliffEvent::CENTER:  cliff_center_detected_ = false; break;
      case kobuki_msgs::CliffEvent::RIGHT:   cliff_right_detected_  = false; break;
    }
  }
  if (!led_cliff_on_ && (cliff_left_detected_ || cliff_center_detected_ || cliff_right_detected_))
  {
    kobuki_msgs::LedPtr led_msg_ptr;
    led_msg_ptr.reset(new kobuki_msgs::Led());
    led_msg_ptr->value = kobuki_msgs::Led::ORANGE;
    led2_publisher_.publish(led_msg_ptr);
    led_cliff_on_ = true;
  }
  else if (led_cliff_on_ && (!cliff_left_detected_ && !cliff_center_detected_ && !cliff_right_detected_))
  {
    kobuki_msgs::LedPtr led_msg_ptr;
    led_msg_ptr.reset(new kobuki_msgs::Led());
    led_msg_ptr->value = kobuki_msgs::Led::BLACK;
    led2_publisher_.publish(led_msg_ptr);
    led_cliff_on_ = false;
  }
  if (change_direction_)
  {
    ROS_INFO_STREAM("Cliff detected. Changing direction. [" << name_ << "]");
  }

}

static void dropCallback(const kobuki_msgs::BumperEventConstPtr msg)
{
  if (msg->state == kobuki_msgs::WheelDropEvent::DROPPED)
  {
    switch (msg->wheel)
    {
      case kobuki_msgs::WheelDropEvent::LEFT:
        if (!wheel_drop_left_detected_)
        {
          wheel_drop_left_detected_ = true;
        }
        break;
      case kobuki_msgs::WheelDropEvent::RIGHT:
        if (!wheel_drop_right_detected_)
        {
          wheel_drop_right_detected_ = true;
        }
        break;
    }
  }
  else // kobuki_msgs::WheelDropEvent::RAISED
  {
    switch (msg->wheel)
    {
      case kobuki_msgs::WheelDropEvent::LEFT:    wheel_drop_left_detected_   = false; break;
      case kobuki_msgs::WheelDropEvent::RIGHT:   wheel_drop_right_detected_  = false; break;
    }
  }
  if (!led_wheel_drop_on_ && (wheel_drop_left_detected_ || wheel_drop_right_detected_))
  {
    kobuki_msgs::LedPtr led_msg_ptr;
    led_msg_ptr.reset(new kobuki_msgs::Led());
    led_msg_ptr->value = kobuki_msgs::Led::RED;
    led1_publisher_.publish(led_msg_ptr);
    led2_publisher_.publish(led_msg_ptr);
    stop_ = true;
    led_wheel_drop_on_ = true;
  }
  else if (led_wheel_drop_on_ && (!wheel_drop_left_detected_ && !wheel_drop_right_detected_))
  {
    kobuki_msgs::LedPtr led_msg_ptr;
    led_msg_ptr.reset(new kobuki_msgs::Led());
    led_msg_ptr->value = kobuki_msgs::Led::BLACK;
    led1_publisher_.publish(led_msg_ptr);
    led2_publisher_.publish(led_msg_ptr);
    stop_ = false;
    led_wheel_drop_on_ = false;
  }
  if (change_direction_)
  {
    ROS_INFO_STREAM("Wheel(s) dropped. Stopping. [" << name_ << "]");
  }
}

static void handle_sensor_input()
{
  if (this->getState()) // check, if the controller is active
  {
    // Velocity commands
    geometry_msgs::TwistPtr cmd_vel_msg_ptr;
    cmd_vel_msg_ptr.reset(new geometry_msgs::Twist());

    if (stop_)
    {
      vel_pub->publish(cmd_vel_msg_ptr); // will be all zero when initialised
      return;
    }

    if (change_direction_)
    {
      change_direction_ = false;
      // calculate a random turning angle (-180 ... +180) based on the set angular velocity
      // time for turning 180 degrees in seconds = M_PI / angular velocity
      turning_duration_ = ros::Duration(((double)std::rand() / (double)RAND_MAX) * (M_PI / vel_ang_));
      // randomly chosen turning direction
      if (((double)std::rand() / (double)RAND_MAX) >= 0.5)
      {
        turning_direction_ = 1;
      }
      else
      {
        turning_direction_ = -1;
      }
      turning_start_ = ros::Time::now();
      turning_ = true;
      ROS_INFO_STREAM("Will rotate " << turning_direction_ * turning_duration_.toSec() * vel_ang_ / M_PI * 180
                      << " degrees. [" << name_ << "]");
    }

    if (turning_)
    {
      if ((ros::Time::now() - turning_start_) < turning_duration_)
      {
        cmd_vel_msg_ptr->angular.z = turning_direction_ * vel_ang_;
        vel_pub->publish(cmd_vel_msg_ptr);
      }
      else
      {
        turning_ = false;
      }
    }
    else
    {
      cmd_vel_msg_ptr->linear.x = vel_lin_;
      vel_pub->publish(cmd_vel_msg_ptr);
    }
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::node::Node::make_shared("avoid_obstacles_node");
  vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",
    rmw_qos_profile_sensor_data);
  led1_pub = node->create_publisher<kobuki_msgs::Led>("commands/led1",
    rmw_qos_profile_sensor_data);
  led2_pub = node->create_publisher<kobuki_msgs::Led> ("commands/led2",
    rmw_qos_profile_sensor_data);
    

  auto bumper_sub = node->create_subscription<geometry_msgs::msg::Twist>(
    "events/bumper", bumperCallback, rmw_qos_profile_sensor_data);

  auto cliff_sub = node->create_subscription<geometry_msgs::msg::Twist>(
    "events/cliff", cliffCallback, rmw_qos_profile_sensor_data);
  
  auto drop_sub = node->create_subscription<geometry_msgs::msg::Twist>(
    "events/drop", dropCallback, rmw_qos_profile_sensor_data);

  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok()) {
      
      rclcpp::spin_some(node);
      handle_sensor_input();
      loop_rate.sleep();
      
  }
  
  return 0;
}
