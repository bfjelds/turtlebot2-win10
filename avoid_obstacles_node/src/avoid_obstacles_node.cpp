#define _USE_MATH_DEFINES // for C++  
#include <cmath>  

#include <algorithm>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/twist.hpp>

#include <kobuki_msgs/msg/bumper_event.hpp>
#include <kobuki_msgs/msg/cliff_event.hpp>
#include <kobuki_msgs/msg/led.hpp>
#include <kobuki_msgs/msg/wheel_drop_event.hpp>

#include <chrono>

//
// Publishers
//
rclcpp::publisher::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
rclcpp::publisher::Publisher<kobuki_msgs::msg::Led>::SharedPtr led1_pub;
rclcpp::publisher::Publisher<kobuki_msgs::msg::Led>::SharedPtr led2_pub;

bool verbose_ = true;

//
// State
//
/// Flag for changing direction
bool change_direction_;
/// Flag for retreating
bool retreat_;
/// Flag for retreating
bool retreating_;
/// retreating start
std::chrono::duration<double> retreating_start_;
/// retreating duration
std::chrono::duration<double> retreating_duration_;
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
std::chrono::duration<double> turning_duration_;
/// Randomly chosen turning direction
int turning_direction_;
/// Start time of turning
std::chrono::duration<double> turning_start_;
/// Flag for turning state
bool turning_;

static void dumpCurrent()
{
  if (verbose_)
  {
    printf("\t**************************************** \r\n");
    printf("\tbumpers on: left=%d middle=%d right=%d \r\n", bumper_left_pressed_, bumper_center_pressed_, bumper_right_pressed_);
    printf("\tcliff found: left=%d middle=%d right=%d \r\n", cliff_left_detected_, cliff_center_detected_, cliff_right_detected_);
    printf("\tdrop found: left=%d right=%d \r\n", wheel_drop_left_detected_, wheel_drop_right_detected_);

    printf("\tstop=%d \r\n", stop_);
    printf("\tretreat=%d retreating=%d \r\n", retreat_, retreating_);
    printf("\tturning=%d \r\n", turning_);
    printf("\t**************************************** \r\n");
  }
}


static void bumperCallback(const kobuki_msgs::msg::BumperEvent::ConstSharedPtr msg)
{
  if (verbose_) printf("bumperCallback %d \r\n", msg->bumper);

  if (msg->state == kobuki_msgs::msg::BumperEvent::PRESSED && !change_direction_)
  {
    switch (msg->bumper)
    {
    case kobuki_msgs::msg::BumperEvent::LEFT:
      if (!bumper_left_pressed_)
      {
        bumper_left_pressed_ = true;
        change_direction_ = true;
      }
      break;
    case kobuki_msgs::msg::BumperEvent::CENTER:
      if (!bumper_center_pressed_)
      {
        bumper_center_pressed_ = true;
        change_direction_ = true;
      }
      break;
    case kobuki_msgs::msg::BumperEvent::RIGHT:
      if (!bumper_right_pressed_)
      {
        bumper_right_pressed_ = true;
        change_direction_ = true;
      }
      break;
    }
  }
  else if (msg->state == kobuki_msgs::msg::BumperEvent::RELEASED)
  {
    switch (msg->bumper)
    {
    case kobuki_msgs::msg::BumperEvent::LEFT:  bumper_left_pressed_ = false; break;
    case kobuki_msgs::msg::BumperEvent::CENTER:  bumper_center_pressed_ = false; break;
    case kobuki_msgs::msg::BumperEvent::RIGHT:   bumper_right_pressed_ = false; break;
    }

    if (change_direction_) return;
  }
  if (!led_bumper_on_ && (bumper_left_pressed_ || bumper_center_pressed_ || bumper_right_pressed_))
  {
    kobuki_msgs::msg::Led::SharedPtr led_msg_ptr;
    led_msg_ptr.reset(new kobuki_msgs::msg::Led());
    led_msg_ptr->value = kobuki_msgs::msg::Led::ORANGE;
    led1_pub->publish(led_msg_ptr);
    led_bumper_on_ = true;
  }
  else if (led_bumper_on_ && (!bumper_left_pressed_ && !bumper_center_pressed_ && !bumper_right_pressed_))
  {
    kobuki_msgs::msg::Led::SharedPtr led_msg_ptr;
    led_msg_ptr.reset(new kobuki_msgs::msg::Led());
    led_msg_ptr->value = kobuki_msgs::msg::Led::BLACK;
    led1_pub->publish(led_msg_ptr);
    led_bumper_on_ = false;
  }
  if (change_direction_)
  {
    retreat_ = true;
  }

  dumpCurrent();
}

static void cliffCallback(const kobuki_msgs::msg::CliffEvent::ConstSharedPtr msg)
{
  if (verbose_) printf("cliffCallback %d \r\n", msg->sensor);

  if (msg->state == kobuki_msgs::msg::CliffEvent::CLIFF && !change_direction_)
  {
    switch (msg->sensor)
    {
    case kobuki_msgs::msg::CliffEvent::LEFT:
      if (!cliff_left_detected_)
      {
        cliff_left_detected_ = true;
        change_direction_ = true;
      }
      break;
    case kobuki_msgs::msg::CliffEvent::CENTER:
      if (!cliff_center_detected_)
      {
        cliff_center_detected_ = true;
        change_direction_ = true;
      }
      break;
    case kobuki_msgs::msg::CliffEvent::RIGHT:
      if (!cliff_right_detected_)
      {
        cliff_right_detected_ = true;
        change_direction_ = true;
      }
      break;
    }
  }
  else if (msg->state == kobuki_msgs::msg::CliffEvent::FLOOR)
  {
    switch (msg->sensor)
    {
    case kobuki_msgs::msg::CliffEvent::LEFT:  cliff_left_detected_ = false; break;
    case kobuki_msgs::msg::CliffEvent::CENTER:  cliff_center_detected_ = false; break;
    case kobuki_msgs::msg::CliffEvent::RIGHT:   cliff_right_detected_ = false; break;
    }

    if (change_direction_) return;
  }
  if (!led_cliff_on_ && (cliff_left_detected_ || cliff_center_detected_ || cliff_right_detected_))
  {
    kobuki_msgs::msg::Led::SharedPtr led_msg_ptr;
    led_msg_ptr.reset(new kobuki_msgs::msg::Led());
    led_msg_ptr->value = kobuki_msgs::msg::Led::ORANGE;
    led2_pub->publish(led_msg_ptr);
    led_cliff_on_ = true;
  }
  else if (led_cliff_on_ && (!cliff_left_detected_ && !cliff_center_detected_ && !cliff_right_detected_))
  {
    kobuki_msgs::msg::Led::SharedPtr led_msg_ptr;
    led_msg_ptr.reset(new kobuki_msgs::msg::Led());
    led_msg_ptr->value = kobuki_msgs::msg::Led::BLACK;
    led2_pub->publish(led_msg_ptr);
    led_cliff_on_ = false;
  }
  if (change_direction_)
  {
    //ROS_INFO_STREAM("Cliff detected. Changing direction. [" << name_ << "]");
  }

  dumpCurrent();
}

static void dropCallback(const kobuki_msgs::msg::WheelDropEvent::ConstSharedPtr msg)
{
  if (verbose_) printf("dropCallback %d \r\n", msg->wheel);

  if (msg->state == kobuki_msgs::msg::WheelDropEvent::DROPPED && !stop_)
  {
    switch (msg->wheel)
    {
    case kobuki_msgs::msg::WheelDropEvent::LEFT:
      if (!wheel_drop_left_detected_)
      {
        wheel_drop_left_detected_ = true;
      }
      break;
    case kobuki_msgs::msg::WheelDropEvent::RIGHT:
      if (!wheel_drop_right_detected_)
      {
        wheel_drop_right_detected_ = true;
      }
      break;
    }
  }
  else if (msg->state == kobuki_msgs::msg::WheelDropEvent::RAISED)
  {
    switch (msg->wheel)
    {
    case kobuki_msgs::msg::WheelDropEvent::LEFT:  wheel_drop_left_detected_ = false; break;
    case kobuki_msgs::msg::WheelDropEvent::RIGHT:   wheel_drop_right_detected_ = false; break;
    }

    if (stop_) return;
  }
  if (!led_wheel_drop_on_ && (wheel_drop_left_detected_ || wheel_drop_right_detected_))
  {
    kobuki_msgs::msg::Led::SharedPtr led_msg_ptr;
    led_msg_ptr.reset(new kobuki_msgs::msg::Led());
    led_msg_ptr->value = kobuki_msgs::msg::Led::RED;
    led1_pub->publish(led_msg_ptr);
    led2_pub->publish(led_msg_ptr);
    stop_ = true;
    led_wheel_drop_on_ = true;
  }
  else if (led_wheel_drop_on_ && (!wheel_drop_left_detected_ && !wheel_drop_right_detected_))
  {
    kobuki_msgs::msg::Led::SharedPtr led_msg_ptr;
    led_msg_ptr.reset(new kobuki_msgs::msg::Led());
    led_msg_ptr->value = kobuki_msgs::msg::Led::BLACK;
    led1_pub->publish(led_msg_ptr);
    led2_pub->publish(led_msg_ptr);
    stop_ = false;
    led_wheel_drop_on_ = false;
  }
  if (change_direction_)
  {
    //ROS_INFO_STREAM("Wheel(s) dropped. Stopping. [" << name_ << "]");
  }

  dumpCurrent();
}

static void handle_sensor_input()
{
  //
  // TODO: synchronize handle_sensor_input and callbacks
  //
  if (verbose_)
  {
    printf("\t handle_sensor_input ... \r\n");
    dumpCurrent();
  }

  {
    // Velocity commands
    geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg_ptr;
    cmd_vel_msg_ptr.reset(new geometry_msgs::msg::Twist());

    if (stop_)
    {
      vel_pub->publish(cmd_vel_msg_ptr); // will be all zero when initialised
      return;
    }

    if (retreat_)
    {
      retreat_ = false;

      std::chrono::high_resolution_clock clock;
      retreating_start_ =
        std::chrono::duration_cast<std::chrono::seconds>(clock.now().time_since_epoch());
      retreating_duration_ = std::chrono::milliseconds(500);
      retreating_ = true;
    }

    if (retreating_)
    {
      retreat_ = false;

      std::chrono::high_resolution_clock clock;
      std::chrono::duration<double> now =
        std::chrono::duration_cast<std::chrono::seconds>(clock.now().time_since_epoch());
      if (now <= (retreating_start_ + retreating_duration_))
      {
        cmd_vel_msg_ptr->linear.x = -1 * vel_lin_;
        if (verbose_) printf("retreating (%f) ... \r\n", cmd_vel_msg_ptr->linear.x);
        vel_pub->publish(cmd_vel_msg_ptr);

        return;
      }
      else
      {
        if (verbose_) printf("done retreating\r\n");
        retreating_ = false;
      }
    }

    if (change_direction_)
    {
      change_direction_ = false;

#ifdef RANDOMLY_TURN
      auto seconds = ((double)std::rand() / (double)RAND_MAX) * (M_PI / vel_ang_);
#else
      //
      // TURN 50% of pi / vel_ang
      //
      auto seconds = ((double)0.25) * (M_PI / vel_ang_);
#endif
      turning_duration_ = std::chrono::duration<double>(seconds);
      if (verbose_) printf("turn for %f seconds ... \r\n", turning_duration_.count());

#ifdef RANDOMLY_TURN
      // randomly chosen turning direction
      if (((double)std::rand() / (double)RAND_MAX) >= 0.5)
      {
        turning_direction_ = 1;
      }
      else
      {
        turning_direction_ = -1;
      }
#else

      //
      // TURN counter-clockwise
      //
      turning_direction_ = 1;

#endif

      std::chrono::high_resolution_clock clock;
      turning_start_ =
        std::chrono::duration_cast<std::chrono::seconds>(clock.now().time_since_epoch());
      if (verbose_) printf("turn starting %f ... \r\n", turning_start_.count());
      turning_ = true;
    }

    if (turning_)
    {
      std::chrono::high_resolution_clock clock;
      std::chrono::duration<double> now =
        std::chrono::duration_cast<std::chrono::seconds>(clock.now().time_since_epoch());
      if (now <= (turning_start_+ turning_duration_))
      {
        cmd_vel_msg_ptr->angular.z = turning_direction_ * vel_ang_;
        if (verbose_) printf("turning (%f) [now - start = %f] ... \r\n", cmd_vel_msg_ptr->angular.z, (now - turning_start_).count());
        vel_pub->publish(cmd_vel_msg_ptr);
      }
      else
      {
        if (verbose_) printf("done turning \r\n");
        turning_ = false;
      }
    }
    else
    {
      cmd_vel_msg_ptr->linear.x = vel_lin_;
      if (verbose_) printf("moving (%f) ... \r\n", cmd_vel_msg_ptr->linear.x);
      vel_pub->publish(cmd_vel_msg_ptr);
    }
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  vel_lin_ = 0.1;
  vel_ang_ = 0.5;

  auto node = std::make_shared<rclcpp::Node>("avoid_obstacles_node");
  vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",
    rmw_qos_profile_sensor_data);
  led1_pub = node->create_publisher<kobuki_msgs::msg::Led>("commands/led1",
    rmw_qos_profile_sensor_data);
  led2_pub = node->create_publisher<kobuki_msgs::msg::Led>("commands/led2",
    rmw_qos_profile_sensor_data);

  
  auto bumper_sub = node->create_subscription<kobuki_msgs::msg::BumperEvent>(
    "events/bumper", bumperCallback, rmw_qos_profile_sensor_data);
  auto cliff_sub = node->create_subscription<kobuki_msgs::msg::CliffEvent>(
    "events/cliff", cliffCallback, rmw_qos_profile_sensor_data);
  auto drop_sub = node->create_subscription<kobuki_msgs::msg::WheelDropEvent>(
    "events/drop", dropCallback, rmw_qos_profile_sensor_data);

  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok()) {

    rclcpp::spin_some(node);
    handle_sensor_input();
    loop_rate.sleep();

  }

  return 0;
}
