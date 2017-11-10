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

#include <sensor_msgs/msg/joy.hpp>

#include <chrono>

rclcpp::Node::SharedPtr node;
//
// Publishers
//
rclcpp::publisher::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
rclcpp::publisher::Publisher<kobuki_msgs::msg::Led>::SharedPtr led1_pub;
rclcpp::publisher::Publisher<kobuki_msgs::msg::Led>::SharedPtr led2_pub;

bool verbose_ = false;
bool callbackverbose_ = false;
bool dumpStateverbose_ = false;

//
// State
//
/// Flag for enabling self direction
bool self_directed_;
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

static void initState(bool self_directed)
{
  self_directed_ = self_directed;
  change_direction_ = false;
  retreat_ = false;
  retreating_ = false;
  retreating_start_ = std::chrono::duration<double>();
  retreating_duration_ = std::chrono::duration<double>();
  stop_ = false;
  bumper_left_pressed_ = false;
  bumper_center_pressed_ = false;
  bumper_right_pressed_ = false;
  cliff_left_detected_ = false;
  cliff_center_detected_ = false;
  cliff_right_detected_ = false;
  wheel_drop_left_detected_ = false;
  wheel_drop_right_detected_ = false;
  led_bumper_on_ = false;
  led_cliff_on_ = false;
  led_wheel_drop_on_ = false;
  vel_lin_ = 0.1;
  vel_ang_ = 0.5;
  turning_duration_ = std::chrono::duration<double>();;
  turning_direction_ = false;
  turning_start_ = std::chrono::duration<double>();;
  turning_ = false;
}

static void dumpCurrent()
{
  if (dumpStateverbose_)
  {
    printf("\t**************************************** \r\n");
    printf("\tself-directed=%d \r\n", self_directed_);

    printf("\tbumpers on: left=%d middle=%d right=%d \r\n", bumper_left_pressed_, bumper_center_pressed_, bumper_right_pressed_);
    printf("\tcliff found: left=%d middle=%d right=%d \r\n", cliff_left_detected_, cliff_center_detected_, cliff_right_detected_);
    printf("\tdrop found: left=%d right=%d \r\n", wheel_drop_left_detected_, wheel_drop_right_detected_);

    printf("\tstop=%d \r\n", stop_);
    printf("\tretreat=%d retreating=%d \r\n", retreat_, retreating_);
    printf("\tturning=%d \r\n", turning_);
    printf("\t**************************************** \r\n");
  }
}

const static uint16_t SELF_DIRECTED_ON = 0;
const static uint16_t SELF_DIRECTED_OFF = 1;
const static uint16_t GAMEPAD_ON = 5;
static void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr joy_msg)
{
  auto previously_self_directed = self_directed_;
  auto button_count = joy_msg->buttons.size();
  if (callbackverbose_ || verbose_) printf("joyCallback self-directed=%d gamepad-directed=%d gamepad-now=%d \r\n", 
    (button_count > SELF_DIRECTED_ON) ? (joy_msg->buttons[SELF_DIRECTED_ON] > 0) : false,
    (button_count > SELF_DIRECTED_OFF) ? (joy_msg->buttons[SELF_DIRECTED_OFF] > 0) : false,
    (button_count > GAMEPAD_ON) ? (joy_msg->buttons[GAMEPAD_ON] > 0) : false);

  if (button_count > SELF_DIRECTED_ON && joy_msg->buttons[SELF_DIRECTED_ON] > 0) self_directed_ = true;
  if (button_count > SELF_DIRECTED_OFF && joy_msg->buttons[SELF_DIRECTED_OFF] > 0) self_directed_ = false;
  if (button_count > GAMEPAD_ON && joy_msg->buttons[GAMEPAD_ON] > 0) self_directed_ = false;

  if (previously_self_directed != self_directed_)
  {
    geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg_ptr;
    cmd_vel_msg_ptr.reset(new geometry_msgs::msg::Twist());
    if (verbose_) printf("\t not self-directed anymore, stopping ... \r\n");
    vel_pub->publish(cmd_vel_msg_ptr); // will be all zero when initialised
    initState(self_directed_);
  }

  dumpCurrent();
}

static void bumperCallback(const kobuki_msgs::msg::BumperEvent::ConstSharedPtr msg)
{
  if (callbackverbose_ || verbose_) printf("bumperCallback %d \r\n", msg->bumper);

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
  if (callbackverbose_ || verbose_) printf("cliffCallback %d \r\n", msg->sensor);

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
    retreat_ = true;
  }

  dumpCurrent();
}

static void dropCallback(const kobuki_msgs::msg::WheelDropEvent::ConstSharedPtr msg)
{
  if (callbackverbose_ || verbose_) printf("dropCallback %d \r\n", msg->wheel);

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
    retreat_ = true;
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
    printf("handle_sensor_input start ... \r\n");
    dumpCurrent();
  }

  if (!self_directed_)
  {
    if (verbose_) printf("\t NOT self-directed \r\n");
  }
  else
  {
    if (verbose_) printf("\t AM self-directed ... \r\n");
    
    // Velocity commands
    geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg_ptr;
    cmd_vel_msg_ptr.reset(new geometry_msgs::msg::Twist());

    if (stop_)
    {
      if (verbose_) printf("\t stopping ... \r\n");
      vel_pub->publish(cmd_vel_msg_ptr); // will be all zero when initialised
      if (verbose_) printf("\t stopped ... \r\n");
      return;
    }

    if (retreat_ && self_directed_)
    {
      retreat_ = false;

      std::chrono::high_resolution_clock clock;
      retreating_start_ =
        std::chrono::duration_cast<std::chrono::seconds>(clock.now().time_since_epoch());
      retreating_duration_ = std::chrono::seconds(1);
      if (verbose_) printf("\t starting retreat ... \r\n");
      retreating_ = true;
    }

    if (retreating_ && self_directed_)
    {
      retreat_ = false;

      std::chrono::high_resolution_clock clock;
      std::chrono::duration<double> now =
        std::chrono::duration_cast<std::chrono::seconds>(clock.now().time_since_epoch());
      if (self_directed_ && (now <= (retreating_start_ + retreating_duration_)))
      {
        cmd_vel_msg_ptr->linear.x = -1 * vel_lin_;
        if (verbose_) printf("\t retreating (%f) ... \r\n", cmd_vel_msg_ptr->linear.x);
        vel_pub->publish(cmd_vel_msg_ptr);
        return;
      }
      else
      {
        if (verbose_) printf("\t done retreating\r\n");
        retreating_ = false;
      }
    }

    if (change_direction_ && self_directed_)
    {
      if (verbose_) printf("\t changing direction ...\r\n");
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
      if (verbose_) printf("\t turn starting %f ... \r\n", turning_start_.count());
      turning_ = true;
    }

    if (turning_ && self_directed_)
    {
      std::chrono::high_resolution_clock clock;
      std::chrono::duration<double> now =
        std::chrono::duration_cast<std::chrono::seconds>(clock.now().time_since_epoch());
      if (self_directed_ && (now <= (turning_start_+ turning_duration_)))
      {
        cmd_vel_msg_ptr->angular.z = turning_direction_ * vel_ang_;
        if (verbose_) printf("\t turning (%f) [now - start = %f] ... \r\n", cmd_vel_msg_ptr->angular.z, (now - turning_start_).count());
        vel_pub->publish(cmd_vel_msg_ptr);
      }
      else
      {
        if (verbose_) printf("\t done turning \r\n");
        turning_ = false;
      }
    }
    else if (self_directed_)
    {
      cmd_vel_msg_ptr->linear.x = vel_lin_;
      if (verbose_) printf("\t moving (%f) ... \r\n", cmd_vel_msg_ptr->linear.x);
      vel_pub->publish(cmd_vel_msg_ptr);
    }
  }
  if (verbose_) printf("handle_sensor_input done\r\n");
};

static void motionHandler()
{
  printf("Motion thread ... \r\n");

  long long iteration = 0;
  while (true) {

    printf("Motion control loop iteration %lld \r\n", iteration++);

    handle_sensor_input();

    std::this_thread::sleep_for(std::chrono::milliseconds(20));

  }
}

static void ros2ListenerHandler()
{
  printf("ROS2 Listener thread ... \r\n");

  rclcpp::WallRate loop_rate(200);
  long long iteration = 0;
  while (rclcpp::ok()) {

    printf("ROS2 listener loop iteration %lld \r\n", iteration++);
    rclcpp::spin_some(node);
    loop_rate.sleep();

  }


}

int main(int argc, char * argv[])
{
  initState(true);

  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("avoid_obstacles_node");
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
  auto joy_sub = node->create_subscription<sensor_msgs::msg::Joy>(
    "joy", joyCallback, rmw_qos_profile_sensor_data);

  std::thread rosListenerThread(ros2ListenerHandler);
  std::thread motionThread(motionHandler);
  rosListenerThread.join();
  motionThread.join();

  return 0;
}
