/*
MIT License

Copyright (c) 2022 Lou Amadio

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <memory>
#include <chrono>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "can_msgs/msg/frame.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

// Keya Motor CAN Commands
// 
// The General format for CAN instructions are:
// Send Data: 0x0600000 + Controller ID (Hex)
// Received Data: 0x0580000 + Controller ID (Hex)
// Heartbeat: 0x0700000  + Controller ID (Hex)

// TODO: Commands must be sent every 1,000 ms or watchdog disables motor? 

// Heartbeat format:
// 0x0700000 + ID, then 8 Data Bytes:
// [0][1] - Angle:  0-1,000 Degrees?
// [2][3] - Motor Speed: +/- rads/s? Degree/s?
// [4][5] - Rated Speed: 0-1,000 
// [6][7] - Fault Code

// TODO: Conflicting data on bytes 4/5 & 2/3. One section says that 4/5 is the actual speed, one says it is the rated speed.


// Command Format:
// 0x0600000 + ID 
// [0][1][2][3][4][5][6][7] Write Address
// [4][5][6][7][8][9][A][B] Command Specific

// The following Commands are supported:
// 
// Motor Enable
// Send: 0x0600000 0x230D2001 0x00000000
// Return: 0x0580000 0x600D2000 0x00000000

// Motor Disable
// Send: 0x0600000 0x230C2001 0x00000000
// Return: 0x0580000 0x600C2000 0x00000000

// Set Motor Speed
// Send: 0x0600000 0x23002001 0xssssssss
// Return: 0x0580000 0x60002000 0x00000000
// Notes:
// Speed: 32bit signed integer, -1,000 to 1,000
// the 1,000 value is interpreted as percent of max set speed, where 1,000 == 100% of max speed
// 0x3E8 = 1000
// 0xFFFFFC18 = -1000
// So: 
// 0x23002001 0x01F40000 is 500
// 0x23002001 0xC1C0FFFF is -500

// Position Control
// Send: 0x0600000 0x23022001 0xpppppppp

// Notes:
// Position: a 32bit signed integer
// A value of 10,000 represents 360 degrees
// Use the Formula: (Degrees * 10,000/360)
// For example:
// 76 degrees * (10,000/360) = 2,111 = 0x0000083F
// So:
// 0x0600000 0x23022001 0x083F0000

// Order is important, so the following commands need to be sent to enter position control mode:
// 0x0600000 0x230C2001 0x00000000  // Disable Motor
// 0x0600000 0x230D2001 0x00000000  // Enable Motor
// 0x0600000 0x23022001 0xpppppppp  // Set Position

constexpr inline float DegToRad(float deg)
{
  return (deg * M_PI) / 180.0f;
}

class KeyaServo : public rclcpp::Node
{
  public:
    KeyaServo()
    : Node("keya_servo")
    {
    }

    void initialize()
    {
      heartbeat_timer_ = this->create_wall_timer(
        heartbeat_interval_, std::bind(&KeyaServo::sendHeartbeat, this));

      can_subscriber_ = this->create_subscription<can_msgs::msg::Frame>(
        "/from_can_bus", 10, std::bind(&KeyaServo::can_callback, this, _1));

      position_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
        "/position", 1, std::bind(&KeyaServo::position_callback, this, _1));

      can_publisher_ = this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 500);

      sendDisable();
      std::this_thread::sleep_for(100ms);
      sendPositionControlMode();
      sendPositionReset();
    }

    void can_callback(const can_msgs::msg::Frame::SharedPtr msg)
    {
      // Check if the message is for this motor
      if (msg->id != (can_id_ + 0x07000000))
      {
        return;
      }

      // Check if the message is a heartbeat
      if (msg->data[0] == 0x07)
      {

      }
    }

    void position_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
      float degrees = msg->data;
      sendEnable();
      std::this_thread::sleep_for(100ms);

      int32_t position = (int32_t)((DegToRad(degrees) * 10000.0f) / 360.0f);
      current_position_ = position;

      sendSetPosition(position);
    }

    void sendPositionControlMode()
    {
      uint8_t data[8] = {0x03, 0x0D, 0x20, 0x31, 0x00, 0x00, 0x00, 0x00};
      send_can_message(can_id_ + 0x06000000, data);
    }

    void sendPositionReset()
    {
      uint8_t data[8] = {0x23, 0x0C, 0x20, 0x09, 0x00, 0x00, 0x00, 0x0};
      send_can_message(can_id_ + 0x06000000, data);
    }

    void sendHeartbeat()
    {
      uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      send_can_message(can_id_ + 0x06000000, data);
    }

    void sendEnable()
    {
      uint8_t data[8] = {0x23, 0x0D, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00};
      send_can_message(can_id_ + 0x06000000, data);
    }

    void sendDisable()
    {
      uint8_t data[8] = {0x23, 0x0C, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00};
      send_can_message(can_id_ + 0x06000000, data);
    }

    void sendSetSpeed(int32_t speed)
    {
      uint8_t data[8];
      data[0] = 0x23;
      data[1] = 0x00;
      data[2] = 0x20;
      data[3] = 0x01;
      data[4] = (speed >> 8) & 0xFF;
      data[5] = speed & 0xFF;
      data[6] = (speed >> 24) & 0xFF;
      data[7] = (speed >> 16) & 0xFF;
      send_can_message(can_id_ + 0x06000000, data);
    }

    void sendSetPosition(float angle)
    {
      // Use the Formula: (angle / 10,000) * 360 to convert to degrees or
      // to convert from degrees to position: (Degrees * 10,000/360)
      // For example:
      // 76 degrees * (10,000/360) = 2,111 = 0x0000083F
      // So:
      // 0x0600000 0x23022001 0x0083F0000

      int32_t position = (int32_t)(angle * 10000.0f / 360.0f);

      uint8_t data[8];
      data[0] = 0x23;
      data[1] = 0x02;
      data[2] = 0x20;
      data[3] = 0x01;
      data[4] = (position >> 8) & 0xFF;
      data[5] = position & 0xFF;
      data[6] = (position >> 24) & 0xFF;
      data[7] = (position >> 16) & 0xFF;

      send_can_message(can_id_ + 0x06000000, data);
    }

    void send_can_message(uint32_t id, uint8_t* data)
    {
      // Create a can frame with a heartbeat message
      can_msgs::msg::Frame frame;
      frame.id = id;
      frame.is_extended = true;
      frame.is_rtr = false;
      frame.is_error = false;
      frame.dlc = 8;
      frame.data[0] = data[0];
      frame.data[1] = data[1];
      frame.data[2] = data[2];
      frame.data[3] = data[3];
      frame.data[4] = data[4];
      frame.data[5] = data[5];
      frame.data[6] = data[6];
      frame.data[7] = data[7];

      // Publish the frame
      can_publisher_->publish(frame);
    }


  private:

  uint32_t can_id_ = 1;
  std::chrono::milliseconds heartbeat_interval_ = 500ms;
  int32_t current_position_ = 0;

  bool speed_or_position_ = true; // True = speed, False = position

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr position_subscriber_;

  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_subscriber_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<KeyaServo>();

    node->initialize();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}