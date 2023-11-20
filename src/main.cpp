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
// Position: 32bit signed integer
// A value of 10,000 represents 360 degrees
// Use the Formula: (Position / 10,000) * 360 to convert to degrees or
// to convert from degrees to position: (Degrees * 10,000/360)
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
      position_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
      "position", 10, std::bind(&KeyaServo::position_callback, this, _1));
    }

    void position_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
    }


  private:

  // TODO: Add a parameter for the CAN ID
  // TODO: Add a parameter for the CAN interface
  // TODO: Add a parameter for the CAN baud rate
  // TODO: Add a parameter for the CAN timeout
  // TODO: Add a parameter for the CAN heartbeat rate

  // ROS subscriber for motor position

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr position_subscriber_;

  // ROS publisher for ros2_socketcan_msgs::msg::Frame
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;

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