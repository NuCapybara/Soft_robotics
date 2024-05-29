#include <iostream>
#include <SerialPort.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "tentacle_interfaces/msg/actuation.hpp"
#include <vector>
#include <chrono> // Include chrono header for chrono literals

// Use 'LibSerial::' instead of 'using namespace LibSerial;'
using LibSerial::SerialPort;
using LibSerial::BaudRate;
using LibSerial::CharacterSize;
using LibSerial::Parity;
using LibSerial::StopBits;
using LibSerial::FlowControl;
using namespace std::chrono_literals; // Add this line for chrono literals

class controller : public rclcpp::Node
{
public:
    controller() : Node("controller")
    {
        // Parameter description
        auto timer_frequency_des = rcl_interfaces::msg::ParameterDescriptor{};
        auto baud_rate_des = rcl_interfaces::msg::ParameterDescriptor{};

        timer_frequency_des.description = "Timer callback frequency [Hz]";
        baud_rate_des.description = "Baud rate for communication [bits/s]";

        // Declare default parameters values
        declare_parameter("timer_frequency", -1.0, timer_frequency_des);     // Hz 
        declare_parameter("baud_rate_int", -1, baud_rate_des);         // Bits per second

        // Get params - Read params from yaml file that is passed in the launch file
        timer_frequency_ = get_parameter("timer_frequency").get_parameter_value().get<double>();
        baud_rate_int_ = get_parameter("baud_rate_int").get_parameter_value().get<int>();
        baud_rate_ = integerToBaudRate(baud_rate_int_);

        // Check all params
        check_yaml_params();

        try 
        {
            serial_port_.Open("/dev/ttyUSB0"); // Adjust the port name as necessary
            serial_port_.SetBaudRate(baud_rate_); // Adjust the baud rate
            serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
            serial_port_.SetParity(Parity::PARITY_NONE);
            serial_port_.SetStopBits(StopBits::STOP_BITS_1);
            serial_port_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);

            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
        }
        catch (const LibSerial::OpenFailed &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
            throw;
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "An error occurred during serial port setup: %s", e.what());
            throw;
        }

        // Initialize actuation
        // Initialize actuator names
        for(int i = 1; i <= actuator_count_; i++)
        {
            states_.push_back(false);
        }

        // Subscribers
        cmd_actuation_subscriber_ = create_subscription<tentacle_interfaces::msg::Actuation>(
        "/cmd_actuation", 10, std::bind(
            &controller::cmd_actuation_callback, this,
            std::placeholders::_1));

        // Create timer to periodically send control commands
        timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / timer_frequency_)), std::bind(&controller::sendArduinoCommands, this));
    }

private:
    double timer_frequency_ = -1.0; // Hz
    int baud_rate_int_ = -1; // bits per second
    BaudRate baud_rate_ = LibSerial::BaudRate::BAUD_INVALID; // bits per second

    SerialPort serial_port_;
    rclcpp::TimerBase::SharedPtr timer_;
    int actuator_count_ = 6;
    std::vector<bool> states_;
 
    rclcpp::Subscription<tentacle_interfaces::msg::Actuation>::SharedPtr cmd_actuation_subscriber_;

    void sendArduinoCommands()
    {
        // WRITE

        // Create character array message
        std::vector<char> msg;

        for(int i = 0; i < actuator_count_;  i++)
        {
            if (states_.at(i))
            {
                msg.push_back(1);
            }
            else
            {
                msg.push_back(0);
            }
        }

        // Send data over serial
        std::vector<char> bytes_written(msg.size() * sizeof(char));
        std::memcpy(bytes_written.data(), msg.data(), bytes_written.size());
        std::string data_string(bytes_written.begin(), bytes_written.end());
        serial_port_.Write(data_string);

        // Debug output
        // RCLCPP_INFO(this->get_logger(), "Sent control commands: Ac1=%d, Ac2=%d, Av=%d", msg.at(0), msg.at(1), msg.at(2));
        // RCLCPP_INFO(this->get_logger(), "Sent control commands: %c, %c, %c", data_string.at(0), data_string.at(1), data_string.at(2));

        // READ

        // Wait for response
        std::vector<char> response;
        try 
        {
            // int timeout_ms = 1000/timer_frequency_; // timeout value in milliseconds
            // int timeout_ms = 1000; // timeout value in milliseconds
            int timeout_ms = 1; // timeout value in milliseconds

            char reading = 0;      // variable to store the read result

            for (int i = 1; i <= actuator_count_ + 2; i++)
            {
                serial_port_.ReadByte( reading, timeout_ms );
                response.push_back(reading);
            }

            // RCLCPP_INFO(this->get_logger(), "UNGA BUNGA: %d", reading);
        } 
        catch (const std::exception& e) 
        {
            RCLCPP_DEBUG(this->get_logger(), "Exception occurred while reading from serial port: %s", e.what());
        }

        if ((response.size() == static_cast<size_t>(actuator_count_ + 2)) && (response.at(0) == '!')) 
        {   
            // RCLCPP_INFO(this->get_logger(), "Received confirmation: %c, %d, %d, %d", response.at(0), response.at(1), response.at(2), response.at(3)); // 3 actuators
            RCLCPP_INFO(this->get_logger(), "Received confirmation: %c, %d, %d, %d, %d, %d, %d", response.at(0), response.at(1), response.at(2), response.at(3), response.at(4), response.at(5), response.at(6)); // 6 actuators
            // RCLCPP_INFO(this->get_logger(), "Received confirmation: %c, %d, %d, %d, %d, %d, %d, %d, %d, %d", response.at(0), response.at(1), response.at(2), response.at(3), response.at(4), response.at(5), response.at(6), response.at(7), response.at(8), response.at(9)); // 9 actuators
        } 
        else 
        {
            RCLCPP_DEBUG(this->get_logger(), "Invalid response from Arduino");
        }
    }

    /// \brief cmd_actuation topic callback
    void cmd_actuation_callback(const tentacle_interfaces::msg::Actuation& msg)
    {
        // speed_ = msg.speed;
        // steering_ = msg.steering;
        // int temp = 0;

        for (int i = 1; i <= actuator_count_; i++)
        {
            states_.at(i-1) = msg.states[i-1];
        }
    }

    void check_yaml_params()
    {
        if (timer_frequency_ == -1.0 ||
            baud_rate_int_ == -1
            )
        {
            RCLCPP_ERROR(this->get_logger(), "Param timer frequency: %f", timer_frequency_);
            RCLCPP_ERROR(this->get_logger(), "Param Baud rate: %d", baud_rate_int_);
            
            throw std::runtime_error("Missing necessary parameters in comm.yaml!");
        }
        if (timer_frequency_ <= 0.0 ||
            baud_rate_ == LibSerial::BaudRate::BAUD_INVALID 
          )
        {
            RCLCPP_ERROR(this->get_logger(), "Param timer frequency: %f", timer_frequency_);
            RCLCPP_ERROR(this->get_logger(), "Param Baud rate: %d", baud_rate_int_);
            
            throw std::runtime_error("Incorrect params in comm.yaml!");
        }
    }

    // Function to convert an integer to a BaudRate enum value
    BaudRate integerToBaudRate(int baud_int)
    {
        // Check the integer value against each possible baud rate using a switch-case
        switch (baud_int)
        {
            case 50:
                return BaudRate::BAUD_50;
            case 75:
                return BaudRate::BAUD_75;
            case 110:
                return BaudRate::BAUD_110;
            case 134:
                return BaudRate::BAUD_134;
            case 150:
                return BaudRate::BAUD_150;
            case 200:
                return BaudRate::BAUD_200;
            case 300:
                return BaudRate::BAUD_300;
            case 600:
                return BaudRate::BAUD_600;
            case 1200:
                return BaudRate::BAUD_1200;
            case 1800:
                return BaudRate::BAUD_1800;
            case 2400:
                return BaudRate::BAUD_2400;
            case 4800:
                return BaudRate::BAUD_4800;
            case 9600:
                return BaudRate::BAUD_9600;
            case 19200:
                return BaudRate::BAUD_19200;
            case 38400:
                return BaudRate::BAUD_38400;
            case 57600:
                return BaudRate::BAUD_57600;
            case 115200:
                return BaudRate::BAUD_115200;
            case 230400:
                return BaudRate::BAUD_230400;
            // Add other cases for additional baud rates as needed
            default:
                return BaudRate::BAUD_INVALID; // Return invalid if no match found
        }
    }
};



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<controller>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
