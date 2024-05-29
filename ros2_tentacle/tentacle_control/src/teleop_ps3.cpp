#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include "tentacle_interfaces/msg/actuation.hpp"

class teleop_ps3 : public rclcpp::Node {
public:
    teleop_ps3() : Node("teleop_ps3") 
    {
        // Initialize actuator names
        for(int i = 1; i <= actuator_count_; i++)
        {
            actuators_.push_back("actuator" + std::to_string(i));
        }
        safe_ = true;

        // Publishers
        cmd_actuation_publisher_ = create_publisher<tentacle_interfaces::msg::Actuation>(
        "/cmd_actuation", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&teleop_ps3::publishJoystickValues, this));
    }

private:

    int actuator_count_ = 6;
    std::vector<std::string> actuators_;
    bool safe_;

    rclcpp::Publisher<tentacle_interfaces::msg::Actuation>::SharedPtr cmd_actuation_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publishJoystickValues() 
    {
        RCLCPP_INFO(this->get_logger(), "Publishing joystick values");

        // Open the joystick device
        int joy_fd = open("/dev/input/js0", O_RDONLY);
        if (joy_fd < 0) 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open joystick device: %s", strerror(errno));
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Joystick device opened successfully");

        // Initialize joystick event
        struct js_event js;

        // Initialize joystick message
        std::vector<double> js_vals;

        for (int i=0; i<4; i++)
        {
            js_vals.push_back(0);
        }

        // Initialize tentacle actuation message
        tentacle_interfaces::msg::Actuation msg;
        msg = initialize_actuation_msg();

        // Joystick closing needs to be handled correctly, hence the infinite time loop
        while (true) 
        {
            // Read joystick event
            int bytes_read = read(joy_fd, &js, sizeof(js));

            // Handle improper reading
            if (bytes_read < 0) 
            {
                RCLCPP_ERROR(this->get_logger(), "Error reading joystick event: %s", strerror(errno));
                break;
            } 
            else if (bytes_read == 0) 
            {
                RCLCPP_WARN(this->get_logger(), "No data read from joystick device");
                break;
            } 
            else if (bytes_read != sizeof(js)) 
            {
                RCLCPP_ERROR(this->get_logger(), "Incomplete joystick event read");
                continue;
            }

            // If joysticks of the PS3 controller have been moved
            if (js.type & JS_EVENT_AXIS) 
            {
                // Handle joystick axes
                if (js.number <= 4) 
                {   /// 0&1 for left joyst
                    if (js.number == 0 || js.number == 1)
                    {
                        js_vals[js.number] = static_cast<float>(js.value) / 32767.0;
                    }
                    /// 3&4 for right joyst
                    else if (js.number == 3 || js.number == 4)
                    {
                        js_vals[js.number-1] = static_cast<float>(js.value) / 32767.0;
                    }
                } 
                else 
                {
                    RCLCPP_ERROR(this->get_logger(), "Axis number out of range: %d", js.number);
                    safe_ = false;
                }
            } 

            if(safe_)
            {
                // Convert joystick values to control inputs

                // // Tri-directional motion

                // // Check if left_joystick is up
                // if(js_vals[1] < 0)
                // {
                //     msg.states[0] = true;
                // }
                // else if(js_vals[1] > 0 && js_vals[0] < 0) // Check if bottom left
                // {
                //     msg.states[1] = true;
                // }
                // else if(js_vals[1] > 0 && js_vals[0] > 0) // Check if bottom right
                // {
                //     msg.states[2] = true;
                // }

                // Six-directional motion
                double up_thresh = 0.8;
                double side_thresh = 0.2;
                // Check if left joystick is up
                if(js_vals[1] < -up_thresh)
                {
                    msg.states[0] = true;
                    RCLCPP_INFO(this->get_logger(), "upper trigger of left one!");
                }
                else if((js_vals[1] <= 0) && (js_vals[1] > -up_thresh) && js_vals[0] < -side_thresh) // Check if left joystick is top-left
                {
                    msg.states[0] = true;
                    msg.states[1] = true;
                    RCLCPP_INFO(this->get_logger(), " left TOP LEFT!");
                }
                else if((js_vals[1] <= 0) && (js_vals[1] > -up_thresh) && js_vals[0] > side_thresh) // Check if left joystick is top-right
                {
                    msg.states[0] = true;
                    msg.states[2] = true;

                    RCLCPP_INFO(this->get_logger(), " left TOP RIGHT!");
                }
                else if((js_vals[1] > 0) && (js_vals[1] < up_thresh) && js_vals[0] < -side_thresh) // Check if left joystick is bottom-left
                {
                    msg.states[1] = true;
                    RCLCPP_INFO(this->get_logger(), " left BOTTOM LEFT!");
                }
                else if((js_vals[1] > 0) && (js_vals[1] < up_thresh) && js_vals[0] > side_thresh) // Check if left joystick is bottom-right
                {
                    msg.states[2] = true;
                    RCLCPP_INFO(this->get_logger(), " left BOTTOM RIGHT!");
                }
                else if(js_vals[1] > up_thresh) // Check if left joystick is down
                {
                    msg.states[1] = true;
                    msg.states[2] = true;
                    RCLCPP_INFO(this->get_logger(), "left DOWN!");
                }
                


                // Check if right Joystick is up
                if(js_vals[3] < -up_thresh)
                {
                    msg.states[3] = true;
                    RCLCPP_INFO(this->get_logger(), " right TOP!");

                }
                else if((js_vals[3] <= 0) && (js_vals[3] > -up_thresh) && js_vals[2] < -side_thresh) // Check if right joystick is top-left
                {
                    msg.states[3] = true;
                    msg.states[4] = true;
                    RCLCPP_INFO(this->get_logger(), " right TOP LEFT!");
                }
                else if((js_vals[3] <= 0) && (js_vals[3] > -up_thresh) && js_vals[2] > side_thresh) // Check if right joystick is top-righ
                {
                    msg.states[3] = true;
                    msg.states[5] = true;
                    RCLCPP_INFO(this->get_logger(), " right TOP RIGHT!");
                }
                else if((js_vals[3] > 0) && (js_vals[3] < up_thresh) && js_vals[2] < -side_thresh) // Check if right joystick is bottom-left
                {
                    msg.states[4] = true;
                    RCLCPP_INFO(this->get_logger(), " right BOTTOM LEFT!");
                }   
                else if((js_vals[3] > 0) && (js_vals[3] < up_thresh) && js_vals[2] > side_thresh) // Check if right joystick is bottom-right
                {
                    msg.states[5] = true;
                    RCLCPP_INFO(this->get_logger(), " right BOTTOM RIGHT!");
                }
                else if(js_vals[3] > up_thresh) // Check if right joystick is down
                {
                    msg.states[4] = true;
                    msg.states[5] = true;
                    RCLCPP_INFO(this->get_logger(), "right DOWN!");
                }



                cmd_actuation_publisher_->publish(msg);

                // Reset after publishing
                msg = initialize_actuation_msg();

                // RCLCPP_INFO(this->get_logger(), "Joystick message published: time: %d, val: %d, type: %d, number: %d", js.time, js.value, js.type, js.number);
            }
            else
            {
                cmd_actuation_publisher_->publish(msg);

                RCLCPP_ERROR(this->get_logger(), "PREVENTED MOTION");
            }
            safe_ = true;
        }

        close(joy_fd);
        RCLCPP_INFO(this->get_logger(), "Joystick device closed");
    }

    tentacle_interfaces::msg::Actuation initialize_actuation_msg()
    {
        tentacle_interfaces::msg::Actuation msg;

        // Initialize actuator names
        for(int i = 1; i <= actuator_count_; i++)
        {
            msg.names.push_back(actuators_.at(i-1));
            msg.states.push_back(false);
            // RCLCPP_INFO(this->get_logger(), "msg.states print out: %ld" , msg.states.size());
        }

        return msg;
    }

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<teleop_ps3>());
    rclcpp::shutdown();
    return 0;
}
