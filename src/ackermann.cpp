#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std::chrono_literals;

class Ackermann : public rclcpp::Node{
    public:
        Ackermann(): Node("ackermann_node"){
            publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
            timer_ = this->create_wall_timer(3s, std::bind(&Ackermann::timer_callback, this));
            flag_= true;
        }

    private:
        void timer_callback(){
            auto message = ackermann_msgs::msg::AckermannDriveStamped();
            
            if(flag_){
                message.drive.speed = 0.5;
                flag_ = false;
            }
            else{
                message.drive.speed = -0.5;
                flag_ = true;
            }

            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Publishing %f", message.drive.speed);
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
        bool flag_;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ackermann>());
  rclcpp::shutdown();
  return 0;
}