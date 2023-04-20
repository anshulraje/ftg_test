#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

//namespaces
using namespace std::chrono_literals;
using namespace sensor_msgs::msg;
using namespace ackermann_msgs::msg;
using namespace nav_msgs::msg;
using namespace geometry_msgs::msg;
using namespace std;
using std::placeholders::_1;

//global variables
LaserScan::SharedPtr scan_data;
int max_gap_index = 0;

struct {
    double x;
    double y;
    double yaw;
} odom;

class Test1 : public rclcpp::Node{
    public:
        Test1(): Node("test1"){
            drive_publisher_ = this->create_publisher<AckermannDriveStamped>("/drive", 10);
            lidar_subscriber_ = this->create_subscription<LaserScan>("/scan", 10, std::bind(&Test1::lidar_callback, this, _1));
            odom_subscriber_ = this->create_subscription<Odometry>("/ego_racecar/odom", 10, std::bind(&Test1::odom_callback, this, _1));
        }

        double yaw_from_quaternion(Quaternion quat) const {
            tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getEulerYPR(yaw, pitch, roll);
            return yaw;
        }

        void lidar_callback(const LaserScan::SharedPtr msg) const {
            scan_data = msg;
            find_gap(5);
        }

        void odom_callback(const Odometry::SharedPtr msg) const {
            odom.x = msg->pose.pose.position.x;
            odom.y = msg->pose.pose.position.y;
            odom.yaw = yaw_from_quaternion(msg->pose.pose.orientation);
            // RCLCPP_INFO(this->get_logger(), "Odom: %f, %f, %f", odom.x, odom.y, odom.yaw);
        }

        void find_gap(int gap_size) const {
            auto ranges = scan_data->ranges;

            vector<float>::iterator it1 = ranges.begin();
            vector<float>::iterator it2 = ranges.begin()+149;
            ranges.erase(it1, it2);

            vector<float>::iterator it3 = ranges.end()-149;
            vector<float>::iterator it4 = ranges.end();
            ranges.erase(it3, it4);

            auto last_index = int(ranges.size()-1);
            cout << "Last Index: " << last_index << endl;

            float max_gap = 0;

            for(int i = 0; i<=last_index-(gap_size-1); i++){
                float temp_sum = 0;
                for(int j = i; j<=i+(gap_size-1); j++){
                    temp_sum = temp_sum + ranges.at(j);
                }

                if(temp_sum > max_gap){
                    max_gap = temp_sum;
                    max_gap_index = i+int(floor(gap_size/2));
                }
                else
                    continue;
            }

            cout << "Max Gap Index: " << max_gap_index << endl;
        }

        void follow_gap(){

        }

        //field declarations
        rclcpp::Publisher<AckermannDriveStamped>::SharedPtr drive_publisher_;
        rclcpp::Subscription<LaserScan>::SharedPtr lidar_subscriber_;
        rclcpp::Subscription<Odometry>::SharedPtr odom_subscriber_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<Test1>());
    rclcpp::shutdown();
    return 0;
}

/*TO DO
lidar scans from -3pi/4 to 3pi/4. have to filter this down to probably -pi/2 to pi/2
info about this can be found with ros2 topic info /scan
the above should fix the gap finding issue
write the follow_gap() fnc to drive towards the gap
*/

/*QUESTIONS FOR MEET
Is my apporoach correct?
Explain how I have gotten odom with yaw
How to overlap lidar map and odom map?

Marker message - map gap on the sim
have to use transforms from yaw to gap
*/