#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cmath>
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "turtlesim_catch_them_all_project_interfaces/msg/turtle_array.hpp"
#include "turtlesim_catch_them_all_project_interfaces/msg/turtle.hpp"

using std::placeholders::_1;

class TurtleControllerNode : public rclcpp::Node
{

    typedef struct
    {
        double_t x;
        double_t y;
    } Point;

public:
    TurtleControllerNode() : Node("turtle_controller")
    {
        alive_turtles_subscriber_ = this->create_subscription<turtlesim_catch_them_all_project_interfaces::msg::TurtleArray>(
            "alive_turtles",
            10,
            std::bind(&TurtleControllerNode::callbackAliveTurtles, this, _1));

        pose_master_turtle_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose",
            10,
            std::bind(&TurtleControllerNode::callbackPoseTurtle1, this, _1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    }

private:

    void CatchTarget()
    {
        RCLCPP_INFO(this->get_logger(), "Catching the " + target_turtle_.name + " turtle");

        auto msg_master_pose_ = geometry_msgs::msg::Twist(); 
        catching_a_turtle_ = true;

        double_t distance_from_master_ = distanceFromMaster(target_turtle_.x, target_turtle_.y);
        double_t distance_k_ = 0.3;

        double_t target_theta_ = atan( abs(master_turtle_pose_.y - target_turtle_.y) / abs(master_turtle_pose_.x - target_turtle_.x) ); 
        double_t target_k_ = 3.5; 




        while(distance_from_master_ > 1)
        {
            if(abs(2*M_PI - target_theta_ - master_turtle_pose_.theta) > target_theta_ - master_turtle_pose_.theta)
                msg_master_pose_.angular.z = (target_theta_ - master_turtle_pose_.theta)*target_k_;
            else
                msg_master_pose_.angular.z = abs(2*M_PI - target_theta_ - master_turtle_pose_.theta)*-target_k_;

           msg_master_pose_.linear.x = distance_from_master_ * distance_k_;

           cmd_vel_publisher_->publish(msg_master_pose_);

           distance_from_master_ = distanceFromMaster(target_turtle_.x, target_turtle_.y);
        }

        catching_a_turtle_ = false;
    }

    /* Return the distance between the master turtle and the point (x,y) */
    double_t distanceFromMaster(double_t x, double_t y)
    {
        return (sqrt(pow(master_turtle_pose_.x - x, 2) + pow(master_turtle_pose_.y - y, 2)));
    }

    void callbackAliveTurtles(turtlesim_catch_them_all_project_interfaces::msg::TurtleArray::SharedPtr alive_turtles_)
    {
        /* If the master turtle is already catching a turtle, we don't need a new target */
        if(catching_a_turtle_ == true)
            return;

        double_t lowest_distance_ = distanceFromMaster(alive_turtles_->alive_turtles.at(0).x, alive_turtles_->alive_turtles.at(0).y);
        double_t new_distance_ = 0;

        target_turtle_ = alive_turtles_->alive_turtles.at(0);

        for (int i = 0; i < int(alive_turtles_->alive_turtles.size()); i++)
        {
            /* Calculate the distance of the current turtle */
            new_distance_ = distanceFromMaster(alive_turtles_->alive_turtles.at(i).x, alive_turtles_->alive_turtles.at(i).y);

            if(lowest_distance_ > new_distance_)
            {
                lowest_distance_ = new_distance_;
                target_turtle_ = alive_turtles_->alive_turtles.at(0);
            }
        }
        threads_.push_back(std::thread(std::bind(&TurtleControllerNode::CatchTarget, this)));
    }

    void callbackPoseTurtle1(turtlesim::msg::Pose::SharedPtr new_turtle1_pose_)
    {
        master_turtle_pose_ = *new_turtle1_pose_.get();
        turtlesim_up_ = true;
    }


    std::vector<std::thread> threads_;
    turtlesim_catch_them_all_project_interfaces::msg::Turtle target_turtle_;
    turtlesim::msg::Pose master_turtle_pose_;
    bool turtlesim_up_ = false;
    bool catching_a_turtle_ = false;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_master_turtle_subscriber_;
    rclcpp::Subscription<turtlesim_catch_them_all_project_interfaces::msg::TurtleArray>::SharedPtr alive_turtles_subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
