#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cmath>
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "turtlesim_catch_them_all_project_interfaces/msg/turtle_array.hpp"
#include "turtlesim_catch_them_all_project_interfaces/msg/turtle.hpp"
#include "turtlesim_catch_them_all_project_interfaces/srv/catch_turtle.hpp"

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
        double_t distance_k_ = 1; 

        double_t theta_ = atan( abs(master_turtle_pose_.y - target_turtle_.y) / abs(master_turtle_pose_.x - target_turtle_.x) ); 
        double_t target_theta_;
        double_t theta_k_ = 7; 
        double_t diff_theta_;
        double_t master_theta_two_pi_;

        auto client = this->create_client<turtlesim_catch_them_all_project_interfaces::srv::CatchTurtle>("catch_turtle");

        /* Check if the service is up */
        while(!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the 'catch_turtle' service to be up...");
        }

        auto request = std::make_shared<turtlesim_catch_them_all_project_interfaces::srv::CatchTurtle::Request>();
        request->name = target_turtle_.name;

        while(distance_from_master_ > 0.5)
        {
            /* Calculate the target theta target turtles quadrant */
            if(target_turtle_.x < master_turtle_pose_.x)
            {
                if(target_turtle_.y < master_turtle_pose_.y)
                    target_theta_ = (3*M_PI)/2 - theta_;
                else
                    target_theta_ = M_PI - theta_;
            }
            else
            {
                if(target_turtle_.y < master_turtle_pose_.y)
                    target_theta_ = (3*M_PI)/2 + theta_;
                else
                    target_theta_ = theta_;
            }

            if(master_turtle_pose_.theta >= 0)
                master_theta_two_pi_ = master_turtle_pose_.theta;
            else
                master_theta_two_pi_ = 2*M_PI + master_turtle_pose_.theta;
            
            diff_theta_ = master_theta_two_pi_ - target_theta_;

            if(diff_theta_ > 0)
            {
                if(diff_theta_ > M_PI)
                    msg_master_pose_.angular.z = diff_theta_*theta_k_;
                else
                    msg_master_pose_.angular.z = -diff_theta_*theta_k_;
            }
            else
            {
                if(abs(diff_theta_) > M_PI)
                    msg_master_pose_.angular.z = diff_theta_*theta_k_;
                else
                    msg_master_pose_.angular.z = -diff_theta_*theta_k_;
            }

            //if(abs(diff_theta_) > M_PI && diff_theta_ > 0)
            //{
            //    /* Counter clockwise */
            //    msg_master_pose_.angular.z = diff_theta_*theta_k_;
            //}
            //else
            //{
            //    /* Clock Wise */
            //    msg_master_pose_.angular.z = -(diff_theta_*theta_k_);
            //}

            msg_master_pose_.linear.x = distance_from_master_ * distance_k_;
            cmd_vel_publisher_->publish(msg_master_pose_);

            distance_from_master_ = distanceFromMaster(target_turtle_.x, target_turtle_.y);
            theta_ = atan( abs(master_turtle_pose_.y - target_turtle_.y) / abs(master_turtle_pose_.x - target_turtle_.x) );
        }

        /* Sendo request to catch the turtle */
        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Service call to 'catch_turtle' succeed.");
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call to 'catch_turtle' failed.");
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
