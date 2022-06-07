#include "rclcpp/rclcpp.hpp"
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

        pose_turtle1_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose",
            10,
            std::bind(&TurtleControllerNode::callbackPoseTurtle1, this, _1));

        cmd_vel_turtle1_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        Point turtle1_coord_;
        Point target_turtle_coord_;

        double_t actual_theta_ = 0;
        double_t desired_theta_ = 0;

        turtlesim_catch_them_all_project_interfaces::msg::Turtle::SharedPtr target_turtle_;

        while (alive_turtles_.size() != 0)
        {
            RCLCPP_INFO(this->get_logger(), "Hiii im here");
            turtle1_coord_.x = turtle1_pose_->x;
            turtle1_coord_.y = turtle1_pose_->y;

            target_turtle_ = nearestTurtle(turtle1_coord_, alive_turtles_);

            target_turtle_coord_.x = target_turtle_->x;
            target_turtle_coord_.y = target_turtle_->y;
            desired_theta_ = atan( abs(turtle1_coord_.y - target_turtle_coord_.y) / abs(turtle1_coord_.x - target_turtle_coord_.x) );

            while (distanceBetweenPoints(turtle1_coord_, target_turtle_coord_) > 1)
            {
                turtle1_coord_.x = turtle1_pose_->x;
                turtle1_coord_.y = turtle1_pose_->y;

                actual_theta_ = turtle1_pose_->theta;

            }
        }


        timer_theta_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this, actual_theta_, desired_theta_]() -> void
            {
                threads_.push_back(std::thread(std::bind(&TurtleControllerNode::thetaPcontroller, this, actual_theta_, desired_theta_)));
            }
        );

        timer_velocity_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this, turtle1_coord_, target_turtle_coord_]() -> void
            {
                threads_.push_back(std::thread(std::bind(&TurtleControllerNode::velocityPcontroller, this, turtle1_coord_, target_turtle_coord_)));
            });
    }

private:
    /* Return the index of the nearestTurtle */
    turtlesim_catch_them_all_project_interfaces::msg::Turtle::SharedPtr nearestTurtle(Point master_turtle_coord_,
                                                                                      std::vector<turtlesim_catch_them_all_project_interfaces::msg::Turtle> turtles_)
    {
        if (turtles_.size() <= 0)
            RCLCPP_ERROR(this->get_logger(), "Vector 'turtles_' is empty!");

        double_t distance_nearest_turtle_ = 0;
        turtlesim_catch_them_all_project_interfaces::msg::Turtle::SharedPtr nearest_turtle_;
        int turtle_index_ = 0;

        double_t distance_;

        Point turtle_coord_;

        /* Get the distance of the first turtle in the turtles' vector */
        turtle_coord_.x = turtles_.at(0).x;
        turtle_coord_.y = turtles_.at(0).y;

        distance_nearest_turtle_ = distanceBetweenPoints(master_turtle_coord_, turtle_coord_);

        int v_size_ = int(turtles_.size());
        /* Find the nearestTurtle in the vector */
        for (int i = 0; i < v_size_; i++)
        {
            turtle_coord_.x = turtles_.at(i).x;
            turtle_coord_.y = turtles_.at(i).y;

            distance_ = distanceBetweenPoints(master_turtle_coord_, turtle_coord_);

            if (distance_ < distance_nearest_turtle_)
            {
                distance_nearest_turtle_ = distance_;
                turtle_index_ = i;
            }
        }

        nearest_turtle_->x = alive_turtles_.at(turtle_index_).x;
        nearest_turtle_->y = alive_turtles_.at(turtle_index_).y;
        nearest_turtle_->name = alive_turtles_.at(turtle_index_).name;

        return nearest_turtle_;
    }

    double_t distanceBetweenPoints(Point p1, Point p2)
    {
        return (sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2)));
    }

    void callbackAliveTurtles(turtlesim_catch_them_all_project_interfaces::msg::TurtleArray::SharedPtr new_alive_turtles_)
    {
        alive_turtles_ = new_alive_turtles_->alive_turtles;
    }

    void callbackPoseTurtle1(turtlesim::msg::Pose::SharedPtr new_turtle1_pose_)
    {
        turtle1_pose_ = new_turtle1_pose_;
    }

    void thetaPcontroller(double_t actual_theta_, double_t desired_theta_)
    {
        double_t error_;
        geometry_msgs::msg::Twist new_theta_;

        error_ = actual_theta_ - desired_theta_;

        if (error_ > M_PI)
            error_ = error_ - M_PI;

        error_ = error_ * 0.3;

        new_theta_.angular.z = error_;
        new_theta_.angular.y = 0;
        new_theta_.angular.x = 0;

        new_theta_.linear.x = 0;
        new_theta_.linear.y = 0;
        new_theta_.linear.z = 0;

        cmd_vel_turtle1_publisher_->publish(new_theta_);
    }

    void velocityPcontroller(Point master_turtle_, Point target_turtle_)
    {
        double_t distance_;
        geometry_msgs::msg::Twist new_x_;

        distance_ = distanceBetweenPoints(master_turtle_, target_turtle_);

        distance_ = distance_ * 0.3;

        new_x_.angular.z = 0;
        new_x_.angular.y = 0;
        new_x_.angular.x = 0;

        new_x_.linear.x = distance_;
        new_x_.linear.y = 0;
        new_x_.linear.z = 0;

        cmd_vel_turtle1_publisher_->publish(new_x_);
    }

    turtlesim::msg::Pose::SharedPtr turtle1_pose_;
    std::vector<std::thread> threads_;
    std::vector<turtlesim_catch_them_all_project_interfaces::msg::Turtle> alive_turtles_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_theta_;
    rclcpp::TimerBase::SharedPtr timer_velocity_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_turtle1_publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_turtle1_subscriber_;
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
