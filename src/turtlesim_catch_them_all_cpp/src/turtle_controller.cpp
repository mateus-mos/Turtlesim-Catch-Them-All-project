#include "rclcpp/rclcpp.hpp"
#include "turtlesim_catch_them_all_project_interfaces/msg/turtle_array.hpp"
#include "turtlesim_catch_them_all_project_interfaces/msg/turtle.hpp"

using std::placeholders::_1;

class TurtleControllerNode: public rclcpp::Node 
{

    typedef struct 
    {
        double_t x;
        double_t y;
    } Point ;

public:
    TurtleControllerNode() : Node("turtle_controller") 
    {
        alive_turtles_subscriber_ = this->create_subscription<turtlesim_catch_them_all_project_interfaces::msg::TurtleArray>(
            "alive_turtles",
            10,
            std::bind(&TurtleControllerNode::callbackAliveTurtles, this, _1)
        );

        
        Point master_coord_;

        master_coord_.x = 5.5;
        master_coord_.y = 5.5;



        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            [this, master_coord_]()->void{
                threads_.push_back(std::thread(std::bind(&TurtleControllerNode::nearestTurtle, this, master_coord_, this->alive_turtles_)));
            }
        );

   }

private:

    /* Return the index of the nearestTurtle */
    int nearestTurtle(Point master_turtle_coord_,
                      std::vector<turtlesim_catch_them_all_project_interfaces::msg::Turtle> turtles_) 
    {
        if(turtles_.size() <= 0)
            RCLCPP_ERROR(this->get_logger(), "Vector 'turtles_' is empty!");

        double_t distance_nearest_turtle_= 0;
        int turtle_id_ = 0;

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

            if(distance_ < distance_nearest_turtle_)
            {
                distance_nearest_turtle_ = distance_;
                turtle_id_ = i;
            }

        }

        RCLCPP_INFO(this->get_logger(), " ID: " + std::to_string(turtle_id_));
        RCLCPP_INFO(this->get_logger(), " x: " + std::to_string(turtles_.at(turtle_id_).x) + " y: " + std::to_string(turtles_.at(turtle_id_).y));
        return turtle_id_;
    } 

    double_t distanceBetweenPoints(Point p1, Point p2)
    {
        return (sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2)));
    }

    void callbackAliveTurtles(turtlesim_catch_them_all_project_interfaces::msg::TurtleArray::SharedPtr new_alive_turtles_)
    {
        alive_turtles_ = new_alive_turtles_->alive_turtles;
    }

    //void catch_turtle(turtle_to_catch)
    //{
    //    go_to(turtle_to_catch.location);
    //    call service to catch turtle;
    //}

    //void go_to(turtle_to_catch.location)
    //{
    //    call thetaPcontroller and xVelocityPcontroller until  turtle1 catch turtle_target

    //}

    //void thetaPcontroller(float theta)
    //{
    //    if(theta > pi)
    //        theta = -(theta-pi)
    //    
    //    sum_to_turtle_theta = (turtle_actual_theta - desired_theta)*k;

    //    call cmd_vel to set angular z = sum_to_turtle_theta 
    //}

    //void xVelocityPcontroller(turtle1.position and target_turtle.positivo)
    //{
    //    distance = calculate distance betwen two turtles

    //    velocity_to_publish = (distance)*k

    //    call cmd_vel to set linear x = velocity to publish
    //}

    
    std::vector<std::thread> threads_;
    std::vector<turtlesim_catch_them_all_project_interfaces::msg::Turtle> alive_turtles_;

    rclcpp::TimerBase::SharedPtr timer_;
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
