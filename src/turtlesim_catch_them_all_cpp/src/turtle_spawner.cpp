#include <random>
#include <thread>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim_catch_them_all_project_interfaces/msg/turtle_array.hpp"


std::mutex mtx;

class TurtleSpawnerNode: public rclcpp::Node 
{
public:
    TurtleSpawnerNode() : Node("turtle_spawner") 
    {
        alive_turtles_publisher_ = this->create_publisher<turtlesim_catch_them_all_project_interfaces::msg::TurtleArray>(
            "alive_turtles",
            10
        );

        timer_alive_turtles_publisher_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]()->void{
                threads_.push_back(std::thread(std::bind(&TurtleSpawnerNode::publisherAliverTurtles, this)));
            }
        );

        timer_ = this->create_wall_timer(
            std::chrono::seconds(5), 
            [this]()->void{
                threads_.push_back(std::thread(std::bind(&TurtleSpawnerNode::spawnTurtle, this)));
            }
        );
       RCLCPP_INFO(this->get_logger(), "Turtle Spawner Node has been started!");
    }

private:
   
    void spawnTurtle()
    { 
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dist(0, 11);

        turtlesim_catch_them_all_project_interfaces::msg::Turtle new_turtle_info;

        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");

        /* Check if the service is up */
        while(!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the 'spawn' service to be up...");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();

        /* Set Random Values for the turtle's position*/
        request->x = dist(gen);
        request->y = dist(gen);
        request->theta = dist(gen);
        request->name = "turtle"+std::to_string(alive_turtles_.size()+2);


        /* Fill the information of the new turtle */
        new_turtle_info.name = request->name;
        new_turtle_info.x = request->x;
        new_turtle_info.y = request->y;

        /* Put the new turtle in the alive turtles' vector */
        alive_turtles_.push_back(new_turtle_info);

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Service call to 'spawn' succeed.");
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call to 'spawn' failed.");
        }
    }

    
    void publisherAliverTurtles()
    {
        auto msg = turtlesim_catch_them_all_project_interfaces::msg::TurtleArray();

        size_t vector_length = alive_turtles_.size(); 

        /* Build the msg */
        for (int i = 0; i < int(vector_length); i++)
           msg.alive_turtles.push_back(alive_turtles_.at(i));

        alive_turtles_publisher_->publish(msg);
    }

    std::vector<turtlesim_catch_them_all_project_interfaces::msg::Turtle> alive_turtles_;
    rclcpp::Publisher<turtlesim_catch_them_all_project_interfaces::msg::TurtleArray>::SharedPtr alive_turtles_publisher_;
    rclcpp::TimerBase::SharedPtr timer_alive_turtles_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::thread> threads_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnerNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}