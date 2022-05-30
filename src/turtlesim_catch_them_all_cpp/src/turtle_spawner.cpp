#include <random>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim_catch_them_all_project_interfaces/msg/turtle.hpp"


class TurtleSpawnerNode: public rclcpp::Node 
{
public:
    TurtleSpawnerNode() : Node("turtle_spawner") 
    {
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), 
            [this]()->void{
                threads_.push_back(std::thread(std::bind(&TurtleSpawnerNode::spawnTurtle, this)));
            }
        );
    }

private:

    void spawnTurtle()
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dist(0, 11);

        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");

        /* Check if the service is up */
        while(!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the 'Spawn' service to be up...");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();

        /* Set Random Values for the turtle's position*/
        request->x = dist(gen);
        request->y = dist(gen);
        request->theta = dist(gen);

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Service call to 'Spawn' succeed.");
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call to 'Spawn' failed.");
        }
    }

    
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