#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <chrono>

#include "action_tutorials_interfaces/action/fibonacci.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using Fibonacci = action_tutorials_interfaces::action::Fibonacci;

class FibonacciClient : public rclcpp::Node
{
public:
    explicit FibonacciClient(const std::string& action_name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("fibonacci_client", options), action_name_(action_name)
    {
        using namespace std::placeholders;
        // Create the action server
        client_ = rclcpp_action::create_client<Fibonacci>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            action_name_);

            RCLCPP_INFO(this->get_logger(), "Client is started.");
    }


    bool send_goal(int order)
    {
        using namespace std::placeholders;

        
        
        if (!client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
            return false;
        }

        auto goal_msg = Fibonacci::Goal();
        goal_msg.order = order;

        

        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&FibonacciClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&FibonacciClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&FibonacciClient::result_callback, this, _1);


         RCLCPP_INFO(this->get_logger(), "Client is sending the goal...");
        this->client_->async_send_goal(goal_msg, send_goal_options);
        return true;
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::string action_name_;
    rclcpp_action::Client<Fibonacci>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    void goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<Fibonacci>> goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        std::shared_ptr<rclcpp_action::ClientGoalHandle<Fibonacci>>,
        const std::shared_ptr<const Fibonacci::Feedback> feedback)
    {
        std::stringstream ss;
        ss << "Next number in sequence received: ";
        for (auto number : feedback->partial_sequence)
        {
            ss << number << " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
        std::stringstream ss;
        ss << "Result received: ";
        for (auto number : result.result->sequence)
        {
            ss << number << ", ";
        }
        RCLCPP_INFO(node_->get_logger(), "Result: %s", ss.str().c_str());
        rclcpp::shutdown();
    }
};


int main(int argc, char** argv)
{
rclcpp::init(argc, argv);

auto fibonacci_client = std::make_shared<FibonacciClient>("fibonacci");
if (fibonacci_client->send_goal(10)) {
rclcpp::spin_some();
}

rclcpp::shutdown();
return 0;
}