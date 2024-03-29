#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <action_tutorials_interfaces/action/fibonacci.hpp>

using Fibonacci = action_tutorials_interfaces::action::Fibonacci;

class FibonacciServer : public rclcpp::Node
{
public:
  explicit FibonacciServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("fibonacci_server", options)
  {
    using namespace std::placeholders;
    using namespace std::placeholders;
    // Create the action server
    server_ = rclcpp_action::create_server<Fibonacci>(
        this,
        "fibonacci",
        std::bind(&FibonacciServer::handle_goal, this, _1, _2),
        std::bind(&FibonacciServer::handle_cancel, this, _1),
        std::bind(&FibonacciServer::handle_accepted, this, _1));

        RCLCPP_INFO(this->get_logger(), "Server is started an listening...");
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr server_;

  void handle_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&FibonacciServer::execute, this, _1), goal_handle}.detach();
  }

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);

    (void)uuid;
    // Let's reject sequences that are over 9000
    if (goal->order > 9000)
    {
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto &sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i)
    {
      // Check if there is a cancel request
      if (goal_handle->is_canceling())
      {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok())
    {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};


int main(int argc, char** argv)
{
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<FibonacciServer>());
rclcpp::shutdown();
return 0;
}