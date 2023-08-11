#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>

using namespace std::chrono_literals;
// using 1s to 1000ms 1h for 1 hour ...

class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode {

public:
  explicit LifecycleTalker(
      const std::string &node_name,
      bool intraProcessComms =
          false) // intra-process: messages are sent from a publisher to
                 // subscriptions via in-process memory
      : rclcpp_lifecycle::LifecycleNode(
            node_name,
            rclcpp::NodeOptions().use_intra_process_comms(intraProcessComms)) {}

  void publish() {

    static size_t count = 0;
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Lifecycle talker says Hello World" + std::to_string(++count);

    if (publisher->is_activated()) {
      RCLCPP_INFO(get_logger(),
                  "Lifecycle publisher is active. Publishing [%s]",
                  msg->data.c_str());
      // You can even set it outside the if, it will not be published
      publisher->publish(std::move(msg));
    } else {
      RCLCPP_WARN(get_logger(), "Lifecycle publisher currently inactive");
    }
  }
  // Overload of the return callbacks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) {
    publisher = this->create_publisher<std_msgs::msg::String>("messages", 10);
    timer =
        this->create_wall_timer(1s, std::bind(&LifecycleTalker::publish, this));

    RCLCPP_INFO(get_logger(), "on_configure() called");

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) {
    publisher->on_activate();
    RCLCPP_INFO(get_logger(), "on_activate() called");
    // Take some time to activate
    std::this_thread::sleep_for(2s);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) {
    publisher->on_deactivate();
    RCLCPP_INFO(get_logger(), "on_deactivate() called");
    std::this_thread::sleep_for(2s);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) {
    publisher.reset();
    timer.reset();
    RCLCPP_INFO(get_logger(), "on_cleanup() called");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(get_logger(), "on_shutdown() called");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

private:
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>>
      publisher;
  std::shared_ptr<rclcpp::TimerBase> timer;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto talkerNode = std::make_shared<LifecycleTalker>("lc_talker");
  executor.add_node(talkerNode->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();
  return 0;
}