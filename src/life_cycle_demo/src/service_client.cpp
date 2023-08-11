#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

using namespace std::chrono_literals;

static constexpr char const *talker_Node = "lc_talker";
static constexpr char const *nodeGetStateTopic = "lc_talker/get_state";
static constexpr char const *nodeChangeStateTopic = "lc_talker/change_state";

template <typename FutureT, typename WaitTimeT>
std::future_status waitForResult(FutureT &future, WaitTimeT timeout) {
  auto end = std::chrono::steady_clock::now() + timeout;
  std::chrono::milliseconds waitPeriod(100);
  std::future_status status = std::future_status::timeout;

  do {
    auto now = std::chrono::steady_clock::now();
    auto timeleft = end - now;

    if (timeleft <= std::chrono::seconds(0)) {
      break;
    }

    status = future.wait_for((timeleft < waitPeriod) ? timeleft : waitPeriod);

  } while (rclcpp::ok() && status != std::future_status::ready);

  return status;
}

class ServiceClient : public rclcpp::Node {
public:
  explicit ServiceClient(std::string node_name) : Node(node_name) {
    clientGetState =
        this->create_client<lifecycle_msgs::srv::GetState>(nodeGetStateTopic);
    clientChangeState = this->create_client<lifecycle_msgs::srv::ChangeState>(
        nodeChangeStateTopic);
  }

  inline unsigned int get_state(std::chrono::seconds timeout = 3s) {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    // Service not available
    if (!clientGetState->wait_for_service(timeout)) {
      RCLCPP_ERROR(get_logger(), "Service %s not avalaible",
                   clientGetState->get_service_name());
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // Asynchronous result
    auto futureResult = clientGetState->async_send_request(request);

    // But we don't wait forever
    auto futureStatus = waitForResult(futureResult, timeout);

    if (futureStatus != std::future_status::ready) {
      RCLCPP_ERROR(
          get_logger(),
          "Service %s timed-out while getting current state of node %s",
          clientGetState->get_service_name(), talker_Node);
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    if (futureResult.get()) {
      auto state = futureResult.get()->current_state.id;
      RCLCPP_INFO(get_logger(), "Node %s has current state %s", talker_Node,
                  futureResult.get()->current_state.label.c_str());
      return state;
    }

    RCLCPP_ERROR(get_logger(), "Failed to get current state of node %s",
                 talker_Node);

    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }

  inline bool change_state(std::uint8_t transition,
                           std::chrono::seconds timeout = 3s) {
    auto request =
        std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    // Service not available
    if (!clientChangeState->wait_for_service(timeout)) {
      RCLCPP_ERROR(get_logger(), "Service %s not avalaible",
                   clientChangeState->get_service_name());
      return false;
    }

    // Asynchronous result
    auto futureResult = clientChangeState->async_send_request(request);

    // But we don't wait forever
    auto futureStatus = waitForResult(futureResult, timeout);

    if (futureStatus != std::future_status::ready) {
      RCLCPP_ERROR(
          get_logger(),
          "Service %s timed-out while changing current state of node %s",
          clientChangeState->get_service_name(), talker_Node);
      return false;
    }

    if (futureResult.get()) {
      auto result = futureResult.get()->success;

      if (result) {
        RCLCPP_INFO(get_logger(),
                    "Node %s state has been successfully changed/n Transition "
                    "%d successfully triggered",
                    talker_Node, static_cast<unsigned int>(transition));
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to trigger transition %d of node %s",
                     static_cast<unsigned int>(transition), talker_Node);
      }

      return result;
    }

    RCLCPP_ERROR(get_logger(), "Failed to trigger transition %d of node %s",
                 static_cast<unsigned int>(transition), talker_Node);

    return false;
  }

private:
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> clientGetState;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>>
      clientChangeState;
};

void callee_script(std::shared_ptr<ServiceClient> ServiceClient) {

  rclcpp::WallRate stateChangeTime(0.1); // 10s

  // configure
  {
    // Si on arrive pas à changet l'état à configure ou récupérer l'état
    if (!ServiceClient->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
      return;
    }

    if (!ServiceClient->get_state()) {
      return;
    }
  }

  // Activate
  {
    stateChangeTime.sleep();
    // Si on arrive pas à changet l'état à configure ou récupérer l'état
    if (!ServiceClient->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
      return;
    }

    if (!ServiceClient->get_state()) {
      return;
    }
  }

  // Deactivate
  {
    stateChangeTime.sleep();
    // Si on arrive pas à changet l'état à configure ou récupérer l'état
    if (!ServiceClient->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
      return;
    }

    if (!ServiceClient->get_state()) {
      return;
    }
  }

  // Activate
  {
    stateChangeTime.sleep();
    // Si on arrive pas à changet l'état à configure ou récupérer l'état
    if (!ServiceClient->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
      return;
    }

    if (!ServiceClient->get_state()) {
      return;
    }
  }

  // Deactivate
  {
    stateChangeTime.sleep();
    // Si on arrive pas à changet l'état à configure ou récupérer l'état
    if (!ServiceClient->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
      return;
    }

    if (!ServiceClient->get_state()) {
      return;
    }
  }

  // cleanup
  {
    stateChangeTime.sleep();
    // Si on arrive pas à changet l'état à configure ou récupérer l'état
    if (!ServiceClient->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP)) {
      return;
    }

    if (!ServiceClient->get_state()) {
      return;
    }
  }

  // shutdown
  {
    stateChangeTime.sleep();
    // Si on arrive pas à changet l'état à configure ou récupérer l'état
    if (!ServiceClient->change_state(lifecycle_msgs::msg::Transition::
                                         TRANSITION_UNCONFIGURED_SHUTDOWN)) {
      return;
    }

    if (!ServiceClient->get_state()) {
      return;
    }
  }
}
int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto serviceClient = std::make_shared<ServiceClient>("serviceClient");
  executor.add_node(serviceClient->get_node_base_interface());

  std::shared_future<void> script =
      std::async(std::launch::async, std::bind(callee_script, serviceClient));

  executor.spin_until_future_complete(script);
  rclcpp::shutdown();
  return 0;
}