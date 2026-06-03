/**
 * Entry point for the goal_strategy_node executable.
 *
 * Kept separate from main.cpp (which holds the GoalStrat implementation) so the
 * class can be linked into unit tests without a duplicate `main` — see
 * test/test_goal_strat_node.cpp.
 */

#include "goal_strategy/goal.h"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalStrat>();
    node->initTF();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
