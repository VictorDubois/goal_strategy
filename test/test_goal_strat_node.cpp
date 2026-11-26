/**
 * Basic ROS 2 node tests for the GoalStrat node, modelled on
 * main-strategy/test/test_core_node.cpp.
 *
 * These exercise GoalStrat through its public interface without any hardware or
 * Gazebo. They catch:
 *   - constructor / destructor crashes (and that shutdown is clean — no leaked
 *     thread std::terminate, thanks to ~GoalStrat / ~Actuators2025),
 *   - publisher / subscriber / service wiring regressions (renamed topic, dropped
 *     QoS, ...),
 *   - the INIT -> WAIT_TIRETTE -> RUN state transition triggered by the tirette,
 *   - the pre-match state and recalage zones published for the IHM.
 *
 * IMPORTANT: Etape stores its strategy graph in process-global static storage, so
 * only ONE GoalStrat (which builds a Coupe20XX graph in init()) may exist per test
 * binary. The fixture therefore shares a single node across all tests via
 * SetUpTestSuite/TearDownTestSuite. This is safe and order-independent: the wiring
 * checks only read publisher/subscriber counts, which do not depend on node state.
 *
 * Run with:  colcon test --packages-select goal_strategy
 */

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "goal_strategy/goal.h"

// The state is also repeated at 1 Hz (heartbeat for the IHM)
static void push_if_new(std::vector<std::string>& states, const std::string& state)
{
    if (states.empty() || states.back() != state)
    {
        states.push_back(state);
    }
}

class GoalStratNodeTest : public ::testing::Test
{
protected:
    static std::shared_ptr<GoalStrat> node_;

    static void SetUpTestSuite()
    {
        if (!node_)
        {
            node_ = std::make_shared<GoalStrat>();
            node_->initTF();
        }
    }

    static void TearDownTestSuite()
    {
        // ~GoalStrat stops the publishAll thread; the m_actuators member stops its
        // own thread. A crash/terminate here would mean the lifecycle fix regressed.
        node_.reset();
    }

    // Spin the node until `condition` is true or `timeout_ms` elapses.
    static bool spin_until(std::function<bool()> condition, int timeout_ms = 3000)
    {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(node_);
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        while (std::chrono::steady_clock::now() < deadline)
        {
            exec.spin_some(std::chrono::milliseconds(50));
            if (condition())
            {
                return true;
            }
        }
        return condition();
    }
};

std::shared_ptr<GoalStrat> GoalStratNodeTest::node_ = nullptr;

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

TEST_F(GoalStratNodeTest, ConstructsWithoutError)
{
    EXPECT_NE(node_, nullptr);
}

TEST_F(GoalStratNodeTest, StartsInInitState)
{
    // No tick has run yet (we don't spin here), so the node is still in WAIT_TIRETTE_IN.
    EXPECT_EQ(node_->getState(), GoalStrat::State::WAIT_TIRETTE_IN);
}

// ---------------------------------------------------------------------------
// Publisher wiring (regression guard — names from publisherCreator.cpp)
// ---------------------------------------------------------------------------

TEST_F(GoalStratNodeTest, PublishesIsBlue)
{
    EXPECT_GT(node_->count_publishers("is_blue"), 0u);
}

TEST_F(GoalStratNodeTest, PublishesGoalPose)
{
    EXPECT_GT(node_->count_publishers("goal_pose"), 0u);
}

TEST_F(GoalStratNodeTest, PublishesDebugEtapes)
{
    EXPECT_GT(node_->count_publishers("debug_etapes"), 0u);
}

TEST_F(GoalStratNodeTest, PublishesStratMovement)
{
    EXPECT_GT(node_->count_publishers("strat_movement"), 0u)
      << "GoalStrat must advertise strat_movement (goal commands to main_strategy)";
}

// ---------------------------------------------------------------------------
// Subscriber wiring (regression guard — names from subscriptionCreator.cpp)
// ---------------------------------------------------------------------------

TEST_F(GoalStratNodeTest, SubscribesToTirette)
{
    EXPECT_GT(node_->count_subscribers("tirette"), 0u)
      << "GoalStrat must subscribe to tirette (match-start signal)";
}

TEST_F(GoalStratNodeTest, SubscribesToRemainingTime)
{
    EXPECT_GT(node_->count_subscribers("/remaining_time"), 0u);
}

TEST_F(GoalStratNodeTest, SubscribesToVacuum)
{
    EXPECT_GT(node_->count_subscribers("vacuum"), 0u);
}

TEST_F(GoalStratNodeTest, SubscribesToStepperInfo)
{
    EXPECT_GT(node_->count_subscribers("stepper_info"), 0u);
}

TEST_F(GoalStratNodeTest, SubscribesToAllFourAx12Info)
{
    EXPECT_GT(node_->count_subscribers("ax12_1_info"), 0u);
    EXPECT_GT(node_->count_subscribers("ax12_2_info"), 0u);
    EXPECT_GT(node_->count_subscribers("ax12_3_info"), 0u);
    EXPECT_GT(node_->count_subscribers("ax12_4_info"), 0u);
}

TEST_F(GoalStratNodeTest, SubscribesToDigitalRead)
{
    EXPECT_GT(node_->count_subscribers("digitalRead"), 0u);
}

TEST_F(GoalStratNodeTest, SubscribesToDynamicObstacles)
{
    EXPECT_GT(node_->count_subscribers("dynamic_obstacles"), 0u);
}

TEST_F(GoalStratNodeTest, SubscribesToCaissesSides)
{
    EXPECT_GT(node_->count_subscribers("caisses_sides"), 0u);
}

// ---------------------------------------------------------------------------
// Service wiring (2026 only)
// ---------------------------------------------------------------------------

#ifdef YEAR_2026
TEST_F(GoalStratNodeTest, AdvertisesGrabFlipDropService)
{
    bool found = false;
    for (const auto& [name, types] : node_->get_service_names_and_types())
    {
        if (name.find("grab_flip_drop") != std::string::npos)
        {
            found = true;
            break;
        }
    }
    EXPECT_TRUE(found) << "GoalStrat (2026) must advertise the grab_flip_drop service";
}
#endif

// ---------------------------------------------------------------------------
// Pre-match state and recalage zones (IHM)
// ---------------------------------------------------------------------------

TEST_F(GoalStratNodeTest, PublishesRecalageZonesToLateSubscribers)
{
    krabi_msgs::msg::RecalageZones::SharedPtr zones;
    auto sub = node_->create_subscription<krabi_msgs::msg::RecalageZones>(
      "recalage_zones",
      rclcpp::QoS(1).transient_local(),
      [&](const krabi_msgs::msg::RecalageZones::SharedPtr msg) { zones = msg; });
    ASSERT_TRUE(spin_until([&]() { return zones != nullptr; }));
    EXPECT_EQ(zones->default_zone, "depart");
    EXPECT_GE(zones->zones.size(), 1u);
    EXPECT_TRUE(zones->errors.empty());
}

// Without the tirette, and without map_odom_publisher: the request is accepted, then fails with
// its cause. The match cannot start.
TEST_F(GoalStratNodeTest, RecalageWithoutTiretteReportsItsFailure)
{
    std::vector<std::string> states;
    auto sub = node_->create_subscription<std_msgs::msg::String>(
      "pre_match_state",
      rclcpp::QoS(10).transient_local(),
      [&](const std_msgs::msg::String::SharedPtr msg) { push_if_new(states, msg->data); });
    ASSERT_TRUE(spin_until([&]() { return !states.empty(); }));
    EXPECT_EQ(states.back(), "wait_tirette_in");

    auto pub = node_->create_publisher<std_msgs::msg::Bool>("recalage", 5);
    std_msgs::msg::Bool msg;
    msg.data = true;
    ASSERT_TRUE(spin_until([&]() { return pub->get_subscription_count() > 0; }));
    pub->publish(msg);
    ASSERT_TRUE(spin_until(
      [&]() { return states.back().rfind("recalage_failed: ", 0) == 0; }, 6000))
      << states.back();
    EXPECT_EQ(states[1], "recalage: préparation");
    EXPECT_NE(states.back().find("correct_pose"), std::string::npos) << states.back();
    EXPECT_EQ(node_->getState(), GoalStrat::State::WAIT_TIRETTE_IN);
}

// ---------------------------------------------------------------------------
// State machine: tirette starts the match (kept last — it drives the node to RUN)
// ---------------------------------------------------------------------------

TEST_F(GoalStratNodeTest, TiretteTransitionsToRun)
{
    // Advance out of INIT into WAIT_TIRETTE (a couple of 100 ms timer ticks).
    spin_until([]() { return node_->getState() == GoalStrat::State::WAIT_TIRETTE_IN; }, 1000);
    ASSERT_EQ(node_->getState(), GoalStrat::State::WAIT_TIRETTE_IN);

    // m_remainig_time defaults to 85 s (> 1 s) in the constructor, so publishing
    // tirette=true is enough to satisfy the WAIT_TIRETTE -> RUN guard.
    auto tirette_pub = node_->create_publisher<std_msgs::msg::Bool>("tirette", 5);
    std::vector<std::string> states;
    auto state_sub = node_->create_subscription<std_msgs::msg::String>(
      "pre_match_state",
      rclcpp::QoS(10),
      [&](const std_msgs::msg::String::SharedPtr l_msg) { push_if_new(states, l_msg->data); });
    std_msgs::msg::Bool msg;
    msg.data = false;

    bool reached_init = spin_until(
      [&]()
      {
          tirette_pub->publish(msg);
          return node_->getState() == GoalStrat::State::INIT;
      },
      3000);

    EXPECT_TRUE(reached_init);
    EXPECT_EQ(node_->getState(), GoalStrat::State::INIT);

    bool reached_wait_tirette = spin_until(
      [&]()
      {
          tirette_pub->publish(msg);
          return node_->getState() == GoalStrat::State::WAIT_TIRETTE;
      },
      10000);

    EXPECT_TRUE(reached_wait_tirette);
    EXPECT_EQ(node_->getState(), GoalStrat::State::WAIT_TIRETTE);

    // The failed recalage (previous test) is still shown after init: the robot is not ready
    EXPECT_TRUE(spin_until(
      [&]() { return !states.empty() && states.back().rfind("recalage_failed: ", 0) == 0; }, 1000));
    EXPECT_NE(std::find(states.begin(), states.end(), "init"), states.end());

    msg.data = true;

    bool reached_run = spin_until(
      [&]()
      {
          tirette_pub->publish(msg);
          return node_->getState() == GoalStrat::State::RUN;
      },
      3000);

    EXPECT_TRUE(reached_run);
    EXPECT_EQ(node_->getState(), GoalStrat::State::RUN);
    EXPECT_TRUE(spin_until([&]() { return states.back() == "match"; }, 1000)) << states.back();
}

// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    const char* ros_args[]
      = { argv[0], "--ros-args", "-p", "recalage.file:=" RECALAGE_YAML };
    rclcpp::init(4, ros_args);
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
