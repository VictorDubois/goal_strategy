/**
 * Tier 2 — SEASONAL characterization tests for THIS YEAR's strategy graph.
 *
 * The whole body is guarded by the same YEAR_2026 macro the production code uses
 * (defined in goal_strategy/goal.h). When the team switches to YEAR_2027, this
 * file compiles down to an empty `main` and stops running automatically — add a
 * test_coupe2027.cpp alongside it instead. That is what keeps the evergreen suite
 * (test_goal_logic.cpp, test_geometry.cpp) free of anything that expires.
 *
 * These tests deliberately DO assert concrete 2026 coordinates/scores: their job
 * is to pin down the current graph so an accidental edit is caught.
 *
 * Run with:  colcon test --packages-select goal_strategy
 */

#include <gtest/gtest.h>

#include "goal_strategy/goal.h" // source of truth for the YEAR_20XX macro

#ifdef YEAR_2026

#include "rclcpp/rclcpp.hpp"

namespace
{
double X(const Position& p)
{
    return static_cast<double>(p.getX());
}
double Y(const Position& p)
{
    return static_cast<double>(p.getY());
}
} // namespace

class Coupe2026Test : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        if (!rclcpp::ok())
        {
            rclcpp::init(0, nullptr);
        }
    }
    static void TearDownTestSuite()
    {
        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }
    }

    // Etape stores its graph in process-global static storage; reset it so each test
    // builds a fresh Coupe2026.
    void SetUp() override
    {
        Etape::resetForTests();
    }
};

TEST_F(Coupe2026Test, BlueGraphCharacterization)
{
    auto node = std::make_shared<rclcpp::Node>("coupe2026_test");
    Coupe2026 strat(/*isYellow=*/false, node);

    // The 2026 match lasts 84 seconds.
    EXPECT_NEAR(strat.getRemainingTime(), 84.0, 1e-6);

    // The start step "NID" is a DEPART at positionC(-1.1, -0.75); for blue with
    // X-symmetry, positionC keeps (x, y) unchanged.
    Etape* start = strat.getEtapeEnCours();
    ASSERT_NE(start, nullptr);
    EXPECT_EQ(start->getEtapeType(), Etape::DEPART);
    EXPECT_NEAR(X(start->getPosition()), -1.1, 1e-4);
    EXPECT_NEAR(Y(start->getPosition()), -0.75, 1e-4);

    // Single 2026 parking spot, blue side.
    Position park = strat.getParkedPosition();
    EXPECT_NEAR(X(park), -1.35, 1e-4);
    EXPECT_NEAR(Y(park), -0.85, 1e-4);
}

// Drive the REAL 2026 graph the way main.cpp does (each update() == an arrival) and
// check it advances off the start and eventually completes (returns -1).
TEST_F(Coupe2026Test, RealGraphProgressesAndTerminates)
{
    auto node = std::make_shared<rclcpp::Node>("coupe2026_progress");
    Coupe2026 strat(/*isYellow=*/false, node);

    int start_num = strat.getEtapeEnCours()->getNumero();
    bool advanced = false;
    int ret = 0;
    for (int i = 0; i < 300; ++i)
    {
        ret = strat.update();
        if (strat.getEtapeEnCours()->getNumero() != start_num)
        {
            advanced = true;
        }
        if (ret == -1)
        {
            break;
        }
    }
    EXPECT_TRUE(advanced) << "the robot moves off the start node";
    EXPECT_EQ(ret, -1) << "consuming every task eventually completes the strategy";
}

// The manual score boost (getBoostManuelDeScore, consumed in Coupe2026::getScoreEtape)
// can override the engine's default goal choice on the real graph.
TEST_F(Coupe2026Test, BoostOverridesRealGoalSelection)
{
    // Baseline: the default first goal.
    int baseline_goal = -1;
    {
        auto node = std::make_shared<rclcpp::Node>("coupe2026_baseline");
        Coupe2026 strat(/*isYellow=*/false, node);
        strat.update();
        baseline_goal = strat.getGoal()->getNumero();
    }

    // Fresh graph: boost a pickup zone that is NOT the default goal, then re-decide.
    Etape::resetForTests();
    auto node = std::make_shared<rclcpp::Node>("coupe2026_boost");
    Coupe2026 strat(/*isYellow=*/false, node);

    int zone = -1;
    for (int i = 0; i < Etape::getTotalEtapes(); ++i)
    {
        Etape* e = Etape::get(i);
        if (e != nullptr && e->getEtapeType() == Etape::ZONE_DE_RAMASSAGE && i != baseline_goal)
        {
            zone = i;
            break;
        }
    }
    ASSERT_GE(zone, 0) << "the 2026 graph has a pickup zone to boost";

    Etape::get(zone)->setBoostManuelDeScore(50); // dwarf the default scores
    strat.update();

    EXPECT_EQ(strat.getGoal()->getNumero(), zone)
      << "a large manual boost makes the zone the selected goal (overrides the default)";
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#else // not YEAR_2026: seasonal suite intentionally compiles to nothing.

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#endif
