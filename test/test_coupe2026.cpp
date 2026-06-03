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
};

// NOTE: Etape stores its graph in process-global static storage, so only ONE
// strategy graph may be built per test binary. Everything is asserted on a single
// blue instance inside one test case.
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
