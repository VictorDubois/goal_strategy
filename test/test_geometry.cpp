/**
 * Tier 1 — EVERGREEN unit tests for the krabilib geometry primitives that the
 * extracted goal_logic helpers (and most of the strategy) lean on.
 *
 * Pure maths, no ROS, no competition rules: these invariants are true forever, so
 * the tests are year-stable by construction. Cheap regression insurance for the
 * foundation everything else stands on.
 *
 * Run with:  colcon test --packages-select goal_strategy
 */

#include <cmath>

#include <gtest/gtest.h>

#include "krabilib/angle.h"
#include "krabilib/pose.h"
#include "krabilib/position.h"

// ---------------------------------------------------------------------------
// Position
// ---------------------------------------------------------------------------

TEST(Position, NormeIsPythagorean)
{
    Position p(Distance(3.0), Distance(4.0));
    EXPECT_NEAR(static_cast<double>(p.getNorme()), 5.0, 1e-9);
}

TEST(Position, NormeOfDifferenceIsEuclideanDistance)
{
    Position a(Distance(1.0), Distance(2.0));
    Position b(Distance(4.0), Distance(6.0));
    EXPECT_NEAR(static_cast<double>((a - b).getNorme()), 5.0, 1e-9);
}

TEST(Position, CoordinateRoundTrip)
{
    Position p(Distance(-1.5), Distance(0.25));
    EXPECT_NEAR(static_cast<double>(p.getX()), -1.5, 1e-9);
    EXPECT_NEAR(static_cast<double>(p.getY()), 0.25, 1e-9);
}

// ---------------------------------------------------------------------------
// AngleTools::wrapAngle  — result always in [-pi, pi]
// ---------------------------------------------------------------------------

TEST(WrapAngle, ZeroStaysZero)
{
    EXPECT_NEAR(static_cast<double>(AngleTools::wrapAngle(Angle(0))), 0.0, 1e-9);
}

TEST(WrapAngle, FullTurnWrapsToZero)
{
    EXPECT_NEAR(static_cast<double>(AngleTools::wrapAngle(Angle(2 * M_PI))), 0.0, 1e-9);
}

TEST(WrapAngle, ResultStaysInRange)
{
    for (double a = -10.0; a <= 10.0; a += 0.37)
    {
        double w = static_cast<double>(AngleTools::wrapAngle(Angle(a)));
        EXPECT_LE(w, M_PI + 1e-9);
        EXPECT_GE(w, -M_PI - 1e-9);
    }
}

// ---------------------------------------------------------------------------
// AngleTools::diffAngle  — shortest signed difference, in [-pi, pi]
// ---------------------------------------------------------------------------

TEST(DiffAngle, ZeroForEqualAngles)
{
    EXPECT_NEAR(static_cast<double>(AngleTools::diffAngle(Angle(1.234), Angle(1.234))), 0.0, 1e-9);
}

TEST(DiffAngle, SmallDifference)
{
    EXPECT_NEAR(static_cast<double>(AngleTools::diffAngle(Angle(0.1), Angle(-0.1))), 0.2, 1e-9);
}

TEST(DiffAngle, TakesShortestWayAroundPi)
{
    // 3.13 and -3.13 are ~1.3 deg apart the short way, not ~6.26 rad the long way.
    double d = std::abs(static_cast<double>(AngleTools::diffAngle(Angle(3.13), Angle(-3.13))));
    EXPECT_LT(d, 0.05);
}

// ---------------------------------------------------------------------------
// Pose  — getter round-trip
// ---------------------------------------------------------------------------

TEST(Pose, PositionAndAngleRoundTrip)
{
    Pose p(Position(Distance(0.5), Distance(-0.5)), Angle(1.0));
    EXPECT_NEAR(static_cast<double>(p.getPosition().getX()), 0.5, 1e-9);
    EXPECT_NEAR(static_cast<double>(p.getPosition().getY()), -0.5, 1e-9);
    EXPECT_NEAR(static_cast<double>(p.getAngle()), 1.0, 1e-9);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
