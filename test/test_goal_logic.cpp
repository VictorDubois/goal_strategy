/**
 * Tier 1 — EVERGREEN unit tests for the pure logic extracted from GoalStrat.
 *
 * These assert only year-agnostic invariants of geometry/maths (a closer point
 * is preferred, reverse gear flips the heading by pi, a straight move preserves
 * distance, ...). They name NO competition-specific coordinate or score, so they
 * must keep passing when the yearly rules change. Anything year-specific lives in
 * test_coupe20XX.cpp instead, guarded by the matching YEAR_20XX macro.
 *
 * Run with:  colcon test --packages-select goal_strategy
 */

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "goal_strategy/goal_logic.h"

using goal_logic::isAlignedWithAngle;
using goal_logic::isParkedAt;
using goal_logic::isWithinReach;
using goal_logic::maxSpeedAtArrival;
using goal_logic::straightLineTarget;

namespace
{
Position pos(double x, double y)
{
    return Position(Distance(x), Distance(y));
}
const Angle TWO_DEG = AngleTools::deg2rad(AngleDeg(2));
} // namespace

// ---------------------------------------------------------------------------
// isWithinReach
// ---------------------------------------------------------------------------

TEST(IsWithinReach, TrueWhenCloserThanThreshold)
{
    EXPECT_TRUE(isWithinReach(pos(0, 0), pos(0.01, 0), Distance(0.02)));
}

TEST(IsWithinReach, FalseWhenFurtherThanThreshold)
{
    EXPECT_FALSE(isWithinReach(pos(0, 0), pos(0.05, 0), Distance(0.02)));
}

TEST(IsWithinReach, BoundaryIsExclusive)
{
    // distance == reach must NOT count as arrived (strict <)
    EXPECT_FALSE(isWithinReach(pos(0, 0), pos(0.02, 0), Distance(0.02)));
}

TEST(IsWithinReach, SymmetricInArguments)
{
    Position a = pos(1.0, -0.5);
    Position b = pos(1.3, -0.2);
    EXPECT_EQ(isWithinReach(a, b, Distance(0.5)), isWithinReach(b, a, Distance(0.5)));
}

// ---------------------------------------------------------------------------
// isAlignedWithAngle
// ---------------------------------------------------------------------------

TEST(IsAlignedWithAngle, AlignedAtZeroError)
{
    EXPECT_TRUE(isAlignedWithAngle(Angle(0), Angle(0), /*reverse=*/false, TWO_DEG));
}

TEST(IsAlignedWithAngle, ReverseGearFlipsHeadingByPi)
{
    // Facing 0, but tool is on the back: we are aligned with a target of pi.
    EXPECT_TRUE(isAlignedWithAngle(Angle(M_PI), Angle(0), /*reverse=*/true, TWO_DEG));
    // ... and NOT aligned with that same target in forward gear.
    EXPECT_FALSE(isAlignedWithAngle(Angle(M_PI), Angle(0), /*reverse=*/false, TWO_DEG));
}

TEST(IsAlignedWithAngle, HandlesWrapAroundNearPi)
{
    // 3.13 rad and -3.13 rad are only ~1.3 deg apart once wrapped → aligned.
    EXPECT_TRUE(isAlignedWithAngle(Angle(3.13), Angle(-3.13), /*reverse=*/false, TWO_DEG));
}

TEST(IsAlignedWithAngle, ToleranceBoundary)
{
    EXPECT_TRUE(isAlignedWithAngle(
      AngleTools::deg2rad(AngleDeg(1)), Angle(0), /*reverse=*/false, TWO_DEG));
    EXPECT_FALSE(isAlignedWithAngle(
      AngleTools::deg2rad(AngleDeg(3)), Angle(0), /*reverse=*/false, TWO_DEG));
}

// ---------------------------------------------------------------------------
// straightLineTarget
// ---------------------------------------------------------------------------

TEST(StraightLineTarget, PreservesDistance)
{
    Position cur = pos(1.0, 1.0);
    Position target = straightLineTarget(cur, Angle(0.7), Distance(0.5), /*recule=*/false);
    double moved = static_cast<double>((target - cur).getNorme());
    EXPECT_NEAR(moved, 0.5, 1e-9);
}

TEST(StraightLineTarget, ForwardAndReverseAreOpposite)
{
    Position cur = pos(2.0, -1.0);
    Angle heading(1.2);
    Position fwd = straightLineTarget(cur, heading, Distance(0.4), /*recule=*/false);
    Position rev = straightLineTarget(cur, heading, Distance(0.4), /*recule=*/true);
    // Forward and reverse displacements are exact negatives of each other.
    EXPECT_NEAR(static_cast<double>(fwd.getX()) - static_cast<double>(cur.getX()),
                static_cast<double>(cur.getX()) - static_cast<double>(rev.getX()),
                1e-9);
    EXPECT_NEAR(static_cast<double>(fwd.getY()) - static_cast<double>(cur.getY()),
                static_cast<double>(cur.getY()) - static_cast<double>(rev.getY()),
                1e-9);
}

TEST(StraightLineTarget, AxisAlignedHeadings)
{
    Position origin = pos(0, 0);
    Position east = straightLineTarget(origin, Angle(0), Distance(1.0), /*recule=*/false);
    EXPECT_NEAR(static_cast<double>(east.getX()), 1.0, 1e-9);
    EXPECT_NEAR(static_cast<double>(east.getY()), 0.0, 1e-9);

    Position north = straightLineTarget(origin, Angle(M_PI / 2), Distance(1.0), /*recule=*/false);
    EXPECT_NEAR(static_cast<double>(north.getX()), 0.0, 1e-9);
    EXPECT_NEAR(static_cast<double>(north.getY()), 1.0, 1e-9);
}

// ---------------------------------------------------------------------------
// maxSpeedAtArrival
// ---------------------------------------------------------------------------

TEST(MaxSpeedAtArrival, PassThroughPointDoesNotFullyStop)
{
    EXPECT_GT(maxSpeedAtArrival(Etape::EtapeType::POINT_PASSAGE), 0.f);
}

TEST(MaxSpeedAtArrival, NonPassThroughBrakesToZero)
{
    EXPECT_FLOAT_EQ(maxSpeedAtArrival(Etape::EtapeType::DEPART), 0.f);
}

// ---------------------------------------------------------------------------
// isParkedAt
// ---------------------------------------------------------------------------

TEST(IsParkedAt, TrueWhenInsideRadiusOfASpot)
{
    EXPECT_TRUE(isParkedAt(pos(0, 0), { pos(0.3, 0.0) }, Distance(0.5)));
}

TEST(IsParkedAt, FalseWhenOutsideEverySpot)
{
    EXPECT_FALSE(isParkedAt(pos(0, 0), { pos(1.0, 0.0), pos(0.0, 1.0) }, Distance(0.5)));
}

TEST(IsParkedAt, FalseWhenNoSpots)
{
    EXPECT_FALSE(isParkedAt(pos(0, 0), {}, Distance(0.5)));
}

TEST(IsParkedAt, TrueWhenAnyOfSeveralSpotsMatches)
{
    EXPECT_TRUE(isParkedAt(pos(0, 0), { pos(5.0, 5.0), pos(0.1, 0.1) }, Distance(0.5)));
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
