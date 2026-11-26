/**
 * TiretteFilter: the match must only start on "stably inserted, then removed".
 * Pure logic, no ROS: time is passed explicitly.
 */

#include <gtest/gtest.h>

#include "krabilib/tirette_filter.h"

namespace
{
constexpr bool REMOVED = true;
constexpr bool INSERTED = false;
} // namespace

TEST(TiretteFilter, BootWithoutTiretteNeverStarts)
{
    TiretteFilter f(1.0);
    for (double t = 0.; t < 120.; t += 0.1)
    {
        EXPECT_FALSE(f.update(REMOVED, t));
    }
    EXPECT_FALSE(f.armed());
    EXPECT_FALSE(f.started());
}

TEST(TiretteFilter, InsertedThenRemovedStartsOnce)
{
    TiretteFilter f(1.0);
    f.update(REMOVED, 0.);
    for (double t = 1.; t < 3.; t += 0.1)
    {
        EXPECT_FALSE(f.update(INSERTED, t));
    }
    EXPECT_TRUE(f.armed());
    EXPECT_TRUE(f.update(REMOVED, 3.));
    EXPECT_TRUE(f.started());
    EXPECT_FALSE(f.update(REMOVED, 3.1)) << "start must be reported only once";
}

TEST(TiretteFilter, ContactBounceAtInsertionDoesNotStart)
{
    TiretteFilter f(1.0);
    double t = 0.;
    for (int i = 0; i < 5; ++i)
    {
        EXPECT_FALSE(f.update(INSERTED, t += 0.1));
        EXPECT_FALSE(f.update(REMOVED, t += 0.1));
    }
    EXPECT_FALSE(f.armed());
    EXPECT_FALSE(f.started());
}

TEST(TiretteFilter, SingleInsertedSampleIsNotStable)
{
    // One "inserted" sample, then nothing for a long time, then "removed"
    TiretteFilter f(1.0);
    f.update(INSERTED, 0.);
    EXPECT_FALSE(f.update(REMOVED, 60.));
    EXPECT_FALSE(f.started());
}

TEST(TiretteFilter, ReinsertionAfterStartDoesNotRestart)
{
    TiretteFilter f(1.0);
    f.update(INSERTED, 0.);
    f.update(INSERTED, 1.5);
    EXPECT_TRUE(f.update(REMOVED, 2.));
    f.update(INSERTED, 3.);
    f.update(INSERTED, 5.);
    EXPECT_FALSE(f.update(REMOVED, 6.));
    EXPECT_TRUE(f.started());
}
