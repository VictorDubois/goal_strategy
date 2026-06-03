/**
 * INTEGRATION tests for the mission-planning engine (StrategieV3 + Dijkstra + Etape).
 *
 * The engine is pure C++ (no ROS node, no Gazebo): we subclass StrategieV3 with a
 * tiny custom graph and drive it exactly as main.cpp's stateRun does —
 *   arrival      -> goToNextMission -> strat.update()
 *   collision    -> abortAction     -> strat.collisionAvoided() (then update())
 * and assert how the graph reacts.
 *
 * These are EVERGREEN: the graphs are generic (no competition coordinates/scores),
 * so they stay valid year to year. They are CHARACTERIZATION tests — they pin down
 * the engine's current behaviour.
 *
 * Run with:  colcon test --packages-select goal_strategy
 */

#include <map>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "krabilib/position.h"
#include "krabilib/strategie/etape.h"
#include "krabilib/strategie/mediumLevelAction.h"
#include "krabilib/strategie/strategiev3.h"

namespace
{
Position p(double x, double y)
{
    return Position(Distance(x), Distance(y));
}

// Minimal StrategieV3 with a graph-building DSL. Tasks are nodes given a positive
// score; reaching one turns it into a POINT_PASSAGE (updateStock) = done.
class TestStrategy : public StrategieV3
{
public:
    explicit TestStrategy(int maxNodes)
      : StrategieV3(/*isYellow=*/false, /*useXSymetry=*/true)
    {
        Etape::initTableauEtapeTotal(maxNodes);
    }

    // NOTE: never place a node at (0,0): Etape::setAction copies the action's
    // pose/type onto a node whose position equals the default Position(). All test
    // coordinates are >= 0.1 to avoid that.
    int addNode(Position pos, Etape::EtapeType type = Etape::POINT_PASSAGE)
    {
        int n = Etape::makeEtape(pos, type);
        return n;
    }

    void connect(int a, int b)
    {
        Etape::get(a)->addVoisin(b, /*autreSens=*/true);
    }

    void setTaskScore(int node, int score)
    {
        m_scores[node] = score;
    }

    void finalize(int garageNode)
    {
        m_numero_etape_garage = garageNode;
        m_nombre_etapes = Etape::getTotalEtapes();
        startDijkstra();
    }

    int etapeEnCours() const
    {
        return m_etape_en_cours;
    }
    int goalNum() const
    {
        return m_goal;
    }
    int status() const
    {
        return m_status_strat;
    }

protected:
    // Mirrors the year-specific Coupe convention: a deactivated (>= PIVOT_DESACTIVEE)
    // or already-done (POINT_PASSAGE/DEPART) node scores 0; a manual boost amplifies
    // an already-scoring node (coupe2026.cpp:718).
    int getScoreEtape(int i) override
    {
        Etape::EtapeType t = Etape::get(i)->getEtapeType();
        if (t >= Etape::PIVOT_DESACTIVEE || t == Etape::POINT_PASSAGE || t == Etape::DEPART)
        {
            return 0;
        }
        auto it = m_scores.find(i);
        int base = (it == m_scores.end()) ? 0 : it->second;
        if (base > 0)
        {
            base += static_cast<int>(Etape::get(i)->getBoostManuelDeScore());
        }
        return base;
    }

private:
    std::map<int, int> m_scores;
};

class MissionGraphTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        Etape::resetForTests(); // fresh process-global graph per scenario
    }
};

} // namespace

// ---------------------------------------------------------------------------
// Goal selection
// ---------------------------------------------------------------------------

TEST_F(MissionGraphTest, SelectsHighestScoreReachableTask)
{
    TestStrategy s(8);
    int start = s.addNode(p(0.1, 0.1), Etape::DEPART);
    int a = s.addNode(p(0.6, 0.1), Etape::GARDE_MANGER); // dist 0.5 from start
    int b = s.addNode(p(0.1, 0.6), Etape::GARDE_MANGER); // dist 0.5 from start
    s.setTaskScore(a, 5);
    s.setTaskScore(b, 9);
    s.connect(start, a);
    s.connect(start, b);
    s.finalize(start);

    s.update();
    EXPECT_EQ(s.goalNum(), b) << "equal distance -> higher score wins";
}

TEST_F(MissionGraphTest, PrefersNearerTaskWhenScoresEqual)
{
    TestStrategy s(8);
    int start = s.addNode(p(0.1, 0.1), Etape::DEPART);
    int near = s.addNode(p(0.4, 0.1), Etape::GARDE_MANGER); // dist 0.3
    int far = s.addNode(p(1.0, 0.1), Etape::GARDE_MANGER);  // dist 0.9
    s.setTaskScore(near, 5);
    s.setTaskScore(far, 5);
    s.connect(start, near);
    s.connect(start, far);
    s.finalize(start);

    s.update();
    EXPECT_EQ(s.goalNum(), near) << "equal score -> nearer wins";
}

TEST_F(MissionGraphTest, BoostOverridesSelection)
{
    TestStrategy s(8);
    int start = s.addNode(p(0.1, 0.1), Etape::DEPART);
    int near = s.addNode(p(0.4, 0.1), Etape::GARDE_MANGER); // would win on distance
    int far = s.addNode(p(1.0, 0.1), Etape::GARDE_MANGER);
    s.setTaskScore(near, 5);
    s.setTaskScore(far, 5);
    Etape::get(far)->setBoostManuelDeScore(10); // amplifies far's score 5 -> 15
    s.connect(start, near);
    s.connect(start, far);
    s.finalize(start);

    s.update();
    EXPECT_EQ(s.goalNum(), far) << "manual boost overrides the default (nearer) pick";
}

// ---------------------------------------------------------------------------
// Collision: failing to reach a node reroutes via an alternate path
// ---------------------------------------------------------------------------

TEST_F(MissionGraphTest, CollisionReroutesViaAlternatePath)
{
    TestStrategy s(8);
    int start = s.addNode(p(0.1, 0.1), Etape::DEPART);
    int A = s.addNode(p(0.5, 0.1), Etape::POINT_PASSAGE); // short branch
    int B = s.addNode(p(0.5, 0.6), Etape::POINT_PASSAGE); // long branch
    int task = s.addNode(p(0.9, 0.1), Etape::GARDE_MANGER);
    s.setTaskScore(task, 9);
    s.connect(start, A);
    s.connect(A, task);
    s.connect(start, B);
    s.connect(B, task);
    s.finalize(start);

    s.update();
    ASSERT_EQ(s.etapeEnCours(), A) << "shorter path goes via A first";

    s.collisionAvoided();
    s.update(); // engine marks A avoided, backtracks, reruns Dijkstra
    EXPECT_TRUE(Etape::get(A)->aEviter()) << "the node we failed to reach is now avoided";

    bool via_b = false;
    bool reached = false;
    for (int i = 0; i < 12 && !reached; ++i)
    {
        s.update();
        if (s.etapeEnCours() == B)
        {
            via_b = true;
        }
        if (s.etapeEnCours() == task)
        {
            reached = true;
        }
    }
    EXPECT_TRUE(reached) << "the task is still reached";
    EXPECT_TRUE(via_b) << "via the alternate B branch";
    EXPECT_TRUE(Etape::get(A)->aEviter()) << "A stayed avoided (rerouted, not forgotten)";
    EXPECT_EQ(s.goalNum(), task);
}

// ---------------------------------------------------------------------------
// Stuck on a sub-graph with no reachable task: avoid-marks are forgotten
// ---------------------------------------------------------------------------

TEST_F(MissionGraphTest, StuckSubgraphForgetsRobotVu)
{
    TestStrategy s(8);
    int start = s.addNode(p(0.1, 0.1), Etape::DEPART);
    int bottleneck = s.addNode(p(0.5, 0.1), Etape::POINT_PASSAGE);
    int task = s.addNode(p(0.9, 0.1), Etape::GARDE_MANGER);
    s.setTaskScore(task, 9);
    s.connect(start, bottleneck);
    s.connect(bottleneck, task); // the ONLY path to the task
    s.finalize(start);

    Etape::get(bottleneck)->robotVu(); // the only route is blocked
    ASSERT_TRUE(Etape::get(bottleneck)->aEviter());

    s.update();

    EXPECT_FALSE(Etape::get(bottleneck)->aEviter())
      << "no reachable task -> the engine forgets the avoid-mark (oublieRobotVu)";
    EXPECT_EQ(s.goalNum(), task) << "the task is reachable again";
}

// ---------------------------------------------------------------------------
// Prerequisite gating: a node deactivated until a prerequisite is completed
// ---------------------------------------------------------------------------

TEST_F(MissionGraphTest, PrerequisiteGatingThenReEnable)
{
    TestStrategy s(8);
    int start = s.addNode(p(0.1, 0.1), Etape::DEPART);
    int pickup = s.addNode(p(0.5, 0.1), Etape::GARDE_MANGER);
    int drop = s.addNode(p(0.9, 0.1), Etape::GARDE_MANGER);
    s.setTaskScore(pickup, 5);
    s.setTaskScore(drop, 9); // higher, but gated until pickup is done
    s.connect(start, pickup);
    s.connect(pickup, drop);
    Etape::get(pickup)->addEtapeActiveApres(drop); // deactivates drop now (+PIVOT_DESACTIVEE)
    s.finalize(start);

    ASSERT_GE(static_cast<int>(Etape::get(drop)->getEtapeType()),
              static_cast<int>(Etape::PIVOT_DESACTIVEE))
      << "drop starts deactivated";

    s.update();
    EXPECT_EQ(s.goalNum(), pickup) << "deactivated drop is skipped; pickup chosen first";

    bool drop_selected = false;
    for (int i = 0; i < 12 && !drop_selected; ++i)
    {
        s.update();
        if (s.goalNum() == drop)
        {
            drop_selected = true;
        }
    }
    EXPECT_TRUE(drop_selected) << "after completing pickup, drop is re-enabled and chosen";
    EXPECT_LT(static_cast<int>(Etape::get(drop)->getEtapeType()),
              static_cast<int>(Etape::PIVOT_DESACTIVEE))
      << "drop's deactivation bit was cleared";
}

// ---------------------------------------------------------------------------
// finir(): completing a node turns its linked etapes into plain waypoints
// ---------------------------------------------------------------------------

TEST_F(MissionGraphTest, FinirTurnsLinkedEtapesIntoWaypoints)
{
    TestStrategy s(8);
    int start = s.addNode(p(0.1, 0.1), Etape::DEPART);
    int x = s.addNode(p(0.5, 0.1), Etape::GARDE_MANGER);
    int y = s.addNode(p(0.9, 0.1), Etape::GARDE_MANGER);
    s.setTaskScore(x, 9);
    s.setTaskScore(y, 0); // never a goal on its own; we watch its type
    s.connect(start, x);
    s.connect(x, y);
    Etape::get(x)->addEtapeLieeParFinirEtape(y);
    s.finalize(start);

    ASSERT_EQ(Etape::get(y)->getEtapeType(), Etape::GARDE_MANGER);

    s.update(); // goal = x (adjacent) -> arrived at x
    s.update(); // completing x runs finir() on its linked etapes
    EXPECT_EQ(Etape::get(y)->getEtapeType(), Etape::POINT_PASSAGE)
      << "y was marked finished together with x";
}

// ---------------------------------------------------------------------------
// All tasks done -> strategy reports completion
// ---------------------------------------------------------------------------

TEST_F(MissionGraphTest, AllTasksDoneReturnsMinusOne)
{
    TestStrategy s(8);
    int start = s.addNode(p(0.1, 0.1), Etape::DEPART);
    int task = s.addNode(p(0.5, 0.1), Etape::GARDE_MANGER);
    s.setTaskScore(task, 9);
    s.connect(start, task);
    s.finalize(start);

    int ret = 0;
    for (int i = 0; i < 12; ++i)
    {
        ret = s.update();
        if (ret == -1)
        {
            break;
        }
    }
    EXPECT_EQ(ret, -1) << "once the task is done and we're back at the garage, update() returns -1";
}

// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv); // engine update() logs via rclcpp::get_logger
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
