# The Krabi strategy engine (how it works inside)

This document explains the **internals** of the mission-planning engine, so you can
read it, debug it, and extend `coupe20XX.cpp` with confidence. For the *what* (node
I/O, topics, "what to do for a new year") see the [README](../README.md).

The engine is **pure C++ / year-agnostic** — no ROS, no Gazebo. The integration
tests in [`test/test_mission_graph.cpp`](../test/test_mission_graph.cpp) drive it
directly and are the best *runnable* companion to this doc.

---

## 1. Big picture & data flow

The robot may only move along the edges of a hand-built **graph**. Each vertex is an
**`Etape`** (a place to act, or a plain waypoint). Choosing where to go is the job of
**`StrategieV3`**, which uses **`Dijkstra`** over the graph.

```
 coupe20XX.cpp ── builds ──▶  Etape graph (global static)
                                   ▲
 main.cpp (GoalStrat::stateRun)    │ reads/mutates
        │  (upon event)            │
        ▼                          │
   StrategieV3::update() ──runs──▶ Dijkstra::run()  (shortest paths from "here")
        │
        ▼  publishes the next hop's pose
   strat_movement ──▶ main_strategy (motion control)
```

**Event model** (driven by `GoalStrat::stateRun` in `src/goal/main.cpp`):

| Event | main.cpp does | engine call |
|---|---|---|
| Reached the current target | `goToNextMission()` | `strat.update()` |
| Move timed out (obstacle) | `abortAction()` | `strat.collisionAvoided()`, then `update()` |

So **every `update()` call means "I just arrived where I was going — recompute"**.
That single idea is enough to read the tests: they call `update()` in a loop to
simulate a sequence of arrivals.

---

## 2. The graph: `Etape`

- An `Etape` has a **position** (metres), a **type**, neighbours
  (`addVoisin`, bidirectional by default), an optional **`MediumLevelAction`**
  (what to *do* there), and a **score** (how much we want to go there).
- Nodes are created with the static `Etape::makeEtape(...)` and stored in a
  **process-global static** table (`tableauEtapesTotal`), numbered `0,1,2,…` by
  creation order. `Etape::get(i)` fetches node `i`.
- **Consequence for tests:** only *one* graph can exist per process. Tests call the
  test-only `Etape::resetForTests()` between scenarios; `coupe20XX` calls
  `initTableauEtapeTotal(NOMBRE_ETAPES)` once at start.
- Edge weights are straight-line distances (`computeChildDistances`), with a `+100`
  penalty when another robot is known to sit on that edge.
- **Avoidance & activation state** — two booleans, *not* part of the type:
  `robotVu()`/`oublieRobotVu()` toggle "a robot was seen here"; `desactive()`/`active()`
  toggle whether a gated node is allowed yet. **`aEviter()` (= robot-seen *or*
  deactivated)** is what makes Dijkstra skip a node, and an inactive node also scores 0.

---

## 3. `update()` walkthrough  (`strategiev3.cpp`)

`update()` returns a **status code**:

- `1` → next target is a **goal** (an objective worth points)
- `2` → next target is an **intermediate** waypoint on the way to the goal
- `-1` → **nothing left to do** (strategy finished)

Three key members, and the thing to internalise: **`m_etape_en_cours` is overloaded**
— it is *both* "where the robot just arrived" and, after `update()`, "the next hop to
drive to".

- `m_etape_en_cours` — current node / next hop (dual meaning above)
- `m_goal` — the chosen objective node
- `m_next_step` — the hop after `m_etape_en_cours` (or `-1`)

`update()` has three branches:

**Phase A — avoiding** (`m_avoiding` set by `collisionAvoided()`):
mark nearby nodes `robotVu()` (`robotVuDansCetteZone`, 10 cm radius), **backtrack to
the parent** node, rerun Dijkstra (now skipping the avoided node), set status `1`.
→ the next `update()` picks a fresh goal that routes around the obstacle.

**Phase B — arrived at an intermediate** (`m_status_strat == 2`):
just `updateIntermedaire()` — walk the Dijkstra **parent chain** from `m_goal` back
toward where we are to find the next hop. No new goal is chosen.

**Phase C — choose a new goal** (`m_status_strat == 1`):
1. `finir()` the etapes linked to the one just completed; **re-enable** any nodes
   gated behind it (via `active()`); `updateStock()` turns the reached node into a
   plain `POINT_PASSAGE` (= done).
2. `updateScores()` → set every node's score from the year-specific `getScoreEtape`.
   If *no* node scores, `oublieRobotVu()` on all nodes and re-check; if still nothing
   and we're at the garage, return `-1`; otherwise make the garage attractive.
3. Run Dijkstra from here, then **select the goal**: the node maximising the
   heuristic (§4) among nodes that score and are reachable.
4. `updateIntermedaire()` to set `m_etape_en_cours` to the first hop.

---

## 4. Goal-selection heuristic

For every node, `update()` computes (`strategiev3.cpp`):

```
heuristic = 100000  −  distance*1000  +  getScore()*1000
```

and picks the **max** among nodes with non-zero score and a known distance
(`distance != -1`). So: **score dominates, distance breaks ties** (each point is
worth ~1 m of detour).

Note: this is a very simple heuristic, it could/should be tuned

`getScore()` returns 0 for an *inactive* node; otherwise the value last set by
`updateScores()` = the year-specific **`getScoreEtape(i)`**. By convention (see
`coupe2026.cpp`), `getScoreEtape` adds the node's **`getBoostManuelDeScore()`** to an
already-positive score — that is how you manually bias the robot toward an etape
(`setBoostManuelDeScore`). The tests `BoostOverridesSelection` /
`BoostOverridesRealGoalSelection` demonstrate it.

---

## 5. Dijkstra & the "stuck" recovery  (`dijkstra.cpp`)

Standard Dijkstra over the graph, with two engine-specific twists.

**Per-node state during a run:** `distance` is `-1` until reached; `state` is `-1`
unvisited, `-2` avoided (set when `aEviter()` at init), `0` for the source, `≥1` for
the visit order. Avoided nodes are skipped both when picking the minimum and when
relaxing neighbours.

**`trouverMin` return codes:** index of the next closest node, or `-2` (no reachable
scoring node left → done), or `-1` (we are *coincés dans un endroit pourri* — stuck
with no reachable task).

**Stuck recovery ("deblocage"):** when no reachable node scores, the engine
`oublieRobotVu()` on **every** node (dropping all the avoid-marks) and reruns Dijkstra
once. This is what lets the robot escape a sub-graph it had fenced itself into with
earlier collision marks. The test `StuckSubgraphForgetsRobotVu` pins this down.

---

## 6. Patterns you'll reuse in `coupe20XX.cpp`

These recurring idioms (all visible in `coupe2026.cpp`) are worth recognising:

- **Same action from two approaches (`_from_top`).** Two etapes for one physical
  spot, linked *mutually* so finishing one finishes the other:
  `Etape::get(a)->addEtapeLieeParFinirEtape(b)` **and** `b → a`.
- **Prerequisite gating.** `Etape::get(prereq)->addEtapeActiveApres(gated)` deactivates until `prereq` is completed (then it's re-enabled in Phase C).
- **Action ends elsewhere.** `setNumeroEtapeFinAction(n)` — the action starts at one
  etape and the robot ends up at `n` (e.g. the thermometre sweep).
- **Us/them symmetry.** `positionC(x, y)` mirrors coordinates for blue vs yellow, so
  you describe one side and call it twice. (This is also the source of copy-paste
  name bugs — double-check the `"name"` strings.)

---

## 7. Add your first etape + action (walkthrough)

1. **Define the action** (if it's an action site, not a waypoint): subclass
   `MediumLevelAction` like `krabilib/strategie/zoneDeRamassage.{h,cpp}` — it carries
   the goal `Pose` and a `getType()` (its `EtapeType`).
2. **Create the etape** in `coupe20XX`'s constructor:
   `int my = Etape::makeEtape(new MyAction(approachPose, restPose), "my_name");`
   (or `Etape::makeEtape(positionC(x, y), Etape::POINT_PASSAGE)` for a waypoint).
3. **Wire neighbours:** `Etape::get(my)->addVoisin(other);` (bidirectional).
4. **Score it:** in `getScoreEtape(i)`, return a positive number for `my`'s type
   while it's still worth doing (and `0` once done / when conditions aren't met).
5. **Run it:** `krabi && ros2 launch goal_strategy goal_strat_launch.py`, then watch
   `debug_etapes` in RViz/Foxglove (legend in the README).

For minimal, runnable examples of each engine behaviour (selection, reroute,
stuck-recovery, gating, boost), read `test/test_mission_graph.cpp`.

---

## 8. Known sharp edges

- **Boost only amplifies.** `getBoostManuelDeScore()` is added *only when the base
  score is already > 0* — a boost cannot activate a zero-score node.
