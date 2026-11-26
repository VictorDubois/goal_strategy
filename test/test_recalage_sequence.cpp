/**
 * Lecture des zones de recalage (YAML) : pure logique, sans ROS.
 */

#include <cmath>
#include <fstream>
#include <sstream>

#include <gtest/gtest.h>

#include "goal_strategy/recalage_sequence.h"

using recalage::Config;
using recalage::load;
using recalage::Pose2D;
using recalage::Step;

namespace
{
const Pose2D START{ -1.159, -0.692, -M_PI / 2 };

const char* GOOD = R"(
default_zone: a
zones:
  a:
    label: Zone A
    pose: launch
    sequence: [wall X_MIN 0.10, wall Y_MIN 0.2, start]
  b:
    pose: -0.9 0.75 -90
    sequence: [wall Y_MAX 0.1, goto -1.0 0.2, rotate 45, start]
)";
} // namespace

TEST(RecalageSequence, LoadsBlue)
{
    const Config c = load(GOOD, true, START);
    ASSERT_TRUE(c.errors.empty()) << c.errors.front();
    ASSERT_EQ(c.zones.size(), 2u);
    EXPECT_EQ(c.default_zone, "a");

    const auto* a = c.find("a");
    ASSERT_NE(a, nullptr);
    EXPECT_EQ(a->label, "Zone A");
    EXPECT_DOUBLE_EQ(a->pose.x, START.x);
    EXPECT_DOUBLE_EQ(a->pose.theta, START.theta);
    ASSERT_EQ(a->steps.size(), 3u);
    EXPECT_EQ(a->steps[0].wall, Wall::X_MIN);
    EXPECT_DOUBLE_EQ(a->steps[1].clearance, 0.2);
    EXPECT_EQ(a->steps[2].type, Step::Type::START);

    const auto* b = c.find("b");
    ASSERT_NE(b, nullptr);
    EXPECT_EQ(b->label, "b") << "label par défaut = nom";
    EXPECT_NEAR(b->pose.theta, -M_PI / 2, 1e-9);
    EXPECT_NEAR(b->steps[2].theta, M_PI / 4, 1e-9);
}

TEST(RecalageSequence, YellowIsMirrored)
{
    const Config c = load(GOOD, false, START);
    const auto* a = c.find("a");
    ASSERT_NE(a, nullptr);
    EXPECT_DOUBLE_EQ(a->pose.x, START.x) << "pose: launch n'est jamais symétrisée";
    EXPECT_EQ(a->steps[0].wall, Wall::X_MAX);
    EXPECT_EQ(a->steps[1].wall, Wall::Y_MIN) << "les murs Y ne changent pas";

    const auto* b = c.find("b");
    ASSERT_NE(b, nullptr);
    EXPECT_DOUBLE_EQ(b->pose.x, 0.9);
    EXPECT_DOUBLE_EQ(b->pose.y, 0.75);
    EXPECT_NEAR(b->pose.theta, -M_PI / 2, 1e-9);
    EXPECT_DOUBLE_EQ(b->steps[1].x, 1.0);
    EXPECT_NEAR(b->steps[2].theta, 3 * M_PI / 4, 1e-9);
}

TEST(RecalageSequence, ExplicitYellowIsNotMirrored)
{
    const char* yaml = R"(
default_zone: a
zones:
  a:
    pose: -1.0 0.2 0
    pose_yellow: 1.1 0.3 90
    sequence: [wall X_MIN 0.1]
    sequence_yellow: [wall Y_MIN 0.1, wall X_MAX 0.2]
)";
    const Config y = load(yaml, false, START);
    ASSERT_NE(y.find("a"), nullptr);
    EXPECT_DOUBLE_EQ(y.find("a")->pose.x, 1.1);
    EXPECT_NEAR(y.find("a")->pose.theta, M_PI / 2, 1e-9);
    ASSERT_EQ(y.find("a")->steps.size(), 2u);
    EXPECT_EQ(y.find("a")->steps[1].wall, Wall::X_MAX);

    const Config b = load(yaml, true, START);
    EXPECT_EQ(b.find("a")->steps.size(), 1u);
}

TEST(RecalageSequence, BadStepsAreRefused)
{
    for (const char* text : { "",
                              "wall",
                              "wall X_MIN",
                              "wall NORD 0.1",
                              "wall X_MIN 0.1 0.2",
                              "wall X_MIN 0,1",
                              "wall X_MIN -0.1",
                              "wall X_MIN 3",
                              "goto 0.1",
                              "goto 2 0",
                              "goto 0 0.95",
                              "goto nan 0",
                              "rotate",
                              "rotate 90deg",
                              "start now",
                              "Wall X_MIN 0.1",
                              "avance 0.1" })
    {
        Step step;
        std::string error;
        EXPECT_FALSE(recalage::parseStep(text, step, error)) << text;
        EXPECT_FALSE(error.empty()) << text;
    }
}

// Une zone fausse est désactivée, les autres restent utilisables
TEST(RecalageSequence, BadZoneIsDisabledOthersStay)
{
    const std::string bad_zones[] = {
        "{pose: launch, sequence: [wall NORD 0.1]}",               // mur inconnu
        "{pose: launch, sequence: [goto 0 0, start]}",             // aucun mur
        "{pose: launch, sequence: []}",                            // vide
        "{pose: launch, sequence: wall X_MIN 0.1}",                // pas une liste
        "{pose: launch}",                                          // pas de séquence
        "{sequence: [wall X_MIN 0.1]}",                            // pas de pose
        "{pose: 2 0 0, sequence: [wall X_MIN 0.1]}",               // hors table
        "{pose: [1, 2], sequence: [wall X_MIN 0.1]}",              // mauvais type
        "{pose: launch, sequence: [start, wall X_MIN 0.1]}",       // start pas en dernier
        "{pose: launch, sequences: [wall X_MIN 0.1]}",             // faute de frappe
        "{pose: launch, sequence: [wall X_MIN 0.1], sequence_yellow: [x]}", // faute en jaune
        "juste du texte",
    };
    for (const auto& bad : bad_zones)
    {
        const std::string yaml = "default_zone: ok\nzones:\n  ok: {pose: launch, sequence: [wall "
                                 "X_MIN 0.1]}\n  bad: "
                                 + bad + "\n";
        const Config c = load(yaml, true, START);
        EXPECT_NE(c.find("ok"), nullptr) << bad;
        EXPECT_EQ(c.find("bad"), nullptr) << bad;
        EXPECT_EQ(c.errors.size(), 1u) << bad;
        EXPECT_EQ(c.default_zone, "ok") << bad;
    }
}

TEST(RecalageSequence, BrokenFileNeverThrows)
{
    for (const char* yaml : { "", "zones: [", "zones: 3", "bonjour", "zones:\n\t- a", "- a\n- b" })
    {
        Config c;
        EXPECT_NO_THROW(c = load(yaml, true, START)) << yaml;
        EXPECT_TRUE(c.zones.empty()) << yaml;
        EXPECT_FALSE(c.errors.empty()) << yaml;
        EXPECT_TRUE(c.default_zone.empty()) << yaml;
    }
}

TEST(RecalageSequence, MissingDefaultZoneIsReportedWithoutFallback)
{
    const Config c
      = load("default_zone: x\nzones: {a: {pose: launch, sequence: [wall X_MIN 0.1]}}", true, START);
    EXPECT_NE(c.find("a"), nullptr);
    EXPECT_TRUE(c.default_zone.empty());
    EXPECT_EQ(c.errors.size(), 1u);
}

// Le fichier livré doit être valide, dans les deux couleurs
TEST(RecalageSequence, ShippedFileIsValid)
{
    std::ifstream file(RECALAGE_YAML);
    ASSERT_TRUE(file.good()) << RECALAGE_YAML;
    std::stringstream content;
    content << file.rdbuf();
    for (const bool blue : { true, false })
    {
        const Config c = load(content.str(), blue, START);
        EXPECT_TRUE(c.errors.empty()) << c.errors.front();
        EXPECT_EQ(c.default_zone, "depart");
        EXPECT_EQ(c.zones.size(), 2u);
    }
}
