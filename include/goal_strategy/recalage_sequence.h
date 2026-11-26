#pragma once

/**
 * Zones de recalage et séquences, lues dans un fichier YAML (config/recalage_2026.yaml).
 *
 * Sans ROS, comme goal_logic : testable seul (test/test_recalage_sequence.cpp).
 * Documentation pour l'utilisateur : commentaires de config/recalage_2026.yaml.
 *
 * Une erreur dans une zone la désactive (message dans RecalageConfig::errors) sans toucher aux
 * autres zones, ni au reste de la stratégie.
 */

#include <string>
#include <vector>

/// Bordures de la table, dans le repère map (origine au centre, table 3 m x 2 m)
enum class Wall
{
    X_MIN, // x = -1.5
    X_MAX, // x = +1.5
    Y_MIN, // y = -1.0
    Y_MAX  // y = +1.0
};

namespace recalage
{

struct Pose2D
{
    double x = 0.;
    double y = 0.;
    double theta = 0.; // rad
};

struct Step
{
    enum class Type
    {
        WALL,   // dos au mur, correction, dégagement
        GOTO,   // aller en (x, y)
        ROTATE, // s'orienter vers theta
        START   // retour à la pose de départ
    };
    Type type = Type::START;
    Wall wall = Wall::X_MIN;
    double clearance = 0.; // m
    double x = 0., y = 0.; // m
    double theta = 0.;     // rad
    std::string text;      // texte d'origine, pour les logs et l'IHM
};

/// Zone prête à l'emploi pour la couleur du robot
struct Zone
{
    std::string name;
    std::string label;
    Pose2D pose; // où on pose le robot
    std::vector<Step> steps;
};

struct Config
{
    std::string default_zone;
    std::vector<Zone> zones;
    std::vector<std::string> errors; // une ligne par problème

    const Zone* find(const std::string& a_name) const;
};

/// Lit une étape. En cas d'erreur, renvoie false et remplit a_error.
bool parseStep(const std::string& a_text, Step& a_step, std::string& a_error);

/// Symétrie bleu -> jaune (x -> -x, comme StrategieV3::positionC avec useXSymetry)
Pose2D mirror(const Pose2D& a_pose);
Step mirror(const Step& a_step);

/**
 * @param a_yaml contenu du fichier
 * @param a_is_blue couleur du robot : en jaune, les poses et séquences sont symétrisées, sauf
 *                  pose_yellow / sequence_yellow qui sont prises telles quelles
 * @param a_start_pose pose de départ du launch (pour "pose: launch"), jamais symétrisée
 */
Config load(const std::string& a_yaml, bool a_is_blue, const Pose2D& a_start_pose);

} // namespace recalage
