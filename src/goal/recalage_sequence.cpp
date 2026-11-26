#include "goal_strategy/recalage_sequence.h"

#include <cmath>
#include <set>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace recalage
{
namespace
{
// Limites de table pour base_link : au-delà, le robot serait dans la bordure
constexpr double kMaxX = 1.40;
constexpr double kMaxY = 0.90;
constexpr double kMaxClearance = 1.0;

double degToRad(double a_deg)
{
    return a_deg * M_PI / 180.;
}

double wrap(double a_rad)
{
    return std::atan2(std::sin(a_rad), std::cos(a_rad));
}

std::vector<std::string> split(const std::string& a_text)
{
    std::istringstream l_stream(a_text);
    std::vector<std::string> l_words;
    std::string l_word;
    while (l_stream >> l_word)
    {
        l_words.push_back(l_word);
    }
    return l_words;
}

/// Nombre strict : "0.1a", "nan" ou "inf" sont refusés
bool toDouble(const std::string& a_text, double& a_value)
{
    try
    {
        size_t l_used = 0;
        a_value = std::stod(a_text, &l_used);
        return l_used == a_text.size() && std::isfinite(a_value);
    }
    catch (const std::exception&)
    {
        return false;
    }
}

bool insideTable(double a_x, double a_y)
{
    return std::abs(a_x) <= kMaxX && std::abs(a_y) <= kMaxY;
}

bool parseWall(const std::string& a_text, Wall& a_wall)
{
    if (a_text == "X_MIN")
        a_wall = Wall::X_MIN;
    else if (a_text == "X_MAX")
        a_wall = Wall::X_MAX;
    else if (a_text == "Y_MIN")
        a_wall = Wall::Y_MIN;
    else if (a_text == "Y_MAX")
        a_wall = Wall::Y_MAX;
    else
        return false;
    return true;
}

/// "x y degrés"
bool parsePose(const std::string& a_text, Pose2D& a_pose, std::string& a_error)
{
    const auto l_words = split(a_text);
    double l_deg = 0.;
    if (l_words.size() != 3 || !toDouble(l_words[0], a_pose.x) || !toDouble(l_words[1], a_pose.y)
        || !toDouble(l_words[2], l_deg))
    {
        a_error = "pose '" + a_text + "' : attendu 'x y degrés' ou 'launch'";
        return false;
    }
    if (!insideTable(a_pose.x, a_pose.y))
    {
        a_error = "pose '" + a_text + "' : hors de la table";
        return false;
    }
    a_pose.theta = wrap(degToRad(l_deg));
    return true;
}

bool parseSequence(const YAML::Node& a_node, std::vector<Step>& a_steps, std::string& a_error)
{
    if (!a_node.IsSequence() || a_node.size() == 0)
    {
        a_error = "séquence absente ou vide";
        return false;
    }
    bool l_has_wall = false;
    for (size_t i = 0; i < a_node.size(); ++i)
    {
        Step l_step;
        if (!parseStep(a_node[i].as<std::string>(), l_step, a_error))
        {
            a_error = "étape " + std::to_string(i + 1) + " : " + a_error;
            return false;
        }
        if (l_step.type == Step::Type::START && i + 1 != a_node.size())
        {
            a_error = "étape " + std::to_string(i + 1) + " : 'start' doit être la dernière";
            return false;
        }
        l_has_wall = l_has_wall || l_step.type == Step::Type::WALL;
        a_steps.push_back(l_step);
    }
    if (!l_has_wall)
    {
        a_error = "séquence sans mur : rien ne serait recalé";
        return false;
    }
    return true;
}

bool parseZone(const std::string& a_name,
               const YAML::Node& a_node,
               bool a_is_blue,
               const Pose2D& a_start_pose,
               Zone& a_zone,
               std::string& a_error)
{
    static const std::set<std::string> l_known
      = { "label", "pose", "pose_yellow", "sequence", "sequence_yellow" };
    if (!a_node.IsMap())
    {
        a_error = "contenu invalide";
        return false;
    }
    for (const auto& l_entry : a_node)
    {
        const auto l_key = l_entry.first.as<std::string>();
        if (l_known.count(l_key) == 0)
        {
            a_error = "champ inconnu '" + l_key + "'";
            return false;
        }
    }

    a_zone.name = a_name;
    a_zone.label = a_node["label"] ? a_node["label"].as<std::string>() : a_name;

    // Les deux couleurs sont toujours vérifiées : une faute dans la version jaune doit se voir
    // aussi quand on démarre en bleu.
    if (!a_node["pose"])
    {
        a_error = "pose absente";
        return false;
    }
    Pose2D l_blue, l_yellow;
    const auto l_pose_text = a_node["pose"].as<std::string>();
    if (l_pose_text == "launch")
    {
        l_blue = a_start_pose;
        l_yellow = a_start_pose; // le launch donne déjà la pose de la bonne couleur
    }
    else
    {
        if (!parsePose(l_pose_text, l_blue, a_error))
        {
            return false;
        }
        l_yellow = mirror(l_blue);
    }
    if (a_node["pose_yellow"]
        && !parsePose(a_node["pose_yellow"].as<std::string>(), l_yellow, a_error))
    {
        return false;
    }
    a_zone.pose = a_is_blue ? l_blue : l_yellow;

    std::vector<Step> l_blue_steps, l_yellow_steps;
    if (!a_node["sequence"])
    {
        a_error = "séquence absente ou vide";
        return false;
    }
    if (!parseSequence(a_node["sequence"], l_blue_steps, a_error))
    {
        return false;
    }
    for (const auto& l_step : l_blue_steps)
    {
        l_yellow_steps.push_back(mirror(l_step));
    }
    if (a_node["sequence_yellow"])
    {
        l_yellow_steps.clear();
        if (!parseSequence(a_node["sequence_yellow"], l_yellow_steps, a_error))
        {
            a_error = "sequence_yellow, " + a_error;
            return false;
        }
    }
    a_zone.steps = a_is_blue ? l_blue_steps : l_yellow_steps;
    return true;
}

} // namespace

const Zone* Config::find(const std::string& a_name) const
{
    for (const auto& l_zone : zones)
    {
        if (l_zone.name == a_name)
        {
            return &l_zone;
        }
    }
    return nullptr;
}

bool parseStep(const std::string& a_text, Step& a_step, std::string& a_error)
{
    const auto l_words = split(a_text);
    a_step = Step();
    a_step.text = a_text;
    a_error = "'" + a_text + "' : ";
    if (l_words.empty())
    {
        a_error += "étape vide";
        return false;
    }

    if (l_words[0] == "wall")
    {
        a_step.type = Step::Type::WALL;
        if (l_words.size() != 3 || !parseWall(l_words[1], a_step.wall)
            || !toDouble(l_words[2], a_step.clearance))
        {
            a_error += "attendu 'wall X_MIN|X_MAX|Y_MIN|Y_MAX dégagement'";
            return false;
        }
        if (a_step.clearance < 0. || a_step.clearance > kMaxClearance)
        {
            a_error += "dégagement entre 0 et 1 m";
            return false;
        }
    }
    else if (l_words[0] == "goto")
    {
        a_step.type = Step::Type::GOTO;
        if (l_words.size() != 3 || !toDouble(l_words[1], a_step.x)
            || !toDouble(l_words[2], a_step.y))
        {
            a_error += "attendu 'goto x y'";
            return false;
        }
        if (!insideTable(a_step.x, a_step.y))
        {
            a_error += "hors de la table";
            return false;
        }
    }
    else if (l_words[0] == "rotate")
    {
        a_step.type = Step::Type::ROTATE;
        double l_deg = 0.;
        if (l_words.size() != 2 || !toDouble(l_words[1], l_deg))
        {
            a_error += "attendu 'rotate degrés'";
            return false;
        }
        a_step.theta = wrap(degToRad(l_deg));
    }
    else if (l_words[0] == "start")
    {
        a_step.type = Step::Type::START;
        if (l_words.size() != 1)
        {
            a_error += "'start' ne prend pas d'argument";
            return false;
        }
    }
    else
    {
        a_error += "étape inconnue (wall, goto, rotate, start)";
        return false;
    }
    a_error.clear();
    return true;
}

Pose2D mirror(const Pose2D& a_pose)
{
    Pose2D l_pose = a_pose;
    l_pose.x = -a_pose.x;
    l_pose.theta = wrap(M_PI - a_pose.theta);
    return l_pose;
}

Step mirror(const Step& a_step)
{
    Step l_step = a_step;
    l_step.x = -a_step.x;
    l_step.theta = wrap(M_PI - a_step.theta);
    if (a_step.wall == Wall::X_MIN)
    {
        l_step.wall = Wall::X_MAX;
    }
    else if (a_step.wall == Wall::X_MAX)
    {
        l_step.wall = Wall::X_MIN;
    }
    l_step.text += " (symétrisé)";
    return l_step;
}

Config load(const std::string& a_yaml, bool a_is_blue, const Pose2D& a_start_pose)
{
    Config l_config;
    try
    {
        const YAML::Node l_root = YAML::Load(a_yaml);
        if (!l_root.IsMap() || !l_root["zones"] || !l_root["zones"].IsMap())
        {
            l_config.errors.push_back("fichier invalide : 'zones' absent");
            return l_config;
        }
        for (const auto& l_entry : l_root["zones"])
        {
            const auto l_name = l_entry.first.as<std::string>();
            Zone l_zone;
            std::string l_error;
            bool l_ok = false;
            try
            {
                if (l_config.find(l_name) != nullptr)
                {
                    l_error = "nom déjà utilisé";
                }
                else
                {
                    l_ok = parseZone(
                      l_name, l_entry.second, a_is_blue, a_start_pose, l_zone, l_error);
                }
            }
            catch (const YAML::Exception& ex)
            {
                l_error = ex.what();
            }
            if (l_ok)
            {
                l_config.zones.push_back(l_zone);
            }
            else
            {
                l_config.errors.push_back("zone '" + l_name + "' désactivée : " + l_error);
            }
        }

        if (l_root["default_zone"])
        {
            l_config.default_zone = l_root["default_zone"].as<std::string>();
        }
        if (l_config.find(l_config.default_zone) == nullptr)
        {
            // Pas de repli sur une autre zone : le robot partirait d'une pose inattendue
            l_config.errors.push_back("zone par défaut '" + l_config.default_zone
                                      + "' absente ou désactivée");
            l_config.default_zone.clear();
        }
    }
    catch (const YAML::Exception& ex)
    {
        l_config.zones.clear();
        l_config.default_zone.clear();
        l_config.errors.push_back(std::string("fichier illisible : ") + ex.what());
    }
    return l_config;
}

} // namespace recalage
