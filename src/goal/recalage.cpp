/**
 * Recalage automatique sur bordure.
 *
 * Principe : plaqué contre un mur, on connaît le cap du robot (perpendiculaire au mur) et UNE
 * coordonnée (celle du mur ± distance base_link -> face de contact). On envoie ces valeurs au
 * service correct_pose (map_odom_publisher) qui corrige map -> odom ; l'autre coordonnée est
 * conservée.
 *
 * Toutes ces fonctions sont BLOQUANTES (usleep) : elles sont appelées depuis loop(), comme les
 * autres mouvements de GoalStrat. Le service est appelé en asynchrone et on attend le future
 * avec un timeout. Le client est dans son propre callback group : le MultiThreadedExecutor peut
 * donc traiter la réponse sur un autre thread pendant que loop() attend.
 */

#include "goal_strategy/goal.h"
#include "goal_strategy/goal_logic.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <unistd.h>

using namespace std::chrono_literals;

namespace
{
constexpr double kTableHalfX = 1.5; // m
constexpr double kTableHalfY = 1.0; // m

/// Direction (cap) qui pointe du mur vers l'intérieur de la table
double inwardHeading(Wall a_wall)
{
    switch (a_wall)
    {
    case Wall::X_MIN:
        return 0.0;
    case Wall::X_MAX:
        return M_PI;
    case Wall::Y_MIN:
        return M_PI / 2;
    case Wall::Y_MAX:
    default:
        return -M_PI / 2;
    }
}

/// Coordonnée du mur dans map
double wallCoordinate(Wall a_wall)
{
    switch (a_wall)
    {
    case Wall::X_MIN:
        return -kTableHalfX;
    case Wall::X_MAX:
        return kTableHalfX;
    case Wall::Y_MIN:
        return -kTableHalfY;
    case Wall::Y_MAX:
    default:
        return kTableHalfY;
    }
}

bool isXWall(Wall a_wall)
{
    return a_wall == Wall::X_MIN || a_wall == Wall::X_MAX;
}

const char* wallName(Wall a_wall)
{
    switch (a_wall)
    {
    case Wall::X_MIN:
        return "X_MIN";
    case Wall::X_MAX:
        return "X_MAX";
    case Wall::Y_MIN:
        return "Y_MIN";
    case Wall::Y_MAX:
    default:
        return "Y_MAX";
    }
}
} // namespace

/**
 * @brief Déclare les paramètres ROS du recalage, crée le client correct_pose et l'abonnement
 * motors_current. Appelé depuis le constructeur.
 */
void GoalStrat::declareRecalageParameters()
{
    // Distance base_link -> bord arrière, mesurée sur le robot : 104 mm (URDF simu : 105 mm).
    // Les roues d'odométrie sont distinctes des roues de propulsion : quand le robot bute,
    // l'odom s'arrête même si les roues motrices patinent => détection "immobile" fiable.
    // En simu, krabi_description/scripts/sim_odometry.py reproduit ce comportement.
    m_recalage.contact_offset_back = this->declare_parameter("recalage.contact_offset_back", 0.104);
    // Face avant : non mesurée (contact avant non utilisé pour l'instant)
    m_recalage.contact_offset_front
      = this->declare_parameter("recalage.contact_offset_front", 0.104);
    m_recalage.still_dist = this->declare_parameter("recalage.still_dist", 0.002);
    m_recalage.still_time = this->declare_parameter("recalage.still_time", 0.3);
    m_recalage.settle_time = this->declare_parameter("recalage.settle_time", 0.2);
    m_recalage.timeout = this->declare_parameter("recalage.timeout", 5.0);
    m_recalage.align_timeout = this->declare_parameter("recalage.align_timeout", 5.0);
    m_recalage.service_timeout = this->declare_parameter("recalage.service_timeout", 1.0);
    // Garde-fous : un robot qui ne démarre pas est immobile, sans être contre un mur
    m_recalage.min_push_dist = this->declare_parameter("recalage.min_push_dist", 0.02);
    m_recalage.max_correction_dist = this->declare_parameter("recalage.max_correction_dist", 0.15);
    m_recalage.max_correction_angle
      = this->declare_parameter("recalage.max_correction_angle", 0.35); // 20°
    m_recalage.use_lidar_loc = this->declare_parameter("recalage.use_lidar_loc", false);

    // Groupe dédié : la réponse du service et motors_current sont traités même si loop()
    // (groupe par défaut) est bloqué dans un recalage.
    m_recalage_cb_group
      = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    m_correct_pose_client = this->create_client<krabi_msgs::srv::CorrectPose>(
      "correct_pose", rclcpp::ServicesQoS(), m_recalage_cb_group);

    rclcpp::SubscriptionOptions l_options;
    l_options.callback_group = m_recalage_cb_group;
    m_motors_current_sub = this->create_subscription<krabi_msgs::msg::MotorsCurrent>(
      "motors_current",
      5,
      std::bind(&GoalStrat::updateMotorsCurrent, this, std::placeholders::_1),
      l_options);

    // ---- Actions avant match ----
    // Pose de départ (map), passée par le launch (= xRobotPos / yRobotPos / zRobotOrientation)
    m_start_pose_x = this->declare_parameter("start_pose.x", -1.25);
    m_start_pose_y = this->declare_parameter("start_pose.y", -0.75);
    m_start_pose_theta = this->declare_parameter("start_pose.theta", M_PI / 2);

    // Client vers Core (main_strategy) : autorise les moteurs avant la tirette
    m_pre_match_motion_client = this->create_client<std_srvs::srv::SetBool>(
      "enable_pre_match_motion", rclcpp::ServicesQoS(), m_recalage_cb_group);

    loadRecalageZones();

    // Conservés pour les abonnés tardifs (IHM relancée...)
    const auto l_latched = rclcpp::QoS(1).transient_local();
    m_pre_match_state_pub
      = this->create_publisher<std_msgs::msg::String>("pre_match_state", l_latched);
    m_recalage_zones_pub
      = this->create_publisher<krabi_msgs::msg::RecalageZones>("recalage_zones", l_latched);
    publishRecalageZones();
    publishPreMatchState();

    // Les services ne font que déposer une demande : l'action s'exécute dans loop()
    m_start_recalage_zone_srv = this->create_service<krabi_msgs::srv::StartRecalage>(
      "start_recalage_zone",
      [this](const std::shared_ptr<krabi_msgs::srv::StartRecalage::Request> req,
             std::shared_ptr<krabi_msgs::srv::StartRecalage::Response> res)
      { res->success = requestPreMatchAction(PRE_MATCH_RECALAGE, req->zone, res->message); },
      rclcpp::ServicesQoS(),
      m_recalage_cb_group);
    m_recalage_sub = this->create_subscription<std_msgs::msg::Bool>(
      "recalage",
      5,
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
          std::string l_message; // déjà dans les logs
          if (msg->data)
          {
              requestPreMatchAction(PRE_MATCH_RECALAGE, "", l_message);
          }
      },
      l_options);
    m_start_recalage_srv = this->create_service<std_srvs::srv::Trigger>(
      "start_recalage",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res)
      { res->success = requestPreMatchAction(PRE_MATCH_RECALAGE, "", res->message); },
      rclcpp::ServicesQoS(),
      m_recalage_cb_group);
    m_go_to_start_srv = this->create_service<std_srvs::srv::Trigger>(
      "go_to_start",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res)
      { res->success = requestPreMatchAction(PRE_MATCH_GO_TO_START, "", res->message); },
      rclcpp::ServicesQoS(),
      m_recalage_cb_group);
}

/**
 * @brief Lit les zones de recalage. Toute erreur est signalée, sans empêcher le reste de tourner.
 */
void GoalStrat::loadRecalageZones()
{
    const auto l_path = this->declare_parameter("recalage.file", std::string(""));
    std::ifstream l_file(l_path);
    if (!l_file.good())
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Recalage : fichier de zones '%s' introuvable, recalage indisponible",
                     l_path.c_str());
        return;
    }
    std::stringstream l_content;
    l_content << l_file.rdbuf();

    recalage::Pose2D l_start;
    l_start.x = m_start_pose_x;
    l_start.y = m_start_pose_y;
    l_start.theta = m_start_pose_theta;
    m_recalage_config = recalage::load(l_content.str(), m_is_blue, l_start);

    for (const auto& l_error : m_recalage_config.errors)
    {
        RCLCPP_ERROR(this->get_logger(), "Recalage : %s", l_error.c_str());
    }
    for (const auto& l_zone : m_recalage_config.zones)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Recalage : zone '%s' (%s), %zu étapes%s",
                    l_zone.name.c_str(),
                    l_zone.label.c_str(),
                    l_zone.steps.size(),
                    l_zone.name == m_recalage_config.default_zone ? ", par défaut" : "");
    }
}

/**
 * @brief Dépose une demande d'action avant match. Acceptée avec ou sans tirette, mais refusée
 * pendant l'initialisation (pinces en mouvement), en match, ou si une action est en cours.
 * @param a_zone zone de recalage ; vide = zone par défaut
 */
bool GoalStrat::requestPreMatchAction(PreMatchRequest a_request,
                                      const std::string& a_zone,
                                      std::string& a_message)
{
    const std::string l_zone = a_zone.empty() ? m_recalage_config.default_zone : a_zone;
    bool l_accepted = false;

    const State l_state = m_state;

    if (m_tirette_filter.started() || m_chrono_running
        || (l_state != State::WAIT_TIRETTE_IN && l_state != State::INIT
            && l_state != State::WAIT_TIRETTE))
    {
        a_message = "refusé : uniquement avant le match";
    }
    else if (l_state == State::INIT)
    {
        a_message = "refusé : initialisation en cours";
    }
    else if (m_pre_match_busy || m_pre_match_request != PRE_MATCH_NONE)
    {
        a_message = "refusé : une action avant match est déjà en cours";
    }
    else if (a_request == PRE_MATCH_RECALAGE && m_recalage.use_lidar_loc)
    {
        a_message = "refusé : recalage désactivé avec use_lidar_loc (AMCL publie map->odom)";
    }
    else if (a_request == PRE_MATCH_RECALAGE && l_zone.empty())
    {
        a_message = "refusé : aucune zone par défaut utilisable (voir les logs)";
    }
    else if (a_request == PRE_MATCH_RECALAGE && m_recalage_config.find(l_zone) == nullptr)
    {
        a_message = "refusé : zone '" + l_zone + "' inconnue ou désactivée (voir les logs)";
    }
    else
    {
        m_pre_match_zone = l_zone;
        m_pre_match_request = a_request;
        l_accepted = true;
        a_message = "accepté, résultat sur pre_match_state";
    }
    RCLCPP_INFO(this->get_logger(), "%s", a_message.c_str());
    return l_accepted;
}

/**
 * @brief Exécute l'action avant match demandée, s'il y en a une. Appelé depuis loop().
 * @return true si une action a été exécutée
 */
bool GoalStrat::handlePreMatchRequest()
{
    const int l_request = m_pre_match_request.load();
    if (l_request == PRE_MATCH_NONE)
    {
        return false;
    }

    if (m_tirette_filter.started() || m_chrono_running)
    {
        // Le match a démarré entre-temps : ne surtout pas écraser la pose du robot
        m_pre_match_request = PRE_MATCH_NONE;
        return false;
    }

    m_pre_match_busy = true;
    m_pre_match_error.clear();
    m_pre_match_step = l_request == PRE_MATCH_RECALAGE ? "préparation" : "retour au départ";
    publishPreMatchState();
    const recalage::Zone* l_zone = m_recalage_config.find(m_pre_match_zone);
    m_pre_match_request = PRE_MATCH_NONE;

    // Avant d'activer les moteurs : la bonne pose, puis une cible sur place. Sinon Core
    // partirait vers son ancienne cible.
    bool l_ok = l_request != PRE_MATCH_RECALAGE || (l_zone != nullptr && setZonePose(*l_zone));
    if (l_ok)
    {
        stopPreMatchMotion();
        l_ok = sleepUnlessMatch(0.2) && setPreMatchMotion(true);
    }
    if (l_ok)
    {
        l_ok = l_request == PRE_MATCH_RECALAGE ? runZone(*l_zone) : goToStartPose();
        // Quel que soit le résultat : ne rien laisser traîner pour le match (vitesse forcée,
        // sens de marche imposé par override_gear...)
        stopPreMatchMotion();
        setPreMatchMotion(false);
    }
    RCLCPP_INFO(this->get_logger(),
                "Action avant match %s : %s",
                l_request == PRE_MATCH_RECALAGE ? "recalage" : "go_to_start",
                l_ok ? "réussie" : "ÉCHEC");

    if (!m_tirette_filter.started())
    {
        const std::string l_error = l_ok                        ? ""
                                    : m_pre_match_error.empty() ? "cause inconnue"
                                                                : m_pre_match_error;
        // Un recalage réussi efface tous les échecs ; go_to_start seulement les siens (il ne
        // corrige pas la pose)
        m_go_to_start_failure = l_request == PRE_MATCH_GO_TO_START ? l_error : "";
        if (l_request == PRE_MATCH_RECALAGE)
        {
            m_recalage_failure = l_error;
            m_recalage_done = l_ok ? l_zone->label : "";
        }
    }
    m_pre_match_busy = false;
    publishPreMatchState();
    return true;
}

/**
 * @brief Note la cause d'échec de l'action en cours (la première : c'est la vraie cause).
 * @return false, pour écrire : return preMatchFail("...");
 */
bool GoalStrat::preMatchFail(const std::string& a_reason)
{
    if (m_tirette_filter.started())
    {
        RCLCPP_INFO(this->get_logger(), "Action interrompue : le match démarre");
        return false;
    }
    RCLCPP_ERROR(this->get_logger(), "Recalage: %s", a_reason.c_str());
    if (m_pre_match_error.empty())
    {
        m_pre_match_error = a_reason;
    }
    return false;
}

/**
 * @brief État affiché par l'IHM : "état" ou "état: détail".
 * wait_tirette_in | init | ready | recalage | recalage_failed | match | end
 */
std::string GoalStrat::preMatchState() const
{
    const State l_state = m_state;
    if (l_state == State::EXIT)
    {
        return "end";
    }
    // Dès le retrait de la tirette, même si l'initialisation n'est pas finie. Et si goal_strat a
    // redémarré en plein match (il attend alors la tirette, mais le chrono de Core tourne)
    if (l_state == State::RUN || l_state == State::ALL_DONE || m_tirette_filter.started()
        || m_chrono_running)
    {
        return "match";
    }
    if (l_state == State::INIT)
    {
        return "init";
    }
    if (m_pre_match_busy)
    {
        return "recalage: " + m_pre_match_step;
    }
    if (!m_recalage_failure.empty() || !m_go_to_start_failure.empty())
    {
        return "recalage_failed: "
               + (m_recalage_failure.empty() ? m_go_to_start_failure : m_recalage_failure);
    }
    const std::string l_text = l_state == State::WAIT_TIRETTE_IN ? "wait_tirette_in" : "ready";
    return m_recalage_done.empty() ? l_text : l_text + ": recalé (" + m_recalage_done + ")";
}

/**
 * @brief Publie l'état s'il a changé. Appelé depuis loop() (et les actions, qui la bloquent), et
 * au départ du match depuis updateTirette() : preMatchState() ne lit alors aucune des chaînes
 * réservées au thread de loop().
 */
void GoalStrat::publishPreMatchState()
{
    std::lock_guard<std::mutex> l_lock(m_pre_match_state_mutex);
    const std::string l_state = preMatchState();
    if (l_state == m_pre_match_state_sent)
    {
        return;
    }
    m_pre_match_state_sent = l_state;
    std_msgs::msg::String l_msg;
    l_msg.data = l_state;
    m_pre_match_state_pub->publish(l_msg);
}

/**
 * @brief Republie le dernier état (1 Hz, thread publishAll) : l'IHM sait ainsi que le robot est
 * vivant, même si loop() est bloquée par une action.
 */
void GoalStrat::repeatPreMatchState()
{
    std::lock_guard<std::mutex> l_lock(m_pre_match_state_mutex);
    if (m_pre_match_state_sent.empty())
    {
        return; // constructeur pas fini : l'éditeur n'existe pas encore
    }
    std_msgs::msg::String l_msg;
    l_msg.data = m_pre_match_state_sent;
    m_pre_match_state_pub->publish(l_msg);
}

void GoalStrat::publishRecalageZones()
{
    krabi_msgs::msg::RecalageZones l_msg;
    l_msg.default_zone = m_recalage_config.default_zone;
    l_msg.errors = m_recalage_config.errors;
    for (const auto& l_zone : m_recalage_config.zones)
    {
        krabi_msgs::msg::RecalageZone l_zone_msg;
        l_zone_msg.name = l_zone.name;
        l_zone_msg.label = l_zone.label;
        l_zone_msg.x = l_zone.pose.x;
        l_zone_msg.y = l_zone.pose.y;
        l_zone_msg.theta = l_zone.pose.theta;
        l_msg.zones.push_back(l_zone_msg);
    }
    m_recalage_zones_pub->publish(l_msg);
}

/**
 * @brief Active/désactive les moteurs avant match côté Core (service enable_pre_match_motion).
 */
bool GoalStrat::setPreMatchMotion(bool a_enable)
{
    const auto l_timeout = std::chrono::duration<double>(m_recalage.service_timeout);
    if (!m_pre_match_motion_client->wait_for_service(l_timeout))
    {
        return preMatchFail("main_strat ne répond pas (enable_pre_match_motion)");
    }
    auto l_req = std::make_shared<std_srvs::srv::SetBool::Request>();
    l_req->data = a_enable;
    auto l_future = m_pre_match_motion_client->async_send_request(l_req);
    if (l_future.wait_for(l_timeout) != std::future_status::ready)
    {
        m_pre_match_motion_client->remove_pending_request(l_future);
        return preMatchFail("main_strat ne répond pas (enable_pre_match_motion)");
    }
    return l_future.get()->success;
}

/**
 * @brief Arrête le robot sur place et rend la main à la stratégie (sens de marche libre).
 */
void GoalStrat::stopPreMatchMotion()
{
    updateCurrentPose();
    m_goal_pose = m_current_pose;
    override_gear = krabi_msgs::msg::StratMovement::FORWARD_OR_REVERSE;
    startAngular(); // quitte RECALAGE_BORDURE
    publishStratMovement();
}

/**
 * @brief Attend, en continuant de publier l'ordre courant (Core coupe les moteurs avant match
 * s'il ne reçoit plus rien).
 * @return false si la tirette a été retirée entre-temps (le match démarre)
 */
bool GoalStrat::sleepUnlessMatch(double a_seconds)
{
    const double l_end = TiretteFilter::steadyNow() + a_seconds;
    while (TiretteFilter::steadyNow() < l_end)
    {
        if (m_tirette_filter.started())
        {
            return false;
        }
        publishStratMovement();
        usleep(50000);
    }
    return !m_tirette_filter.started();
}

/**
 * @brief Date de la dernière mesure d'odométrie (odom -> base_link).
 */
bool GoalStrat::getOdomStamp(rclcpp::Time& a_stamp)
{
    try
    {
        const auto l_tf = m_tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
        a_stamp = rclcpp::Time(l_tf.header.stamp, this->now().get_clock_type());
        return true;
    }
    catch (const tf2::TransformException&)
    {
        return false;
    }
}

/**
 * @brief Va en (a_x, a_y) en ligne droite, en avant ou en arrière.
 */
bool GoalStrat::goToPosition(double a_x, double a_y)
{
    const Position l_target{ Distance(a_x), Distance(a_y) };
    updateCurrentPose();
    m_goal_pose.setPosition(l_target);
    m_goal_pose.setAngle(m_current_pose.getAngle());
    override_gear = krabi_msgs::msg::StratMovement::FORWARD_OR_REVERSE;
    startAngular(); // orient = GO_TO_GOALPOSE_POSITION
    m_strat_mvnt.max_speed_at_arrival = 0.f;

    const double l_distance = (m_current_pose.getPosition() - l_target).getNorme();
    const auto l_deadline = this->now() + rclcpp::Duration::from_seconds(10. + 10. * l_distance);
    while (this->now() < l_deadline && !m_tirette_filter.started())
    {
        publishStratMovement();
        usleep(50000);
        updateCurrentPose();
        if ((m_current_pose.getPosition() - l_target).getNorme() < REACH_DIST)
        {
            return true;
        }
    }
    char l_reason[96];
    snprintf(l_reason, sizeof(l_reason), "position (%.2f, %.2f) non atteinte (obstacle ?)", a_x, a_y);
    return preMatchFail(l_reason);
}

/**
 * @brief Va à la pose de départ (start_pose.*), puis s'oriente.
 */
bool GoalStrat::goToStartPose()
{
    RCLCPP_INFO(this->get_logger(),
                "Retour au départ : (%.3f, %.3f, %.3f)",
                m_start_pose_x,
                m_start_pose_y,
                m_start_pose_theta);
    if (!goToPosition(m_start_pose_x, m_start_pose_y)
        || !alignInPlace(Angle(m_start_pose_theta),
                         rclcpp::Duration::from_seconds(m_recalage.align_timeout)))
    {
        return false;
    }
    updateCurrentPose();
    RCLCPP_INFO_STREAM(this->get_logger(), "Au départ, pose : " << m_current_pose);
    return true;
}

void GoalStrat::updateMotorsCurrent(krabi_msgs::msg::MotorsCurrent::SharedPtr a_msg)
{
    // != 0 : la STM32 a détecté un calage et bloque la roue pendant ce nombre de ms
    m_left_wheel_stalled = a_msg->left_wheel_unstalled_in_ms != 0;
    m_right_wheel_stalled = a_msg->right_wheel_unstalled_in_ms != 0;
}

/**
 * @brief Tourne sur place jusqu'au cap a_angle (cap de base_link, marche avant).
 * Contrairement à alignWithAngle(), la position cible est la position courante : le robot ne
 * se déplace pas vers l'étape en cours.
 */
bool GoalStrat::alignInPlace(Angle a_angle, rclcpp::Duration a_timeout)
{
    updateCurrentPose();
    m_goal_pose.setPosition(m_current_pose.getPosition());
    m_goal_pose.setAngle(a_angle);

    // FORWARD : en mode orientation, Core vise goal_angle (+π si REVERSE). On veut le cap brut.
    override_gear = krabi_msgs::msg::StratMovement::FORWARD;
    startAngular();
    m_strat_mvnt.orient = krabi_msgs::msg::StratMovement::ORIENT_TOWARD_GOALPOSE_ORIENTATION;

    const auto l_deadline = this->now() + a_timeout;
    while (this->now() < l_deadline && !m_tirette_filter.started())
    {
        publishStratMovement();
        updateCurrentPose();
        if (isAlignedWithAngle(a_angle))
        {
            return true;
        }
        usleep(50000);
    }
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "alignInPlace: non aligné, cap " << m_current_pose.getAngle() << " au lieu de "
                                                     << a_angle);
    return preMatchFail("rotation impossible (robot bloqué ?)");
}

/**
 * @brief Attend le contact avec le mur pendant la poussée.
 * Contact = déplacement < still_dist pendant still_time, OU les deux roues calées (STM32),
 * et seulement si le robot a d'abord avancé d'au moins min_push_dist : un robot qui ne démarre
 * pas (PWM trop faible, roues bloquées) est immobile sans être contre le mur.
 * L'immobilité est mesurée sur les dates des mesures d'odométrie, pas sur l'horloge : si
 * l'odométrie s'arrête (CAN coupé...), la pose se fige mais aucun contact n'est déclaré.
 * @return false si timeout, odométrie perdue ou tirette retirée
 */
bool GoalStrat::waitForWallContact(rclcpp::Duration a_timeout)
{
    const auto l_deadline = this->now() + a_timeout;

    updateCurrentPose();
    const Position l_start_position = m_current_pose.getPosition();

    // On ignore le tout début : le robot doit d'abord se mettre en mouvement
    if (!sleepUnlessMatch(m_recalage.still_time))
    {
        return false;
    }

    updateCurrentPose();
    Position l_ref_position = m_current_pose.getPosition();
    rclcpp::Time l_odom_stamp = this->now();
    getOdomStamp(l_odom_stamp);
    rclcpp::Time l_ref_time = l_odom_stamp;

    bool l_has_moved = false;
    while (this->now() < l_deadline)
    {
        if (m_tirette_filter.started())
        {
            return false;
        }
        publishStratMovement();
        usleep(50000);
        updateCurrentPose();

        if (!getOdomStamp(l_odom_stamp) || (this->now() - l_odom_stamp).seconds() > 0.5)
        {
            return preMatchFail("odométrie perdue");
        }

        l_has_moved = l_has_moved
                      || (m_current_pose.getPosition() - l_start_position).getNorme()
                           >= m_recalage.min_push_dist;

        if (l_has_moved && m_left_wheel_stalled && m_right_wheel_stalled)
        {
            RCLCPP_INFO(this->get_logger(), "Recalage: contact détecté (roues calées)");
            return true;
        }

        const double l_moved = (m_current_pose.getPosition() - l_ref_position).getNorme();
        if (l_moved > m_recalage.still_dist)
        {
            // Le robot bouge encore : on repart de cette position
            l_ref_position = m_current_pose.getPosition();
            l_ref_time = l_odom_stamp;
        }
        else if (l_has_moved && (l_odom_stamp - l_ref_time).seconds() >= m_recalage.still_time)
        {
            RCLCPP_INFO(this->get_logger(), "Recalage: contact détecté (robot immobile)");
            return true;
        }
    }
    return preMatchFail(l_has_moved ? "pas de contact avec le mur"
                                    : "le robot n'a pas bougé en poussant (déjà contre le mur ? "
                                      "roues bloquées ? PWM trop faible ?)");
}

/**
 * @brief Appelle correct_pose (asynchrone + attente avec timeout).
 */
bool GoalStrat::callCorrectPose(krabi_msgs::srv::CorrectPose::Request::SharedPtr a_request)
{
    if (!m_correct_pose_client->wait_for_service(std::chrono::duration<double>(
          m_recalage.service_timeout)))
    {
        return preMatchFail("service correct_pose indisponible");
    }

    auto l_future = m_correct_pose_client->async_send_request(a_request);
    // Pas de spin_until_future_complete ici (on est dans un callback) : l'executor
    // multithread traite la réponse dans m_recalage_cb_group.
    if (l_future.wait_for(std::chrono::duration<double>(m_recalage.service_timeout))
        != std::future_status::ready)
    {
        m_correct_pose_client->remove_pending_request(l_future);
        return preMatchFail("pas de réponse de correct_pose");
    }

    auto l_res = l_future.get();
    if (!l_res->success)
    {
        return preMatchFail("correct_pose a échoué : " + l_res->message);
    }
    return true;
}

/**
 * @brief Recalage sur un mur.
 * @param a_wall le mur visé
 * @param a_clearance distance dont on s'écarte du mur ensuite (m)
 * @param a_reverse true = contact par l'arrière (le robot recule vers le mur)
 */
bool GoalStrat::recalageBordure(Wall a_wall, double a_clearance, bool a_reverse)
{
    // Cap au contact : dos au mur => on regarde vers l'intérieur de la table
    const double l_inward = inwardHeading(a_wall);
    const Angle l_heading = AngleTools::wrapAngle(Angle(a_reverse ? l_inward : l_inward + M_PI));
    const double l_offset
      = a_reverse ? m_recalage.contact_offset_back : m_recalage.contact_offset_front;
    // base_link est à l_offset du mur, du côté intérieur
    const double l_inward_sign = (a_wall == Wall::X_MIN || a_wall == Wall::Y_MIN) ? 1.0 : -1.0;
    const Distance l_coord = Distance(wallCoordinate(a_wall) + l_inward_sign * l_offset);

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Recalage sur " << wallName(a_wall) << (a_reverse ? " (arrière)" : " (avant)")
                                       << " : cap visé " << l_heading << ", "
                                       << (isXWall(a_wall) ? "x" : "y") << " = " << l_coord);

    // 1. Alignement perpendiculaire au mur
    if (!alignInPlace(l_heading, rclcpp::Duration::from_seconds(m_recalage.align_timeout)))
    {
        return false;
    }

    // 2. Poussée (Core : PWM fixe + courant limité ; en simu, petite vitesse cmd_vel)
    m_left_wheel_stalled = false;
    m_right_wheel_stalled = false;
    recalage_bordure();
    override_gear = a_reverse ? krabi_msgs::msg::StratMovement::REVERSE
                              : krabi_msgs::msg::StratMovement::FORWARD;
    publishStratMovement();

    // 3. Détection du contact
    const bool l_contact
      = waitForWallContact(rclcpp::Duration::from_seconds(m_recalage.timeout));

    if (l_contact)
    {
        // 4. On laisse le robot se plaquer (on pousse toujours)
        sleepUnlessMatch(m_recalage.settle_time);
    }

    // Fin de poussée AVANT la correction : si on corrigeait en poussant encore, l'odom
    // continuerait de bouger (patinage en simu) entre la correction et l'arrêt effectif.
    // Cible = pose courante, rotation bloquée => le robot s'arrête contre le mur.
    updateCurrentPose();
    m_goal_pose = m_current_pose;
    startAngular();
    m_strat_mvnt.orient = krabi_msgs::msg::StratMovement::STOP_ANGULAR;
    override_gear = krabi_msgs::msg::StratMovement::FORWARD_OR_REVERSE;
    publishStratMovement();

    bool l_corrected = false;
    // Attente que Core (boucle 100 ms) ait appliqué l'arrêt et que l'odom soit stable
    if (l_contact && sleepUnlessMatch(std::max(m_recalage.settle_time, 0.3)))
    {
        // Garde-fou : une grosse correction veut dire que ce n'est pas le bon mur, qu'un objet
        // a bloqué le robot, ou qu'il n'était pas là où on le croyait.
        updateCurrentPose();
        const double l_coord_error
          = std::abs(static_cast<double>(isXWall(a_wall) ? m_current_pose.getPosition().getX()
                                                         : m_current_pose.getPosition().getY())
                     - static_cast<double>(l_coord));
        const double l_angle_error = std::abs(
          static_cast<double>(AngleTools::diffAngle(l_heading, m_current_pose.getAngle())));
        if (l_coord_error > m_recalage.max_correction_dist
            || l_angle_error > m_recalage.max_correction_angle)
        {
            char l_reason[96];
            snprintf(l_reason,
                     sizeof(l_reason),
                     "correction refusée, trop grande (%.0f mm, %.0f deg)",
                     l_coord_error * 1000.,
                     l_angle_error * 180. / M_PI);
            return preMatchFail(l_reason);
        }

        // 5. Correction de map -> odom
        auto l_request = std::make_shared<krabi_msgs::srv::CorrectPose::Request>();
        l_request->theta = static_cast<double>(l_heading);
        l_request->set_theta = true;
        (isXWall(a_wall) ? l_request->x : l_request->y) = static_cast<double>(l_coord);
        (isXWall(a_wall) ? l_request->set_x : l_request->set_y) = true;
        l_corrected = callCorrectPose(l_request);

        // La pose map a sauté : nouvelle pose courante comme cible, pour que Core ne reparte
        // pas vers la cible d'avant correction.
        usleep(100000); // laisse le temps au nouveau map->odom d'arriver dans le buffer TF
        updateCurrentPose();
        m_goal_pose = m_current_pose;
        publishStratMovement();
    }

    if (!l_corrected)
    {
        return false;
    }

    // 6. Dégagement : on s'éloigne du mur (sens opposé à la poussée), sans tourner.
    // (pas avanceDroit() : goToDroit() force override_gear = REVERSE)
    updateCurrentPose();
    Position l_target = goal_logic::straightLineTarget(m_current_pose.getPosition(),
                                                       m_current_pose.getAngle(),
                                                       Distance(a_clearance),
                                                       !a_reverse /* sensRecule */);
    m_goal_pose.setPosition(l_target);
    m_goal_pose.setAngle(m_current_pose.getAngle());
    override_gear = a_reverse ? krabi_msgs::msg::StratMovement::FORWARD
                              : krabi_msgs::msg::StratMovement::REVERSE;
    m_strat_mvnt.orient = krabi_msgs::msg::StratMovement::GO_TO_GOALPOSE_POSITION;

    const auto l_deadline = this->now() + rclcpp::Duration::from_seconds(3. + 10. * a_clearance);
    bool l_clear = false;
    while (!l_clear && this->now() < l_deadline && !m_tirette_filter.started())
    {
        publishStratMovement();
        usleep(50000);
        updateCurrentPose();
        l_clear = (m_current_pose.getPosition() - l_target).getNorme() < REACH_DIST;
    }
    override_gear = krabi_msgs::msg::StratMovement::FORWARD_OR_REVERSE;
    updateCurrentPose();
    m_goal_pose = m_current_pose;
    publishStratMovement();

    if (!l_clear || m_tirette_filter.started())
    {
        // La suite de la séquence suppose le robot écarté du mur (place pour pivoter)
        return preMatchFail("dégagement du mur impossible (obstacle ?)");
    }

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Recalage " << wallName(a_wall) << " terminé, pose: " << m_current_pose);
    return true;
}

/**
 * @brief Le robot est posé dans la zone : sa pose devient celle de la zone (approximative,
 * la séquence la corrige ensuite contre les murs).
 */
bool GoalStrat::setZonePose(const recalage::Zone& a_zone)
{
    auto l_request = std::make_shared<krabi_msgs::srv::CorrectPose::Request>();
    l_request->x = a_zone.pose.x;
    l_request->y = a_zone.pose.y;
    l_request->theta = a_zone.pose.theta;
    l_request->set_x = l_request->set_y = l_request->set_theta = true;
    if (!callCorrectPose(l_request))
    {
        return false;
    }
    usleep(100000); // laisse le temps au nouveau map->odom d'arriver dans le buffer TF
    return true;
}

/**
 * @brief Exécute la séquence d'une zone. S'arrête à la première étape en échec.
 */
bool GoalStrat::runZone(const recalage::Zone& a_zone)
{
    RCLCPP_INFO(this->get_logger(),
                "Recalage zone '%s' (%s)",
                a_zone.name.c_str(),
                m_is_blue ? "bleu" : "jaune");
    for (size_t i = 0; i < a_zone.steps.size(); ++i)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Étape %zu/%zu : %s",
                    i + 1,
                    a_zone.steps.size(),
                    a_zone.steps[i].text.c_str());
        m_pre_match_step = std::to_string(i + 1) + "/" + std::to_string(a_zone.steps.size()) + " "
                           + a_zone.steps[i].text;
        publishPreMatchState();
        if (!runStep(a_zone.steps[i]) || m_tirette_filter.started())
        {
            m_pre_match_error = "étape " + m_pre_match_step + " : " + m_pre_match_error;
            RCLCPP_ERROR(this->get_logger(),
                         "Recalage zone '%s' : arrêt à l'étape %zu (%s)",
                         a_zone.name.c_str(),
                         i + 1,
                         a_zone.steps[i].text.c_str());
            return false;
        }
    }
    return true;
}

bool GoalStrat::runStep(const recalage::Step& a_step)
{
    switch (a_step.type)
    {
    case recalage::Step::Type::WALL:
        return recalageBordure(a_step.wall, a_step.clearance);
    case recalage::Step::Type::GOTO:
        return goToPosition(a_step.x, a_step.y);
    case recalage::Step::Type::ROTATE:
        return alignInPlace(Angle(a_step.theta),
                            rclcpp::Duration::from_seconds(m_recalage.align_timeout));
    case recalage::Step::Type::START:
        return goToStartPose();
    }
    return false;
}
