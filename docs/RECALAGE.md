# Recalage sur bordures

Avant le match, le robot peut corriger sa position en venant toucher les bordures de la table, dos
au mur. Sans recalage, il se croit à sa pose de départ. Avec un recalage : on le pose dans une
**zone**, il exécute la **séquence** de cette zone, revient à sa pose de départ et attend la tirette.

- [1. Au match, sur l'IHM](#1-au-match-sur-lihm)
- [2. En simulation](#2-en-simulation)
- [3. Définir une zone, pas à pas](#3-définir-une-zone-pas-à-pas)
- [4. Écrire une séquence](#4-écrire-une-séquence)
- [5. Changer la pose de départ](#5-changer-la-pose-de-départ)
- [6. Paramètres](#6-paramètres)
- [7. À mesurer sur le vrai robot](#7-à-mesurer-sur-le-vrai-robot)
- [8. Dépannage](#8-dépannage)
- [9. Comment ça marche](#9-comment-ça-marche)

---

## 1. Au match, sur l'IHM

Le robot démarre **sans tirette** : la pile ROS est lancée une fois, au démarrage du Raspberry.
Insérer la tirette ne relance plus ROS.

### Déroulé

1. Allumer le robot. Le bandeau passe à **TIRETTE EN ATTENTE** (jaune, clignotant). Les menus
   restent utilisables.
2. Vérifier la couleur sur la page **Prépa** (« COULEUR LUE PAR LE ROBOT »). Si elle est fausse :
   changer l'interrupteur, puis **RESTART COLOR** (la couleur n'est lue qu'au lancement de ROS).
3. *(Facultatif)* Recalage, onglet **Recalage** :
   1. choisir la zone, sur la carte ou dans la liste ;
   2. poser le robot dans le cercle vert, flèche = avant du robot. Quelques centimètres et
      quelques degrés d'écart suffisent : la séquence les corrige ;
   3. **LANCER LE RECALAGE**. Le robot bouge tout de suite, avec ou sans tirette.
   4. À la fin, le bandeau indique « recalé (nom de la zone) ».
4. Insérer la tirette : **INITIALISATION…** (les pinces bougent), puis **ROBOT PRÊT** (vert).
5. Retirer la tirette : le match démarre.

Entre deux matchs : **RESTART COLOR** (pile neuve, chrono et recalage remis à zéro).

### Le bandeau

| Bandeau | Signification |
|---|---|
| ROBOT DÉMARRE… (gris) | Pas de nouvelles de `goal_strat` depuis 3 s : ROS démarre, ou est planté |
| TIRETTE EN ATTENTE (jaune clignotant) | Prêt à recevoir la tirette ; recalage possible |
| INITIALISATION… (bleu) | Tirette insérée, initialisation des pinces. **Ne pas retirer la tirette** : le match partirait à la fin de l'initialisation, chrono déjà lancé |
| RECALAGE EN COURS — étape (bleu) | Séquence en cours, étape affichée (ex. `2/3 wall Y_MIN 0.10`) |
| ROBOT PRÊT (vert) | Tirette insérée, initialisation faite, aucun diagnostic ROS en erreur |
| PAS PRÊT : cause (rouge) | Recalage échoué (cause complète sur l'onglet Recalage), ou diagnostic ROS en erreur. Le match reste possible |
| MATCH EN COURS | Tirette retirée. RESTART COLOR et recalage sont bloqués |
| MATCH TERMINÉ — RESTART COLOR | Relancer la pile avant le match suivant |

« recalé (zone) » s'ajoute après un recalage réussi. Un recalage **raté** reste affiché, même après
l'insertion de la tirette, jusqu'au prochain recalage réussi : le robot n'est peut-être plus à sa
pose de départ.

### RESTART COLOR

Même bouton qu'avant (page Prépa, en bas à droite), avec une confirmation. Il fait
`sudo systemctl restart krabi_color` : couleur relue, pose de départ, recalage et chrono remis à
zéro. Il est **bloqué pendant un match**, sauf si le robot ne répond plus.

---

## 2. En simulation

La tirette simulée démarre **absente**, comme sur le robot.

```bash
krabiRunAndSimu gz_world:=table2026Full.world   # alias du README de krabi ; avec les caisses
# Recalage, zone par défaut :
ros2 service call /krabi_ns/start_recalage std_srvs/srv/Trigger
# Recalage, zone choisie :
ros2 service call /krabi_ns/start_recalage_zone krabi_msgs/srv/StartRecalage "{zone: 'public'}"
# Revenir à la pose de départ (sans recalage) :
ros2 service call /krabi_ns/go_to_start std_srvs/srv/Trigger
# Tirette :
ros2 service call /krabi_ns/insert_tirette std_srvs/srv/Trigger
ros2 service call /krabi_ns/start_match std_srvs/srv/Trigger
# État et zones (ce que lit l'IHM) :
ros2 topic echo /krabi_ns/pre_match_state
ros2 topic echo --qos-durability transient_local /krabi_ns/recalage_zones
```

- `auto_start:=True` : ancien comportement, tirette insérée à 9 s et retirée à 14 s.
- `recalage_file:=/chemin/vers/zones.yaml` : essayer un autre fichier de zones sans toucher au
  fichier livré.
- L'IHM marche aussi face à la simu : `python3 krabi_gui.py --simu` dans `krabi_gui/krabi_gui`.

---

## 3. Définir une zone, pas à pas

Les zones sont dans [`config/recalage_2026.yaml`](../config/recalage_2026.yaml), lu au
démarrage de `goal_strat`. **Pas besoin de recompiler** si le workspace est compilé avec
`--symlink-install` ; sinon `colcon build --packages-select goal_strategy` (quelques secondes).
L'IHM reçoit les zones du robot : rien à changer côté IHM.

```yaml
default_zone: depart          # zone de « start_recalage » et du bouton de l'ancienne IHM

zones:
  public:                     # identifiant (lettres, chiffres, _)
    label: Côté public        # texte affiché sur l'IHM
    pose: -0.9 0.75 -90       # où on pose le robot, côté BLEU : x (m), y (m), cap (degrés)
    sequence:                 # côté BLEU
      - wall Y_MAX 0.10
      - goto -1.0 0.2
      - wall X_MIN 0.30
      - goto -1.05 -0.25
      - goto -1.2 -0.55
      - start
```

1. **Choisir l'endroit** : contre deux bordures perpendiculaires si possible, loin des caisses et
   du grenier. Le robot doit pouvoir y être posé à la main en quelques secondes.
2. **Écrire la pose côté bleu.** Repère `map` : origine au centre de la table, x de −1,5 (bord
   bleu) à +1,5, y de −1 (grenier) à +1 (public). Cap en degrés : 0 = vers +x, 90 = vers +y.
   `pose: launch` = la pose de départ du robot (voir §5).
3. **Écrire la séquence** (§4) : en général un ou deux murs, puis `start` pour revenir au départ.
4. **Le jaune est automatique** : symétrie x → −x, X_MIN ↔ X_MAX, cap θ → 180° − θ. Si la
   symétrie ne convient pas, écrire `pose_yellow:` et/ou `sequence_yellow:`, en coordonnées
   jaunes (voir la zone `depart`).
5. **Lancer `goal_strat` et lire ses logs** : une ligne par zone chargée, ou la raison pour
   laquelle elle est désactivée. Une erreur désactive **cette zone seulement**, le reste marche.
   L'IHM affiche aussi les erreurs, sous le bouton de l'onglet Recalage.
6. **Essayer en simu, dans les deux couleurs, avec les caisses** (§4, « Vérifier »).

Le fichier est lu strictement : champ inconnu, étape inconnue, coordonnée hors de la table
(|x| > 1,40 ou |y| > 0,90), dégagement hors de 0–1 m, séquence sans mur, `start` ailleurs
qu'en dernier → zone désactivée. Si la zone par défaut est désactivée, `start_recalage` est refusé (pas de
repli sur une autre zone).

---

## 4. Écrire une séquence

### Étapes

| Étape | Effet |
|---|---|
| `wall <MUR> <dégagement>` | S'oriente dos au mur, recule jusqu'au contact, corrige **le cap et une coordonnée** (x pour X_MIN/X_MAX, y pour Y_MIN/Y_MAX), puis s'écarte du mur de `dégagement` mètres (arrêt à 2 cm près : 0,10 donne 8 à 10 cm) |
| `goto <x> <y>` | Va au point en ligne droite, en avant ou en arrière |
| `rotate <degrés>` | S'oriente (cap absolu), sur place |
| `start` | Va à la pose de départ, puis s'oriente. Seulement en dernier ; sans `start`, le robot reste où la séquence le laisse |

MUR : `X_MIN` (x = −1,5, côté bleu), `X_MAX` (x = +1,5), `Y_MIN` (y = −1, grenier), `Y_MAX`
(y = +1, public). Si une étape échoue, le robot s'arrête sur place et n'exécute pas la suite.

### Règles

- **Il n'y a pas d'évitement.** La séquence doit éviter seule les caisses, le grenier
  (|x| < 0,9 et y < −0,55) et le nid adverse. C'est la responsabilité de celui qui l'écrit.
- **Les obstacles fixes du lidar s'appliquent** (`lidar_strategy` : bordures, grenier, nid adverse).
  Core freine puis s'arrête si l'un d'eux est **devant** le robot (trop près, dans ±60°).
  Exemple réel : `goto -1.0 -0.45` s'arrêtait devant le coin du grenier ; la zone `public` passe
  maintenant par `-1.05 -0.25` puis `-1.2 -0.55`. Pendant la **poussée** contre le mur, les
  obstacles sont ignorés ; pendant les `goto`, une personne ou un objet près du robot l'arrête
  (sans danger : l'étape échoue).
- **Laisser de la place pour pivoter.** Le robot pivote avant chaque mur et en fin de `goto`. Sans
  le billig, il lui faut ~17 cm autour de son centre (coin avant). **Le billig dépasse de ~20 cm à droite**
  (§7) : à vérifier avant de recaler près des murs côté billig.
- **Deux murs perpendiculaires** pour corriger x, y et le cap. Un seul mur ne corrige qu'une
  coordonnée.
- **Moins de mouvements = moins de risques** : choisir l'ordre des murs selon le cap de départ
  (c'est pourquoi `depart` a une `sequence_yellow`).
- **Dégagement** : 0,10 m suffit pour pivoter loin d'un coin ; plus si une étape suivante passe
  près du mur.

### Vérifier en simulation

```bash
krabiRunAndSimu gz_world:=table2026Full.world recalage_file:=$PWD/mes_zones.yaml   # isBlue:=False pour le jaune
# Poser le robot (gz) dans la zone, avec quelques cm d'écart, puis :
ros2 service call /krabi_ns/start_recalage_zone krabi_msgs/srv/StartRecalage "{zone: 'ma_zone'}"
ros2 topic echo /krabi_ns/pre_match_state
```

À regarder : bandeau / `pre_match_state` qui finit en `recalé`, aucune caisse déplacée, robot
revenu au départ. Tester **bleu et jaune**. Voir §8 si le lidar simulé arrête le robot sans raison.

---

## 5. Changer la pose de départ

La pose de départ est celle où le robot est posé pour le match, et celle où `start` le ramène.

- **Vrai robot** : `start_ros_color.py` (dépôt `krabi-startscripts`), variables `xPos`, `yPos`,
  `zRot` pour chaque couleur. Elle est passée au launch (`xRobotPos`, `yRobotPos`,
  `zRobotOrientation`) et sert à `map_odom_publisher` et à `goal_strat` (`start_pose.*`).
- **Simu** : mêmes arguments sur la ligne de commande (`xRobotPos:=…`).
- La zone `depart` utilise `pose: launch` : elle suit automatiquement la pose de départ.
- Changer de couleur ou de pose demande de relancer ROS (RESTART COLOR).

### Poses actuelles et cales

`start_ros_color.py` donne :

| Couleur | x | y | cap | Mur en y |
|---|---|---|---|---|
| Bleu | −1,159 | −0,692 | −90° (avant vers le grenier) | avant à 188,5 mm (« cale arrière bleu ») |
| Jaune | +1,159 | −0,692 | +90° (arrière vers le grenier) | arrière à 219 mm (« cale arrière jaune ») |

Ces poses ont été calculées avec **89 mm** entre l'axe des roues et l'arrière (commentaire du
script). La mesure actuelle est **104 mm** :

- **Jaune** (arrière vers le mur) : y = −1 + 0,219 + 0,104 = **−0,677** au lieu de −0,692 (15 mm).
- **Bleu** (avant vers le mur) : y = −1 + 0,1885 + 0,1195 = −0,692 ; dépend de la distance avant
  (119,5 mm), **pas remesurée**.
- **x** (±1,159, soit 341 mm du bord) : ne correspond pas à « cale côté 144 mm + 115 mm de
  demi-largeur » (259 mm). La cale côté est sans doute mesurée depuis autre chose que la bordure.
  **À vérifier.**

Ces valeurs **n'ont pas été changées** dans `start_ros_color.py` : à mesurer d'abord (§7). Un
recalage corrige de toute façon ces écarts.

---

## 6. Paramètres

`goal_strat` (namespace `krabi_ns`), réglables dans
[`launch/goal_strat_launch.py`](../launch/goal_strat_launch.py) :

| Paramètre | Défaut | Rôle |
|---|---|---|
| `recalage.file` | `config/recalage_2026.yaml` | Fichier des zones (argument de launch `recalage_file`) |
| `recalage.use_lidar_loc` | `false` | Vrai : AMCL publie `map → odom`, recalage refusé |
| `recalage.contact_offset_back` | 0,104 m | Axe des roues → face arrière (contact) |
| `recalage.contact_offset_front` | 0,104 m | Face avant (contact avant non utilisé) |
| `recalage.still_dist` | 0,002 m | Déplacement maximal pour « immobile » |
| `recalage.still_time` | 0,3 s | Durée d'immobilité = contact |
| `recalage.min_push_dist` | 0,02 m | Déplacement minimal avant d'accepter un contact (un robot qui ne démarre pas est immobile sans être au mur) |
| `recalage.settle_time` | 0,2 s | Poussée prolongée après le contact |
| `recalage.timeout` | 5 s | Durée maximale de poussée |
| `recalage.align_timeout` | 5 s | Durée maximale d'une rotation |
| `recalage.max_correction_dist` | 0,15 m | Correction refusée au-delà (mauvais mur, objet, robot mal posé) |
| `recalage.max_correction_angle` | 0,35 rad (20°) | Idem pour le cap |
| `recalage.service_timeout` | 1 s | Attente des services `correct_pose`, `enable_pre_match_motion` |
| `start_pose.x / y / theta` | `xRobotPos`… | Pose de départ (cible de `start`) |

`main_strat` (Core) :

| Paramètre | Défaut | Rôle |
|---|---|---|
| `recalageCmdVel` | 0 (robot), 0,1 (simu) | Vitesse de poussée en simu. Sur le robot, la poussée est un PWM fixe à courant limité (`core.cpp`) |
| *(constante)* `PRE_MATCH_MOTION_TIMEOUT` | 1 s | Moteurs coupés si `goal_strat` ne commande plus rien avant le match |

---

## 7. À mesurer sur le vrai robot

Rien de tout ceci n'a été essayé sur le vrai robot. Avant le premier essai :

1. **Sens et force de poussée** : le robot recule-t-il bien vers le mur, à une vitesse sûre ?
   (Robot sur cales, puis contre un mur, main sur l'arrêt d'urgence.)
2. **Détection du contact** : l'odométrie s'arrête-t-elle nettement au contact (roues
   d'odométrie distinctes des roues motrices) ? Sinon, ajuster `still_dist` / `still_time`.
3. **Distance axe des roues → arrière** : 104 mm (`contact_offset_back`). Vérifier sur le robot
   complet, pare-chocs compris.
4. **Emprise du billig** : la TF `billig` est à 20 cm **à droite** de `base_link` (y = −0,2,
   `krabi_launch.py`). Si le billig dépasse réellement autant, il peut toucher la bordure en
   pivotant à 8–10 cm d'un mur ; côté **jaune** au départ, la droite du robot regarde la bordure
   X_MAX. Mesurer, puis augmenter le dégagement ou changer l'ordre des murs si besoin.
5. **Pinces au repos** : sans tirette, le recalage ne lance pas l'initialisation ; vérifier que
   rien ne dépasse dans cette position.
6. **Mètre ruban** : après un recalage, comparer la position réelle et la pose affichée (onglet
   Carte). En simu l'écart est de 1 à 2 mm.
7. **Poses de départ** (§5) : distance avant, cales, x.

---

## 8. Dépannage

| Symptôme | Cause probable |
|---|---|
| `refusé : zone '…' inconnue ou désactivée` | Faute dans le YAML : lire les logs de `goal_strat` au démarrage, ou l'onglet Recalage |
| `refusé : initialisation en cours` / `uniquement avant le match` / `une action avant match est déjà en cours` | Attendre la fin, ou RESTART COLOR après un match |
| `refusé : recalage désactivé avec use_lidar_loc` | Lancé avec la localisation lidar (AMCL) |
| `pas de contact avec le mur` | Le robot n'est pas allé jusqu'au mur en 5 s : trop loin, ou bloqué |
| `le robot n'a pas bougé en poussant` | Déjà contre le mur, roues bloquées, PWM trop faible, moteurs non activés |
| `correction refusée, trop grande (… mm, … deg)` | Mauvaise zone, robot mal posé (> 15 cm / 20°), ou un objet a arrêté le robot avant le mur |
| `position (x, y) non atteinte (obstacle ?)` / `dégagement du mur impossible` | Obstacle lidar devant : personne, objet, obstacle fixe (grenier…). Revoir le trajet (§4) |
| `rotation impossible` | Robot coincé |
| `odométrie perdue` | Plus de mesure d'odométrie (CAN, carte moteur) |
| `main_strat ne répond pas` / `service correct_pose indisponible` | Nœud planté ou pas encore démarré |
| Bandeau gris en permanence | `goal_strat` ne publie pas : ROS pas lancé (`systemctl status krabi_color`) |
| En simu, le robot s'arrête sans obstacle | Sur une machine à GPU virtuel, le lidar simulé renvoie des points fantômes à 20 cm, et Core croit voir un adversaire. Vérifier `ros2 topic echo /krabi_ns/scan_obstacles` robot immobile |
| La tirette simulée ne réagit pas à `insert_tirette` | `krabi_bringup` copie ses scripts : `colcon build --packages-select krabi_bringup` |

---

## 9. Comment ça marche

- **Correction** : plaqué contre un mur, le robot connaît son cap (perpendiculaire au mur) et une
  coordonnée (mur ± 104 mm). `goal_strat` appelle `correct_pose` (`map_odom_publisher`,
  main-strategy), qui ne modifie que `map → odom` : l'odométrie n'est jamais réinitialisée (REP 105).
- **Moteurs avant le match** : `goal_strat` les active par `enable_pre_match_motion` (Core),
  seulement pendant l'action. Core les coupe si `goal_strat` se tait plus d'une seconde.
- **Tirette** : le match ne démarre que sur insérée **puis** retirée (`TiretteFilter`, commun à
  Core et `goal_strat`). Tirette retirée pendant un recalage : la séquence s'arrête, le match part.
- **IHM** : `pre_match_state` (texte, répété chaque seconde) et `recalage_zones`, publiés par
  `goal_strat` ; service `start_recalage_zone` (`krabi_msgs/srv/StartRecalage`).

| Fichier | Contenu |
|---|---|
| `goal_strategy/config/recalage_2026.yaml` | Zones et séquences |
| `goal_strategy/src/goal/recalage_sequence.cpp` | Lecture du YAML, symétrie (sans ROS, testé) |
| `goal_strategy/src/goal/recalage.cpp` | Exécution, services, état |
| `main-strategy/src/map_odom_publisher.cpp` | `map → odom`, service `correct_pose` |
| `main-strategy/src/core.cpp` | Mouvement avant match, poussée, coupure à 1 s |
| `krabi_gui` : `pre_match.py`, `banner.py`, `RecalagePage.qml` | Bandeau, onglet Recalage, RESTART COLOR |
| `krabi-startscripts/start_ros.py` | Lancement unique de ROS au démarrage |
| `krabi_bringup/src/tirette.py` | Tirette simulée |
