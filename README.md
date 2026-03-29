# SO-ARM101 Simulation

Simulation du bras robotique open-source [SO-ARM101](https://github.com/TheRobotStudio/SO-ARM100) (TheRobotStudio / HuggingFace) dans Gazebo Harmonic avec ROS2 Jazzy.

Un carré rouge se déplace aléatoirement sur une table. Une caméra le filme, un détecteur de couleur publie ses coordonnées, et le bras se déplace pour le suivre. Les mouvements sont enregistrés en CSV.

```
Gazebo (carré bouge)
  → Caméra overhead
  → ros_gz_bridge
  → color_detector (OpenCV HSV)
  → /target/position
  → arm_planner (IK via ikpy)
  → /arm_controller (ros2_control)
  → le bras bouge dans Gazebo
  → recorder → CSV
```

---

## 0. Origine des fichiers et reconstruction from scratch

### Fichiers de géométrie (meshes STL)

Les 13 fichiers STL proviennent du dépôt officiel :

- **Dépôt** : https://github.com/TheRobotStudio/SO-ARM100
- **Chemin** : `Simulation/SO101/assets/*.stl`
- **Commit utilisé** : branche `main` au 29 mars 2026

Pour les re-télécharger manuellement :

```bash
# Cloner le dépôt officiel
git clone --depth 1 https://github.com/TheRobotStudio/SO-ARM100.git /tmp/SO-ARM100

# Copier les meshes
cp /tmp/SO-ARM100/Simulation/SO101/assets/*.stl \
   ros2_ws/src/soarm_description/meshes/

# Copier l'URDF original (pour référence)
cp /tmp/SO-ARM100/Simulation/SO101/so101_new_calib.urdf \
   ros2_ws/src/soarm_description/urdf/so101_original.urdf

# Nettoyage
rm -rf /tmp/SO-ARM100
```

### URDF

Le fichier `so101_original.urdf` est l'URDF officiel tel quel. Le fichier `so101.urdf.xacro` est notre version adaptée qui :

- Remplace les chemins meshes relatifs (`assets/xxx.stl`) par des chemins ROS2 (`package://soarm_description/meshes/xxx.stl`)
- Nettoie les valeurs numériques quasi-nulles (ex: `8.97657e-09` → `0`)
- Ajoute des tags `<dynamics>` (damping/friction) aux joints pour la stabilité en simulation
- Ajoute le bloc `<ros2_control>` pour l'intégration avec `gz_ros2_control`
- Ajoute le plugin Gazebo `GazeboSimROS2ControlPlugin`
- Ajoute les couleurs Gazebo (les materials URDF ne sont pas automatiquement repris par Gazebo)

### Spécifications du robot

| Propriété | Valeur |
|---|---|
| Nom | SO-ARM101 (SO-100 v2) |
| DOF | 6 (5 bras + 1 gripper) |
| Joints | shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper |
| Servos | Feetech STS3215 |
| Concepteur | TheRobotStudio, en collaboration avec HuggingFace |
| Licence | Apache 2.0 |

### Autres ressources consultées

| Ressource | URL | Ce qu'on en a tiré |
|---|---|---|
| Dépôt officiel SO-ARM100 | https://github.com/TheRobotStudio/SO-ARM100 | URDF, meshes STL |
| LeRobot (HuggingFace) | https://github.com/huggingface/lerobot | Contexte logiciel (non utilisé directement) |
| brukg/SO-100-arm | https://github.com/brukg/SO-100-arm | Référence pour l'intégration ROS2 + Gazebo Harmonic |
| ROS2 Jazzy docs | https://docs.ros.org/en/jazzy/ | Documentation ROS2 |

---

## 1. Setup

### Prérequis

| Outil | Version minimale | Vérification |
|---|---|---|
| Docker | 24+ | `docker --version` |
| Docker Compose | v2+ | `docker compose version` |
| NVIDIA Container Toolkit | (si GPU NVIDIA) | `nvidia-smi` |
| X11 | (pour afficher Gazebo/RViz) | `echo $DISPLAY` |

### Construction de l'image Docker

```bash
# Cloner le projet
git clone <url-du-repo> simu_robot
cd simu_robot

# Builder l'image (première fois : ~10-15 min selon la connexion)
make build
```

L'image Docker inclut :
- ROS2 Jazzy Desktop (RViz2, rqt, etc.)
- Gazebo Harmonic (`ros-jazzy-ros-gz`)
- ros2_control + controllers (`ros-jazzy-ros2-controllers`, `ros-jazzy-gz-ros2-control`)
- Python : ikpy, opencv-python, numpy (installés via uv)

### Structure du projet

```
simu_robot/
├── docker/
│   ├── Dockerfile              # Image multi-stage (base → dev)
│   ├── docker-compose.yml      # Services : sim, rviz
│   └── entrypoint.sh           # Source ROS2 + workspace + GZ paths
│
├── ros2_ws/src/
│   ├── soarm_description/      # Modèle robot (URDF, meshes, controllers)
│   │   ├── urdf/
│   │   │   ├── so101.urdf.xacro       # XACRO principal (utilisé par les launch)
│   │   │   └── so101_original.urdf    # URDF officiel (référence)
│   │   ├── meshes/                     # 13 fichiers STL
│   │   ├── config/controllers.yaml     # Config ros2_control
│   │   ├── launch/rviz.launch.py       # Launch RViz standalone
│   │   └── rviz/default.rviz
│   │
│   ├── soarm_gazebo/           # Simulation Gazebo
│   │   ├── worlds/tracking_world.sdf   # Monde : sol, table, caméra, carré rouge
│   │   ├── config/gz_bridge.yaml       # Bridge Gazebo ↔ ROS2
│   │   └── launch/
│   │       ├── gazebo.launch.py        # Gazebo + robot + controllers
│   │       └── full_sim.launch.py      # Tout (Gazebo + nodes tracking)
│   │
│   └── soarm_tracking/         # Nodes applicatifs (Python)
│       └── soarm_tracking/
│           ├── square_mover.py         # Déplace le carré aléatoirement
│           ├── color_detector.py       # Détection HSV → coordonnées 3D
│           ├── arm_planner.py          # IK (ikpy) → trajectoire bras
│           └── recorder.py             # Enregistre en CSV
│
├── recordings/                 # Sorties CSV (volume Docker monté)
├── pyproject.toml              # Dépendances Python (uv)
└── Makefile                    # Commandes raccourcies
```

### Architecture ROS2 (topics)

```
/camera/image_raw        (sensor_msgs/Image)         gz_bridge → color_detector
/target/position         (geometry_msgs/PointStamped) color_detector → arm_planner, recorder
/target/image_debug      (sensor_msgs/Image)         color_detector → RViz
/joint_states            (sensor_msgs/JointState)     joint_state_broadcaster → recorder, RViz
/arm_controller/follow_joint_trajectory  (action)     arm_planner → arm_controller
```

---

## 2. Utilisation

### Commandes make disponibles

| Commande | Description |
|---|---|
| `make build` | Construit l'image Docker |
| `make up` | Démarre le container sim en arrière-plan |
| `make down` | Arrête tous les containers |
| `make shell` | Ouvre un bash dans le container sim |
| `make rebuild` | Recompile le workspace ROS2 (colcon) dans le container |
| `make sim` | Lance directement la simulation complète |
| `make rviz` | Lance RViz seul (nécessite que sim tourne) |
| `make clean` | Supprime containers et images |

### Lancer la simulation complète

```bash
# 1. Builder l'image (si pas déjà fait)
make build

# 2. Lancer la simulation (Gazebo + tous les nodes)
make sim
```

Cela lance :
- Gazebo Harmonic avec le monde (table, carré rouge, caméra)
- Le robot SO-ARM101 spawné sur la table
- Les controllers ros2_control (arm_controller + gripper_controller)
- Le bridge ros_gz (caméra → ROS2)
- Les 4 nodes : square_mover, color_detector, arm_planner, recorder

### Lancer en mode interactif (debug)

```bash
# Démarrer le container
make up

# Ouvrir un shell
make shell

# Dans le container — lancer uniquement Gazebo + robot :
ros2 launch soarm_gazebo gazebo.launch.py

# Ou lancer la simulation complète :
ros2 launch soarm_gazebo full_sim.launch.py
```

### Lancer les nodes individuellement (debug)

Depuis un shell dans le container (`make shell`) :

```bash
# Lancer Gazebo + robot d'abord
ros2 launch soarm_gazebo gazebo.launch.py &

# Puis les nodes un par un
ros2 run soarm_tracking square_mover
ros2 run soarm_tracking color_detector
ros2 run soarm_tracking arm_planner
ros2 run soarm_tracking recorder
```

### Visualiser avec RViz

```bash
# Dans un second terminal (pendant que sim tourne)
make rviz

# Ou dans le container :
ros2 launch soarm_description rviz.launch.py
```

### Inspecter les topics

```bash
# Depuis un shell dans le container
ros2 topic list
ros2 topic echo /target/position
ros2 topic echo /joint_states
ros2 topic hz /camera/image_raw
```

### Voir l'image caméra

```bash
# Via rqt
rqt_image_view /camera/image_raw

# Ou l'image debug (avec la détection annotée)
rqt_image_view /target/image_debug
```

### Récupérer les enregistrements

Les fichiers CSV sont écrits dans le dossier `recordings/` (monté comme volume Docker) :

```bash
ls recordings/
# target_positions_20260329_143052.csv
# joint_states_20260329_143052.csv
```

Format des fichiers :

**target_positions_*.csv** :
```
timestamp,x,y,z
1711712345.123456,0.250000,0.100000,0.055000
```

**joint_states_*.csv** :
```
timestamp,shoulder_pan_pos,shoulder_pan_vel,shoulder_lift_pos,shoulder_lift_vel,...
1711712345.123456,0.523599,0.010000,-0.261799,0.005000,...
```

### Paramètres ajustables

Les nodes acceptent des paramètres ROS2. Pour les modifier, éditer `ros2_ws/src/soarm_gazebo/launch/full_sim.launch.py` ou passer des arguments :

```bash
# Exemple : changer l'intervalle de déplacement du carré
ros2 run soarm_tracking square_mover --ros-args -p move_interval:=3.0

# Exemple : changer les bornes du workspace
ros2 run soarm_tracking square_mover --ros-args \
  -p x_min:=0.1 -p x_max:=0.35 -p y_min:=-0.15 -p y_max:=0.15
```

| Node | Paramètre | Défaut | Description |
|---|---|---|---|
| square_mover | `move_interval` | 2.0 | Intervalle entre déplacements (s) |
| square_mover | `x_min/x_max` | 0.05/0.40 | Bornes X du workspace |
| square_mover | `y_min/y_max` | -0.20/0.20 | Bornes Y du workspace |
| color_detector | `hsv_lower_1` | [0,120,70] | Borne basse HSV (rouge 1) |
| color_detector | `hsv_upper_1` | [10,255,255] | Borne haute HSV (rouge 1) |
| color_detector | `table_height` | 0.055 | Hauteur Z de la table |
| arm_planner | `move_duration` | 1.5 | Durée de chaque mouvement (s) |
| arm_planner | `min_move_interval` | 1.0 | Intervalle min entre commandes (s) |
| recorder | `output_dir` | /ws/recordings | Répertoire de sortie CSV |

---

## Dépannage

### Gazebo ne s'affiche pas

```bash
# Sur la machine hôte, autoriser Docker à accéder à X11
xhost +local:docker

# Vérifier que DISPLAY est défini
echo $DISPLAY  # doit afficher :0 ou :1
```

### Le robot n'apparaît pas dans Gazebo

```bash
# Vérifier que le spawn a fonctionné
ros2 topic echo /robot_description --once | head -5

# Vérifier les logs du spawn
ros2 run ros_gz_sim create -name so101 -topic robot_description
```

### Les controllers ne démarrent pas

```bash
# Lister les controllers
ros2 control list_controllers

# Vérifier le controller_manager
ros2 service list | grep controller_manager
```

### L'image caméra est noire

```bash
# Vérifier que le bridge fonctionne
ros2 topic hz /camera/image_raw

# Vérifier les topics Gazebo
gz topic -l | grep camera
```

# 3. Explications

1. Le monde Gazebo — tracking_world.sdf
C'est la scène 3D. Comme un plateau de tournage avant d'y poser les acteurs.


Plugins système (lignes 13-19)
Physics — le moteur physique ODE (gravité, collisions, forces)
SceneBroadcaster — permet à l'interface graphique Gazebo d'afficher la scène
UserCommands — permet de téléporter/supprimer des modèles via des services (utilisé par square_mover)
Sensors — active le rendu des capteurs (caméra). Sans ce plugin, la caméra ne produit rien

Modèles statiques
ground_plane — le sol infini
table — une boîte de 60×60cm, 5cm d'épaisseur, centrée en x=0.2 (devant le robot)
overhead_camera — la caméra fixe, en plongée à 70cm de haut

Modèle dynamique
colored_square — le petit carré rouge 4×4cm, posé sur la table. static=false car le square_mover le téléporte
2. Le modèle du robot — so101.urdf.xacro
C'est l'ADN du robot. Il décrit 3 choses :

a) Les links (lignes 25-255) — les pièces physiques
Chaque <link> est un corps rigide avec :

<inertial> — masse + tenseur d'inertie (nécessaire pour la simulation physique)
<visual> — le mesh STL pour l'affichage 3D
<collision> — la forme utilisée pour les collisions (ici = même mesh)
La chaîne : base_link → shoulder_link → upper_arm_link → lower_arm_link → wrist_link → gripper_link → moving_jaw_link

b) Les joints (lignes 260-318) — les articulations
Chaque <joint> relie deux links :

type="revolute" — rotation autour d'un axe
<axis> — l'axe de rotation (ex: 0 0 1 = rotation autour de Z)
<limit> — butées angulaires (en radians), effort max, vitesse max
<dynamics> — amortissement et friction du joint
<origin> — la position/orientation du joint par rapport au link parent
Joint	Mouvement
shoulder_pan	Rotation de la base (gauche/droite)
shoulder_lift	Lever/baisser l'épaule
elbow_flex	Plier le coude
wrist_flex	Plier le poignet
wrist_roll	Rotation du poignet
gripper	Ouvrir/fermer la pince
c) Le bloc ros2_control (lignes 323-357) — l'interface de commande
C'est le pont entre ros2_control et Gazebo. Il dit :

Le plugin hardware est gz_ros2_control/GazeboSimSystem (simulé, pas un vrai moteur)
Pour chaque joint : on peut commander en position (command_interface) et lire la position et la vitesse (state_interface)
initial_value = la position au démarrage (tous à 0.0 actuellement → bras déplié → il tombe)
d) Le plugin Gazebo (lignes 362-366)

<plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
  <parameters>$(find soarm_description)/config/controllers.yaml</parameters>
</plugin>
Ce plugin injecté dans Gazebo fait le lien : il lit les commandes venant de ros2_control et les applique comme des forces/positions sur les joints simulés. Il charge la config depuis controllers.yaml.

3. La config des contrôleurs — controllers.yaml
C'est la configuration de ros2_control, le framework qui gère les contrôleurs.


controller_manager:          # Le chef d'orchestre
  update_rate: 100           # Boucle de contrôle à 100 Hz
Il déclare 3 contrôleurs :

Contrôleur	Type	Rôle
joint_state_broadcaster	Broadcaster	Ne commande rien. Il lit les positions/vitesses de tous les joints et les publie sur /joint_states
arm_controller	JointTrajectoryController	Reçoit des trajectoires (séquence de positions dans le temps) via une action ROS2. Interpole et envoie les commandes de position aux 5 joints du bras
gripper_controller	JointGroupPositionController	Plus simple : reçoit une position directe sur un topic pour le joint gripper
La différence clé :

arm_controller utilise une action (/arm_controller/follow_joint_trajectory) — requête/réponse avec feedback, adapté aux mouvements complexes
gripper_controller utilise un topic (/gripper_controller/commands) — fire-and-forget, adapté à ouvrir/fermer
4. Le bridge Gazebo↔ROS2 — gz_bridge.yaml
Gazebo et ROS2 sont deux mondes séparés avec leurs propres systèmes de messages. Le bridge traduit :

/camera/image (Gazebo) → /camera/image_raw (ROS2) — l'image caméra
/clock (Gazebo) → /clock (ROS2) — le temps simulé (pour use_sim_time: True)
Sans ce bridge, les nodes ROS2 ne voient ni la caméra ni le temps.

5. Le launch Gazebo — gazebo.launch.py
L'orchestrateur du démarrage. L'ordre compte :

robot_state_publisher — publie le URDF sur /robot_description (pour que Gazebo sache quoi spawner)
gazebo — lance le simulateur avec le monde
spawn_robot — injecte le robot dans le monde
gz_bridge + gz_set_pose_bridge — les bridges
Après que spawn_robot finit → joint_state_broadcaster
Après que joint_state_broadcaster finit → arm_controller + gripper_controller
Ce chaînage (lignes 113-126) est crucial : si les contrôleurs démarrent avant que le robot existe dans Gazebo, ils crashent.

6. Le launch complet — full_sim.launch.py
Inclut gazebo.launch.py + ajoute les 4 nodes de tracking :

square_mover — téléporte le carré rouge
color_detector — détecte le rouge dans l'image
arm_planner — calcule l'IK et envoie les trajectoires
recorder — enregistre les données
7. Les nodes de tracking — soarm_tracking/
square_mover.py — Toutes les 2 secondes, appelle le service Gazebo set_pose pour déplacer colored_square à une position aléatoire sur la table.

color_detector.py — Écoute /camera/image_raw, filtre le rouge en HSV, trouve le plus grand contour, projette le centroïde pixel en coordonnées 3D monde (via la pose connue de la caméra + intersection avec le plan de la table). Publie sur /target/position.

arm_planner.py — Écoute /target/position, calcule la cinématique inverse avec ikpy, envoie la trajectoire au arm_controller.

recorder.py — Enregistre les données (pas lu en détail, mais sauvegarde dans /ws/recordings).