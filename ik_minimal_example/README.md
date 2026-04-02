# IK Minimal Example

Test minimal de cinématique inverse pour le SO-ARM101.
Place une sphère verte à la position cible dans Gazebo, résout l'IK, et envoie la trajectoire au robot.

## Prérequis

```bash
make up        # Démarrer le container
make rebuild   # Rebuilder le workspace (si nécessaire)
```

## Lancement

### Terminal 1 : Gazebo + robot

```bash
make shell
ros2 launch soarm_gazebo gazebo.launch.py
```

### Terminal 2 : Test IK

```bash
make shell
python3 /ws/ik_minimal_example/test_ik.py --target 0.25 0.0 0.155
```

### Options

```bash
# Position cible (x y z en mètres, repère world)
python3 /ws/ik_minimal_example/test_ik.py --target 0.3 0.1 0.15

# Durée du mouvement (défaut: 3s)
python3 /ws/ik_minimal_example/test_ik.py --target 0.25 0.0 0.155 --duration 5.0

# Résoudre l'IK sans bouger le robot
python3 /ws/ik_minimal_example/test_ik.py --target 0.25 0.0 0.155 --dry-run
```

## Repères utiles

- **Table** : centrée en x=0.2, y=0, surface à z=0.05, dimensions 0.6x0.6m
- **Carré rouge** : posé à z=0.055 sur la table
- **Robot** : base à x=0, y=0, z=0.05 (bord de la table)
- **Zone atteignable** : environ x=[0.05, 0.40], y=[-0.20, 0.20]

## Commandes Gazebo utiles

```bash
# Supprimer le marqueur vert manuellement
gz service -s /world/tracking_world/remove \
  --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
  --timeout 5000 --req 'name: "ik_target_marker" type: MODEL'

# Voir les modèles présents dans le monde
gz model --list

# Voir les topics Gazebo
gz topic --list
```

## Commandes ROS2 utiles

```bash
# Vérifier que les contrôleurs sont actifs
ros2 control list_controllers

# Voir les positions articulaires en temps réel
ros2 topic echo /joint_states

# Envoyer manuellement une position articulaire
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
    joint_names: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll],
    points: [{positions: [0.0, -0.5, 0.5, 0.3, 0.0], time_from_start: {sec: 2}}]
  }}"
```
