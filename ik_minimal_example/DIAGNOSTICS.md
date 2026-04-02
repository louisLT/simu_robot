# Diagnostics IK - SO-ARM101

Guide de debug pour la cinématique inverse, basé sur les problèmes rencontrés.

## Prérequis

Gazebo doit tourner avec le robot spawné :

```bash
# Terminal 1
make shell
ros2 launch soarm_gazebo gazebo.launch.py
```

Toutes les commandes suivantes sont dans un 2e terminal (`make shell`).

## 1. Vérifier les contrôleurs

```bash
ros2 control list_controllers
```

On doit voir :
```
joint_state_broadcaster [active]
arm_controller          [active]
gripper_controller      [active]
```

## 2. Vérifier les positions articulaires

```bash
ros2 topic echo /joint_states
```

Attention : l'ordre des joints dans le message est **alphabétique**, pas dans l'ordre de la chaîne cinématique.

## 3. Envoyer manuellement des angles connus

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {
    joint_names: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll],
    points: [{positions: [0.0, -0.5, 0.5, 0.3, 0.0], time_from_start: {sec: 2}}]
  }}"
```

## 4. Comparer FK ikpy vs FK Gazebo (KDL)

C'est le diagnostic clé pour détecter si une bibliothèque IK interprète le URDF correctement.

### Méthode

1. Envoyer le robot à des angles connus (via le script `--diag` ou manuellement)
2. Lire la position réelle du gripper selon Gazebo/KDL :

```bash
ros2 run tf2_ros tf2_echo world gripper_link
```

3. Comparer avec la prédiction FK du solver :

```bash
python3 /ws/ik_minimal_example/test_ik.py --diag
```

4. Le script place une **sphère bleue** à la position FK prédite. Si elle n'est pas à la base du gripper dans Gazebo, le modèle cinématique du solver diverge.

### Résultat observé avec ikpy

Pour la target `[0.25, 0.0, 0.155]` :

| Source | X | Y | Z |
|--------|-------|--------|-------|
| ikpy FK | 0.250 | 0.000 | 0.155 |
| Gazebo TF | 0.239 | -0.018 | 0.184 |
| **Erreur** | -0.011 | -0.018 | +0.029 |

Erreur totale : ~3.6 cm. Cause : ikpy interprète les rotations composées du URDF différemment de KDL (utilisé par Gazebo et ros2_control).

## 5. Vérifier la chaîne IK

```bash
python3 -c "
import ikpy.chain
chain = ikpy.chain.Chain.from_urdf_file(
    '/tmp/so101.urdf',
    base_elements=['base_link'],
)
for i, link in enumerate(chain.links):
    print(f'[{i}] {link.name}')
"
```

Vérifier que la chaîne se termine au bon link et que le nombre de links correspond au masque `active_links_mask`.

## 6. Topics utiles

```bash
# Image caméra brute
ros2 topic hz /camera/image_raw

# Position de la cible détectée
ros2 topic echo /target/position

# Image de debug (détection couleur)
ros2 run rqt_image_view rqt_image_view /target/image_debug

# TF d'un link quelconque
ros2 run tf2_ros tf2_echo world <link_name>
```

## 7. Commandes Gazebo

```bash
# Lister les modèles
gz model --list

# Supprimer un marqueur
gz service -s /world/tracking_world/remove \
  --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
  --timeout 5000 --req 'name: "ik_target_marker" type: MODEL'

# Spawner un marqueur manuellement
gz service -s /world/tracking_world/create \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'sdf: "<sdf version=\"1.9\"><model name=\"test\"><static>true</static><pose>0.25 0 0.15 0 0 0</pose><link name=\"link\"><visual name=\"v\"><geometry><sphere><radius>0.02</radius></sphere></geometry><material><ambient>0 1 0 1</ambient><diffuse>0 1 0 1</diffuse></material></visual></link></model></sdf>"'
```
