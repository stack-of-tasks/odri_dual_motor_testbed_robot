# Modèle géométrique direct du five-bar — spec du plugin Gazebo

Fichier robot : `motkin_dual_motor_testbed_description/urdf/fivebar_2dof.urdf.xacro`
Plugin : `libFiveBarClosurePlugin.so` / `motkin_gz::FiveBarClosurePlugin`
(sources : `motkin_dual_motor_testbed_gazebo/src/FiveBarClosurePlugin.cc`)

> **Mise à jour (mécanisme MOTKINBENCH).** Le robot décrit ici provient désormais de
> `MOTKINBENCH/urdf/five_bar/robot.urdf` (Onshape `581c9cc3…`). C'est un mécanisme
> *différent* de l'ancien, pas une révision : noms, dimensions et orientation du plan
> ont tous changé. Les correspondances avec l'ancienne version :
>
> | ancien | nouveau |
> |---|---|
> | `base_asm` | `base_link` (racine muette) + `case` (masse réelle) |
> | `rotor_asm` / `rotor_asm_2` | `rotor` / `rotor_2` |
> | `l2r` / `l2l` | `arm_l2` / `arm_r2` |
> | `motorL` / `motorR` | `motor_1` / `motor_2` |
> | `elbowL` / `elbowR` | `passive_1` / `passive_2` |
> | `closing_ee_1` / `closing_ee_2` | `closing_tip_2` / `closing_tip_1` |
>
> Le plan du mécanisme est maintenant le plan **(x,y) de `case`** (les deux axes moteur
> pointent selon son -z local), alors que l'ancien évoluait dans le plan (x,z) de
> `base_asm` (axe y). Les paramètres `a_z` / `b_z` du plugin et les suffixes `_z` du code
> gardent leur nom historique mais portent désormais les coordonnées **y** : l'algèbre ne
> les traite jamais que comme « deuxième axe dans le plan ».
>
> Conséquence physique : posé à plat, ce five-bar est **horizontal**. La gravité est
> perpendiculaire au plan et ne produit aucun couple sur `motor_1`/`motor_2` — la
> compensation de gravité des contrôleurs `motkin_five_bar_force_velocity_*` est donc
> désactivée par défaut (voir leurs README).

## 1. Historique : l'ancien plugin à pénalité

Avant le passage au modèle géométrique exact, un ressort-amortisseur de pénalité tirait un
point de fermeture vers l'autre en appliquant une force sur les deux manivelles
(`kp=50000`, `kd=500`, `max_force=20`). C'était une approximation numérique de la fermeture
de boucle, sensible aux gains et instable si la configuration initiale ne respectait pas la
contrainte. Il a été **entièrement remplacé** par le modèle ci-dessous.

## 2. Modèle géométrique direct (exact, sans intégration ni gains)

Le mécanisme est un five-bar plan. Entrées : θ1 = `motor_1`, θ2 = `motor_2`. Sorties
recherchées : β1 = `passive_1`, β2 = `passive_2`, et P = position du point de fermeture dans
le plan (x,y) de `case`.

### Paramètres (extraits des origins du xacro, vérifiés numériquement)

```
A  = (-0.0421006, 0.0302628)   # axe de motor_1 dans le plan (x,y) de case
B  = ( 0.0578994, 0.0302628)   # axe de motor_2
L1 = 0.06              # longueur manivelle (motor -> elbow), identique gauche/droite
L2 = 0.1               # longueur bielle (elbow -> point de fermeture), identique g/d
phi1 = 1.5707963267948966   # offset angulaire manivelle gauche (rad)
phi2 = 1.5707963267948966   # offset angulaire manivelle droite (rad)
psi1 = 0.21205399999927563   # offset angulaire bielle gauche (rad)
psi2 = 2.511306362956145   # offset angulaire bielle droite (rad)
```

> **Zéro moteur.** Le yaw des origins de `motor_1` / `motor_2` a été mis à 0 (au lieu
> de 0.662203 / -0.0662491 exportés) pour que θ = 0 corresponde au zéro des codeurs :
> les deux manivelles sont alors parallèles, orientées selon +y de `case`
> (d'où phi1 = phi2 = π/2). Les anciennes valeurs (phi1 = 2.2329993267948964,
> phi2 = 1.5045472267948965, psi1 = 0.8742569999992758, psi2 = 2.4450572629561456)
> correspondent au zéro CAO et ne doivent plus être utilisées. À θ = 0, le mécanisme
> se ferme avec β1 ≈ 0.8351, β2 ≈ -0.4169, d_EE = 0.1 m.

### Étape 1 — position des coudes (directe)

```
E_L(theta1) = A + L1 * [cos(phi1 - theta1), sin(phi1 - theta1)]
E_R(theta2) = B + L1 * [cos(phi2 - theta2), sin(phi2 - theta2)]
```

### Étape 2 — P par intersection de deux cercles de rayon L2

```
d_EE = norm(E_R - E_L)
u    = (E_R - E_L) / d_EE
M    = (E_L + E_R) / 2
h    = sqrt(L2**2 - (d_EE/2)**2)       # exige d_EE <= 2*L2 (sinon hors espace de travail)
P    = M + h * perp(u)                 # perp(ux,uy) = (-uy, ux)  -- branche vérifiée
```

La branche `+perp(u)` est bien celle de l'assemblage réel : au zéro CAO elle redonne
`closing_tip_*` à 1.0e-07 m près, l'autre branche à 1.4e-01 m.

### Étape 3 — angles des coudes

```
beta1 = atan2((P-E_L)_y, (P-E_L)_x) - psi1 + theta1
beta2 = atan2((P-E_R)_y, (P-E_R)_x) - psi2 + theta2
```

### Convention de signe des joints passifs — important

Ces formules supposent que le coude tourne dans le sens **opposé** à son moteur, ce qui
était le cas de l'ancien mécanisme (`elbowL`/`elbowR` portaient un roll de π par rapport à
leur manivelle). Dans l'export MOTKINBENCH, `passive_1`/`passive_2` n'ont pas ce flip et
tournent donc dans le **même** sens que les moteurs, ce qui donnerait
`beta = psi - theta - atan2(...)`, soit l'opposé.

Plutôt que de dupliquer un signe dans le plugin, le xacro publie ces deux joints avec
`axis="0 0 -1"` au lieu du `"0 0 1"` exporté (voir l'en-tête de
`fivebar_2dof.urdf.xacro`, delta 2). Le plugin et les deux contrôleurs restent inchangés.

**Si vous régénérez le xacro depuis onshape-to-robot, ce flip doit être réappliqué** —
sans lui la fermeture de boucle est fausse de ~10 cm.

### Validation numérique

Vérifié en composant les transformations homogènes complètes du xacro (origins + rpy de
`motor_1`, `passive_1`, `closing_tip_2_frame`, et symétrique côté droit), avec l'algorithme
du plugin non modifié et l'axe passif inversé :

| quantité | valeur |
|---|---|
| résidu de fermeture max, θ ∈ [-0.7, 0.7]² (zéro CAO) | **6.6e-07 m** |
| idem, avec l'axe passif tel qu'exporté | 9.6e-02 m (cassé) |
| résidu de l'export CAO lui-même (θ=β=0, zéro CAO) | 6.5e-07 m |
| d_EE au zéro CAO / limite 2·L2 | 0.1414 m / 0.2 m |
| résidu de fermeture max, θ ∈ [-0.9, 0.9]² (zéro codeur, yaw moteur = 0) | **2.7e-07 m** |
| d_EE à θ = 0 (zéro codeur) | 0.1 m |

(L'ancien export CAO présentait un résidu de ~0.5 mm ; celui-ci est propre.)

## 3. Ce que fait le plugin

Le plugin ne pilote plus les joints passifs : les deux bras sont reliés en P par une
contrainte de pivot plane, et c'est la force transmise en P qui ferme la boucle.

1. Au démarrage uniquement (`<initialize_with_mgd>true`, défaut) : calculer `beta1`,
   `beta2` avec les formules ci-dessus et appeler `ResetPosition` une fois sur `passive_1`
   / `passive_2`, pour que la simulation parte d'une configuration fermée.
2. À chaque step, pour chaque bras i : position P_i de la pointe (`arm_l2` + `tip_offset1`,
   `arm_r2` + `tip_offset2`), jacobienne J_i = ∂P_i/∂(θ_i, β_i) et matrice de masse 2×2
   M_i construite à partir des inerties des liens dans l'ECM.
3. Contrainte c = P_1 − P_2 projetée dans le plan du mécanisme (normal aux axes moteur),
   J = [J_1, −J_2]. Le multiplicateur de Lagrange
   `λ = (J M⁻¹ Jᵀ)⁻¹ (−β c/dt² − J q̇/dt − J a_libre)`
   est la force transmise en P ; elle impose `J q̇⁺ = −β c/dt` après le step (Baumgarte
   en vitesse, `<baumgarte>` = 0.2 par défaut).
   `a_libre` (accélération articulaire sans contrainte) est estimée au step précédent :
   accélération mesurée moins la part due à λ. Couples moteur, frottements, Coriolis et
   contacts sont ainsi pris en compte sans être modélisés.
4. Appliquer +λ en P_1 sur `arm_l2` et −λ en P_2 sur `arm_r2` (`Link::AddWorldForce`).
   |λ| est écrêté à `<max_force>` (100 N par défaut) : ne sert qu'au démarrage si la
   configuration initiale est ouverte.

Vérifié en simulation (plan vertical, gravité seule, dt = 1 ms, 5 s) : |c| ≤ 5e-06 m ;
sans assemblage initial (boucle ouverte de 7 cm), fermeture en quelques steps puis
|c| ≤ 7e-06 m.

Les joints passifs doivent rester de type `continuous` : `atan2` renvoie dans (-π, π], donc
`beta = atan2(...) - psi + theta` peut sortir de [-π, π] (jusqu'à ≈ -5.03 rad sur la grille
testée). Avec un joint `revolute` borné à ±π, l'assemblage initial par `ResetPosition` serait
écrêté.

## 3 bis. Robot réel

Sur le robot réel, seuls `motor_1` / `motor_2` ont des codeurs. Le plugin ros2_control
`motkin_dual_motor_testbed_hardware/FiveBarSystem` enveloppe le plugin de la carte
(`inner_plugin`, par défaut `SystemPicoDualDrv8316CHardware`) et exporte `passive_1` /
`passive_2` comme interfaces d'état seules (position, vitesse, effort = 0), recalculées
après chaque `read()` avec ce modèle ; `joint_state_broadcaster` publie donc tout le
five-bar, comme en simulation. Il est activé par `five_bar="true"` dans
`system_motkin.ros2_control.xacro`, dont les paramètres géométriques doivent
rester identiques à ceux du plugin Gazebo.

Vitesses passives : en dérivant `r_i · (Ṗ - Ė_i) = 0` avec `r_i = P - E_i`, on obtient Ṗ
par un système 2×2, puis `β̇_i = (r_i × (Ṗ - Ė_i)) / L2² + θ̇_i`.

## 4. Régénération depuis la CAO

Source : `MOTKINBENCH/urdf/five_bar/` (`robot.urdf`, `assets/`, `config.json`). Les maillages
sont recopiés dans `motkin_dual_motor_testbed_description/meshes/assets/five_bar/`. Après
régénération, réappliquer les cinq deltas listés en en-tête de `fivebar_2dof.urdf.xacro`, et
recalculer phi/psi si la CAO a bougé.
