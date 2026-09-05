# Robot (Raspberry Pi 5)

Code embarqué sur la Raspberry Pi n°1 du projet : pilotage moteurs, télécommande,
asservissement PID et navigation GPS/DGPS pour un robot mobile d'extérieur.

Le serveur web (Flask, sur la Raspberry Pi n°2) qui envoie les ordres à ce robot
vit dans un dépôt séparé.

## Structure du projet

```
.
├── motor_control/      # Pilotage moteurs (GPIO/PWM) et entrée manette
│   ├── pwm.py
│   ├── remote_control.py
│   ├── gps_condition_logger.py      # Moteur commun aux 2 scripts ci-dessous (logique de journalisation GPS conditionnelle)
│   ├── gps_log_on_full_throttle.py  # Variante de remote_control.py : journalise le GPS en ligne droite a fond (reponse a l'echelon, translation)
│   └── gps_log_on_full_rotation.py  # Variante de remote_control.py : journalise le GPS en rotation sur place a fond (reponse a l'echelon, rotation)
├── pid/                # Asservissement PID (distance / cap)
│   ├── pid_controller.py
│   ├── pid_plot.py               # Simulation/visualisation hors robot (ancien)
│   ├── simulate_pid.py           # Simulateur de réglage PID (1 boucle, coefficients réels)
│   └── simulate_motor_commands.py # Simulateur distance+angle -> commandes moteur PWM
├── gps/                # Lecture GPS, correction DGPS (NTRIP), calculs géo
│   ├── gps_parse.py
│   ├── gps_transfer.py
│   ├── dgps_transfer.py
│   └── gps_delta.py
├── link/               # Serveur TCP recevant les commandes du site web (voir plus bas)
│   ├── nmea.py         # Construction/analyse des trames + checksum
│   ├── robot_state.py  # Validation et état (PWM, mode, cible GPS, gains PID, appelle camera/ en HTTP pour CAM,SNAP)
│   ├── gps_reader.py   # Lecture GPS en tâche de fond (voir plus bas)
│   └── server.py        # Serveur TCP (socketserver), aucune dépendance externe
├── camera/             # Flux caméra en direct + snapshots (voir plus bas) -- processus séparé de link/, relié par HTTP local
│   ├── stream_server.py
│   └── snapshots.py    # Stockage des snapshots, jamais plus de 5 fichiers
├── archive/            # Anciennes versions gardées pour référence (voir plus bas)
├── tests/              # Tests automatisés (pytest)
├── requirements.txt
└── .env.example        # Modèle pour les identifiants NTRIP (voir plus bas)
```

## Installation

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

Certains modules nécessitent du matériel Raspberry Pi (`gpiod`, `evdev`) ou un
GPS branché en USB/série (`/dev/ttyACM0`) — ils ne peuvent être exécutés que sur
la Raspberry Pi n°1, pas sur un PC de développement classique.

## Configuration (identifiants NTRIP)

`gps/dgps_transfer.py` utilisait auparavant des identifiants NTRIP écrits en
dur dans le code. Ils sont maintenant lus depuis un fichier `.env` (jamais
commité, voir `.gitignore`) :

```bash
cp .env.example .env
# puis éditer .env si vous avez vos propres identifiants NTRIP
```

Sans fichier `.env`, le script retombe sur le compte public de test du réseau
Centipede (`crtk.net`), comme avant.

## Réglage des PID (hors robot)

Deux outils de simulation, sans matériel nécessaire :

```bash
# Comparer plusieurs jeux de coefficients sur une seule boucle
python3 pid/simulate_pid.py --y0 5 --setpoint 0 --kp 1 --ki 0 --kd 0,0.5,2 \
  --ylabel "Distance restante (m)"

# Voir l'effet combiné distance+angle sur les commandes moteur (left_speed/right_speed)
python3 pid/simulate_motor_commands.py --dist-kp 1 --dist-kd 0.5 --angle-kp 0.5 --angle-kd 1
```

Le second script reproduit exactement le mixage de `move_to_target()` (distance
+ angle -> `left_speed`/`right_speed`) et trace des lignes de repère à ±255
pour repérer visuellement une saturation — voir "Corrections apportées"
plus bas, deux problèmes réels ont été détectés puis corrigés grâce à cet outil.

Les coefficients par défaut dans `pid_controller.py`
(`pid_distance = PIDController(kp=1, ki=0, kd=0.5, ...)`,
`pid_angle = PIDController(kp=0.5, ki=0, kd=1, ...)`) sont ceux validés par ces
simulations (pas de dépassement ni de saturation moteur sur le modèle simulé).
Ils restent à confirmer sur le robot réel, à basse vitesse, avant un usage
complet en extérieur.

## Serveur de commandes (`link/`)

Reçoit les trames NMEA envoyées par le site web (Raspberry Pi n°2, voir
`pages/protocole_controle.html` dans le dépôt `robot-webserver` pour le
format complet des trames) et y répond par `ACK`/`ERR`/`STA`.

```bash
python3 -m link                                    # écoute sur 0.0.0.0:5050
CONTROL_HOST=0.0.0.0 CONTROL_PORT=5050 python3 -m link   # port personnalisé
```

État actuel de l'implémentation :

- `STP` (arrêt d'urgence) et `DRV` (pilotage direct) valident les entrées et
  mettent à jour un état en mémoire (`link/robot_state.py`), avec la bonne
  réponse `ACK`/`ERR` — mais ne pilotent pas encore les moteurs réels.
  `motor_control/pwm.py` est aujourd'hui un script bloquant qui ouvre le GPIO
  dès son import et boucle sur `stdin` : ce n'est pas une fonction qu'on peut
  appeler depuis `link/`. Le brancher pour de vrai demande soit de le
  refactoriser en fonction (comme `pid_controller.py` l'a déjà été), soit de
  faire cohabiter ce serveur avec le pipeline existant
  (`dgps_transfer.py | pid_controller.py | pwm.py`) — à décider avant de
  continuer sur ce point precis.
- `MOD` et `NAV` et `PID` enregistrent la valeur reçue et répondent `ACK`,
  sans encore agir dessus (pas de lien avec le pipeline GPS/PID pour
  l'instant).
- `CAM,SNAP` appelle pour de vrai la route `GET /snap` de
  `camera/stream_server.py` (processus séparé, en HTTP local — voir section
  dédiée ci-dessous) et répond `ACK`/`ERR` selon que ça réussit ou non
  (caméra éteinte, pas encore de trame disponible...). `CAM,REC_START` et
  `CAM,REC_STOP` répondent toujours `ERR` (`CAM_NOT_IMPLEMENTED`) —
  l'enregistrement vidéo n'existe pas dans le projet à ce jour.
- `STA` (sans champ, en requête) répond avec l'état courant : position
  *courante* (`lat`/`lat_dir`/`lon`/`lon_dir`), `cap` et `speed` sont lus
  pour de vrai depuis un récepteur GPS série par `link/gps_reader.py` (voir
  section dédiée ci-dessous) — `0.0` tant qu'aucun récepteur n'est branché
  ou qu'aucune trame valide n'a été reçue. Position *cible*
  (`target_lat`/`target_lat_dir`/`target_lon`/`target_lon_dir`, dernière
  trame `NAV` reçue), `left_pwm`/`right_pwm` et `mode` sont réels dès
  aujourd'hui. `batterie` reste à 0 (aucun capteur de batterie dans le
  projet). Tout ça alimente le bandeau de statut et l'onglet "TCP" de
  `/control` sur le site web.

## Lecture GPS (`link/gps_reader.py`)

Tourne automatiquement en arrière-plan dans `link/server.py` (désactivable
avec `GPS_ENABLED=false`) : lit un récepteur GPS série (même matériel que
`gps/gps_transfer.py`/`gps/gps_parse.py` — trames `GPRMC`/`GGA` via
`pynmea2`) et met à jour la position courante, le cap et la vitesse
utilisés par `STA`. Se dégrade proprement si aucun récepteur n'est
branché (ou si `pynmea2`/`pyserial` ne sont pas installés) : un
avertissement dans les logs, et le serveur de contrôle continue de
fonctionner normalement avec la position courante à `0.0`.

```bash
python3 -m link                          # GPS sur /dev/ttyACM0 @ 57600 bauds par défaut
GPS_DEVICE=/dev/ttyACM1 python3 -m link  # port série personnalisé
GPS_ENABLED=false python3 -m link        # désactive la lecture GPS (utile si gps_transfer.py
                                          # ou gps_parse.py utilise déjà le port série --
                                          # un seul processus peut l'ouvrir à la fois)
```

**Non testé sur un vrai récepteur** : `pynmea2` et `pyserial` (déjà dans
`requirements.txt`) n'ont pas pu être installés dans l'environnement où
ce module a été écrit (pas d'accès PyPI, même limitation que pour
`pytest`). Le calcul de conversion de coordonnées (`link/nmea.py`,
`decimal_to_nmea`/`nmea_to_decimal`) est testé pour de vrai ; le parsing
`GPRMC`/`GGA` (`tests/test_gps_reader.py`) est écrit contre l'API
documentée de `pynmea2` mais n'a jamais tourné pour de vrai — lancer
`pytest tests/test_gps_reader.py` sur la Pi avant de lui faire confiance.
La dégradation propre (récepteur absent, bibliothèques absentes) a en
revanche été vérifiée pour de vrai : le serveur démarre et répond
normalement dans les deux cas.

## Journalisation GPS pour courbes de réponse à l'échelon (`motor_control/gps_log_on_full_*.py`)

Deux variantes de `motor_control/remote_control.py` (même manette, mêmes
moteurs, code de `Remote` inchangé), pensées pour définir les courbes de
réponse à l'échelon du robot — une en translation, une en rotation —
sans avoir à trier tout le reste du trajet dans les données GPS :
chacune ajoute une tâche de fond (`motor_control/gps_condition_logger.py`,
moteur commun aux deux) qui lit en continu le GPS série (même
matériel/port que `gps/gps_parse.py` et `link/gps_reader.py`) et n'écrit
dans un fichier de log que lorsque les deux moteurs remplissent une
condition précise :

- **`gps_log_on_full_throttle.py`** — ligne droite à fond :
  `dutyCycleLeft` ET `dutyCycleRight` == 255 (même sens, pleine
  puissance) → `motor_control/full_throttle_gps.log`, marqueurs
  `FULL_THROTTLE_START`/`FULL_THROTTLE_END`. Utile pour la vitesse de
  pointe et la dérive en ligne droite.
- **`gps_log_on_full_rotation.py`** — rotation sur place à fond :
  `dutyCycleLeft`/`dutyCycleRight` == +255/-255 ou -255/+255 (sens
  opposés, pleine puissance) → `motor_control/full_rotation_gps.log`,
  marqueurs `FULL_ROTATION_START`/`FULL_ROTATION_END`. Ici c'est le
  **cap** GPS qui est le signal intéressant, pas la position — le robot
  pivote quasiment sur place, sa position GPS ne bouge quasiment pas.

```bash
python3 -m motor_control.gps_log_on_full_throttle   # translation
python3 -m motor_control.gps_log_on_full_rotation   # rotation
```

Dans les deux cas, les trames NMEA brutes sont horodatées et le fichier
de log (déjà ignoré par git, comme tout `*.log`) reste vide (ou ne
contient que des marqueurs) tant que la condition exacte n'a jamais été
atteinte pendant la session — ce n'est pas un bug.

**Non testé sur le robot réel** : comme pour `link/gps_reader.py`,
`pyserial`, `evdev`, `pygame` et `gpiod` n'ont pas pu être installés dans
l'environnement où ces scripts ont été écrits (pas d'accès PyPI). Seules
les deux détections pures et sans matériel (`is_full_throttle()` et
`is_full_rotation()`) sont réellement testées
(`tests/test_gps_log_on_full_throttle.py`,
`tests/test_gps_log_on_full_rotation.py`) ; le reste (lecture série,
intégration avec `Remote`, dans `gps_condition_logger.py`) est écrit
contre les API documentées mais n'a jamais tourné pour de vrai — à
vérifier sur la Pi, manette et récepteur GPS branchés, avant de leur
faire confiance.

## Flux caméra en direct (`camera/`)

Diffuse l'image d'une webcam USB branchée sur cette Pi en MJPEG sur HTTP
(`multipart/x-mixed-replace`), un format que tout navigateur affiche
nativement dans une simple balise `<img>` — pas de plugin, pas de
WebRTC. Le site web (Raspberry Pi n°2) ne charge jamais ce flux
directement : il le relaie depuis sa propre route `/media/camera` (voir le
README de `robot-webserver`), pour qu'il reste derrière le login du site
plutôt que d'être exposé directement sur le réseau local. Sur la page
`/control`, ce flux en direct prend automatiquement la place de la
playlist vidéo dès qu'il est disponible, et on retombe sur la playlist
s'il devient injoignable.

```bash
python3 -m camera                    # écoute sur 0.0.0.0:8000
CAMERA_DEVICE=/dev/video2 CAMERA_PORT=8000 python3 -m camera
```

Variables d'environnement disponibles (toutes optionnelles) :
`CAMERA_DEVICE` (index ou chemin du périphérique vidéo, défaut `0`),
`CAMERA_WIDTH`/`CAMERA_HEIGHT`/`CAMERA_FPS` (résolution et cadence de
capture), `CAMERA_HOST`/`CAMERA_PORT` (interface d'écoute),
`CAMERA_SNAPSHOT_DIR` (dossier des snapshots, voir juste en dessous).

Ce module reste volontairement indépendant du protocole NMEA de `link/` en
tant que processus (deux scripts séparés, lancés indépendamment), mais
`link/robot_state.py` lui parle en HTTP pour `CAM,SNAP` (voir plus haut) :
`camera/` gère à la fois l'aperçu vidéo continu et, depuis peu, les
snapshots à la demande.

### Snapshots (`camera/snapshots.py`)

`GET /snap` sur ce même serveur (port `8000` par défaut) sauvegarde
l'image actuelle dans un dossier tampon (`camera/tmp/` par défaut,
personnalisable avec `CAMERA_SNAPSHOT_DIR`) destiné à un traitement
ultérieur (pipeline de vision, export...), pas à un archivage permanent :
`SnapshotStore` n'y garde jamais plus de **5 fichiers** — sauvegarder un
6e supprime automatiquement le plus ancien. C'est ce que `CAM,SNAP`
déclenche (voir plus haut) ; l'endpoint reste aussi appelable directement
(`curl http://<IP Pi 1>:8000/snap`) pour du test ou un script externe.

**Non testé sur du vrai matériel** : le code a été relu et vérifié
syntaxiquement, et le flux MJPEG lui-même (encodage, multipart, relais par
le site web, bascule automatique sur la page `/control`) a été testé de
bout en bout avec une fausse webcam simulée, tout comme `/snap` et le
plafond à 5 fichiers de `SnapshotStore` (voir `tests/test_link_server.py`
et `tests/test_snapshots.py`). Mais il n'y a pas de vraie webcam ni de
Raspberry Pi dans l'environnement où il a été écrit — à tester avec la
caméra réellement branchée avant de s'y fier.

## Tests

```bash
pytest
```

`pid/pid_controller.py`, `link/nmea.py` et `link/server.py` sont testables
sans matériel (pas de GPIO ni de port série) — `tests/test_link_server.py`
démarre même un vrai serveur TCP sur un port local pour vérifier le
protocole de bout en bout. Les autres modules ouvrent une ressource
matérielle dès leur import et nécessitent la Raspberry Pi pour être
exécutés.

## Dossier `archive/`

Contient les versions précédentes (2025) de certains modules, gardées pour
référence pendant la transition. Objectif à terme : ne garder qu'une seule
version par module et s'appuyer sur l'historique git (branches/tags) plutôt que
sur des noms de fichiers datés.

## Points connus à vérifier

- `gps/gps_delta.py` : la fonction `angle_to_target_radius` référence des
  variables `A` et `B` qui ne sont pas définies dans la fonction (elle prend
  `latA_rad, lonA_rad, latB_rad, lonB_rad` en paramètres mais les réécrit à
  partir de `A["Lat"]` etc.) — la fonction plantera telle quelle. À corriger
  avant utilisation. De même, `distance_to_target_meter(A, B)` utilise des
  variables `latA_rad`/`latB_rad`/`lonA_rad`/`lonB_rad` qui ne sont pas
  calculées dans la fonction elle-même.
- `requirements.txt` a été complété : `keyboard`, `pygnssutils`, `matplotlib`
  et `scipy` étaient utilisés par certains scripts mais absents du fichier
  d'origine.
## Corrections apportées (2026-08-29)

Ces deux problèmes ont été repérés grâce à `pid/simulate_motor_commands.py`
puis corrigés dans `pid_controller.py` :

- **`left_speed`/`right_speed` non bornés** : `move_to_target()` combinait
  `speed_pwm` et `angular_pwm` (chacun jusqu'à ±255) sans reborner la somme,
  qui pouvait donc atteindre ±510 — hors de la plage physique attendue par
  `motor_control/pwm.py`. Un `max(min(...))` a été ajouté après le mixage,
  comme cela existait déjà pour `speed`/`angular_velocity`.
- **"Derivative kick" au premier appel** : `PIDController.previous_error`
  démarrait à `0`, donc au tout premier appel — avec une erreur de départ déjà
  grande (5 m, 45°) — le terme dérivé calculait `(erreur - 0) / dt` sur un
  intervalle très court, créant un pic artificiel énorme (jusqu'à saturer les
  deux moteurs simultanément dès t≈0, avant même le bug ci-dessus). Corrigé en
  initialisant `previous_error` avec la première erreur mesurée au lieu de 0.

## Documentation réseau et architecture

Les schémas réseau (IP des deux Raspberry Pi, chez vous et chez vos parents)
et l'architecture globale du projet (robot ↔ serveur web ↔ routeur) sont tenus
à jour dans un document séparé, en dehors de ce dépôt de code.
