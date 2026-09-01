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
│   └── remote_control.py
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
│   ├── robot_state.py  # Validation et état (PWM, mode, cible GPS, gains PID)
│   └── server.py        # Serveur TCP (socketserver), aucune dépendance externe
├── camera/             # Flux caméra en direct (voir plus bas), indépendant de link/
│   └── stream_server.py
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
- `CAM` répond systématiquement `ERR` (`CAM_NOT_IMPLEMENTED`) — pas de code
  caméra dans le projet à ce jour.
- `STA` (sans champ, en requête) répond avec l'état courant ; `lat`/`lon`/
  `cap`/`batterie` restent à 0 tant qu'ils ne sont pas branchés sur une
  vraie source.

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
capture), `CAMERA_HOST`/`CAMERA_PORT` (interface d'écoute).

Ce module est volontairement indépendant du protocole NMEA de `link/` : le
type de trame `CAM` (SNAP/REC_START/REC_STOP) reste une fonctionnalité
séparée, non implémentée à ce jour — `camera/` fournit uniquement
l'aperçu vidéo continu, tant que le script tourne.

**Non testé sur du vrai matériel** : le code a été relu et vérifié
syntaxiquement, et le flux MJPEG lui-même (encodage, multipart, relais par
le site web, bascule automatique sur la page `/control`) a été testé de
bout en bout avec une fausse webcam simulée. Mais il n'y a pas de vraie
webcam ni de Raspberry Pi dans l'environnement où il a été écrit — à
tester avec la caméra réellement branchée avant de s'y fier.

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
