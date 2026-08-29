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
│   └── pid_plot.py     # Simulation/visualisation hors robot
├── gps/                # Lecture GPS, correction DGPS (NTRIP), calculs géo
│   ├── gps_parse.py
│   ├── gps_transfer.py
│   ├── dgps_transfer.py
│   └── gps_delta.py
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

## Tests

```bash
pytest
```

Seul `pid/pid_controller.py` est actuellement testable sans matériel (pas de
GPIO ni de port série). Les autres modules ouvrent une ressource matérielle dès
leur import et nécessitent la Raspberry Pi pour être exécutés.

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


