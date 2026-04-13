# Télémétrie réseau (UDP)

Le robot envoie ses données de télémétrie en **JSON via UDP** (port 9870) vers une RPI réceptrice sur le réseau WiFi 5GHz.

## Architecture

```
OPOS6UL (robot)                              RPI (récepteur)
┌──────────────────┐       UDP 9870        ┌──────────────────┐
│ TelemetryAppender│ ──── WiFi 5GHz ────→  │ Réception JSON   │
│ (flush 300ms)    │    192.168.3.101      │                  │
└──────────────────┘                       └──────────────────┘
```

| Paramètre | Valeur |
|---|---|
| Protocole | UDP |
| Port | 9870 |
| IP récepteur (RPI) | `192.168.3.101` |
| Format | JSON Lines (un objet JSON par paquet) |
| Fréquence de flush | 300 ms |

## Format des paquets JSON

Chaque paquet UDP contient un objet JSON avec l'ID du robot, un timestamp, le temps écoulé depuis le démarrage, et les données du logger :

```json
{"OPOS6UL":{"timestamp":1774733396.268,"elapsedtime_ms":10009.463,"LedBar":{"pos":0,"color":6}}}
{"OPOS6UL":{"timestamp":1774733396.288,"elapsedtime_ms":10029.68,"LedBar":{"hex":255,"hexcolor":6}}}
```

| Champ | Description |
|---|---|
| `timestamp` | Epoch Unix en secondes (avec millisecondes) |
| `elapsedtime_ms` | Temps écoulé depuis le démarrage du programme (ms) |
| `<NomLogger>` | Données spécifiques au logger (ex: LedBar, Asserv, Sensors) |

## Réception des trames

### Afficher en direct sur le terminal

Sur la RPI ou tout PC avec l'IP `192.168.3.101` :

```bash
socat -u UDP-RECV:9870,reuseaddr -
```

### Afficher et sauvegarder dans un fichier

```bash
socat -u UDP-RECV:9870,reuseaddr - | tee telemetry.json
```

### Sauvegarder uniquement dans un fichier

```bash
socat -u UDP-RECV:9870,reuseaddr OPEN:./telemetry.json,creat,trunc
```

> **Note** : `socat` doit être installé (`sudo apt install socat`).

## Configuration côté robot

### Ligne de commande

L'IP et le port de la cible télémétrie peuvent être modifiés via la ligne de commande :

```bash
./robot                                    # défaut: 192.168.3.101:9870
./robot /i 192.168.1.50                    # IP custom, port défaut
./robot /p 9871                            # IP défaut, port custom
./robot /i 192.168.1.50 /p 9871           # IP et port custom
```

| Option | Description | Défaut |
|---|---|---|
| `/i ip` | Adresse IP du récepteur télémétrie | `192.168.3.101` |
| `/p port` | Port UDP du récepteur télémétrie | `9870` |

### Code source

La télémétrie est initialisée dans `robot/src/bot-opos6ul/LoggerInitialize.cpp` avec les valeurs par défaut, puis reconfigurée si `/i` ou `/p` sont passés en arguments :

```cpp
// Création de l'appender télémétrie vers la RPI (valeurs par défaut)
add("net", new TelemetryAppender("OPOS6UL", "192.168.3.101", 9870));

// Rootlogger (erreurs) sur la télémétrie + console
add(logs::Level::ERROR, "", "net");

// Brancher un logger sur la télémétrie
add(logs::Level::INFO, "LedBar", "net");
```

### Ajouter un logger à la télémétrie

1. Dans `LoggerInitialize.cpp`, changer l'appender du logger de `"console"` vers `"net"` :
   ```cpp
   add(logs::Level::INFO, "MonLogger", "net");
   ```

2. Dans le code, envoyer des données JSON avec `telemetry()` :
   ```cpp
   nlohmann::json j;
   j["distance"] = 42.5;
   j["angle"] = 90.0;
   logger().telemetry(j.dump());
   ```

> Les messages de niveau `TELEM` sont envoyés en UDP. Les autres niveaux (`INFO`, `DEBUG`, `WARN`) sont affichés sur la console. Les `ERROR` sont envoyés en UDP **et** affichés sur la console.

## Test en simulation locale

Pour tester sur le PC de dev sans la RPI, la VM doit être en mode **bridge** (pas NAT) avec l'IP `192.168.3.101` :

```bash
cd robot/build-simu-release

# Terminal 1 : réception
socat -u UDP-RECV:9870,reuseaddr - | tee telemetry.json

# Terminal 2 : lancer le test LED
./bot-opos6ul t /n 1
```

## Réseau

Le robot et la RPI sont sur le réseau WiFi 5GHz AP :

| Appareil | IP |
|---|---|
| OPOS6UL (WiFi 5GHz AP) | `192.168.3.103` |
| RPI (récepteur télémétrie) | `192.168.3.101` |
| VM dev (bridge) | `192.168.3.x` |

## Robustesse

- Chaque paquet UDP est **atomique** : envoyé en entier ou pas du tout
- Format **JSON Lines** : chaque ligne est un JSON indépendant, pas de structure globale à corrompre
- Si le robot crashe : les trames déjà envoyées sont valides, au pire on perd les messages en attente (max 300 ms de buffer)
- Si le récepteur n'est pas joignable : les `sendto` échouent silencieusement, pas d'impact sur le robot
