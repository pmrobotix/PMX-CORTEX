# PMX-CORTEX

Robot de l'équipe **PM-ROBOTIX** pour la **Coupe de France de Robotique 2026**.

## Architecture matérielle

| Carte | Processeur | Rôle | Environnement |
|---|---|---|---|
| OPOS6UL (Armadeus Systems) | NXP i.MX6ULL — ARM Cortex-A7 @ 900MHz | Cerveau principal (stratégie, navigation, décision) | C++17, CMake, Buildroot, cross-compilation ARM |
| Teensy 4.1 | NXP i.MX RT1062 — ARM Cortex-M7 @ 600MHz | Asservissement moteurs et capteurs temps réel | PlatformIO / Arduino (C/C++) |
| micro:bit | ARM Cortex-M0 | PAMIs (petits robots autonomes) | MakeCode (makecode.microbit.org) |

## Structure du projet

```
PMX-CORTEX/
├── brain/                    # Code OPOS6UL (cerveau Linux)
│   └── CMakeLists.txt
├── teensy/                   # Projets PlatformIO Teensy 4.1
│   └── motor-control/        # Asservissement moteurs
│       ├── platformio.ini
│       └── src/main.cpp
├── pamis/                    # PAMIs micro:bit
│   └── pami-01/
├── docs/                     # Documentation
└── CLAUDE.md                 # Ce fichier
```

Chaque sous-dossier dans `teensy/` est un projet PlatformIO indépendant.
Chaque sous-dossier dans `pamis/` est un PAMI indépendant.

## Communication entre cartes

- OPOS6UL ↔ Teensy 4.1 : liaison série (UART) ou USB (à définir)
- Les PAMIs (micro:bit) sont des robots indépendants du robot principal

## Toolchain OPOS6UL

- BSP : Armadeus 7.0 (Buildroot)
- Toolchain : `arm-none-linux-gnueabihf-gcc` (ARM A-profile 10.3)
- Noyau : Linux 5.10.167
- Chemin toolchain : `~/armadeus-7.0/buildroot/output/host/usr/bin/`
- Device tree : `imx6ul-opos6uldev.dts`

## Teensy 4.1

- Framework : Arduino via PlatformIO
- Upload : `teensy-cli`
- Fonctions principales : asservissement PID moteurs, lecture encodeurs, capteurs rapides

## Conventions de code

- C++17 pour brain/ et teensy/
- Style : noms de classes en PascalCase, méthodes en camelCase, constantes en UPPER_SNAKE_CASE
- Chaque module a un .h et un .cpp
- Pas de `using namespace std;` dans les headers
- Commenter en français ou anglais (cohérent par fichier)

## Environnement de développement

- OS : Kubuntu 22.04 LTS (VM VMware)
- IDE : VSCode avec extensions C/C++, PlatformIO, Claude Code, GitLens
- IA : Claude Code CLI + extension VSCode (compte Claude Max)
- Repo : https://github.com/pmrobotix/PMX-CORTEX (public)
- Ancien projet (référence) : https://github.com/pmrobotix/PMX

## Contexte compétition

- Coupe de France de Robotique 2026 (mai 2026)
- Le robot doit : se déplacer de façon autonome, éviter les obstacles
- Deadline serrée : chaque suggestion doit être pragmatique et fonctionnelle
- Privilégier la fiabilité à l'élégance

## Règles pour Claude Code

- Toujours fournir un résumé des modifications avec explications avant de coder — Présenter ce qui va être changé, pourquoi, et dans quels fichiers.
- Avant de coder, vérifier la cohérence avec l'architecture existante en consultant les fichiers de référence (voir ci-dessous). Si une modification impacte l'architecture, le signaler et attendre validation.
- Toujours demander confirmation avant de modifier des fichiers existants
- Ne jamais modifier la toolchain Armadeus ou le BSP Buildroot
- Proposer des solutions simples et testables
- Expliquer les choix d'architecture quand c'est pertinent
- Pas d'hallucination : si tu ne connais pas un détail hardware, dis-le

## Fichiers de référence (robot/)

Consulter ces fichiers avant toute modification pour vérifier la cohérence architecturale :

| Fichier | Contenu |
|---|---|
| `robot/ARCHITECTURE.md` | Architecture globale, threads, timers, structure des dossiers, plan de migration AAsservDriver |
| `robot/HARDWARE_CONFIG.md` | Configuration hardware dynamique (activation/désactivation drivers) |
| `robot/ASSERV_MIGRATION_COMMUNICATION.md` | Migration communication série ancien raspIO → SerialIO/RaspIO |
| `robot/SENSORS_DETECTION_MIGRATION.md` | Refactoring Sensors/ObstacleZone, beacon, détection, SVG |
| `robot/ASSERV_BUG_GLITCH_I2C.md` | Bug glitch I2C asserv |
| `robot/BUILD.md` | Instructions de build |
| `robot/config/opos6ul/FLASH-OPOS6UL.md` | Procédure flash OPOS6UL |
| `robot/config/opos6ul/CONFIG-STATUS.md` | Status configuration hardware |
