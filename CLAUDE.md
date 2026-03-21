# PMX-CORTEX

Robot de l'équipe **PM-ROBOTIX** pour la **Coupe de France de Robotique 2026**.

## Architecture matérielle

| Carte | Processeur | Rôle | Environnement |
|---|---|---|---|
| OPOS6UL (Armadeus Systems) | ARM Cortex-A7, Linux Buildroot | Cerveau principal du robot (stratégie, navigation, communication) | C/C++, Buildroot |
| Teensy 4.1 | ARM Cortex-M7 | Asservissement moteurs et capteurs temps réel | PlatformIO / Arduino (C/C++) |
| micro:bit | ARM Cortex-M0 | PAMIs (petits robots autonomes) | MakeCode (makecode.microbit.org) |

## Communication

- OPOS6UL ↔ Teensy 4.1 : liaison série / USB
- Les PAMIs (micro:bit) sont des robots indépendants

## Conventions

- Le code embarqué Teensy utilise le framework Arduino via PlatformIO
- Le code OPOS6UL cible un système Linux Buildroot (cross-compilation ARM)
- Les PAMIs sont développés via l'éditeur en ligne MakeCode
