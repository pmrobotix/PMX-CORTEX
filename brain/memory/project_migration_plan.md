---
name: Plan de migration brain PMX → PMX-CORTEX
description: Migration progressive du code robot depuis l'ancien projet PMX (Eclipse) vers PMX-CORTEX (CMake). Architecture documentée dans brain/ARCHITECTURE.md.
type: project
---

Migration du code robot de l'ancien projet PMX (Eclipse, /home/pmx/git/PMX/) vers PMX-CORTEX (CMake, brain/).

**Why:** Préparer le robot pour la Coupe de France 2026, passer d'Eclipse à CMake/VSCode, et améliorer l'architecture (AAsservDriver, détection obstacles).

**How to apply:** L'architecture cible est documentée dans brain/ARCHITECTURE.md. Migration progressive, phase par phase. Le code existant est migré tel quel d'abord, les refactorings (AAsserv/AMotorDriver, ObstacleDetector, isOnPath) viendront après.

Points clés discutés :
- AAsservDriver (47 méthodes) → à terme AAsserv (19) + AMotorDriver (14) + suppression code mort
- SensorsTimer mélange lecture/filtrage/décision → séparer en couches
- Détection obstacles : zones robot (sécurité) + vérification trajectoire (reprise) — piste isOnPath
- Carte ST asserv externe (asserv_chibios) : retour CBOR toutes les 100ms, manque motion_phase mais isOnPath pourrait rendre ça inutile
- Le robot OPOS6UL est stable d'année en année, seuls les states/stratégie changent
