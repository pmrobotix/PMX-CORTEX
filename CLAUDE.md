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

- **Avant toute action de code** : (a) résumer ce qui sera changé, pourquoi, dans quels fichiers ; (b) si la tâche touche >1 fichier → préparer un brief subagent et **demander confirmation avant délégation** ; (c) sinon, attendre validation avant édition directe.
- Avant de coder, vérifier la cohérence avec l'architecture existante en consultant les fichiers de référence (voir ci-dessous). Si une modification impacte l'architecture, le signaler et attendre validation.
- Toujours demander confirmation avant de modifier des fichiers existants
- Ne jamais modifier la toolchain Armadeus ou le BSP Buildroot
- Proposer des solutions simples et testables
- Expliquer les choix d'architecture quand c'est pertinent
- Avant chaque commit demandé par l'utilisateur, proposer le message de commit avec un résumé des modifications et attendre sa validation avant de commiter

## Anti-hallucination — règles strictes

### Vérifier avant d'affirmer
- Avant de mentionner un fichier, une fonction, un champ struct, une option CLI, un flag CMake : vérifier son existence (Read/Grep). Pas de noms "plausibles".
- Avant de citer une signature, un type, une valeur de constante, un chemin : la lire dans le code, ne pas la deviner.
- Avant de proposer une API d'une lib externe : vérifier la version utilisée (CMakeLists.txt, platformio.ini, submodules) et la doc/code de cette version exacte.
- Pour tout détail hardware (registres, broches, timings, adresses I2C, datasheets) : citer la source ou marquer explicitement "à vérifier".
- Les mémoires (auto memory) sont des snapshots datés : toujours vérifier qu'un fait mémorisé est encore vrai dans le code actuel avant de l'utiliser.

### Formulations à bannir sans vérification préalable
- "Je pense que…", "normalement…", "ça devrait…" → vérifier puis affirmer, sinon dire "je ne sais pas".
- "X existe / fait Y" sans avoir lu X → dire "je vais vérifier" et le faire.
- Inventer un nom de fonction/fichier/flag plausible → toujours grep avant de le citer.

### Quand admettre l'incertitude
- Préférer "je ne sais pas, je vérifie" à une affirmation plausible mais non contrôlée.
- Si vérification impossible rapidement : marquer "[à confirmer]" dans la réponse.
- Pour les conventions floues : citer le fichier de référence (`robot/md/…`) ou demander.

## Pattern orchestrateur — Claude principal = architecte

L'IA principale (cette conversation) reste **architecte** du projet PMX-CORTEX.
Les tâches d'implémentation sont **déléguées à des subagents** (outil Agent) avec un contexte propre, non pollué par l'historique de conversation.

### Confirmation obligatoire avant délégation
**Avant CHAQUE délégation à un subagent, présenter à l'utilisateur :**
1. L'objectif que recevra le subagent.
2. Les fichiers de référence à lire et fichiers de code à toucher.
3. Le format de retour attendu.

**Attendre validation explicite** avant de lancer l'outil Agent. Pas d'exception, même pour une tâche "évidente".

### Rôle de l'IA principale (ne jamais déléguer)
- Vision d'ensemble : architecture, conventions, cohérence inter-modules (brain/teensy/pamis).
- Planification : découpage des tâches, identification des fichiers impactés, effets de bord.
- Brief des subagents : objectif borné, contexte, fichiers de référence à lire en priorité, conventions, format de retour attendu.
- Review du retour subagent : valider la cohérence avec l'architecture, intégrer, corriger.
- Décisions techniques structurantes et trade-offs.
- Communication avec l'utilisateur : présentation, choix, confirmations, message de commit.

### À déléguer systématiquement à un subagent
- Implémentation d'une feature touchant **>1 fichier**.
- Refactoring multi-fichiers (mécanique, transversal).
- Recherche/exploration nécessitant >3 grep/find (`subagent_type: Explore`).
- Debug d'un module bien isolé.
- Audit (sécurité, perf, dead code, doublons).

### À NE PAS déléguer
- Petites éditions (1 seul fichier connu).
- Décisions d'architecture, validation de cohérence globale.
- Préparation et validation d'un message de commit.
- Toute interaction directe avec l'utilisateur.

### Brief type d'un subagent (toujours inclure)
1. **Objectif** : précis et borné (ce qui doit être fait, pas plus).
2. **Contexte** : pourquoi cette tâche, dans quelle initiative elle s'inscrit.
3. **Lectures préalables** : fichiers `robot/md/*.md` pertinents + fichiers de code à lire en premier (chemins absolus).
4. **Conventions** : pointer vers la section appropriée du CLAUDE.md (ex: liens absolus, style C++17, args CLI préfixe `/`).
5. **Hors scope** : ce que le subagent ne doit PAS faire.
6. **Format de retour** : court, factuel, avec chemins/lignes vérifiés (`file_path:line_number`).
7. **Anti-hallucination** : rappeler la règle "vérifier avant d'affirmer".

## Liens cliquables dans le chat

Le workspace est multi-root (`pmx.code-workspace`). Les chemins relatifs dans les liens markdown `[texte](chemin)` ne sont pas toujours résolus correctement par VSCode selon le dossier actif. **Toujours utiliser des chemins absolus** pour que les liens soient cliquables :

- `/home/pmx/git/PMX-CORTEX/...` pour les fichiers du dépôt principal (robot/, brain/, simulator/, libs/, etc.)
- `/home/pmx/git/asserv_chibios/...`, `/home/pmx/git/plotjuggler_asservstream_plugin/...`, `/home/pmx/git/PMX-CORTEX.wiki/...` pour les dépôts externes référencés dans le workspace

Pour les lignes précises, utiliser `#L<n>` ou `#L<n>-L<m>` (ex: `.../fichier.cpp#L42-L51`).

## Fichiers de référence (robot/)

Consulter ces fichiers avant toute modification pour vérifier la cohérence architecturale :

| Fichier | Contenu |
|---|---|
| `robot/md/ARCHITECTURE.md` | Architecture globale, threads, timers, structure des dossiers, plan de migration AAsservDriver |
| `robot/md/HARDWARE_CONFIG.md` | Configuration hardware dynamique (activation/désactivation drivers) |
| `robot/md/ASSERV_MIGRATION_COMMUNICATION.md` | Migration communication série ancien raspIO → SerialIO/RaspIO |
| `robot/md/SENSORS_DETECTION_MIGRATION.md` | Refactoring Sensors/ObstacleZone, beacon, détection, SVG |
| `robot/md/DETECTION_ADV_CONVENTION.md` | Convention canonique x=avant/y=gauche partagée balise/simu/filtre/tests |
| `robot/md/ASSERV_BUG_GLITCH_I2C.md` | Bug glitch I2C asserv |
| `robot/md/O_STATE_NEW_INIT.md` | Refactor O_State_Init multi-sources (shield LCD2x16 + LCD tactile balise) avec phase machine |
| `robot/md/STRATEGY_DECISION_RUNNER.md` | Évolution runner : skip+continue au lieu d'abort, sémantique TRAJ_STATE, variables JSON pour homologation |
| `robot/md/PUSH_ELEMENTS_2026.md` | Manipulation `push_elements_P{N}` : pousse les éléments selon config beacon LCD tactile, tables D1..D4 |
| `robot/md/BUILD.md` | Instructions de build |
| `robot/config/opos6ul/FLASH-OPOS6UL.md` | Procédure flash OPOS6UL |
| `robot/config/opos6ul/CONFIG-STATUS.md` | Status configuration hardware |
