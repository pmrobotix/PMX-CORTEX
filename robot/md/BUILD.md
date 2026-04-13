# Guide de build — PMX-CORTEX / robot

## Prérequis

| Outil | Installation | Usage |
|---|---|---|
| CMake ≥ 3.21 | `sudo apt install cmake` | Build system |
| GCC / G++ | `sudo apt install build-essential` | Compilateur C++17 |
| Doxygen | `sudo apt install doxygen graphviz` | Génération de doc (optionnel) |
| Toolchain ARM | BSP Armadeus 7.0 | Cross-compilation OPOS6UL (optionnel) |

## Cloner le projet

Le projet utilise des **git submodules** pour les bibliothèques externes (`libs/`).

```bash
# Premier clone (récupère tout, y compris les libs)
git clone --recursive git@github.com:pmrobotix/PMX-CORTEX.git

# Si déjà cloné sans --recursive
git submodule update --init --recursive
```

Les submodules sont :

| Submodule | Chemin | Description |
|---|---|---|
| `simple-svg` | `libs/simple-svg/` | Génération de fichiers SVG |
| `PathFinding` | `libs/PathFinding/` | Algorithme de pathfinding (pmr-pathfinding) |

## Commandes rapides

```bash
cd robot/

# Configurer + compiler + tester (SIMU)
cmake --preset simu-debug
cmake --build build-simu-debug --target common-test
./build-simu-debug/common-test
```

## Presets CMake

Tous les presets sont définis dans `CMakePresets.json`.

### Configuration

| Commande | Plateforme | Mode | Dossier de build |
|---|---|---|---|
| `cmake --preset simu-debug` | PC (simulation) | Debug | `build-simu-debug/` |
| `cmake --preset simu-release` | PC (simulation) | Release | `build-simu-release/` |
| `cmake --preset arm-debug` | OPOS6UL (ARM) | Debug | `build-arm-debug/` |
| `cmake --preset arm-release` | OPOS6UL (ARM) | Release | `build-arm-release/` |

### Compilation

```bash
# Tout compiler (toutes les targets)
cmake --build build-simu-debug

# Compiler une target spécifique
cmake --build build-simu-debug --target common-test
cmake --build build-simu-debug --target pmx-common
```

## Targets disponibles

| Target | Type | Description |
|---|---|---|
| `pmx-common` | Bibliothèque (STATIC) | Briques de base : thread, utils, timer, log |
| `pmx-suite` | Bibliothèque (STATIC) | Framework de test unitaire maison |
| `simple-svg` | Bibliothèque (STATIC) | Génération de fichiers SVG (submodule) |
| `pmr-pathfinding` | Bibliothèque (STATIC) | Algorithme de pathfinding (submodule) |
| `common-test` | Exécutable | Tests unitaires du code commun |

Futures targets (à venir) :

| Target | Type | Description |
|---|---|---|
| `pmx-driver-arm` | Bibliothèque | Drivers hardware ARM OPOS6UL |
| `pmx-driver-simu` | Bibliothèque | Drivers simulés (PC) |
| `driver-test` | Exécutable | Tests unitaires des drivers |
| `bench` | Exécutable | Benchmarks (pas d'assertions) |
| `manual-test` | Exécutable | Tests manuels hardware (sur la carte) |

## Lancer les tests

```bash
# Tests unitaires common (sur PC)
./build-simu-debug/common-test

# Résultat attendu :
# Start Unit tests
#    Début d'éxecution de <MutexTest>
#    Fin d'éxecution de <MutexTest>
#    ...
# End of Unit tests
```

Un test qui échoue affiche un message `ERROR` avec le nom du test et la raison.

## Générer la documentation

```bash
# Générer la doc HTML
cd robot/
doxygen

# Ouvrir dans le navigateur
xdg-open docs/html/index.html
```

La doc est générée dans `robot/docs/html/`. Elle inclut :
- Diagrammes de classes (héritage)
- Graphes d'appels de fonctions
- Documentation de toutes les classes et méthodes

Dans VSCode : `Ctrl+Shift+P` → **Tasks: Run Task** → **Doxygen: Générer la doc**

## Cross-compilation ARM

La cross-compilation nécessite le BSP Armadeus 7.0 installé.

```bash
# Configurer pour ARM
cmake --preset arm-debug

# Compiler
cmake --build build-arm-debug --target common-test

# Le binaire est dans build-arm-debug/common-test
# À copier sur la carte OPOS6UL via scp :
scp build-arm-debug/common-test root@<ip-opos6ul>:/root/
```

## Utilisation dans VSCode

1. Ouvrir le workspace `pmx.code-workspace`
2. Sélectionner le dossier actif : `Ctrl+Shift+P` → **CMake: Select Active Folder** → `robot`
3. Sélectionner le preset : barre du bas → cliquer sur le preset → choisir `SIMU Debug`
4. Compiler : `F7` ou `Ctrl+Shift+B`
5. Lancer les tests : `Ctrl+Shift+P` → **CMake: Run Without Debugging**

## Structure du projet

```
PMX-CORTEX/
├── libs/                        # Bibliothèques externes (git submodules)
│   ├── simple-svg/              #   Génération SVG
│   └── PathFinding/             #   Algorithme de pathfinding
├── robot/
│   ├── CMakeLists.txt           # Build unique (inclut les libs via add_subdirectory)
│   ├── CMakePresets.json        # 4 presets (simu/arm × debug/release)
│   ├── Doxyfile                 # Config Doxygen
│   ├── cmake/
│   │   └── toolchain-arm-opos6ul.cmake
│   ├── src/
│   │   └── common/              # Code source (pmx-common)
│   │       ├── thread/          #   Thread, Mutex
│   │       ├── utils/           #   Chronometer, PointerList, json.hpp
│   │       ├── timer/           #   ITimerPosixListener, ITimerListener
│   │       └── log/             #   Logger, LoggerFactory, Appenders
│   ├── test/
│   │   ├── suite/               # Framework de test (pmx-suite)
│   │   └── common/              # Tests unitaires (common-test)
│   └── docs/                    # Doc générée par Doxygen (gitignored)
└── pmx.code-workspace           # Workspace VSCode
```

## Lier les libs dans votre code

Pour utiliser `simple-svg` ou `pmr-pathfinding` dans une target :

```cmake
target_link_libraries(ma-cible PRIVATE simple-svg pmr-pathfinding)
```

Les headers sont automatiquement accessibles grâce aux `target_include_directories PUBLIC` des libs :
- `#include "simple_svg_1.0.0.hpp"`
- `#include "pmr_pathfinding.h"`
