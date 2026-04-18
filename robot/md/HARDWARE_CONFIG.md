# Migration : Configuration hardware dynamique

## Statut implémentation

| Composant | Statut |
|---|---|
| Classe `HardwareConfig` singleton (`common/HardwareConfig.hpp/cpp`) | ✅ fait |
| Fichier `hardware.conf` (à côté de l'exécutable) | ✅ fait |
| `Main.cpp` : `HardwareConfig::instance().load(argv[0])` | ✅ fait |
| Factory `create()` — LedDriver | ✅ fait |
| Factory `create()` — LcdShieldDriver | ✅ fait |
| Factory `create()` — ButtonDriver | ✅ fait |
| Factory `create()` — SwitchDriver | ✅ fait |
| Factory `create()` — SensorsDriver | ✅ fait |
| Factory `create()` — ServoDriver | ✅ fait |
| Factory `create()` — AsservCborDriver | ✅ fait |
| Factory `create()` — ColorDriver (ajout hors spec) | ✅ fait |
| Log au démarrage (`HardwareConfig: XxxDriver=ON/OFF`) | ✅ fait |
| Lib `pmx-driver-stub` séparée (build SIMU pur) | ⬜ pas fait |
| `DriverFactories.cpp` regroupant les `create()` SIMU | ⬜ pas fait |

## Problème

Au démarrage, `OPOS6UL_ActionsExtended` et `OPOS6UL_AsservExtended` initialisent **tous** les drivers matériels (I2C LCD, capteurs, servos, liaison série asserv...) dans leur constructeur. Si un périphérique n'est pas branché, on obtient des erreurs bloquantes (ex: `Can't write on i2c`).

Pendant l'intégration progressive du robot, on a besoin de tester chaque composant individuellement sans que les autres drivers échouent.

## Solution : fichier `hardware.conf`

Un fichier texte placé **à côté de l'exécutable** qui déclare quels drivers sont actifs. Les drivers désactivés utilisent le **driver simu (stub)** déjà existant.

### Format du fichier

```ini
# hardware.conf — Configuration des drivers matériels
# 1 = driver ARM réel, 0 = driver simu (stub)

# Actions
LedDriver=1
LcdShieldDriver=0
ButtonDriver=0
SwitchDriver=0
SensorsDriver=0
ServoDriver=0

# Asservissement
AsservDriver=0
```

En match, tout passe à `1`. Pendant l'intégration, on active un par un.

### Fichier absent = tout activé

Si `hardware.conf` n'existe pas à côté de l'exe, **tous les drivers sont à 1** (comportement actuel inchangé). Aucune régression en match.

## Impacts sur le code

### 1. Nouvelle classe `HardwareConfig` (common/)

```
src/common/HardwareConfig.hpp
src/common/HardwareConfig.cpp
```

- Lit `hardware.conf` dans le répertoire de l'exécutable (`argv[0]` ou `/proc/self/exe`)
- Singleton, chargé une seule fois au démarrage
- API simple :

```cpp
class HardwareConfig {
public:
    static HardwareConfig& instance();
    void load(const std::string& exePath);  // appelé depuis main()
    bool isEnabled(const std::string& driverName) const;  // ex: "LcdShieldDriver"
private:
    std::map<std::string, bool> drivers_;
    bool loaded_ = false;  // si false, tout est enabled
};
```

### 2. Modification des factory `create()` (7 fichiers)

Chaque interface abstraite a une méthode `static create()` avec **deux implémentations** :
- `src/driver-arm/XxxDriver.cpp` (ARM réel)
- `src/driver-simu/XxxDriver.cpp` (stub)

Actuellement, la sélection est faite **à la compilation** (ARM vs SIMU).

**Changement** : sur la cible ARM, la factory consulte `HardwareConfig` pour décider si elle instancie le driver ARM réel ou le driver simu.

| Fichier factory (driver-arm/) | Nom dans hardware.conf |
|---|---|
| `LedDriver.cpp` → `ALedDriver::create()` | `LedDriver` |
| `LcdShieldDriver.cpp` → `ALcdShieldDriver::create()` | `LcdShieldDriver` |
| `ButtonDriver.cpp` → `AButtonDriver::create()` | `ButtonDriver` |
| `SwitchDriver.cpp` → `ASwitchDriver::create()` | `SwitchDriver` |
| `SensorsDriver.cpp` → `ASensorsDriver::create()` | `SensorsDriver` |
| `ServoDriver.cpp` → `AServoDriver::create()` | `ServoDriver` |
| `AsservDriver.cpp` → `AAsservDriver::create()` | `AsservDriver` |

#### Exemple complet : LcdShieldDriver (celui qui cause l'erreur I2C)

**Problème** : actuellement les classes ARM et SIMU s'appellent toutes les deux `LcdShieldDriver`. On ne peut pas les lier ensemble dans le même binaire.

**Étape A — Renommer la classe simu**

`src/driver-simu/LcdShieldDriver.hpp` :

```cpp
// AVANT
class LcdShieldDriver: public ALcdShieldDriver

// APRÈS
class LcdShieldDriverSimu: public ALcdShieldDriver
```

`src/driver-simu/LcdShieldDriver.cpp` — renommer constructeur, destructeur et toutes les méthodes :

```cpp
// AVANT
ALcdShieldDriver * ALcdShieldDriver::create(std::string botId)
{
    return new LcdShieldDriver();
}
LcdShieldDriver::LcdShieldDriver() { }
LcdShieldDriver::~LcdShieldDriver() { }
bool LcdShieldDriver::is_connected() { return true; }
// ...

// APRÈS — on supprime create() d'ici (c'est driver-arm qui fait le routing)
LcdShieldDriverSimu::LcdShieldDriverSimu() { }
LcdShieldDriverSimu::~LcdShieldDriverSimu() { }
bool LcdShieldDriverSimu::is_connected() { return true; }
// ...
```

**Étape B — Modifier la factory ARM**

`src/driver-arm/LcdShieldDriver.cpp` — seul le `create()` change :

```cpp
// AVANT
#include "LcdShieldDriver.hpp"

ALcdShieldDriver * ALcdShieldDriver::create(std::string botId)
{
    return new LcdShieldDriver();
}
// ... le reste (constructeur, clear(), write(), etc.) ne change PAS

// APRÈS
#include "LcdShieldDriver.hpp"
#include "HardwareConfig.hpp"
#include "../driver-simu/LcdShieldDriver.hpp"  // LcdShieldDriverSimu

ALcdShieldDriver * ALcdShieldDriver::create(std::string botId)
{
    if (!HardwareConfig::instance().isEnabled("LcdShieldDriver")) {
        return new LcdShieldDriverSimu();  // stub, pas d'I2C
    }
    return new LcdShieldDriver();  // vrai hardware
}
// ... le reste ne change PAS
```

**Étape C — Ajouter un `create()` pour le build SIMU pur**

En mode SIMU on ne compile pas `driver-arm/`, il faut un `create()` dédié.

`src/driver-simu/DriverFactories.cpp` (nouveau fichier, regroupe tous les `create()` simu) :

```cpp
#include "interface/ALcdShieldDriver.hpp"
#include "LcdShieldDriver.hpp"  // LcdShieldDriverSimu

ALcdShieldDriver * ALcdShieldDriver::create(std::string botId)
{
    return new LcdShieldDriverSimu();
}
// ... idem pour les 6 autres drivers
```

**Flux résumé :**

```
hardware.conf: LcdShieldDriver=0
        │
        ▼
ALcdShieldDriver::create()          ← driver-arm/LcdShieldDriver.cpp
        │
        ├─ isEnabled? OUI → new LcdShieldDriver()         → I2C réel
        │
        └─ isEnabled? NON → new LcdShieldDriverSimu()     → stub (log only)
```

Le reste du code (`LcdShield`, `OPOS6UL_ActionsExtended`) ne voit rien, il manipule toujours un `ALcdShieldDriver*`. C'est le même pattern pour les 7 drivers.

### 3. Compilation : lier driver-simu dans la cible ARM

Actuellement dans `CMakeLists.txt`, `pmx-driver-arm` et `pmx-driver-simu` sont des libs séparées, mutuellement exclusives. Il faut que la cible ARM puisse aussi accéder aux classes simu (stubs).

**Option retenue** : extraire les stubs simu dans une lib `pmx-driver-stub` toujours compilée, liée en ARM et en SIMU.

```cmake
# Stubs toujours disponibles (fallback quand hardware désactivé)
add_library(pmx-driver-stub STATIC
    src/driver-simu/LedDriver.cpp
    src/driver-simu/LcdShieldDriver.cpp
    src/driver-simu/ButtonDriver.cpp
    src/driver-simu/SwitchDriver.cpp
    src/driver-simu/SensorsDriver.cpp
    src/driver-simu/ServoDriver.cpp
    src/driver-simu/AsservDriver.cpp
)

if(TARGET_PLATFORM STREQUAL "ARM")
    target_link_libraries(bot-opos6ul ... pmx-driver-arm pmx-driver-stub)
endif()
```

**Attention** : les `create()` sont actuellement définis **à la fois** dans driver-arm et driver-simu (symbol duplicate). Il faudra :
- Supprimer les `create()` de driver-simu
- Garder uniquement les `create()` dans driver-arm (qui font le routing)
- En mode SIMU pur, ajouter un fichier `create()` dédié dans driver-simu qui instancie toujours le stub

### 4. Modification de `Main.cpp`

Charger la config avant la création du robot :

```cpp
int main(int argc, char** argv)
{
    // Charger la config hardware depuis le répertoire de l'exécutable
    HardwareConfig::instance().load(argv[0]);

    mlockall(MCL_CURRENT | MCL_FUTURE);
    // ... suite inchangée
}
```

### 5. Log au démarrage

`HardwareConfig::load()` logge l'état de chaque driver :

```
[INFO] HardwareConfig: LedDriver=ON
[INFO] HardwareConfig: LcdShieldDriver=OFF (stub)
[INFO] HardwareConfig: AsservDriver=OFF (stub)
```

## Fichiers impactés (résumé)

| Action | Fichiers |
|---|---|
| **Créer** | `src/common/HardwareConfig.hpp`, `src/common/HardwareConfig.cpp`, `hardware.conf` (défaut) |
| **Modifier** | 7 factory `create()` dans `src/driver-arm/` |
| **Modifier** | 7 factory `create()` dans `src/driver-simu/` (supprimer les `create()`) |
| **Modifier** | `CMakeLists.txt` (lib stub + linkage) |
| **Modifier** | `src/bot-opos6ul/Main.cpp` (chargement config) |
| **Modifier** | `test/driver/Main.cpp` (chargement config) |
| **Modifier** | `test/manual/Main.cpp` (chargement config) |

## Exécutables concernés

| Exécutable | Lié aux drivers | `hardware.conf` | Remarque |
|---|---|---|---|
| **bot-opos6ul** | oui (`${DRIVER_LIB}`) | **Oui** | Cas principal : robot complet |
| **driver-test** | oui (`${DRIVER_LIB}`) | **Oui** | Tests unitaires drivers, active au cas par cas |
| **manual-test** | oui (`${DRIVER_LIB}`) | **Oui** | Tests manuels hardware, même besoin |
| **common-test** | non (`pmx-suite` seul) | **Non** | Pas de drivers, que du common |
| **bench** | non (`pmx-suite` seul) | **Non** | Performance pure (timers POSIX), pas de hardware |

Les 3 exécutables concernés bénéficient **automatiquement** du mécanisme car le routing se fait dans les factory `create()`. Le seul ajout par exécutable est l'appel `HardwareConfig::instance().load(argv[0])` dans leur `Main.cpp`.

Chaque exécutable peut avoir son propre `hardware.conf` à côté de lui, adapté à ce qu'il teste. Par exemple, `driver-test` avec uniquement `LedDriver=1` pour tester les LEDs isolément.

## Cas d'usage

| Situation | hardware.conf |
|---|---|
| Test LED seule | `LedDriver=1`, tout le reste à `0` |
| Test LED + LCD | `LedDriver=1`, `LcdShieldDriver=1`, reste à `0` |
| Intégration complète | Tout à `1` |
| Match (fichier absent) | Tout à `1` par défaut |

## Risques et points d'attention

1. **Symbol duplicates** : la réorganisation des `create()` entre driver-arm et driver-simu nécessite de bien séparer les responsabilités pour éviter des erreurs de linkage
2. **Headers driver-simu** : les classes simu doivent être accessibles depuis driver-arm (include path)
3. **Pas de changement en SIMU** : le build SIMU continue de ne lier que les stubs, pas de régression
4. **Thread safety** : `HardwareConfig::load()` est appelé une seule fois avant toute création de thread, pas de problème de concurrence
5. **`OPOS6UL_ActionsExtended`** : les membres sont des objets directs (pas des pointeurs), donc le driver simu est instancié à l'intérieur de chaque objet via la factory — **aucun changement nécessaire** dans cette classe
