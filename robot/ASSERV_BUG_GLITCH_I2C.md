# Bugs de boot : Glitches I2C (MD22) et UART (SerialIO)

**Date** : avril 2026
**Projet** : asserv_chibios (PMX — PM-ROBOTIX)
**Cartes concernées** : Nucleo F446-RE (STM32) + MD22 (Robot Electronics, 24V 5A dual H-bridge) + OPOS6UL (Armadeus, UART)

---

## Symptômes

### Symptôme 1 — Moteurs à fond au boot (I2C)

Au démarrage du robot, les moteurs se mettent à tourner **à pleine puissance pendant 2 à 4 secondes**, puis s'arrêtent.
Le comportement est **non systématique** mais très fréquent.
C'est dangereux : le robot peut tomber de la table ou tirer sur le câble USB.

### Symptôme 2 — Mouvements aléatoires sans OPOS6UL (UART)

Sans la carte OPOS6UL connectée, le robot exécute des **mouvements aléatoires** au démarrage (rotations de 45°, avancer/reculer de 20cm). Le robot tourne puis ne s'asservit plus.
Le comportement est aléatoire et ne se produit que quand l'OPOS6UL n'est pas alimentée.

---

## Architecture d'alimentation

```
[Switch ON]  ──► 5V ──► MD22 (logique) + Balise

[Arrêt d'urgence (ARU)]  ──► 16V ──► MD22 (puissance moteurs)
                          ──► 16V → 7V ──► Nucleo F446-RE
                          ──► 16V → 12V ──► Servomoteurs
```

- Le **5V** (MD22 logique + balise) est allumé **en premier** via un switch.
- L'**ARU** est relâché ensuite, alimentant simultanément le 16V moteurs et le 7V Nucleo.
- La **Nucleo** communique avec la **MD22** via **I2C** (PB6=SCL, PB7=SDA, 400kHz).
- La **carte OPOS6UL** (Linux) communique avec la Nucleo via **UART** (SD4, PA0=TX, PA1=RX) — liaison série séparée du bus I2C.

---

## Analyse — Cause racine

### Spécification MD22 (source : robot-electronics.co.uk)

- Au power-on, la MD22 démarre en **Mode 0** (vitesse unsigned : 0=full reverse, 128=stop, 255=full forward).
- **"No motor will move until directly after speed or speed2/turn registers are changed"** → les moteurs ne bougent pas tant qu'aucune écriture I2C n'est faite sur les registres de vitesse.
- **Pas de watchdog/timeout en mode I2C** : la MD22 maintient la dernière vitesse commandée indéfiniment. Le timeout (200ms) n'existe qu'en mode RC servo.

### Ce qui se passe au boot

1. Le **5V est déjà allumé** → la MD22 est prête et écoute sur le bus I2C.
2. L'**ARU est relâché** → le Nucleo reçoit son 7V et commence à booter. Le 16V moteur arrive simultanément sur la MD22.
3. **Pendant le boot du STM32**, les pins PB6 (SCL) et PB7 (SDA) sont dans un **état indéfini** (floating input par défaut au reset ARM). Les pull-ups I2C externes + les transitoires électriques génèrent des **transitions aléatoires** sur le bus.
4. La MD22 interprète ces transitions comme des **transactions I2C valides** → elle écrit une **valeur de vitesse aléatoire** dans ses registres.
5. **Pas de timeout** → la MD22 **maintient cette vitesse** indéfiniment.
6. **2 à 4 secondes plus tard**, le firmware ChibiOS a fini de booter, `Md22::init()` envoie Mode 1 + vitesse 0 → les moteurs s'arrêtent.

### Pourquoi `enableMotors(false)` ne protège pas

La ligne `mainAsserv->enableMotors(false)` dans `main.cpp:232` est une protection logicielle **dans le firmware du Nucleo**. Elle désactive l'asservissement côté software.

Mais les glitches I2C écrivent **directement dans les registres hardware de la MD22**, en bypassant complètement le firmware. Le code ChibiOS n'est même pas encore chargé en mémoire quand les glitches se produisent.

### Pourquoi c'est non systématique

Le bruit sur les lignes I2C pendant le boot est **aléatoire**. Parfois les transitions forment une transaction I2C valide avec l'adresse 0x58 (MD22), parfois non. Quand ça match l'adresse + un registre de vitesse, les moteurs démarrent.

---

## Analyse — Bug 2 : Commandes parasites UART (SerialIO)

### Protocole série sans protection

Le `SerialIO` utilise un protocole **caractère par caractère sans aucune validation** :
- Pas de handshake ni de synchronisation
- Pas de checksum ni CRC
- Pas de marqueur de début de trame
- Chaque octet reçu sur RX est **immédiatement traité** comme une commande

### Commandes déclenchées par du bruit sur RX

| Octet aléatoire | Caractère | Effet |
|---|---|---|
| `0x7A` | `z` | **Avancer 20cm** |
| `0x73` | `s` | **Reculer 20cm** |
| `0x71` | `q` | **Tourner 45° à gauche** |
| `0x64` | `d` | **Tourner 45° à droite** |
| `0x68` | `h` | **Arrêt d'urgence logiciel** |
| `0x67` + bruit | `g...` | **GoTo position aléatoire** |
| `0x4D 0x31` | `M1` | **Activer les moteurs** |

### Ce qui se passe

1. L'OPOS6UL n'est pas connectée ou pas alimentée
2. La pin PA1 (RX UART4) est configurée en **floating** (pas de pull-up)
3. Le thread `serialIoWrapperCommandInput` lit en boucle sur SD4 via `streamGet()`
4. Le bruit électrique sur RX est interprété comme des octets valides
5. Si un octet correspond à `q` ou `d` → le robot tourne de 45°
6. La commande se termine → plus de consigne → le robot ne maintient plus sa position

### Pourquoi ça ne se produit pas avec l'OPOS6UL

Quand l'OPOS6UL est connectée et alimentée, sa pin TX maintient la ligne à **3.3V** (état idle UART) → pas de bruit → pas de commandes parasites.

---

## Use cases de validation

### Use case 1 — Démarrage normal (5V → ARU)

| Étape | Action | Résultat |
|-------|--------|----------|
| 1 | Switch ON → 5V (MD22 logique + balise) | MD22 prête, moteurs immobiles |
| 2 | ARU relâché → 16V moteurs + 7V Nucleo | **Moteurs à fond 2-4s**, puis stop |
| 3 | OPOS6UL envoie M1 via UART | Asservissement activé, robot OK |

**Verdict** : Bug présent, fréquent mais non systématique.

### Use case 2 — Sans carte OPOS6UL, sans liaison série

| Étape | Action | Résultat |
|-------|--------|----------|
| 1 | Switch ON → 5V | MD22 prête |
| 2 | ARU relâché → Nucleo + moteurs | **À-coups dans les roues** (glitches I2C), puis moteurs immobiles |
| 3 | On bouge le robot manuellement | Pas d'asservissement, roues libres |

**Explication** : Les à-coups sont les glitches I2C. Ensuite `enableMotors(false)` désactive l'asserv. Sans OPOS6UL, personne n'envoie la commande `M1` pour activer les moteurs → pas d'asservissement.

### Use case 3 — Avec carte OPOS6UL

| Étape | Action | Résultat |
|-------|--------|----------|
| 1 | Switch ON → 5V | MD22 prête |
| 2 | Allumage OPOS6UL (boot Linux) | Rien de spécial, pas d'interaction I2C |
| 3 | ARU relâché → Nucleo + moteurs | Glitches possibles, puis asservissement OK |

**Explication** : L'OPOS6UL finit par envoyer `M1` via UART → moteurs activés → asservissement fonctionne. Le robot s'asservit **à chaque fois** dans cette configuration.

### Use case 4 — Toggle ARU répété (5V + OPOS6UL restent allumés)

| Étape | Action | Résultat |
|-------|--------|----------|
| 1 | 5V ON + OPOS6UL ON | MD22 prête, OPOS6UL tourne |
| 2 | ARU ON → Nucleo + moteurs | **Moteurs à fond 1-2s** |
| 3 | ARU OFF → Nucleo éteint, moteurs coupés | Stop |
| 4 | ARU ON → Nucleo reboot | **Moteurs à fond 1-2s** |
| 5 | Répéter... | **Reproductible à chaque toggle** |

**Explication** : C'est le cas le plus reproductible. La MD22 (5V) reste alimentée et écoute l'I2C en permanence. Chaque reboot du Nucleo génère des glitches I2C → la MD22 reçoit des commandes de vitesse aléatoires → moteurs à fond. Preuve définitive que le problème vient du bus I2C pendant le boot du STM32.

### Use case 5 — Sans OPOS6UL, moteurs activés, robot bouge tout seul

| Étape | Action | Résultat |
|-------|--------|----------|
| 1 | Switch ON → 5V, OPOS6UL **non alimentée** | MD22 prête |
| 2 | ARU relâché → Nucleo + moteurs | Boot, moteurs activés (`enableMotors(true)`) |
| 3 | Attente, robot posé sur la table | **Le robot tourne de ~45° aléatoirement**, puis ne résiste plus |

**Explication** : La pin RX (PA1) de l'UART4 est flottante sans l'OPOS6UL. Le bruit est interprété comme des commandes série (`q`=tourner 45° gauche, `d`=tourner 45° droite, etc.). Après exécution de la commande parasite, la file de commandes est vide → le robot ne maintient plus sa position (pas d'asservissement en position au repos).

---

## Timeline détaillée du bug

```
t=0.000s  ARU relâché
          ├── 16V arrive sur MD22 (puissance moteurs)
          └── 7V arrive sur Nucleo (début boot STM32)

t=0.000s  Pins PB6/PB7 (I2C) en état indéfini (floating)
  à       Pull-ups I2C + transitoires = bruit sur SDA/SCL
t=0.010s  MD22 interprète un glitch comme une écriture de vitesse
          ► MOTEURS À FOND (vitesse aléatoire, maintenue sans timeout)

t=0.001s  STM32 sort du reset, exécute le startup code
t=0.005s  main() → halInit() + chSysInit()
t=0.010s  Config LEDs, démarrage SD2 (UART debug)
t=0.510s  chThdSleepMilliseconds(500)  ← 500ms perdues !
t=0.520s  initAsserv() → création de tous les objets
t=0.600s  AsservThread démarre → Md22::init()
t=0.610s  I2C pins configurées + i2cStart()
t=0.710s  chThdSleepMilliseconds(100) (attente boot MD22)
t=0.720s  Envoi Mode 1 + vitesse 0 via I2C
          ► MOTEURS STOP

t=0.730s  enableMotors(false) — asserv désactivé
          Attente commande M1 de l'OPOS6UL...
```

Fenêtre vulnérable : **~700ms à 2-4s** (selon la vitesse de boot et les allocations mémoire).

---

## Fichiers concernés

| Fichier | Rôle |
|---------|------|
| `src/Robots/PMX/main.cpp` | Séquence de boot, création des objets, thread asserv |
| `src/motorController/Md22.cpp` | Init I2C + communication MD22 |
| `src/motorController/Md22.h` | Classe MD22 (pins I2C : PB6=SCL, PB7=SDA) |
| `src/Communication/SerialIO.cpp` | Protocole série OPOS6UL ↔ Nucleo (commandes caractère) |

---

## Solutions implémentées

### Fix 1 — Bit-bang I2C avant halInit() (Bug I2C + rétention MD22)

Ajouté au tout début de `main()`, **avant `halInit()`/`chSysInit()`** — code bare-metal qui :
1. Force PB6/PB7 en open-drain HIGH (état idle I2C) pour stopper les glitches
2. Envoie en bit-bang I2C : Mode 1 + vitesse moteur 1 = 0 + vitesse moteur 2 = 0

```cpp
// 1. Init GPIO — PB6 (SCL) et PB7 (SDA) en open-drain output HIGH
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
GPIOB->OTYPER |= (1 << 6) | (1 << 7);
GPIOB->ODR |= (1 << 6) | (1 << 7);
GPIOB->MODER = (GPIOB->MODER & ~(0xF << 12)) | (0x5 << 12);

// 2. Bit-bang I2C : envoyer Mode 1 + vitesse 0 aux 2 moteurs
//    Séquences START → 0xB0 (adresse MD22 write) → registre → valeur → STOP
i2cWriteReg(0x00, 0x01);  // Mode 1 (signed: 0=stop)
i2cWriteReg(0x01, 0x00);  // Motor 1 = 0
i2cWriteReg(0x02, 0x00);  // Motor 2 = 0
```

**Impact** : réduit la fenêtre vulnérable à quelques microsecondes. Résout aussi le problème de rétention des registres MD22 après ARU (la MD22 conserve la dernière vitesse car son 5V n'est jamais coupé).

### Fix 2 — Handshake `#` dans SerialIO (Bug UART) ✅ Validé

Ajouté dans `SerialIO::commandInput()` : le thread attend de recevoir le caractère `#` avant d'accepter toute commande. Tant que le handshake n'est pas reçu, tous les octets parasites sont ignorés.

```cpp
// Attente du handshake '#' envoyé par l'OPOS6UL
while(true)
{
    char c = streamGet(m_serialDriver);
    if (c == '#')
        break;
}
```

**Impact** : résout complètement le problème de commandes parasites UART. L'OPOS6UL doit envoyer `#` à l'ouverture de la connexion série avant toute commande.

**Note** : le `positionOutput` envoie `#%d;%d;...` toutes les 100ms. Si l'OPOS6UL est connectée, le `#` en tête de ces trames peut servir de handshake naturel (écho/retour sur RX si TX et RX sont proches). Mais le plus fiable est que l'OPOS6UL envoie explicitement `#` au démarrage.

### ~~Fix 2b — Pull-up interne sur RX/TX UART~~ ❌ Abandonné

Testé : `PAL_STM32_PUPDR_PULLUP` sur PA0 (TX) et PA1 (RX). Le pull-up interne (~40kΩ) est **trop faible** pour filtrer le bruit. Le problème persistait, voire empirait (possiblement un conflit avec le driver UART qui écrit sur TX toutes les 100ms via `positionOutput`).

Ref : [Zephyr UART pin pull-up issue #15349](https://github.com/zephyrproject-rtos/zephyr/issues/15349) — problème connu sur les STM32, mais le pull-up interne ne suffit pas dans notre cas.

---

## Solutions hardware envisagées (non implémentées)

### 3. Hardware — Résistances série sur SDA/SCL

Ajouter des résistances de 330Ω en série sur SDA et SCL entre le Nucleo et la MD22. Combinées avec les pull-ups, elles atténuent les transitoires pendant le boot.

### 4. Hardware — GPIO Nucleo contrôle le 5V MD22

Un MOSFET piloté par un GPIO du Nucleo (LOW par défaut au reset) coupe le 5V de la MD22. Au boot : MD22 éteinte → pas de glitch. Après init I2C : GPIO HIGH → MD22 alimentée → Nucleo envoie immédiatement les commandes stop.

### 5. Hardware — Mettre le 5V MD22 sur l'ARU

Alimenter la MD22 (5V logique) via l'ARU au lieu du switch séparé → la MD22 se reset à chaque coupure ARU → les registres de vitesse sont perdus → plus de rétention.

---

## Références

- [MD22 Technical Documentation](https://www.robot-electronics.co.uk/htm/md22tech.htm)
- [Zephyr UART pin pull-up issue #15349](https://github.com/zephyrproject-rtos/zephyr/issues/15349)
- Code source : `asserv_chibios/src/motorController/Md22.cpp`
- Code source : `asserv_chibios/src/Communication/SerialIO.cpp`
- STM32F446 Reference Manual — GPIO default state after reset : floating input
