# pami-test-codal — Build Instructions

Projet CODAL (runtime natif micro:bit V2) avec touch logo capacitif.

## Prérequis

```bash
sudo apt install gcc git cmake gcc-arm-none-eabi binutils-arm-none-eabi python3
pip3 install --user intelhex
```

## Premier build

```bash
cd pamis/pami-test-codal
python3 build.py
```

Le premier build télécharge automatiquement les dépendances CODAL (~5 min).

## Flash

Copier le fichier `MICROBIT.hex` généré sur le lecteur USB MICROBIT :

```bash
cp MICROBIT.hex /media/$USER/MICROBIT/
```

## Utilisation

- **Bouton A** : affiche un smiley
- **Bouton B** : affiche un visage triste
- **A + B** : affiche une croix
- **Touch logo** : affiche un coeur
- La LED centrale clignote quand le logo est touché
