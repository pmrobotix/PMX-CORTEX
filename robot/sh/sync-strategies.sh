#!/bin/bash
# ============================================================
# sync-strategies.sh — Sync strategy + init PMX{0,1,2,3}.json simu -> bot-opos6ul
#
# Source canonique : simulator/resources/2026/{strategy,init}PMXN.json
# Destination      : robot/src/bot-opos6ul/{strategy,init}PMXN.json
#
# Regle : on edite TOUJOURS dans le simu, puis on lance ce script.
#
# Comportement :
#   - identique     -> skip
#   - dest absente  -> copie directe
#   - dest plus ancienne -> copie (cas normal)
#   - dest plus recente  -> WARN + prompt y/n (risque ecrasement edition oubliee)
# ============================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SIMU_DIR="$(cd "$ROBOT_DIR/../simulator/resources/2026" && pwd)"
DST_DIR="$ROBOT_DIR/src/bot-opos6ul"

FILES=(
    strategyPMX0.json strategyPMX1.json strategyPMX2.json strategyPMX3.json
    initPMX0.json     initPMX1.json     initPMX2.json     initPMX3.json
)

# Couleurs
C_OK='\033[0;32m'
C_WARN='\033[0;33m'
C_ERR='\033[0;31m'
C_DIM='\033[0;90m'
C_RST='\033[0m'

copied=0
skipped=0
aborted=0

echo -e "${C_DIM}SRC: $SIMU_DIR${C_RST}"
echo -e "${C_DIM}DST: $DST_DIR${C_RST}"
echo

for f in "${FILES[@]}"; do
    src="$SIMU_DIR/$f"
    dst="$DST_DIR/$f"

    if [ ! -f "$src" ]; then
        echo -e "${C_ERR}[MISS]${C_RST} $f : source absente ($src)"
        continue
    fi

    if [ -f "$dst" ] && cmp -s "$src" "$dst"; then
        echo -e "${C_DIM}[ ==  ] $f${C_RST}"
        skipped=$((skipped+1))
        continue
    fi

    if [ ! -f "$dst" ]; then
        cp -p "$src" "$dst"
        echo -e "${C_OK}[NEW ]${C_RST} $f"
        copied=$((copied+1))
        continue
    fi

    # Les deux existent et different
    src_mtime=$(stat -c %Y "$src")
    dst_mtime=$(stat -c %Y "$dst")

    if [ "$dst_mtime" -gt "$src_mtime" ]; then
        # Bot plus recent que simu : suspect, prompt
        echo -e "${C_WARN}[WARN]${C_RST} $f : la copie BOT est plus recente que la source SIMU"
        echo -e "       SIMU mtime: $(date -d @$src_mtime '+%Y-%m-%d %H:%M:%S')"
        echo -e "       BOT  mtime: $(date -d @$dst_mtime '+%Y-%m-%d %H:%M:%S')"
        echo -e "${C_DIM}--- diff (SIMU -> BOT) ---${C_RST}"
        diff -u "$src" "$dst" || true
        echo -e "${C_DIM}--------------------------${C_RST}"
        read -r -p "Ecraser $f cote BOT par la version SIMU ? [y/N] " ans || ans=""
        if [[ "$ans" =~ ^[yY]$ ]]; then
            cp -p "$src" "$dst"
            echo -e "${C_OK}[COPY]${C_RST} $f (force)"
            copied=$((copied+1))
        else
            echo -e "${C_DIM}[SKIP] $f (conserve la version BOT)${C_RST}"
            aborted=$((aborted+1))
        fi
    else
        # Cas normal : simu plus recent
        cp -p "$src" "$dst"
        echo -e "${C_OK}[COPY]${C_RST} $f"
        copied=$((copied+1))
    fi
done

echo
echo -e "${C_OK}copied=$copied${C_RST}  ${C_DIM}skipped=$skipped${C_RST}  ${C_WARN}aborted=$aborted${C_RST}"
