#include <Arduino.h>

// Matrice LED 5x5 du micro:bit V2
// Numéros de pins Arduino (définis dans le variant BBCmicrobitV2)
// ROW = anodes, COL = cathodes
static const int ROW_PINS[5] = {21, 22, 23, 24, 25};
static const int COL_PINS[5] = {4, 7, 3, 6, 10};

// Boutons A et B : actifs LOW
static const int BUTTON_A = 5;
static const int BUTTON_B = 11;

// Logo tactile : Arduino pin 26 (nRF52 P1.04)
static const int LOGO_TOUCH = 26;
static const int TOUCH_THRESHOLD_US = 50; // seuil en microsecondes (à ajuster)

// Motifs 5x5 (1 = LED allumée)
static const uint8_t SMILEY[5][5] = {
    {0, 1, 0, 1, 0},
    {0, 1, 0, 1, 0},
    {0, 0, 0, 0, 0},
    {1, 0, 0, 0, 1},
    {0, 1, 1, 1, 0}
};

static const uint8_t SAD[5][5] = {
    {0, 1, 0, 1, 0},
    {0, 1, 0, 1, 0},
    {0, 0, 0, 0, 0},
    {0, 1, 1, 1, 0},
    {1, 0, 0, 0, 1}
};

static const uint8_t HEART[5][5] = {
    {0, 1, 0, 1, 0},
    {1, 1, 1, 1, 1},
    {1, 1, 1, 1, 1},
    {0, 1, 1, 1, 0},
    {0, 0, 1, 0, 0}
};

static const uint8_t CROSS[5][5] = {
    {1, 0, 0, 0, 1},
    {0, 1, 0, 1, 0},
    {0, 0, 1, 0, 0},
    {0, 1, 0, 1, 0},
    {1, 0, 0, 0, 1}
};

static const uint8_t (*currentPattern)[5] = SMILEY;

// Lecture capacitive du logo tactile
// Le PCB a un pull-up 10MOhm sur le logo → on décharge à LOW puis
// on mesure le temps de remontée à HIGH via le pull-up.
// Un doigt ajoute de la capacité → remontée plus lente.
unsigned long readTouchUs() {
    // Décharger le pin à LOW
    pinMode(LOGO_TOUCH, OUTPUT);
    digitalWrite(LOGO_TOUCH, LOW);
    delayMicroseconds(50);

    // Passer en entrée (sans pull-up interne) et mesurer la remontée
    unsigned long start = micros();
    pinMode(LOGO_TOUCH, INPUT);
    while (!digitalRead(LOGO_TOUCH)) {
        if (micros() - start > 5000) break; // timeout 5ms
    }
    return micros() - start;
}

bool isLogoTouched() {
    return readTouchUs() > TOUCH_THRESHOLD_US;
}

void displayMatrix() {
    for (int row = 0; row < 5; row++) {
        digitalWrite(ROW_PINS[row], HIGH);

        for (int col = 0; col < 5; col++) {
            digitalWrite(COL_PINS[col], currentPattern[row][col] ? LOW : HIGH);
        }

        delayMicroseconds(2000);

        digitalWrite(ROW_PINS[row], LOW);
        for (int col = 0; col < 5; col++) {
            digitalWrite(COL_PINS[col], HIGH);
        }
    }
}

void setup() {
    for (int i = 0; i < 5; i++) {
        pinMode(ROW_PINS[i], OUTPUT);
        pinMode(COL_PINS[i], OUTPUT);
        digitalWrite(ROW_PINS[i], LOW);
        digitalWrite(COL_PINS[i], HIGH);
    }

    pinMode(BUTTON_A, INPUT_PULLUP);
    pinMode(BUTTON_B, INPUT_PULLUP);
}

void loop() {
    bool btnA = digitalRead(BUTTON_A) == LOW;
    bool btnB = digitalRead(BUTTON_B) == LOW;

    if (btnA && btnB) {
        currentPattern = CROSS;
    } else if (btnA) {
        currentPattern = SMILEY;
    } else if (btnB) {
        currentPattern = SAD;
    } else if (isLogoTouched()) {
        currentPattern = HEART;
    }

    displayMatrix();
}
