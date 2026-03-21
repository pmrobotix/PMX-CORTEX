#include "MicroBit.h"

MicroBit uBit;

// Motifs 5x5
const MicroBitImage SMILEY(
    "0,255,0,255,0\n"
    "0,255,0,255,0\n"
    "0,0,0,0,0\n"
    "255,0,0,0,255\n"
    "0,255,255,255,0\n"
);

const MicroBitImage SAD(
    "0,255,0,255,0\n"
    "0,255,0,255,0\n"
    "0,0,0,0,0\n"
    "0,255,255,255,0\n"
    "255,0,0,0,255\n"
);

const MicroBitImage HEART(
    "0,255,0,255,0\n"
    "255,255,255,255,255\n"
    "255,255,255,255,255\n"
    "0,255,255,255,0\n"
    "0,0,255,0,0\n"
);

const MicroBitImage CROSS(
    "255,0,0,0,255\n"
    "0,255,0,255,0\n"
    "0,0,255,0,0\n"
    "0,255,0,255,0\n"
    "255,0,0,0,255\n"
);

static void onButtonA(MicroBitEvent)
{
    uBit.display.print(SMILEY);
}

static void onButtonB(MicroBitEvent)
{
    uBit.display.print(SAD);
}

static void onButtonAB(MicroBitEvent)
{
    uBit.display.print(CROSS);
}

static void onTouchLogo(MicroBitEvent)
{
    uBit.display.print(HEART);
}

int main()
{
    uBit.init();

    uBit.messageBus.listen(MICROBIT_ID_BUTTON_A,  MICROBIT_BUTTON_EVT_CLICK, onButtonA);
    uBit.messageBus.listen(MICROBIT_ID_BUTTON_B,  MICROBIT_BUTTON_EVT_CLICK, onButtonB);
    uBit.messageBus.listen(MICROBIT_ID_BUTTON_AB, MICROBIT_BUTTON_EVT_CLICK, onButtonAB);
    uBit.messageBus.listen(MICROBIT_ID_LOGO,      MICROBIT_BUTTON_EVT_CLICK, onTouchLogo);

    uBit.display.print(SMILEY);

    release_fiber();
}
