// Test MakeCode micro:bit V2 - PMX PAMI
// Affiche une icône au démarrage puis fait défiler "PMX" en boucle
// Boutons A, B et logo tactile affichent des icônes

basic.showIcon(IconNames.Heart)
basic.pause(1000)

input.onButtonPressed(Button.A, function () {
    basic.showIcon(IconNames.Sword)
})

input.onButtonPressed(Button.B, function () {
    basic.showIcon(IconNames.Skull)
})

input.onLogoEvent(TouchButtonEvent.Pressed, function () {
    basic.showIcon(IconNames.Diamond)
})

basic.forever(function () {

    basic.showIcon(IconNames.Happy)
    basic.pause(500)
})
