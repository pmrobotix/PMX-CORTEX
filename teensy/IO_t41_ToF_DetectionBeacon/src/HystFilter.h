/*
 * Class: HystFilter [Hysteresis filter].
 * Apply hysteresis to an input value and deliver a lower resolution, stabilised output value.
 *Hysteresis

This exists in science and nature and is broadly defined but basically something similar to this: Hysteresis is the dependence of the state of a system on its history. 27 Anyway, its application here in the Arduino world is very concrete, generally to minimise oscillation between switching states.

Problem Area:

You've designed a system which determines a state based on an input. For example, it could be setting the brightness of a display based on
the ambient light measured via an analog pin. It could be a potentiometer which you want to use to input numbers on the range 0-9 into an Arduino. It could be thermostat which controls a heater. You discover problems with your display when the ambient light falls very close to the boundary of two brightness levels resulting in flicker. Or in the potentiometer example, you've set it close to the boundary of two values and it wanders between them. Or the heater constantly switches on and off when the ambient temperature is close to the thermostat setting.

Solutions:

There is a number of solutions. One is to add a time delay between changes of state and this prevents rapid oscillation between two states. This would minimise display flicker (etc.) in the example above but could make the system less responsive.
Another solution, which is the theme of this article, is to simply add some hysteresis, that is 'stickiness' or reluctance to leave the current state.
This hysteresis solution can be implemented in hardware or software. However the one presented here is a pure software solution.

Diagram

The diagram below illustrates the situation where you are reading an analog value in the range 0 to 1023 (say it is derived from light sensor) and you want to convert that into 5 levels that is level 0 through to level 4 (say to control the brightness of a display using pulse width modulation).
Broadly, the diagram illustrates that the conditions for leaving an existing state are different (within a margin, tougher) than entering it. Taking a detailed look, let's say the initial state of this system is that an analog value of 500 is read from the sensor. From the graph, it is clear that this yields an output level of 2. Now let's further assume that the light level drops slowly so the analog value moves downwards. At the point where the analog value reaches 380, the output level switches from 2 to 1. If, after dropping past 380, the analog value now starts rising, it must reach 420 before the output level switches back from 1 to 2. So, it is this difference (40 analog units in this case ) between leaving and entering adjacent states which provides the protection against flicker in this example. The arrowed squares in this graph are, incidentally, similar to the classic Schmitt trigger graph.

hysteresis.JPG
hysteresis.JPG
1221×737 70.8 KB
Software implementation.
https://forum.arduino.cc/t/hysteresis/506190

This is a simple function which is parameterised for a (linear) potentiometer across the power rails with the wiper connected to analog pin A0 yielding values in the range 0 to 9. It can easily be adapted for other purposes such as controlling display flicker by altering the number of levels and the middle transition points between levels to suit the application.
 */

#ifndef HYSTFILTER_H_
#define HYSTFILTER_H_

#include <Arduino.h>


class HystFilter
{
  public:

    HystFilter( uint16_t inputValues, uint16_t outputValues, uint16_t margin  ) ;

    // constructor

    // inputValues:  the total number of discrete values delivered by the input.
    //               For example, a 10 bit ADC delivers 1024 values.
    //               8 bit ADC = 256, 9 bit ADC = 512, 10 bit ADC = 1024, 11 bit ADC = 2048, 12 bit ADC = 4096 etc.

    // outputValues: the number of discrete output values delivered. This governs the resolution of the function.
    //               For example a 6 bit resolution yields 64 values. This should ideally be no higher that the input resolution
    //               minus 3 bits. For example if the input resolution is 10 bits (1024), this should not exceed 7 bits (128)

    // margin:       margin sets the 'stickyness' of the hysteresis or the reluctance to leave the current state.
    //               It is measured in units of the the input level. As a general rule, this should be about 10% to 25% of the
    //               range of input values that map to 1 output value. For example, if the inputValues is 1024 and the outputValues
    //               is 128, then 8 input values map to 1 output value so the margin should be 2 (25% of 8 ).
    //               Probably a value of 2 is OK. For low resolutions or noisy environments, it can be higher. Don't make it too high
    //               or ranges overlap and some output values become unreachable.


    uint16_t getOutputLevel( uint16_t inputLevel )  ;

    // converts an input level (eg in the range 0 to 1023 to an aoutput value of 1 to 127 with hyteresis.

  private:
    uint16_t _inputValues ;
    uint16_t _outputValues ;
    uint16_t _margin ;
    uint16_t _currentOutputLevel ;
} ;

#endif /* HYSTFILTER_H_ */
