#include "HystFilter.h"

/*

   HystFilter class [Hysteresis]

   This code example shows obtaining values 0-64 from one potentiometer wired
   as a voltage divider and 0-10 from another.

   These values remain stable even if the potentiometer
   is set on a border point between two values.

   The same principle could be used to set the brightness of a display
   dependent on the ambient light, but free from flicker or switching a heater
   on based on a thermostat etc.

   6v6gt 17.apr.2018

   ver 0.01 03.02.2018 Original verson with table to represent range end points
   ver 0.02 13.04.2018 Original verson with calculation to determine range end points.
   ver 0.04 17.03.2018 Class implementation for multiple potentiometers (or other analog sources)
*/
/*
#include "HystFilter.h"

HystFilter potA( 1024, 64, 3 ) ;  // 10 bit ADC = 1024, 64 discrete output values required, margin = 3 units (of 1024)
HystFilter potB( 1024, 10, 5 ) ;  // 10 bit ADC = 1024, 10 discrete output values required, margin = 5 units (of 1024)


void setup() {
  Serial.begin( 9600 ) ;
}

void loop() {

  Serial.print("potA:  " ) ;
  Serial.print(  potA.getOutputLevel( analogRead(A0) ) ) ;
  Serial.print("  " ) ;
  Serial.println( analogRead(A0) ) ;

  Serial.print("potB:  " ) ;
  Serial.print(  potB.getOutputLevel( analogRead(A1) ) ) ;
  Serial.print("  " ) ;
  Serial.println( analogRead(A1) ) ;
  Serial.println("  " ) ;


  delay ( 500 ) ;
}
*/

HystFilter::HystFilter( uint16_t inputValues, uint16_t outputValues, uint16_t margin  ) :
  _inputValues( inputValues ) ,
  _outputValues(  outputValues ) ,
  _margin(  margin ) ,
  _currentOutputLevel( 0  )
{  }


uint16_t HystFilter::getOutputLevel( uint16_t inputLevel ) {

  // get lower and upper bounds for currentOutputLevel
  uint16_t lb =   (float) ( (float) _inputValues / _outputValues ) * _currentOutputLevel  ;
  if ( _currentOutputLevel > 0 ) lb -= _margin  ;   // subtract margin

  uint16_t ub =   ( (  (float) ( (float) _inputValues / _outputValues ) * ( _currentOutputLevel + 1 )  )  - 1 )  ;
  if ( _currentOutputLevel < _outputValues ) ub +=  _margin  ;  // add margin
  // now test if input is outside the outer margins for current output value
  // If so, caclulate new output level.
  if ( inputLevel < lb || inputLevel > ub ) {
    // determine new output level
    _currentOutputLevel =   (  ( (float) inputLevel * (float) _outputValues ) /  _inputValues ) ;
  }
  return _currentOutputLevel ;
}


