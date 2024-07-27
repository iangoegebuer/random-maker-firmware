
/*
 * Extending with additions by Ian Goegebuer and RTClib
 *
 * cheapStepper_newMoveTo.ino
 * ///////////////////////////////////////////
 * using CheapStepper Arduino library v.0.2.0
 * created by Tyler Henry, 7/2016
 * ///////////////////////////////////////////
 * 
 * This sketch illustrates the library's
 * "non-blocking" move functions -
 * i.e. you can perform moves with the stepper over time
 * while still running other code in your loop()
 * 
 * This can be useful if your Arduino is multi-tasking,
 * but be careful: if the other code in your loop()
 * slows down your Arduino, the stepper motor may
 * slow down or move with a stutter
 * 
 * //////////////////////////////////////////////////////
 */

// first, include the library :)

#include <CheapStepper.h>
#include <DS3231.h>
#include <Wire.h>
#include <EEPROM.h>

DS3231 adcClock;
RTClib myRTC;

bool century = false;
bool h12Flag;
bool pmFlag;

DateTime reset(2000, 1, 1, 0, 0, 0);

// next, declare the stepper
// and connect pins 8,9,10,11 to IN1,IN2,IN3,IN4 on ULN2003 board

CheapStepper stepper1 (10,11,12,13);  
CheapStepper stepper2 (6,7,8,9);  
CheapStepper stepper3 (2,3,4,5);  


 // let's also create a boolean variable to save the direction of our rotation
 // and a timer variable to keep track of move times

bool moveClockwise = true;
unsigned long moveStartTime = 0; // this will save the time (millis()) when we started each new move
unsigned int moveAmount = 30;
long currentValue = -1 ;

struct CounterObject {
  byte s1Flap;
  byte s2Flap;
  byte s3Flap;
};

CounterObject flapCurrentValues;


void setup() {
  EEPROM.get(0, flapCurrentValues);
  pinMode(A0, INPUT_PULLUP);
 // myRTC.setClockMode(false);

  // let's run the stepper at 12rpm (if using 5V power) - the default is ~16 rpm

  stepper1.setRpm(12);
  stepper2.setRpm(12);
  stepper3.setRpm(12);

  // let's print out the RPM to make sure the setting worked
  
  Serial.begin(9600);
  Serial.print("Flap1: ");Serial.println(flapCurrentValues.s1Flap,DEC);
  Serial.print("Flap2: ");Serial.println(flapCurrentValues.s2Flap,DEC);
  Serial.print("Flap3: ");Serial.println(flapCurrentValues.s3Flap,DEC);
//  flapCurrentValues.s1Flap = 0;
//  flapCurrentValues.s2Flap = 0;
//  flapCurrentValues.s3Flap = 0;

//  EEPROM.put(0,flapCurrentValues);
  Wire.begin();
  Serial.print("stepper RPM: "); Serial.print(stepper1.getRpm());
  Serial.println();

  // and let's print the delay time (in microseconds) between each step
  // the delay is based on the RPM setting:
  // it's how long the stepper will wait before each step

  Serial.print("stepper delay (micros): "); Serial.print(stepper1.getDelay());
  Serial.println(); Serial.println();

  // now let's set up our first move...
  // let's move a half rotation from the start point

//  stepper.newMoveTo(moveClockwise, 2048);
//    stepper1.newMoveToDegree(false, moveAmount);
  /* this is the same as: 
//   * stepper.newMoveToDegree(clockwise, 180);
   * because there are 4096 (default) steps in a full rotation
   */
  moveStartTime = millis(); // let's save the time at which we started this move
  
}

void setDisplay(int f1, int f2, int f3) {
    int nFlap1 = 0;
    int nFlap2 = 0;
    int nFlap3 = 0;

    Serial.print("Flaps1: "); Serial.println(f1);
    Serial.print("Flaps2: "); Serial.println(f2);
    Serial.print("Flaps3: "); Serial.println(f3);

    if(f1 > flapCurrentValues.s1Flap)
      nFlap1 = f1 - flapCurrentValues.s1Flap;
    else if ( f1 < flapCurrentValues.s1Flap)
      nFlap1 = 12 - (flapCurrentValues.s1Flap - f1);

    if(f2 > flapCurrentValues.s2Flap)
      nFlap2 = f2 - flapCurrentValues.s2Flap;
    else if ( f2 < flapCurrentValues.s2Flap)
      nFlap2 = 12 - (flapCurrentValues.s2Flap - f2);

    if(f3 > flapCurrentValues.s3Flap)
      nFlap3 = f3 - flapCurrentValues.s3Flap;
    else if ( f3 < flapCurrentValues.s3Flap)
      nFlap3 = 12 - (flapCurrentValues.s3Flap - f3);


    Serial.print("Number flaps1: "); Serial.println(nFlap1);
    Serial.print("Number flaps2: "); Serial.println(nFlap2);
    Serial.print("Number flaps3: "); Serial.println(nFlap3);
  
    flapCurrentValues.s1Flap = f1;
    flapCurrentValues.s2Flap = f2;
    flapCurrentValues.s3Flap = f3;
    EEPROM.put(0,flapCurrentValues);
  
    stepper1.newMoveDegrees (false, moveAmount*nFlap1); // move 180 degrees from current position
    stepper2.newMoveDegrees (false, moveAmount*nFlap2); // move 180 degrees from current position
    stepper3.newMoveDegrees (false, moveAmount*nFlap3); // move 180 degrees from current position
  
}

void loop() {

  // we need to call run() during loop() 
  // in order to keep the stepper moving
  // if we are using non-blocking moves
  
  stepper1.run();
  stepper2.run();
  stepper3.run();

  ////////////////////////////////
  // now the stepper is moving, //
  // let's do some other stuff! //
  ////////////////////////////////

  // let's check how many steps are left in the current move:
  
  int stepsLeft = stepper1.getStepsLeft() + stepper2.getStepsLeft() + stepper3.getStepsLeft();
  

  // if the current move is done...
  
  if (stepsLeft == 0){
    int nFlap1 = 0;
    int nFlap2 = 0;
    int nFlap3 = 0;
    long inVal = currentValue;
    int sensorValue = digitalRead(A0);
    
    if (Serial.available() > 0) {
      
      inVal = Serial.parseInt();
      adcClock.setEpoch();
//      setDisplay(inVal%10, (inVal/10)%10, (inVal/100)%10);
    } else if(sensorValue == 0 ) {
      inVal = 0;
      Serial.print("Sensor: "); Serial.println(sensorValue);
      adcClock.setEpoch();
      while(sensorValue == 0) sensorValue = digitalRead(A0);
      delay(1000);
      
    }

    else {
      delay(1000);
      DateTime now = myRTC.now();
//      
      Serial.print(now.year(), DEC);
      Serial.print('/');
      Serial.print(now.month(), DEC);
      Serial.print('/');
      Serial.print(now.day(), DEC);
      Serial.print(' ');
      Serial.print(now.hour(), DEC);
      Serial.print(':');
      Serial.print(now.minute(), DEC);
      Serial.print(':');
      Serial.print(now.second(), DEC);
      Serial.println();
      Serial.print(sensorValue, DEC);
      Serial.println();
//      
//      Serial.print(" since midnight 1/1/1970 = ");
//      Serial.print(now.unixtime());
//      Serial.print("s = ");
//      Serial.print((now.unixtime() - reset.unixtime()));
//      Serial.println("d");
      inVal = (now.unixtime() - reset.unixtime())/3600L;
    }

      if(inVal == currentValue) return;

      int nFlaps = 0;

      if(inVal > currentValue) {
        nFlaps = inVal - currentValue;
      } else {
        nFlaps = 12 - (currentValue - inVal);
      }

      Serial.print("Current value: "); Serial.println(currentValue);
      Serial.print("New value:     "); Serial.println(inVal);
      
      setDisplay(inVal%10, (inVal/10)%10, (inVal/100)%10);
      currentValue = inVal;

  
      Serial.print("stepper position: "); Serial.print(stepper1.getStep());
      Serial.println();
  
      // and now let's print the time the move took
  
      unsigned long timeTook = millis() - moveStartTime; // calculate time elapsed since move start
      Serial.print("move took (ms): "); Serial.print(timeTook);
      Serial.println(); Serial.println();
      
//      stepper1.newMoveDegrees (false, moveAmount*nFlap1); // move 180 degrees from current position
//      stepper2.newMoveDegrees (false, moveAmount*nFlap2); // move 180 degrees from current position
      moveStartTime = millis(); // reset move start time

      Serial.print("Available: ");Serial.println(Serial.available());
      Serial.read();
      Serial.print("Available: ");Serial.println(Serial.available());


    
    }

}
