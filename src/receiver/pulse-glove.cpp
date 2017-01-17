#include <receiver/common.h>
#include <math.h>
#include <TimerThree.h>

// For the Bluetooth micro.
const int right = 9;
const int front = 6;
const int left  = 11;
const int back  = 10;

#include <math.h>

const float strength = 255.0;//0.001;//625 / 2.0;

volatile int16_t xscaled, yscaled, pulseLength, period, counter;

const float minPeriod = 2.0;
const float maxPeriod = 0.5;
const float normPower = 0.5;

// How many periods in a second?
const float periodScaler = 100.0;

// Defines a deadzone around zero.
const float eps = 0.1;

const float overlap = 0.5;
const bool midTimeOff = false;

void timerCallback() {
  if(!calibrated) {
    digitalWrite(left, LOW);
    digitalWrite(right, LOW);
    digitalWrite(front, LOW);
    digitalWrite(back, LOW);
    return;
  }

  if(counter < overlap*pulseLength) {
    if(xscaled > 0) {
      analogWrite(right, xscaled);
      analogWrite(left , 0);
    } else {
      analogWrite(left, -xscaled);
      analogWrite(right , 0);
    }

    if(yscaled > 0) {
      analogWrite(front, yscaled);
      analogWrite(back , 0);
    } else {
      analogWrite(back, -yscaled);
      analogWrite(front, 0);
    }
  } else if(counter < pulseLength) {
    if(midTimeOff) {
      digitalWrite(left, LOW);
      digitalWrite(right, LOW);
      digitalWrite(front, LOW);
      digitalWrite(back, LOW);
    } else {

      if(xscaled > 0) {
        analogWrite(right, xscaled);
        analogWrite(left , xscaled);
      } else {
        analogWrite(left, -xscaled);
        analogWrite(right , -xscaled);
      }

      if(yscaled > 0) {
        analogWrite(front, yscaled);
        analogWrite(back , yscaled);
      } else {
        analogWrite(back, -yscaled);
        analogWrite(front,-yscaled);
      }
    }
  } else if(counter < (1 + overlap)*pulseLength) {
    if(xscaled > 0) {
      analogWrite(left, xscaled);
      analogWrite(right, 0);
    } else {
      analogWrite(right, -xscaled);
      analogWrite(left , 0);
    }

    if(yscaled > 0) {
      analogWrite(back , yscaled);
      analogWrite(front, 0);
    } else {
      analogWrite(front, -yscaled);
      analogWrite(back , 0);
    }
  } else {
    analogWrite(front, 0);
    analogWrite(back, 0);
    analogWrite(left, 0);
    analogWrite(right, 0);
  }


  digitalWrite(LED_BUILTIN, counter < pulseLength);
  counter++;
  if(counter >= period) {
    counter -= period;
  }
}

void scaleXY(float x, float y) {
  // max(abs(x)) should be 1.0, and similarly for y.
  float norm = sqrt(x * x + y * y);

  noInterrupts();
  period = periodScaler * ((maxPeriod - minPeriod) * pow(norm, normPower) + minPeriod);

  //period = 50.0;

  // Start with maximum feedback
  xscaled = 1.0 * strength * x  / (eps + norm);
  yscaled = 1.0 * strength * y  / (eps + norm);
  interrupts();
}

// the setup function runs once when you press reset or power the board
void setupGlove() {
  // Initialize the pins for output mode
  pinMode(front, OUTPUT);
  pinMode(back, OUTPUT);
  pinMode(left, OUTPUT);
  pinMode(right, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  xscaled = 0;
  yscaled = 0;
  counter = 0;
  period = 75;
  pulseLength = 25;

  // Callbacks every 10 ms
  Timer3.initialize();
  Timer3.attachInterrupt(timerCallback, 10000);
}
