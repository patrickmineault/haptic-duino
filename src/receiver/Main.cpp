/* RECEIVER TEST
 *  RFM69 library and code by Felix Rusu - felix@lowpowerlab.com
// Get libraries at: https://github.com/LowPowerLab/
// Make sure you adjust the settings in the configuration section below !!!
// **********************************************************************************
// Copyright Felix Rusu, LowPowerLab.com
// Library and code by Felix Rusu - felix@lowpowerlab.com
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will
// be useful, but WITHOUT ANY WARRANTY; without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A
// PARTICULAR PURPOSE. See the GNU General Public
// License for more details.
//
// You should have received a copy of the GNU General
// Public License along with this program.
// If not, see <http://www.gnu.org/licenses></http:>.
//
// Licence can be viewed at
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************/

#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <radio-config.h>
#include <radio-packet.h>
//*********************************************************************************************
#define SERIAL_BAUD   115200
#include <Adafruit_NeoPixel.h>
#include <receiver/pulse-glove.h>
#include <receiver/common.h>

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define PIN            A5

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      1

// Various states and their respective color
const uint32_t noSignal = 0x220000;
const uint32_t unsynced = 0x111100;
const uint32_t syncing  = 0x000022;
const uint32_t synced   = 0x002200;
const uint32_t readingHigh = 0x230000;
const uint32_t readingLow  = 0x120000;
const uint32_t calibrationFailure = 0x2d0f00;

uint32_t state = noSignal;
bool calibrated = false;

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, PIN, NEO_GRB + NEO_KHZ800);
//#define LED           0 //use 0 on ESP8266

int16_t packetnum = 0;  // packet counter, we increment per xmission

RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);

#define SYNC_BUTTON A1
#define SYNC_INTERVAL 1000
unsigned long lastSyncPress = 0;

void setup() {
  //while (!Serial); // wait until serial console is open, remove if not tethered to computer. Delete this line on ESP8266
  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(32, 0, 0)); // Red.
  pixels.show(); // This sends the updated pixel color to the hardware.

  Serial.begin(SERIAL_BAUD);

  Serial.println("Feather RFM69HCW Receiver");

  // Hard Reset the RFM module
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);

  // Initialize radio
  radio.initialize(FREQUENCY, RECEIVER, NETWORKID);
  if (IS_RFM69HCW) {
    radio.setHighPower();    // Only for RFM69HCW & HW!
  }
  radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)

  radio.encrypt(ENCRYPTKEY);

  pinMode(LED, OUTPUT);

  Serial.print("\nListening at ");
  Serial.print(FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(" MHz");

  pinMode(SYNC_BUTTON, INPUT);

  setupGlove();
}

#define MSG_LEN 16
#define TIMEOUT_POKE 500
#define TIMEOUT_CALIBRATE 4000

typedef void(*waitCallback)(const unsigned long elapsed);
void emptyCb(const unsigned long elapsed) {
  ;
}

typedef struct {
  bool found;
  Payload payload;
} Response;

Response waitForPacket(const char type, unsigned long timeout, waitCallback cb) {
  Response response;
  response.found = false;
  long start = millis();
  unsigned long elapsed = 0;
  while(!response.found && elapsed < timeout) {
    if (radio.receiveDone()) {
      if (radio.DATALEN != sizeof(Payload)) {
        Serial.println("Invalid payload received!");
      }
      else
      {
        //*data = *(Payload*)radio.DATA; //assume radio.DATA actually contains
        // our struct and not something else.
        Serial.println("Before copy");
        for(int i = 0; i < radio.DATALEN; i++) {
          Serial.print((char) radio.DATA[i]);
        }
        Serial.println(response.payload.type);
        response.payload = deserialize(radio.DATA);
        Serial.println("After copy");
        Serial.println(response.payload.type);
        if(type == 'X' || response.payload.type == type) {
          // Found it!
          response.found = true;
        } else {
          Serial.println("Invalid type");
          Serial.println(response.payload.type);
          Serial.println(type);
        }
      }

      if (radio.ACKRequested())
      {
        radio.sendACK();
      }
    }
    delay(1);
    elapsed = millis() - start;
    if(cb != NULL) {
      cb(elapsed);
    }
  }
  return response;
}

void blinkSyncing(const unsigned long elapsed) {
  // Moderately bright green color.
  pixels.setPixelColor(0, elapsed % 200 < 100 ? syncing : 0x000000);
  pixels.show(); // This sends the updated pixel color to the hardware
}

void pulseRed(const unsigned long elapsed) {
  pixels.setPixelColor(0, elapsed % 100 < 50 ? readingHigh : readingLow);
  pixels.show(); // This sends the updated pixel color to the hardware
}

void recalibrate() {
  Serial.println("Initiating recalibration");
  radio.receiveDone();
  char msg[MSG_LEN] = "RECALIBRATE";
  if(radio.sendWithRetry(SENDER, msg, sizeof(msg), 2, 255)) {
    Response r;
    for(int i = 0; i < 2; i++) {
      Serial.println("R phase");
      Serial.println(millis());
      r = waitForPacket('R', 1500, blinkSyncing);
      if(!r.found) {
        goto bailout;
      }
      Serial.println("R phase");
      Serial.println(millis());
      r = waitForPacket('S', 1500, pulseRed);
      if(!r.found) {
        goto bailout;
      }
      Serial.println(millis());
    }

    r = waitForPacket('X', 250, blinkSyncing);
    if( !r.found || r.payload.type == '(') {
      // Couldn't find the right calibration :(
      Serial.println("Calibration failure");
      pixels.setPixelColor(0, calibrationFailure);
      pixels.show(); // This sends the updated pixel color to the hardware
      delay(2500);
      goto bailout;
    }

    // Great! Now we're calibrated!
    Serial.println("Calibration complete!");
    state = synced;
    return;
  }
  Serial.println("Did not receive acknowledgement of calibration");

  bailout:
  Serial.println("Error during calibration calibration!");
  if(calibrated) {
    state = synced;
  } else {
    state = unsynced;
  }
}
unsigned long lastPacketTime = 0;

#define TIMEOUT 500

int lastSyncButtonState = LOW;

void loop() {
  if(state != noSignal) {
    if(digitalRead(SYNC_BUTTON) == HIGH) {
      Serial.println("Sync button off");
      if(millis() - lastSyncPress > SYNC_INTERVAL && lastSyncButtonState == LOW) {
        calibrated = false;
        recalibrate();
      }
      lastSyncButtonState = HIGH;
    } else {
      if(lastSyncButtonState == HIGH) {
        lastSyncPress = millis();
      }
      Serial.println("Sync button on");
      if(millis() - lastSyncPress > SYNC_INTERVAL) {
        state = syncing;
      }
      lastSyncButtonState = LOW;
    }
  }

  Response r;
  r = waitForPacket('X', 100, NULL);

  if(r.found) {
    lastPacketTime = millis();
    Serial.println("Packet received!");
    Serial.println(r.payload.type);
    if(r.payload.type == 'U') {
      calibrated = false;
    } else if(r.payload.type == 'C') {
      Serial.println("Before");
      calibrated = true;
      scaleXY(r.payload.x, r.payload.y);
      Serial.print(r.payload.x);
      Serial.print(":");
      Serial.println(r.payload.y);
      Serial.println("After");
    }

    if(state == noSignal || state == synced || state == unsynced) {
      // Upgrade the start to unsynced or synced depending on calibration
      // status
      if(calibrated) {
        Serial.println("Synced");
        state = synced;
      } else {
        Serial.println("Unsynced");
        state = unsynced;
      }
    }
  } else {
    Serial.println("Packet not received!");
  }

  if(millis() - lastPacketTime > TIMEOUT) {
    state = noSignal;
  }

  //radio.receiveDone(); //put radio in RX mode
  //Serial.flush(); //make sure all serial data is clocked out before sleeping the MCU
  pixels.setPixelColor(0, state); // Moderately bright green color.
  pixels.show(); // This sends the updated pixel color to the hardware.
  delay(10);
}
