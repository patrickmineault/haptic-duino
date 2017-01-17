#include <Arduino.h>
#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <radio-config.h>
#include <radio-packet.h>

// Setup for 9-dof lib
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

#include <Vector.h>
#include <Welford.h>

/* Setup for 9-dof */
// In IMU mode, the 9 dof unit works at 100 Hz.
#define BNO055_SAMPLERATE_DELAY_MS (10)

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

#define SERIAL_BAUD   115200

int16_t packetnum = 0;  // packet counter, we increment per xmission

// Set up radio and 9-dof
RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Vector g0, gf, gr;

// Acquire one second of data from the unit for calibration purposes.
float acquireCalibration(uint16_t sampleSize, Vector *calibration) {
  Serial.println("Acquiring");
  const int nsamples = sampleSize / BNO055_SAMPLERATE_DELAY_MS;
  // Compute the mean gravity vector.
  Welford<float> rx = Welford<float>{};
  Welford<float> ry = Welford<float>{};
  Welford<float> rz = Welford<float>{};

  for(int i = 0; i < nsamples; i++) {
    imu::Vector<3> v = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    rx.add_value(v.x());
    ry.add_value(v.y());
    rz.add_value(v.z());

    // Wait for a legit number of samples before moving on.
    delay(BNO055_SAMPLERATE_DELAY_MS);
  }

  calibration->x = rx.mean();
  calibration->y = ry.mean();
  calibration->z = rz.mean();

  return rx.var() + ry.var() + rz.var();
}

void Blink(byte PIN, byte DELAY_MS, byte loops)
{
  for (byte i=0; i<loops; i++)
  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}

void setupSensor() {

  /* Initialise the sensor */
  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS))
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    digitalWrite(LED_BUILTIN, HIGH);
    while (1);
  }

  int eeAddress = 0;
  long bnoID;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  /*
  *  Look for the sensor's unique ID at the beginning oF EEPROM.
  *  This isn't foolproof, but it's better than nothing.
  */
  bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id)
  {
    Serial.println("No Calibration Data for this sensor exists in EEPROM; run Examples > Adafruit BNO055 > sensor_offsets first");
    digitalWrite(LED_BUILTIN, HIGH);
    while (1);
  }
  else
  {
    Serial.println("Found Calibration for this sensor in EEPROM.");
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibrationData);

    Serial.println("Restoring Calibration data to the BNO055...");
    bno.setSensorOffsets(calibrationData);

    Serial.println("Calibration data loaded into BNO055");
  }

  bno.setExtCrystalUse(true);

  // Great -- now we're in the uncalibrated state. We just have to wait
  // a bit to that we get a request to calibrate and we're a-go!
}

void setup() {
  //while (!Serial); // wait until serial console is open, remove if not tethered to computer. Delete this line on ESP8266
  delay(3000);
  Serial.begin(SERIAL_BAUD);
  Serial.println("Feather RFM69HCW Transmitter");

  // Hard Reset the RFM module
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);

  // Initialize radio
  radio.initialize(FREQUENCY, SENDER, NETWORKID);
  if (IS_RFM69HCW) {
    radio.setHighPower();    // Only for RFM69HCW & HW!
  }
  radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)

  radio.encrypt(ENCRYPTKEY);

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.print("\nTransmitting at ");
  Serial.print(FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(" MHz");

  setupSensor();
}



bool calibrated = false;

void calibrate() {
  // Enter into the calibration routine.
  // Now perform the calibration sequence.
  //Serial.println("Beginning secondary calibration sequence.");
  //waitForChar("Keep head straight and press C to calibrate.", 'C');

  delay(1000);
  radioPacket.type = 'R';
  radioPacket.x = 0.0;
  radioPacket.y = 0.0;

  radio.sendWithRetry(RECEIVER, (const void*)(&radioPacket), sizeof(radioPacket));
  float v0 = acquireCalibration(uint16_t(1000), &g0);
  radioPacket.type = 'S';
  radio.sendWithRetry(RECEIVER, (const void*)(&radioPacket), sizeof(radioPacket));

  delay(1000);
  radioPacket.type = 'R';
  radio.sendWithRetry(RECEIVER, (const void*)(&radioPacket), sizeof(radioPacket));
  float vf = acquireCalibration(uint16_t(1000), &gf);
  radioPacket.type = 'S';
  radio.sendWithRetry(RECEIVER, (const void*)(&radioPacket), sizeof(radioPacket));

  Serial.println(1000.0*v0);
  Serial.println(1000.0*vf);

  // Compute the gravity vector.
  g0 = normalize(g0);

  // test that the cross product is sufficiently large.
  float n = norm(crossProduct(gf, g0));
  Serial.println(n);
  if(n < 1 || v0 > .01 || vf > .01) {
    // Calibration was not successful, because the sensor didn't move over a
    // large enough range, or it wasn't stable enough.  :(
    radioPacket.type = '(';
    radio.sendWithRetry(RECEIVER, (const void*)(&radioPacket), sizeof(radioPacket));
    return;
  }

  // BULD SUCCEFUL
  radioPacket.type = ')';
  radio.sendWithRetry(RECEIVER, (const void*)(&radioPacket), sizeof(radioPacket));

  // And the front vector, by projecting out the gravity vector.
  gf = normalize(projectOut(gf, g0));
  // And the right vector, as a cross product of the other two.
  gr = normalize(crossProduct(g0, gf));

  // Now it's calibrated and ready to go!
  calibrated = true;
  Serial.println("Exiting calibration mode");
}

void loop() {
  Blink(LED_BUILTIN, 100, 1);
  delay(100);  // Wait 1 second between transmits, could also 'sleep' here!

  Serial.println("Receiving");
  if (radio.receiveDone() && radio.DATALEN > 0)
  {
    Serial.println("Received packet");
    Serial.print(radio.DATALEN);
    Serial.println(" bytes.");
    for(int i = 0; i < radio.DATALEN; i++) {
      Serial.print((char) radio.DATA[i]);
    }
    Serial.println("");
    Serial.println("===============");
    if (radio.ACKRequested())
    {
      radio.sendACK();
    }

    calibrate();
  }

  if(!calibrated) {
    Serial.println("Not calibrated yet.");
    radioPacket.type = 'U';
    radioPacket.x = 0.0;
    radioPacket.y = 0.0;
  } else {
    Serial.println("Calibrated");
    // Read the data from the BNO, then.
    imu::Vector<3> v = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    Vector w = {.x = v.x(), .y = v.y(), .z = v.z()};
    w = normalize(w);

    // Take dot products wrt. gravity.
    float x = dot(w, gr);
    float y = dot(w, gf);

    radioPacket.type = 'C';
    radioPacket.x = x;
    radioPacket.y = y;
    Serial.print(x);
    Serial.print(":");
    Serial.println(y);
    Serial.println("===");
  }

  long now = millis();
  Serial.println("Sending");
  Serial.println(calibrated);
  if (radio.sendWithRetry(RECEIVER, (const void*)(&radioPacket), sizeof(radioPacket))) {
    Serial.println("ACK received");
    //Blink(LED, 50, 3); //blink LED 3 times, 50ms between blinks
  } else {
    Serial.println("ACK not received");
  }
  Serial.println(millis() - now);

  //radio.receiveDone(); //put radio in RX mode
  Serial.flush(); //make sure all serial data is clocked out before sleeping the MCU
}
