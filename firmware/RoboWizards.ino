/* * HEMISPHERICAL TOF SCANNER (Pololu Library Version)
 * --------------------------------------------------
 * REQUIRED LIBRARY: "VL53L0X" by Pololu
 * * WIRING:
 * - Servo Pan: Pin 9
 * - Servo Tilt: Pin 10
 * - VL53L0X SDA: A4
 * - VL53L0X SCL: A5
 * - VL53L0X VCC: 5V
 * - VL53L0X GND: GND
 */

#include <Wire.h>
#include <Servo.h>
#include <VL53L0X.h>

Servo panServo;
Servo tiltServo;
VL53L0X sensor;

// --- SCANNING CONFIGURATION ---
const int PAN_STEP = 5;    // Step angle (Lower = Higher Resolution)
const int TILT_STEP = 5;   // Vertical step angle
const int MAX_TILT = 180;   // Limit scan to upper hemisphere

void setup() {
  Serial.begin(115200); // High speed serial
  Wire.begin();

  panServo.attach(9);
  tiltServo.attach(10);

  // Initialize Sensor
  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

  // Increase timing budget for higher accuracy (optional)
  // Default is ~33ms. 
  // sensor.setMeasurementTimingBudget(200000); 

  // Move to Home Position
  panServo.write(0);
  tiltServo.write(0);
  delay(1000);
}

void loop() {
  // Wait for Python command 'S'
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'S') {
      Serial.println("START_SCAN");
      performScan();
      Serial.println("END_SCAN");
    }
  }
}

void performScan() {
  // Loop Tilt from 0 to 90
  for (int t = 0; t <= MAX_TILT; t += TILT_STEP) {
    tiltServo.write(t);
    delay(50); // Allow tilt to settle

    // Zig-Zag Logic: Even rows go Right, Odd rows go Left
    if ((t / TILT_STEP) % 2 == 0) {
      // Sweep 0 -> 180
      for (int p = 0; p <= 180; p += PAN_STEP) {
        scanPoint(p, t);
      }
    } else {
      // Sweep 180 -> 0
      for (int p = 180; p >= 0; p -= PAN_STEP) {
        scanPoint(p, t);
      }
    }
  }
  // Return Home
  panServo.write(0);
  tiltServo.write(0);
}

void scanPoint(int p, int t) {
  panServo.write(p);
  
  // Mechanical delay: Give servo time to reach position
  // 30ms is usually enough for small steps (2 degrees)
  delay(30); 

  // Take Single Shot Measurement
  // This function blocks until the sensor has a value
  int distance = sensor.readRangeSingleMillimeters();

  // Check for timeouts
  if (sensor.timeoutOccurred()) { 
    distance = 8190; // Error code
  }

  // Send Data: "PAN,TILT,DISTANCE"
  Serial.print(p);
  Serial.print(",");
  Serial.print(t);
  Serial.print(",");
  Serial.println(distance);
}