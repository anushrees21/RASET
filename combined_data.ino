#include <Wire.h>
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_accel;
const int ledPin1 = 2;
const int ledPin2 = 3;
const int rollThreshold = 10;   // Threshold for roll angle (in degrees)
const int pitchThreshold = 30;  // Threshold for pitch angle (in degrees)
float altitudethresh = 1.3;     // Threshold for altitude (in meters)

#include <Adafruit_BMP280.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp; 

void setup() {
  Wire.begin();
  Serial.begin(9600);
  mpu.begin();

  pinMode(ledPin1, OUTPUT);
  digitalWrite(ledPin1, LOW);
  pinMode(ledPin2, OUTPUT);
  digitalWrite(ledPin2, LOW);
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }  

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) {
      delay(10);
    }  
  }

  // Initialize filter variables
  mpu_accel = mpu.getAccelerometerSensor();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // Set accelerometer range if needed
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);      // Set gyro range if needed
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void loop() {
  // Read accelerometer data
  sensors_event_t accel;
  mpu_accel->getEvent(&accel);

  // Calculate roll and pitch angles
  float roll = atan2(accel.acceleration.y, accel.acceleration.z) * (180.0 / PI);
  float pitch = atan2(-accel.acceleration.x, sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.z * accel.acceleration.z)) * (180.0 / PI);

  // Display the roll and pitch values
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(" degrees, Pitch: ");
  Serial.print(pitch);
  Serial.println(" degrees");

  // Check if roll and pitch values are within the thresholds
  if (abs(roll) >= rollThreshold && abs(pitch) >= pitchThreshold) {
    digitalWrite(ledPin1, HIGH);  // Turn on the LED
  } else {
    digitalWrite(ledPin1, LOW);   // Turn off the LED
  }

  if (bmp.takeForcedMeasurement()) {
    // can now print out the new measurements
    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1013.25));
    float al = bmp.readAltitude(1013.25); // Read the altitude in meters
    al=al/100;
    /* Adjusted to local forecast! */
    Serial.println("m");

    // Check if altitude value is above the threshold
    if (al > altitudethresh) {
      digitalWrite(ledPin2, HIGH);  // Turn on the LED
    } else {
      digitalWrite(ledPin2, LOW);   // Turn off the LED
    }
  }

  delay(1000);  // You can adjust the delay based on your needs.
}
