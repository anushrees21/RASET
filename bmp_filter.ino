#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// Initialize the BMP280 sensor
Adafruit_BMP280 bmp280;

// Set the BMP280 sensor settings
float seaLevelPressure = 1013.25;  // Set the sea level pressure for altitude calculation

float alpha = 0.2;
float filteredValue = 0.0;
// Define the filter parameters
int filterWindow = 5;  // Number of previous pressure values to consider for filtering
float pressureValues[5];  // Array to store the previous pressure values
int currentIndex = 0;  // Index for storing the current pressure value

// Function to update the filtered value using EMA
float updateEMA(float newValue) {
  filteredValue = alpha * newValue + (1 - alpha) * filteredValue;
  return filteredValue;
}

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize the BMP280 sensor
  if (!bmp280.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  // Set the initial pressure values in the array
  for (int i = 0; i < filterWindow; i++) {
    pressureValues[i] = bmp280.readPressure();
    float filteredPressure = updateEMA(pressureValues[i]);
  }
}

void loop() {
  // Read the current pressure value from the BMP280 sensor
  float pressure = bmp280.readPressure();
  float new_pressure=updateEMA(pressure);
  

  // Store the current pressure value in the array
  pressureValues[currentIndex] = new_pressure;

  // Increment the current index
  currentIndex = (currentIndex + 1) % filterWindow;

  // Calculate the filtered pressure value using a simple moving average
  float filteredPressure = 0;
  for (int i = 0; i < filterWindow; i++) {
    filteredPressure += pressureValues[i];
  }
  filteredPressure /= filterWindow;
  Serial.print("Filtered Pressure: ");
  Serial.print(filteredPressure);
  Serial.println(" hPa");

  // Optional: Add a delay to control the sampling rate
  delay(500);
}

  // Print the filtered
