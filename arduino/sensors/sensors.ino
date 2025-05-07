// Define a struct to hold the sensor data
struct SensorData {
  int leftMotorPotentiometer;
  int rightMotorPotentiometer;
};

// Define the sensor pins here
#define LEFT_MOTOR_POT_PIN A0
#define RIGHT_MOTOR_POT_PIN A1

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // No need to configure analog pins explicitly for potentiometers
  // as they are used as inputs by default.
}

void loop() {
  // Create a SensorData struct
  SensorData data;

  // Read from the analog inputs (potentiometers)
  data.leftMotorPotentiometer = analogRead(LEFT_MOTOR_POT_PIN);  // Read left motor potentiometer value
  data.rightMotorPotentiometer = analogRead(RIGHT_MOTOR_POT_PIN); // Read right motor potentiometer value

  // Send the struct over the serial bus to the Nvidia Jetson
  Serial.write((byte *)&data, sizeof(SensorData));

  // Wait 100ms before the next loop iteration
  delay(100);
}
