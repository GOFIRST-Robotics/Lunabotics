// Define a struct to hold the sensor data
struct SensorData {
  int leftMotorPotentiometer;
  int rightMotorPotentiometer;
};

// Define the sensor pins here
#define LEFT_MOTOR_POT_PIN A0
#define RIGHT_MOTOR_POT_PIN A1
#define RELAY_PIN 7

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  pinMode(RELAY_PIN, OUTPUT);  // Set relay pin as an output
  digitalWrite(RELAY_PIN, LOW);  // Ensure the motor is off at the start

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

  // Check for incoming relay commands from the Jetson
  if (Serial.available()) {
    byte cmd = Serial.read();
    static bool relayState = LOW;
    switch (cmd) {
      case '0':  // Off
        relayState = LOW;
        break;
      case '1':  // On
        relayState = HIGH;
        break;
      case '2':  // Toggle
        relayState = !relayState;
        break;
      default:  // Unknown command
        break;
    }
    digitalWrite(RELAY_PIN, relayState);
  }

  // Wait 100ms before the next loop iteration
  delay(100);
}
