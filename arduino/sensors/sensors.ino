// Define a struct to hold the sensor data
struct SensorData {
  bool topLimitSwitch;
  bool bottomLimitSwitch;
};

// Define the sensor pins here
#define TOP_LIMIT_SWITCH 2
#define BOTTOM_LIMIT_SWITCH 3

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize the digital inputs (limit switches)
  pinMode(TOP_LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(BOTTOM_LIMIT_SWITCH, INPUT_PULLUP);
}

void loop() {
  // Create a SensorData struct
  SensorData data;

  // Read from the digital inputs (limit switches)
  data.topLimitSwitch = (bool)digitalRead(TOP_LIMIT_SWITCH);
  data.bottomLimitSwitch = (bool)digitalRead(BOTTOM_LIMIT_SWITCH);

  // Send the struct over the serial bus to the Nvidia Jetson
  Serial.write((byte *)&data, sizeof(SensorData));

  // Wait 100ms before the next loop iteration
  delay(100);
}
