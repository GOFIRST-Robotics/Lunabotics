// Define a struct to hold the sensor data
struct SensorData {
  bool diggerTopLimitSwitch;
  bool diggerBottomLimitSwitch;
  bool dumperTopLimitSwitch;
  bool dumperBottomLimitSwitch;
};

// Define the sensor pins here
#define DIGGER_TOP_LIMIT_SWITCH 2
#define DIGGER_BOTTOM_LIMIT_SWITCH 3
#define DUMPER_TOP_LIMIT_SWITCH 4
#define DUMPER_BOTTOM_LIMIT_SWITCH 5
void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize the digital inputs (limit switches)
  pinMode(DIGGER_TOP_LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(DIGGER_BOTTOM_LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(DUMPER_TOP_LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(DUMPER_BOTTOM_LIMIT_SWITCH, INPUT_PULLUP);
}

void loop() {
  // Create a SensorData struct
  SensorData data;

  // Read from the digital inputs (limit switches)
  data.diggerTopLimitSwitch = (bool)digitalRead(DIGGER_TOP_LIMIT_SWITCH);
  data.diggerBottomLimitSwitch = (bool)digitalRead(DIGGER_BOTTOM_LIMIT_SWITCH);
  data.dumperTopLimitSwitch = (bool)digitalRead(DUMPER_TOP_LIMIT_SWITCH);
  data.dumperBottomLimitSwitch = (bool)digitalRead(DUMPER_BOTTOM_LIMIT_SWITCH);

  // Send the struct over the serial bus to the Nvidia Jetson
  Serial.write((byte *)&data, sizeof(SensorData));

  // Wait 100ms before the next loop iteration
  delay(100);
}
