// Define a struct to hold the sensor data
struct SensorData {
  bool diggerTopLimitSwitch;
  bool diggerBottomLimitSwitch;
  bool dumperTopLimitSwitch;
  bool dumperBottomLimitSwitch;
  int leftMotorPotentiometer;
  int rightMotorPotentiometer;
};

// Define the sensor pins here
#define DIGGER_TOP_LIMIT_SWITCH 2
#define DIGGER_BOTTOM_LIMIT_SWITCH 3
#define DUMPER_TOP_LIMIT_SWITCH 4
#define DUMPER_BOTTOM_LIMIT_SWITCH 5
#define LEFT_MOTOR_POT_PIN A0
#define RIGHT_MOTOR_POT_PIN A1

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize the digital inputs (limit switches)
  pinMode(DIGGER_TOP_LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(DIGGER_BOTTOM_LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(DUMPER_TOP_LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(DUMPER_BOTTOM_LIMIT_SWITCH, INPUT_PULLUP);

  // No need to configure analog pins explicitly for potentiometers
  // as they are used as inputs by default.
}

void loop() {
  // Create a SensorData struct
  SensorData data;

  // Read from the digital inputs (limit switches)
  data.diggerTopLimitSwitch = (bool)digitalRead(DIGGER_TOP_LIMIT_SWITCH);
  data.diggerBottomLimitSwitch = (bool)digitalRead(DIGGER_BOTTOM_LIMIT_SWITCH);
  data.dumperTopLimitSwitch = (bool)digitalRead(DUMPER_TOP_LIMIT_SWITCH);
  data.dumperBottomLimitSwitch = (bool)digitalRead(DUMPER_BOTTOM_LIMIT_SWITCH);

  // Read from the analog inputs (potentiometers)
  data.leftMotorPotentiometer = analogRead(LEFT_MOTOR_POT_PIN);  // Read left motor potentiometer value
  data.rightMotorPotentiometer = analogRead(RIGHT_MOTOR_POT_PIN); // Read right motor potentiometer value

  // Send the struct over the serial bus to the Nvidia Jetson
  Serial.write((byte *)&data, sizeof(SensorData));

  // Wait 100ms before the next loop iteration
  delay(100);
}
