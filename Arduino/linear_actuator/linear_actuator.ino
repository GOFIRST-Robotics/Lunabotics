#include <ezButton.h> // This library makes it easier for us to work with limit switches

const int extend_pin = 10; // motor driver forwards pin
const int retract_pin = 9; // motor driver reverse pin

ezButton limit_switch_extend(8); // extend limit switch pin
ezButton limit_switch_retract(7); // retract limit switch pin

// These booleans help keep track of the current state of the linear actuator
bool extending = false;
bool retracting = false;

int speed = 50; // PWM value ranges between 0-255

// put setup code in this method to run once:
void setup() {
  Serial.begin(9600); // open serial communication with a baud rate of 9600 bps

  pinMode(extend_pin, OUTPUT); // Declare extend_pin to be an OUTPUT pin
  pinMode(retract_pin, OUTPUT); // Declare retract_pin to be an OUTPUT pin
  pinMode(LED_BUILTIN, OUTPUT); // This is the built-in LED on the Arduino

  stop_actuator(); // stop the linear actuator
}

// put main loop code in this method to run repeatedly:
void loop() {
  limit_switch_extend.loop(); // this method from the ezButton library needs to be called every code cycle
  limit_switch_retract.loop(); // this method from the ezButton library needs to be called every code cycle

  if (Serial.available()) { // check if data is available to be read from the serial buffer
    char data_received = Serial.read(); // read one byte from the serial buffer and save to data_received
    if (data_received == 'e') extend(speed); // extend the linear actuator
    if (data_received == 'r') retract(speed); // retract the linear actuator
  }

  // check if the extend limit switch is pressed and we are currently extending
  if(limit_switch_extend.isPressed() && extending) {
    stop_actuator(); // stop the linear actuator
    Serial.write('f'); // send a message over serial to the Jetson
  }

  // check if the retract limit switch is pressed and we are currently retracting
  if(limit_switch_retract.isPressed() && retracting) {
    stop_actuator(); // stop the linear actuator
    Serial.write('s'); // send a message over serial to the Jetson
  }
}

// extend the linear actuator by running the motor forwards
void extend(int speed) {
  extending = true;
  retracting = false;
  analogWrite(extend_pin, speed);
  analogWrite(retract_pin, 0);
  digitalWrite(LED_BUILTIN, HIGH); // Turn on the built-in LED
}

// retract the linear actuator by running the motor in reverse
void retract(int speed) {
  retracting = true;
  extending = false;
  analogWrite(extend_pin, 0);
  analogWrite(retract_pin, speed);
  digitalWrite(LED_BUILTIN, HIGH); // Turn on the built-in LED
}

// stop the linear actuator motor
void stop_actuator() {
  analogWrite(extend_pin, 0);
  analogWrite(retract_pin, 0);
  digitalWrite(LED_BUILTIN, LOW); // Turn off the built-in LED
  extending = false;
  retracting = false;
}