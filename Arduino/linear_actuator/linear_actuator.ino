#include <ezButton.h>

const int extend_pin = 10; // motor driver pin
const int retract_pin = 9; // motor driver pin

ezButton limit_switch_extend(8); // limit switch pin
ezButton limit_switch_retract(7); // limit switch pin

// put setup code here to run once:
void setup() {
  Serial.begin(9600); // open serial communication with a baud rate of 9600 bps

  pinMode(extend_pin, OUTPUT);
  pinMode(retract_pin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT); // The built-in LED

  stop_actuator(); // stop the linear actuator
  digitalWrite(LED_BUILTIN, LOW); // Turn off the built-in LED
}

// put main code here to run repeatedly:
void loop() {
  if (Serial.available()) { // check if data is available to be read
    char data_received = Serial.read(); // read one byte from the serial buffer and save to data_received
    if (data_received == 'e') extend(); // extend the linear actuator
    if (data_received == 'r') retract(); // retract the linear actuator
  }
}

// extend the linear actuator, using a limit switch for feedback
void extend() {
  digitalWrite(LED_BUILTIN, HIGH); // Turn on the built-in LED
  digitalWrite(extend_pin, HIGH);
  digitalWrite(retract_pin, LOW);

  while (true) { // loop until the limit switch is pressed
    limit_switch_extend.loop();
    if(limit_switch_extend.isPressed()) {
      break;
    }

    if (Serial.available()) { // check if data is available to be read
      char data_received = Serial.read(); // read one byte from the serial buffer and save to data_received
      if (data_received == 'e') {
        extend(); // extend the linear actuator
        break;
      }
      if (data_received == 'r') {
        retract(); // retract the linear actuator
        break;
      }
    }
  }

  stop_actuator(); // stop the linear actuator
  Serial.write('f'); // send a confirmation message back to the Jetson
  digitalWrite(LED_BUILTIN, LOW); // Turn off the built-in LED
}

// retract the linear actuator, using a limit switch for feedback
void retract() {
  digitalWrite(LED_BUILTIN, HIGH); // Turn on the built-in LED
  digitalWrite(extend_pin, LOW);
  digitalWrite(retract_pin, HIGH);
  
  while (true) { // loop until the limit switch is pressed
    limit_switch_retract.loop();
    if(limit_switch_retract.isPressed()) {
      break;
    }

    if (Serial.available()) { // check if data is available to be read
      char data_received = Serial.read(); // read one byte from the serial buffer and save to data_received
      if (data_received == 'e') {
        extend(); // extend the linear actuator
        break;
      }
      if (data_received == 'r') {
        retract(); // retract the linear actuator
        break;
      }
    }
  }

  stop_actuator(); // stop the linear actuator
  Serial.write('s'); // send a confirmation message back to the Jetson
  digitalWrite(LED_BUILTIN, LOW); // Turn off the built-in LED
}

// stop the linear actuator
void stop_actuator() {
  digitalWrite(extend_pin, LOW);
  digitalWrite(retract_pin, LOW);
}