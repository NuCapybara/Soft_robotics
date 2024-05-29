#include <Arduino.h>

// Declare pins for controlling actuators
const int actuatorPins[] = {2, 3, 4, 5, 6, 7, 8, 9, 10};

// Parametrize the number of actuators
const int num_actuators_used = 6;

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < num_actuators_used; i++)
  {
    pinMode(actuatorPins[i], OUTPUT);
  }

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // Check if there's enough data available to read
  // if (Serial.available() >= num_actuators_used) 
  if (Serial.available() >= 9) 
  { 
    // Read incoming data
    char buffer[9];
    Serial.readBytes(buffer, 9);

    // Parse received data
    char actuator_states[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

    for (int i = 0; i < num_actuators_used; i++)
    {
      // Assign actuator states
      memcpy(&actuator_states[i], &buffer[i*sizeof(char)], sizeof(char));

      if (actuator_states[i] != 0)
      {
        // Turn actuator ON
        digitalWrite(actuatorPins[i], HIGH);

        // Trigger LED_BUILTIN
        digitalWrite(LED_BUILTIN, HIGH);
      }
      else
      {
        // Turn actuator ON
        digitalWrite(actuatorPins[i], LOW);

        // Trigger LED_BUILTIN
        digitalWrite(LED_BUILTIN, LOW);
      }
    }

    // Relay the data back through serial
    Serial.print('!'); // Start character
    for (int i = 0; i < num_actuators_used; i++)
    {
      Serial.print(static_cast<char>(actuator_states[i])); // Actuator state as char
    }
    Serial.println(); // End of message
  }
}
