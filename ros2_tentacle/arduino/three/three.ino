#include <Arduino.h>

// Declare pins for controlling actuators
const int actuatorPins[] = {2, 3, 4};

// int n = 8;

void setup() {
  Serial.begin(115200);


}

void loop() {
  if (Serial.available() >= 9) { // Check if there's enough data available to read
    // Read incoming data
    // uint8_t buffer[n];
    char buffer[9];
    Serial.readBytes(buffer, 9);
    Serial.flush();

    // Parse received data
    char state1, state2, state3;
    memcpy(&state1, &buffer[0], sizeof(char));
    memcpy(&state2, &buffer[1], sizeof(char));
    memcpy(&state3, &buffer[2], sizeof(char));

    // Relay the data back through serial
    Serial.print('!'); // Start character
    Serial.print(static_cast<char>(state1)); // Speed value as char
    Serial.print(static_cast<char>(state2)); // Steering value as char
    Serial.print(static_cast<char>(state3)); // Steering value as char
    Serial.println(); // End of message
  }
}
