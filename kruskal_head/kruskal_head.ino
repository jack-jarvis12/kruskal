#include <Servo.h>

Servo servoA;  // Servo on pin 9
Servo servoB;  // Servo on pin 8
Servo servoC;  // Servo on pin 7

void setup() {
  Serial.begin(9600);
  servoA.attach(9);
  servoB.attach(8);
  servoC.attach(7);

  Serial.println("Send a <number>, b <number>, or c <number> to control servos on pins 9, 8, and 7. Send d <number> to control 7 & 8.");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Read incoming data
    input.trim();  // Remove any whitespace

    if (input.length() > 1) {  // Ensure the command is at least two characters
      char servoID = input.charAt(0);  // Get the first character
      String numberStr = input.substring(1);
      numberStr.trim();
      int angle = numberStr.toInt();  // Convert the string to an integer

      if (angle >= 0 && angle <= 180) {
        switch (servoID) {
          case 'a':
            servoA.write(angle);
            Serial.print("Servo a moved to: ");
            break;
          case 'b':
            servoB.write(angle);
            Serial.print("Servo b moved to: ");
            break;
          case 'c':
            servoC.write(angle);
            Serial.print("Servo c moved to: ");
            break;
          case 'd':
            servoB.write(angle);
            servoC.write(180 - angle);
            Serial.print("Servo b & c moved to: ");
            break;
          default:
            Serial.println("Error: Invalid servo identifier. Use a, b, c, or d.");
            return;
        }
        Serial.println(angle);
      } else {
        Serial.println("Error: Angle must be between 0 and 180.");
      }
    } else {
      Serial.println("Error: Invalid command format. Use a <number>, b <number>, c <number>, or d <number>.");
    }
  }
}


// Debug Script
// void loop() {
//     // Move to 0 degrees
//     servoA.write(0);
//     servoB.write(0);
//     servoC.write(0);
//     delay(1000);

//     // Move to 180 degrees
//     servoA.write(180);
//     servoB.write(180);
//     servoC.write(180);
//     delay(1000);
// }