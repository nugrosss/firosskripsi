/*
 * Arduino - Kontrol Servo Tilt
 * Menerima perintah serial dari ROS2 node servo_tilt
 * Format: T<derajat>  contoh: T45.00
 */

#include <Servo.h>

Servo servoTilt;
const int SERVO_PIN = 9;

void setup() {
  Serial.begin(9600);
  servoTilt.attach(SERVO_PIN);
  servoTilt.write(90);  // posisi tengah
  Serial.println("Servo Tilt siap");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("T")) {
      float degree = input.substring(1).toFloat();
      degree = constrain(degree, 0, 180);
      servoTilt.write((int)degree);
      Serial.print("OK ");
      Serial.println(degree);
    }
  }
}
