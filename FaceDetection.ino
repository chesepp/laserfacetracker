#include <Servo.h>

Servo servoX;
Servo servoY;
int ledPin1 = 13;//red
int ledPin2 = 12;//green
int laserPin = 8;
void setup()
{
  servoX.attach(9);
  servoY.attach(10);
  Serial.begin(9600);
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(laserPin, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n'); // Read incoming data
    if (data == "LED1_ON") {
      digitalWrite(ledPin1, HIGH);  // Turn LED1 ON
      digitalWrite(ledPin2, LOW);   // Turn LED2 OFF
      Serial.println("LED1 turned ON, LED2 turned OFF");
    } else if (data == "LED2_ON") {
      digitalWrite(ledPin1, LOW);   // Turn LED1 OFF
      digitalWrite(ledPin2, HIGH);  // Turn LED2 ON
      Serial.println("LED1 turned OFF, LED2 turned ON");
    } else if (data == "LED_OFF") {
      digitalWrite(ledPin1, LOW);   // Turn LED1 OFF
      digitalWrite(ledPin2, LOW);   // Turn LED2 OFF
      Serial.println("Both LEDs turned OFF");
    }
    else if (data == "LASER_ON") {
      digitalWrite(laserPin, HIGH);
    }
    else if (data == "LASER_OFF") {
      digitalWrite(laserPin, LOW);
    }
    else if (data.startsWith("Y:"))
    {
      int angleY = data.substring(2).toInt();
      angleY = constrain(angleY, 0,180); // Ensure the value is within servo range
      servoY.write(angleY);
    }
    else if (data.startsWith("X:"))
    {
      int angleX = data.substring(2).toInt();
      angleX = constrain(angleX, 0, 180); // Ensure the value is within servo range
      servoX.write(angleX);
    }
       // Convert data to an integer for the angle

    // Move the servo to the specified angle

    
  }
}
