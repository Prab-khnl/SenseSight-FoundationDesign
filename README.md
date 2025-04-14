# SenseSight-FoundationDesign

## Team Members
- **Brandon Brown** — `bjb754` — EE  
- **Juan Mateo** — `jm5487`  
- **Micah Coleman** — `mac1354` — CPE  
- **Prabesh Khanal** — `pk571` — CPE  

---

## Project Description

This repository contains the Arduino Uno code for **SenseSight**, a wearable assistive device for visually impaired individuals.

The system combines:
- **ESP32-CAM** trained with a FOMO (Faster Objects, More Objects) model using **Edge Impulse**
- **Ultrasonic sensor** for distance measurement
- **IR obstacle detection sensor** for detecting close objects
- **DFPlayer Mini** module with a speaker for audio feedback


### Object Detection:
The ESP32-CAM detects four key objects:
- Wall
- Chair
- Door
- Stool

These are critical for obstacle navigation. Detected objects are sent via serial communication to the Arduino Uno, which then triggers corresponding audio signal using the DFPlayer Mini.

### Collision Avoidance:
In addition to object recognition, the ultrasonic and IR sensors detect nearby obstacles. When a potential collision is detected (object closer than 25 cm), a buzzer alerts the user.
 In this way, it can help blind person navigate safely without coliding with the objects.

---

## Arduino Uno Code

```cpp
// Team Members:-
// Brandon Brown bjb754 EE
// Juan Mateo Jm5487
// Micah Coleman mac1354 CPE
// Prabesh Khanal pk571 CPE

#define TRIG_PIN 9
#define ECHO_PIN 12
#define irSensorPin 2
#define buzzerPin 3
#define switchPin 4

#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

// SoftwareSerial for DFPlayer Mini (RX = 5, TX = 6)
SoftwareSerial mySoftwareSerial(5, 6); // DFPlayer Mini RX, TX
DFRobotDFPlayerMini myDFPlayer;

void setup() {
  Serial.begin(115200); // Communication with ESP32-CAM
  mySoftwareSerial.begin(9600); // DFPlayer Mini communication

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(irSensorPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP);  // Internal pull-up resistor

  Serial.println(F("Initializing DFPlayer..."));
  if (!myDFPlayer.begin(mySoftwareSerial)) {
    Serial.println(F("DFPlayer Mini not detected!"));
    while (1);
  }
  myDFPlayer.volume(30);  // Max volume
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
  Serial.println(F("DFPlayer Initialized!"));
}

void loop() {
  int irValue = digitalRead(irSensorPin);
  int switchState = digitalRead(switchPin);  // LOW = Pressed

  Serial.print("Switch state: ");
  Serial.println(switchState == LOW ? 1 : 0);

  // Check for messages from ESP32-CAM
  if (Serial.available()) {
    String message = Serial.readStringUntil('\n');
    message.trim();

    if (message.length() > 0) {
      Serial.println(message);

      if ((message == "Chair" || message.startsWith("C")) && switchState == LOW) {
        myDFPlayer.play(1);
      } else if ((message == "Door" || message.startsWith("D")) && switchState == LOW) {
        myDFPlayer.play(2);
      } else if ((message == "Stool" || message.startsWith("S")) && switchState == LOW) {
        myDFPlayer.play(4);  // 4 corresponds to 3.mp3
      } else if ((message == "Wall" || message.startsWith("W")) && switchState == LOW) {
        myDFPlayer.play(3);  // 3 corresponds to 4.mp3
      } else if ((message == "None" || message.startsWith("N")) && switchState == LOW) {
        myDFPlayer.stop();
      }
    }
  }

  // IR Obstacle Detection
  if (irValue == LOW) {
    tone(buzzerPin, 3000);
    delay(500);
    noTone(buzzerPin);
  } else {
    // Ultrasonic Distance Detection
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    if (duration > 0) {
      long distance = (duration / 2) / 29.1;
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");

      if (distance <= 25) {
        tone(buzzerPin, 1000);
        delay(500);
        noTone(buzzerPin);
      }
    } else {
      Serial.println("Ultrasonic error: no echo");
    }
  }

  delay(100);
}
