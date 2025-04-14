# SenseSight-FoundationDesign
Team Members:-
Brandon Brown bjb754 EE
Juan Mateo Jm5487
Micah Coleman mac1354 CPE
Prabesh Khanal pk571 CPE

This repo contains the code for sunglasses built for blind people using ESP32 AI Module, Ultrasonic Sensor, Infrared Sesnsor and Speakers.

This code integrates ESP32 CAM, Ultrasonic Sensor, IR obstacle detection sensor and HF Mini player with the Arduino Uno. The code communicates with ESP32 CAM over Serial communication which outputs the prediction of the object the ESP32 CAM detected. The ESP32 CAM was trained on FOMO (Faster Objects, More Objects) using EdgeImpuse. The objects of interest are Wall, Chair, Door and Stool. We believe these objects are crucial for blind person to detect so that they could navigate easily.Based upon the object detected, the Arduino Uno sends the corresponding sound (eg. Wall) for the HF Mini player to play on the speaker. The Ultrasonic and IR sensors are also working side by side to play the buzzing sound when the object is very close to the blind person. In this way, it can help blind person navigate safely with out coliding with the objects.

Working Arduino Uno code :-



//Team Members:-
//Brandon Brown bjb754 EE
//Juan Mateo Jm5487
//Micah Coleman mac1354 CPE
//Prabesh Khanal pk571 CPE



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
  // Start hardware serial for communication with ESP32-CAM (pins 0 and 1)
  Serial.begin(115200); // For Arduino to Serial Monitor and ESP32 communication

  // Start SoftwareSerial for DFPlayer Mini
  mySoftwareSerial.begin(9600); // For DFPlayer Mini communication

  // Set pin modes for other components
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(irSensorPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP);  // Pull-up resistor enabled

  // Initialize DFPlayer Mini
  Serial.println(F("Initializing DFPlayer..."));
  if (!myDFPlayer.begin(mySoftwareSerial)) {
    Serial.println(F("DFPlayer Mini not detected!"));
    while (1);  // Stay here if DFPlayer is not found
  }
  myDFPlayer.volume(30);  // Set volume level
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);  // Set EQ
  Serial.println(F("DFPlayer Initialized!"));
}

void loop() {
  // Read input from IR sensor and switch
  int irValue = digitalRead(irSensorPin);
  int switchState = digitalRead(switchPin);  // Read the state of the switch

  // Debugging: print the switch state
  Serial.print("Switch state: ");
  Serial.println(switchState == LOW ? 1 : 0); // LOW means the switch is pressed, as we are using INPUT_PULLUP

  // Check for incoming data from ESP32 (via hardware Serial)
  if (Serial.available()) {
    String message = Serial.readStringUntil('\n');
    message.trim();  // Trim any leading or trailing spaces

    if (message.length() > 0) {
      Serial.println(message);
      
      // Update the condition to check if the switch is pressed
      if ((message == "Chair" || message.startsWith("C")) && switchState == LOW) {
        myDFPlayer.play(1);  // Play track 1
      } else if ((message == "Door" || message.startsWith("D")) && switchState == LOW) {
        myDFPlayer.play(2);  // Play track 2
      } else if ((message == "Stool" || message.startsWith("S")) && switchState == LOW) {
        myDFPlayer.play(4);  // Somehow 4 is corresponding to 3.mp3 on memory card
      } else if ((message == "Wall" || message.startsWith("W")) && switchState == LOW) {
        myDFPlayer.play(3);  // Somehow 3 is corresponding to 4.mp3 on memory card
      } else if ((message == "None" || message.startsWith("N")) && switchState == LOW) {
        // Optional: Do nothing or stop the music
        myDFPlayer.stop();
      }
    }
  }

  // IR sensor logic
  if (irValue == LOW) {
    tone(buzzerPin, 3000);  // Make sound at 3000Hz
    delay(500);
    noTone(buzzerPin);  // Stop sound
  } else {
    // Measure distance using ultrasonic sensor
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // Read echo signal duration
    if (duration > 0) {
      long distance = (duration / 2) / 29.1;  // Calculate distance in cm
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");

      // Trigger buzzer if distance is less than or equal to 25 cm
      if (distance <= 25) {
        tone(buzzerPin, 1000);  // Make sound at 1000Hz
        delay(500);
        noTone(buzzerPin);  // Stop sound
      }
    } else {
      Serial.println("Ultrasonic error: no echo");
    }
  }

  delay(100);  // Small delay to stabilize loop
}
