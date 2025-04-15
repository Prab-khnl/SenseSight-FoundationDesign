// Define the IR sensor pin
const int irSensorPin = 2;

// Define the buzzer pin
const int buzzerPin = 3;

void setup() {
  // Set the IR sensor pin as an input
  pinMode(irSensorPin, INPUT);

  // Set the buzzer pin as an output
  pinMode(buzzerPin, OUTPUT);
}

void loop() {
  // Read the IR sensor value
  int sensorValue = digitalRead(irSensorPin);

  // If the sensor detects an object (or whatever state you want to trigger the buzzer)
  if (sensorValue == LOW) { // Or LOW, depending on your sensor's output
    // Turn on the buzzer
    tone(buzzerPin, 3000); // Play a tone at 3000 Hz
    delay(500); // Delay for 500 milliseconds
    noTone(buzzerPin); // Stop the tone
  }
  delay(10);
}



































/*const int IRSensorPin = 3;
const int LEDPin = 10;

void setup() {
  Serial.begin(9600);

  pinMode(IRSensorPin, INPUT);

  pinMode(LEDPin, OUTPUT);
}

void loop() {
  int sensorValue = digitalRead(IRSensorPin); // Read the IR sensor output

  if (sensorValue == LOW) {
    digitalWrite(LEDPin, HIGH);
    Serial.println("There is an obstacle!");
  } 
  
  else {
    digitalWrite(LEDPin, LOW);
    Serial.println("There is no obstacle!");
  }


  delay(2);
}
*/
