#define TRIG_PIN 9
#define ECHO_PIN 10
#define irSensorPin 2
#define buzzerPin 3

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(irSensorPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
}

void loop() {
  // --- IR Sensor Check ---
  int irValue = digitalRead(irSensorPin);

  if (irValue == LOW) {
    // IR sensor detects an object (usually LOW for detection)
    tone(buzzerPin, 3000);  // 3000 Hz tone
    delay(500);
    noTone(buzzerPin);
  } else {
    // --- Ultrasonic Sensor Check ---
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // Timeout after 30 ms

    if (duration == 0) {
      Serial.println("Error: No echo received.");
      noTone(buzzerPin);
    } else {
      long distance = (duration / 2) / 29.1;

      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");

      if (distance <= 25) {
        tone(buzzerPin, 1000);  // 1000 Hz tone
        delay(500);
        noTone(buzzerPin);
      } else {
        noTone(buzzerPin);  // Nothing detected, buzzer off
      }
    }
  }

  delay(100); // Small delay to ease sensor polling
}
