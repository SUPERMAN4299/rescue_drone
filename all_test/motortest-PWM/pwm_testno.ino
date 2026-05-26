// Battery GND
// MOSFET Source
// Arduino GND
// 9600 baund 
// battery - 3.3v
// mAh - more and more

// motor increase -> motor decrease -> next loop

const int motorPin = 5;

int throttle = 0;

void setup() {

  pinMode(motorPin, OUTPUT);

  Serial.begin(9600);

  Serial.println("CORELESS MOTOR TEST");
}

void loop() {

  // =========================
  // START MOTOR SLOWLY
  // =========================

  for(throttle = 0; throttle <= 255; throttle++) {

    analogWrite(motorPin, throttle);

    Serial.print("Throttle: ");
    Serial.println(throttle);

    delay(25);
  }

  // =========================
  // FULL SPEED HOLD
  // =========================

  Serial.println("FULL SPEED");

  delay(3000);

  // =========================
  // SLOW DOWN
  // =========================

  for(throttle = 255; throttle >= 0; throttle--) {

    analogWrite(motorPin, throttle);

    Serial.print("Throttle: ");
    Serial.println(throttle);

    delay(25);
  }

  Serial.println("STOPPED");

  delay(3000);
}