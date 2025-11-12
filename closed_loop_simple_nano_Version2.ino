// closed_loop_simple_nano.ino
// Basit kapalı çevrim deneme (Arduino Nano) - düşük hız/test amaçlı
// STEP -> D9, DIR -> D8, ENABLE -> D10 (opsiyonel)
// A -> D2, B -> D3
// Harici pull-ups A,B -> 2.7k -> 5V

const uint8_t PIN_STEP = 9;
const uint8_t PIN_DIR  = 8;
const uint8_t PIN_EN   = 10;

const uint8_t PIN_A = 2;
const uint8_t PIN_B = 3;
volatile long encoder_count = 0;

const long motor_full_steps_per_rev = 200;
const int microstep = 16;
const long motor_steps_per_rev = motor_full_steps_per_rev * (long)microstep;

volatile long target_steps = 0;

void setup() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, LOW);

  pinMode(PIN_A, INPUT);
  pinMode(PIN_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_A), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), isrB, CHANGE);

  Serial.begin(115200);
  while(!Serial);

  Serial.println("Basit closed-loop test hazır. 'g' ileri 1000, 'r' geri 1000.");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'g') { target_steps += 1000; Serial.println("Hedef +1000"); }
    else if (c == 'r') { target_steps -= 1000; Serial.println("Hedef -1000"); }
  }

  noInterrupts();
  long enc = encoder_count;
  long tgt = target_steps;
  interrupts();

  if (tgt > enc) {
    digitalWrite(PIN_DIR, HIGH);
    singleStep();
    delay(2);
  } else if (tgt < enc) {
    digitalWrite(PIN_DIR, LOW);
    singleStep();
    delay(2);
  }

  static unsigned long last=0;
  if (millis()-last>500) {
    Serial.print("Encoder counts: "); Serial.print(enc);
    Serial.print(" | Target steps: "); Serial.println(tgt);
    last = millis();
  }
}

void singleStep() {
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(8);
  digitalWrite(PIN_STEP, LOW);
  delayMicroseconds(8);
}

void isrA() {
  bool a = digitalRead(PIN_A);
  bool b = digitalRead(PIN_B);
  if (a == b) encoder_count++; else encoder_count--;
}
void isrB() {
  bool a = digitalRead(PIN_A);
  bool b = digitalRead(PIN_B);
  if (a != b) encoder_count++; else encoder_count--;
}