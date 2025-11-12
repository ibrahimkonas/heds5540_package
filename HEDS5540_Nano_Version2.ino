// HEDS5540_Nano.ino
// Arduino Nano test sketch (A->D2, B->D3). Harici 2.7k pull-up kullanın.

const uint8_t pinA = 2; // INT0
const uint8_t pinB = 3; // INT1
const uint8_t pinZ = 4; // index (opsiyonel)

volatile long position = 0;
volatile unsigned long pulses = 0;

const unsigned int PPR = 128; // datasheet'ten doğrulayın
const unsigned int countsPerRev = PPR * 4;

unsigned long lastReport = 0;
const unsigned long reportIntervalMs = 500;

void setup() {
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  pinMode(pinZ, INPUT);

  attachInterrupt(digitalPinToInterrupt(pinA), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), isrB, CHANGE);

  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("HEDS-5540 test (Nano) başlatıldı");
}

void loop() {
  unsigned long now = millis();
  if (now - lastReport >= reportIntervalMs) {
    noInterrupts();
    unsigned long p = pulses;
    pulses = 0;
    long pos = position;
    interrupts();

    double intervalSec = (now - lastReport) / 1000.0;
    double revs = (double)p / (double)countsPerRev;
    double rpm = (revs / intervalSec) * 60.0;
    double angle = (double)pos * 360.0 / (double)countsPerRev;

    Serial.print("RPM: ");
    Serial.print(rpm, 2);
    Serial.print(" | Pos(counts): ");
    Serial.print(pos);
    Serial.print(" | Angle(deg): ");
    Serial.println(angle, 2);

    lastReport = now;
  }
}

void isrA() { updateFromPins(); }
void isrB() { updateFromPins(); }

inline void updateFromPins() {
  bool a = digitalRead(pinA);
  bool b = digitalRead(pinB);
  if (a == b) position++;
  else position--;
  pulses++;
}