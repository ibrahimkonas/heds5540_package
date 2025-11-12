// HEDS5540_Nano_fast.ino
// Tek-kanal RISING yöntemi (performans için), doğrudan port okuma.
// D2 = A (INT0), D3 = B, harici 2.7k pull-up kullanın.

const uint8_t pinA = 2; // INT0 (PD2)
const uint8_t pinB = 3; // PD3

volatile long position = 0;
volatile unsigned long pulses = 0;

const unsigned int PPR = 200; // datasheet'teki gerçek değeri buraya yazın
const unsigned int countsPerRev = PPR * 4;

unsigned long lastReport = 0;
const unsigned long reportIntervalMs = 500;

void setup() {
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);

  Serial.begin(115200);
  while (!Serial) { ; }

  attachInterrupt(digitalPinToInterrupt(pinA), isrA_rising, RISING);

  Serial.println("HEDS-5540 Nano test (tek-kanal RISING) başladı");
}

void loop() {
  unsigned long now = millis();
  if (now - lastReport >= reportIntervalMs) {
    noInterrupts();
    unsigned long p = pulses;
    long pos = position;
    pulses = 0;
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

void isrA_rising() {
  bool b = (PIND & (1 << PD3)); // PD3 = D3
  if (b) position++;
  else position--;
  pulses += 1;
}