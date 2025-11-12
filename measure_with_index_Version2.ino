// measure_with_index.ino
// Otomatik bir devir ölçümü: A->D2, B->D3 (CHANGE), Z->D4 (index)
// Harici pull-ups 2.7k kullanın.

const uint8_t pinA = 2;
const uint8_t pinB = 3;
const uint8_t pinZ = 4;

volatile unsigned long quadCount = 0;
volatile bool newRev = false;
volatile unsigned long lastRevCount = 0;

void isrAB() {
  quadCount++;
}

void isrZ() {
  lastRevCount = quadCount;
  quadCount = 0;
  newRev = true;
}

void setup() {
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  pinMode(pinZ, INPUT);

  attachInterrupt(digitalPinToInterrupt(pinA), isrAB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), isrAB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinZ), isrZ, RISING);

  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("Z ile PPR ölçüm - döndürün.");
}

void loop() {
  if (newRev) {
    noInterrupts();
    unsigned long measured = lastRevCount;
    newRev = false;
    interrupts();

    double quadCounts = (double)measured;
    double PPR_est = quadCounts / 4.0;
    Serial.print("Quadrature edges/rev = ");
    Serial.print(measured);
    Serial.print("  -> Estimated PPR (cycles/rev) = ");
    Serial.println(PPR_est, 4);
    Serial.println("--- bekleniyor ---");
  }
}