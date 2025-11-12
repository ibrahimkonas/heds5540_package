// measure_manual_one_rev.ino
// Elle 1 tur ölçümü (Z yoksa). A->D2, B->D3. Harici pull-ups 2.7k.

const uint8_t pinA = 2;
const uint8_t pinB = 3;

volatile unsigned long quadCount = 0;
bool measuring = false;

void isrAB() {
  if (measuring) quadCount++;
}

void setup() {
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);

  attachInterrupt(digitalPinToInterrupt(pinA), isrAB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), isrAB, CHANGE);

  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("Elle 1 tur ölçümü - 's' başlat, 'e' bitir");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's') {
      noInterrupts();
      quadCount = 0;
      measuring = true;
      interrupts();
      Serial.println("Ölçüm başladı - bir tur döndürün, sonra 'e' ile bitirin.");
    }
    else if (c == 'e') {
      noInterrupts();
      unsigned long measured = quadCount;
      measuring = false;
      interrupts();

      double ppr_est = measured / 4.0;
      Serial.print("Quadrature edges counted = ");
      Serial.println(measured);
      Serial.print("Estimated PPR (cycles/rev) = ");
      Serial.println(ppr_est, 4);
      Serial.println("--- ölçüm tamamlandı ---");
    }
  }
}