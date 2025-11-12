/*
  closed_loop_nano_tb6600.ino
  Arduino Nano + TB6600 + HEDS-5540 (no LS7366R / no STM32)
  - A -> D2 (INT0), B -> D3
  - STEP -> D9 (OC1A), DIR -> D8, ENABLE -> D10 (optional)
  - Use external 2.7k pull-ups on A and B to +5V (datasheet recommended)
  - Timer1 used to generate step square wave by toggling OC1A (D9)
  - ISR on A (RISING) reads B via direct port read to determine direction
  - PID loop runs in main loop to adjust step frequency
  NOTES:
  - You MUST set encoder_counts_per_rev to the measured quadrature edges per revolution.
  - encoder_counts_per_step = encoder_counts_per_rev / motor_steps_per_rev
  - If encoder_counts_per_step << 1 then encoder cannot resolve single microstep.
*/

#include <Arduino.h>

// Pins
const uint8_t PIN_A   = 2;  // INT0
const uint8_t PIN_B   = 3;
const uint8_t PIN_Z   = 4;  // optional index
const uint8_t PIN_DIR = 8;
const uint8_t PIN_STEP = 9; // OC1A
const uint8_t PIN_EN  = 10; // optional enable on TB6600 (active low/high depends on board)

// Hardware / mechanical params (SET THESE)
const long motor_full_steps_per_rev = 200; // e.g. 200 for 1.8deg motor
const int microstep = 16; // TB6600 DIP setting - must match
const long motor_steps_per_rev = motor_full_steps_per_rev * (long)microstep;

// ENCODER: you MUST measure this (quadrature edges per revolution)
volatile long encoder_counts_per_rev = 800; // <-- REPLACE with measured quadrature counts/rev
// If you only measured A rising edges and want to use that, adjust code accordingly.

double encoder_counts_per_step = 0.0; // computed later

// Control limits
const double MAX_STEP_FREQ_HZ = 2000.0; // maximum step frequency to allow (tune as needed)
const double MIN_STEP_FREQ_HZ = 0.5;

// PID params (tune these)
double Kp = 0.8;
double Ki = 0.04;
double Kd = 0.0;

volatile long encoder_count = 0; // quadrature counts (edges) — updated in ISR
volatile bool index_flag = false; // Z pulse detected

// Target in motor microsteps
volatile long target_steps = 0;

// Internal for PID
double integral = 0;
double lastError = 0;

// function prototypes
void isrA_rising();
void isrZ_rising();
void setupTimer1Disabled();
bool setStepFrequency(double freqHz);
void stopSteps();

void setup() {
  cli();
  pinMode(PIN_A, INPUT);
  pinMode(PIN_B, INPUT);
  pinMode(PIN_Z, INPUT);

  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_EN, OUTPUT);

  // Ensure STEP low initially
  digitalWrite(PIN_STEP, LOW);
  // Enable driver if active low/high: adjust according to your TB6600 board
  digitalWrite(PIN_EN, LOW); // try this; if enable logic is reversed set HIGH

  // compute conversion factor
  if (encoder_counts_per_rev > 0) {
    encoder_counts_per_step = (double)encoder_counts_per_rev / (double)motor_steps_per_rev;
  } else {
    encoder_counts_per_step = 0.0;
  }

  // Attach interrupts: use RISING on A to reduce ISR rate
  attachInterrupt(digitalPinToInterrupt(PIN_A), isrA_rising, RISING);
  // optional index homing
  // attachInterrupt(digitalPinToInterrupt(PIN_Z), isrZ_rising, RISING);

  // Disable Timer1 toggling at start
  setupTimer1Disabled();

  Serial.begin(115200);
  while(!Serial);

  Serial.println("Closed-loop Nano+TB6600 example started.");
  Serial.print("Encoder counts/rev (configured): "); Serial.println(encoder_counts_per_rev);
  Serial.print("Motor steps/rev: "); Serial.println(motor_steps_per_rev);
  Serial.print("Encoder counts per motor step (counts/step): ");
  Serial.println(encoder_counts_per_step, 6);
  Serial.println("Commands: 'g' add +1000 steps, 'r' -1000 steps, 's' stop, 'p' print status");
  sei();
}

unsigned long lastLoopMs = 0;
const unsigned long loopDtMs = 20; // control loop period (~50 Hz)

void loop() {
  // Serial commands for testing
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'g') {
      target_steps += 1000;
      Serial.println("Target += 1000");
    } else if (c == 'r') {
      target_steps -= 1000;
      Serial.println("Target -= 1000");
    } else if (c == 's') {
      target_steps = 0;
      stopSteps();
      Serial.println("Target = 0, steps stopped");
    } else if (c == 'p') {
      noInterrupts();
      long enc = encoder_count;
      interrupts();
      Serial.print("enc_counts="); Serial.print(enc);
      Serial.print(" target_steps="); Serial.print(target_steps);
      Serial.print(" enc_counts_per_step="); Serial.println(encoder_counts_per_step, 6);
    }
  }

  unsigned long now = millis();
  if (now - lastLoopMs >= loopDtMs) {
    lastLoopMs = now;

    // Read encoder count atomically
    noInterrupts();
    long enc = encoder_count;
    interrupts();

    // Convert encoder counts -> motor microsteps (float)
    double actual_steps = 0.0;
    if (encoder_counts_per_step > 0.0) actual_steps = (double)enc / encoder_counts_per_step;
    long actual_steps_round = (long)round(actual_steps);

    // Compute error (in microsteps)
    double error = (double)target_steps - actual_steps;

    // Simple PID (output interpreted as desired step freq in steps/sec)
    double dt = (double)loopDtMs / 1000.0;
    integral += error * dt;
    double derivative = (error - lastError) / dt;
    double output = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;

    // Convert PID output to frequency (magnitude)
    double desired_freq = fabs(output); // steps per second

    // Limit frequency
    if (desired_freq > MAX_STEP_FREQ_HZ) desired_freq = MAX_STEP_FREQ_HZ;
    if (desired_freq < MIN_STEP_FREQ_HZ) desired_freq = 0.0;

    // Set direction according to sign(output) (or sign(error))
    if (output >= 0) digitalWrite(PIN_DIR, HIGH);
    else digitalWrite(PIN_DIR, LOW);

    // Update Timer1 to produce step pulses at desired_freq
    if (desired_freq > 0.0) {
      bool ok = setStepFrequency(desired_freq);
      if (!ok) {
        // could not set exact frequency (out of range) - clamp and/or stop
        stopSteps();
      }
    } else {
      stopSteps();
    }

    // Periodic debug print
    static unsigned long lastPrint = 0;
    if (now - lastPrint >= 500) {
      Serial.print("Tgt:"); Serial.print(target_steps);
      Serial.print(" Act:"); Serial.print(actual_steps_round);
      Serial.print(" Err:"); Serial.print((long)round(error));
      Serial.print(" Freq:"); Serial.print(desired_freq,1);
      Serial.print(" enc_counts:"); Serial.println(enc);
      lastPrint = now;
    }
  }
}

/*** ISR: A rising edge, read B via direct port read for direction ***/
void isrA_rising() {
  // PD3 is D3 - read port D bit 3
  bool b = (PIND & (1 << PD3));
  if (b) encoder_count++;
  else encoder_count--;
}

void isrZ_rising() {
  // optional homing index
  noInterrupts();
  encoder_count = 0;
  index_flag = true;
  interrupts();
}

/*** Timer1 control functions ***/

// Disable Timer1 output (disconnect OC1A) and stop timer
void setupTimer1Disabled() {
  cli();
  // Disconnect OC1A, clear timer config
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0;
  OCR1A = 0;
  // Ensure pin low
  digitalWrite(PIN_STEP, LOW);
  sei();
}

// Map prescaler code
struct Presc { uint16_t val; uint8_t csbits; };
const Presc prescalers[] = {
  {1,   (1<<CS10)},
  {8,   (1<<CS11)},
  {64,  (1<<CS11)|(1<<CS10)},
  {256, (1<<CS12)},
  {1024,(1<<CS12)|(1<<CS10)}
};

// Set Timer1 to toggle OC1A at frequency = desired step frequency (rising edges per sec = freq)
// Returns true if set, false if desired_freq == 0 or out of range
bool setStepFrequency(double freqHz) {
  if (freqHz <= 0.0) {
    setupTimer1Disabled();
    return false;
  }

  const double F_CPU_D = (double)F_CPU; // typically 16000000
  // We use CTC with toggle OC1A: f = F_CPU / (2 * prescaler * (OCR1A + 1))
  // => OCR1A = F_CPU / (2 * prescaler * f) - 1

  uint16_t bestOCR = 0;
  uint8_t bestCS = 0;
  bool found = false;

  for (unsigned i = 0; i < sizeof(prescalers)/sizeof(Presc); ++i) {
    double pres = (double)prescalers[i].val;
    double ocrd = F_CPU_D / (2.0 * pres * freqHz) - 1.0;
    if (ocrd >= 0.0 && ocrd <= 65535.0) {
      // choose this prescaler (prefer smallest prescaler for better resolution)
      bestOCR = (uint16_t)round(ocrd);
      bestCS = prescalers[i].csbits;
      found = true;
      break;
    }
  }

  if (!found) {
    // Can't represent this frequency with Timer1 prescalers; clamp or fail
    return false;
  }

  cli();
  // Set CTC mode (WGM12 = 1), toggle OC1A on compare match (COM1A0 = 1)
  TCCR1A = (1 << COM1A0); // toggle OC1A on compare match
  TCCR1B = (1 << WGM12);  // CTC mode
  OCR1A = bestOCR;
  // Set prescaler bits
  TCCR1B = (TCCR1B & ~((1<<CS12)|(1<<CS11)|(1<<CS10))) | bestCS;
  sei();

  return true;
}

void stopSteps() {
  setupTimer1Disabled();
}

/* 
  TUNING NOTES:
  - Measure encoder_counts_per_rev first and put it into encoder_counts_per_rev.
  - If encoder_counts_per_step < 0.5, single microstep won't be seen by encoder -> consider lowering microstep or using higher-res encoder.
  - Tune PID: start Ki = 0, Kd = 0. Increase Kp until you see stable response with acceptable overshoot. Then add Ki to remove steady-state error.
  - Apply acceleration limits externally if needed (limit rate of change of desired_freq).
  - Ensure TB6600 STEP pulse width is satisfied: with hardware toggling the pulse width = 1/(2*freq). At high freq this may be too short; ensure TB6600 accepts it.
  - For very low-level reliable pulses you may prefer Timer ISR to generate defined pulse width rather than hardware toggle.
*/