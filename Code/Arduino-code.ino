float distance_ft =10;                 // one-way distance (5–10 ft)
float wheel_diam_in = 3.0;             // inches
const long COUNTS_PER_WHEEL_REV = 250; // measured wheel counts per rev

int PWM_MAX = 175;          // peak power (0–255)
int PWM_MIN = 60;           // minimum to actually move (tune 40–90)
int PWM_STEP = 5;           // ramp step size (kept for "manual" ramp fallback)
int RAMP_DT_MS = 30;        // time between updates (smaller = smoother)

// Buffers (counts)
long ACCEL_BUFFER_COUNTS = 750; // One rev is 250 counts. So 750 means after 3 wheel revolution the acceleration is at the desired value
long DECEL_BUFFER_COUNTS = 750; 

// Timing waits
const unsigned long STARTUP_DELAY_MS = 2000; // wait after power-up
const unsigned long DWELL_MS = 2000;         // wait at each end

// ====== PINS ======
const int IN1_PIN = 5;   // forward PWM
const int IN2_PIN = 6;   // reverse PWM
const int ENC_A   = 2;   // encoder A (interrupt)
const int ENC_B   = 3;   // encoder B (direction)

// Encoder 
volatile long count = 0;

void encoderISR() {
  // Quadrature direction using B state at A edge
  bool b = digitalRead(ENC_B);
  if (b) count--;
  else   count++;
}

long readCount() {
  noInterrupts();
  long c = count;
  interrupts();
  return c;
}

void resetCount() {
  noInterrupts();
  count = 0;
  interrupts();
}

// Motor control
void setMotorPWM(int pwmSigned) {
  int u = abs(pwmSigned);
  if (u > 255) u = 255;

  if (pwmSigned > 0) {            // forward
    analogWrite(IN1_PIN, u);
    digitalWrite(IN2_PIN, LOW);
  } else if (pwmSigned < 0) {     // reverse
    digitalWrite(IN1_PIN, LOW);
    analogWrite(IN2_PIN, u);
  } else {                        // stop
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
  }
}

// Distance 
long computeTargetCounts() {
  float wheel_diam_m = wheel_diam_in * 0.0254f;
  float C = PI * wheel_diam_m;         // m per wheel rev
  float D = distance_ft * 0.3048f;     // m
  float revs = D / C;
  return (long)lround(revs * (float)COUNTS_PER_WHEEL_REV);
}

// Move one-way with accel + decel
void moveOneWay(int dir, long targetCounts) {
  // dir = +1 forward, -1 backward
  resetCount();

  long traveled = 0;

  // Run until target, shaping speed with accel + decel buffers 
  while (true) {
    traveled = labs(readCount());
    long remaining = targetCounts - traveled;

    if (remaining <= 0) break;

    //  Accel scaling over ACCEL_BUFFER_COUNTS 
    float accelFrac = 1.0f;
    if (ACCEL_BUFFER_COUNTS > 0 && traveled < ACCEL_BUFFER_COUNTS) {
      accelFrac = (float)traveled / (float)ACCEL_BUFFER_COUNTS;
      if (accelFrac < 0.0f) accelFrac = 0.0f;
      if (accelFrac > 1.0f) accelFrac = 1.0f;
    }

    //  Decel scaling over DECEL_BUFFER_COUNTS 
    float decelFrac = 1.0f;
    if (DECEL_BUFFER_COUNTS > 0 && remaining < DECEL_BUFFER_COUNTS) {
      decelFrac = (float)remaining / (float)DECEL_BUFFER_COUNTS;
      if (decelFrac < 0.0f) decelFrac = 0.0f;
      if (decelFrac > 1.0f) decelFrac = 1.0f;
    }

    // Combine: whichever is smaller limits the speed (slow at start, slow at end)
    float frac = accelFrac;
    if (decelFrac < frac) frac = decelFrac;

    // Command PWM from MIN..MAX using combined fraction
    int pwmCmd = (int)lround(PWM_MIN + (PWM_MAX - PWM_MIN) * frac);

    // Safety clamps
    if (pwmCmd > PWM_MAX) pwmCmd = PWM_MAX;
    if (pwmCmd < PWM_MIN) pwmCmd = PWM_MIN;

    setMotorPWM(dir * pwmCmd);
    delay(RAMP_DT_MS);
  }

  // stop
  setMotorPWM(0);
}

void setup() {
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, RISING);

  Serial.begin(115200);
  delay(100);

  delay(STARTUP_DELAY_MS);

  Serial.println("Distance move start");
}

void loop() {
  long targetCounts = computeTargetCounts();
  Serial.print("Target counts = ");
  Serial.println(targetCounts);

  // OUT
  moveOneWay(+1, targetCounts);
  delay(DWELL_MS);

  // BACK
  moveOneWay(-1, targetCounts);
  delay(DWELL_MS);

  while(1);
}
}
