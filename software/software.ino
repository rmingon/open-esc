#define AH_PIN 8
#define BH_PIN 7
#define CH_PIN 6

#define AL_PIN 9
#define BL_PIN 10
#define CL_PIN 11

#define PHASE_A_ADC A2
#define PHASE_B_ADC A1
#define PHASE_C_ADC A0

#define PWM_INPUT_PIN 2  // RC-style PWM input (1–2 ms pulse)

const int8_t commutation_table[6][3] = {
  {1, 0, -1}, {1, -1, 0}, {0, 1, -1},
  {-1, 1, 0}, {0, -1, 1}, {-1, 0, 1}
};

volatile uint8_t step = 0;
volatile bool closedLoop = false;
volatile uint16_t dutyCycle = 0;
volatile uint16_t targetDutyCycle = 128;
volatile unsigned long lastStepTime = 0;
volatile unsigned int commutationDelay = 2000;
volatile bool zeroCrossDetected = false;
volatile int16_t lastADCValue = 512;
int8_t direction = 1;

uint16_t zeroCrossThresholdA = 512;
uint16_t zeroCrossThresholdB = 512;
uint16_t zeroCrossThresholdC = 512;
bool calibrationDone = false;

volatile unsigned long pwmRiseTime = 0;
volatile uint16_t pwmPulseWidth = 1500;

char inputBuffer[10];
uint8_t inputIndex = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("ESC Starting...");

  // Set pin modes
  pinMode(AH_PIN, OUTPUT); digitalWrite(AH_PIN, LOW);
  pinMode(BH_PIN, OUTPUT); digitalWrite(BH_PIN, LOW);
  pinMode(CH_PIN, OUTPUT); digitalWrite(CH_PIN, LOW);

  pinMode(AL_PIN, OUTPUT); digitalWrite(AL_PIN, LOW);
  pinMode(BL_PIN, OUTPUT); digitalWrite(BL_PIN, LOW);
  pinMode(CL_PIN, OUTPUT); digitalWrite(CL_PIN, LOW);

  pinMode(PWM_INPUT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PWM_INPUT_PIN), pwmISR, CHANGE);

  setupTimer1PWM();
  setupADC();
  calibrateZeroCrossThresholds();

  lastStepTime = micros();
}

void loop() {
  handlePWMAction();
  handleSerial();

  if (!closedLoop) {
    openLoopStartup();
  } else {
    if (zeroCrossDetected) {
      zeroCrossDetected = false;
      commutateStep();
    }
  }
}

// === INTERRUPT-BASED PWM INPUT ===
void pwmISR() {
  if (digitalRead(PWM_INPUT_PIN) == HIGH) {
    pwmRiseTime = micros();
  } else {
    unsigned long now = micros();
    pwmPulseWidth = now - pwmRiseTime;
  }
}

// === HANDLE RC INPUT TO SET SPEED/DIRECTION ===
void handlePWMAction() {
  if (pwmPulseWidth > 1520) {
    direction = 1;
    targetDutyCycle = map(pwmPulseWidth, 1520, 2000, 0, 255);
  } else if (pwmPulseWidth < 1480) {
    direction = -1;
    targetDutyCycle = map(pwmPulseWidth, 1000, 1480, 255, 0);
  } else {
    direction = 0;
    targetDutyCycle = 0;
    brakeMotor();
  }
}

// === BRAKING ===
void brakeMotor() {
  disableAllOutputs();
  digitalWrite(AL_PIN, HIGH);
  digitalWrite(BL_PIN, HIGH);
  digitalWrite(CL_PIN, HIGH);
}

void setupTimer1PWM() {
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10); // Fast PWM 8-bit
  TCCR1B = (1 << WGM12) | (1 << CS11);                  // prescaler 8
  OCR1A = 0;
  OCR1B = 0;
}

void setupADC() {
  ADMUX = (1 << REFS0) | (PHASE_C_ADC & 0x07);
  ADCSRA = (1 << ADEN) | (1 << ADATE) | (1 << ADIE) | (1 << ADSC)
         | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  sei();
}

void calibrateZeroCrossThresholds() {
  Serial.println("Calibrating...");
  zeroCrossThresholdA = calibrateADCPhase(PHASE_A_ADC);
  zeroCrossThresholdB = calibrateADCPhase(PHASE_B_ADC);
  zeroCrossThresholdC = calibrateADCPhase(PHASE_C_ADC);
  calibrationDone = true;
  Serial.println("Calibration complete.");
}

uint16_t calibrateADCPhase(uint8_t channel) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 50; i++) {
    ADMUX = (ADMUX & 0xF8) | (channel & 0x07);
    delay(2);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    sum += ADC;
  }
  return sum / 50;
}

void openLoopStartup() {
  unsigned long now = micros();
  if (now - lastStepTime >= commutationDelay) {
    commutateStep();
    lastStepTime = now;

    if (commutationDelay > 800) commutationDelay -= 5;
    if (dutyCycle < targetDutyCycle) dutyCycle++;

    OCR1A = dutyCycle;
    OCR1B = dutyCycle;

    if (commutationDelay <= 800) {
      closedLoop = true;
      Serial.println("Entering closed-loop mode");
    }
  }
}

void commutateStep() {
  step = (step + direction + 6) % 6;

  int8_t a = commutation_table[step][0];
  int8_t b = commutation_table[step][1];
  int8_t c = commutation_table[step][2];

  disableAllOutputs();
  delayMicroseconds(5); // Dead-time

  OCR1A = (a == -1) ? dutyCycle : 0;
  OCR1B = (b == -1) ? dutyCycle : 0;
  digitalWrite(CL_PIN, (c == -1) ? HIGH : LOW);

  digitalWrite(AH_PIN, (a == 1) ? HIGH : LOW);
  digitalWrite(BH_PIN, (b == 1) ? HIGH : LOW);
  digitalWrite(CH_PIN, (c == 1) ? HIGH : LOW);
}

void disableAllOutputs() {
  digitalWrite(AH_PIN, LOW);
  digitalWrite(BH_PIN, LOW);
  digitalWrite(CH_PIN, LOW);
  OCR1A = 0;
  OCR1B = 0;
  digitalWrite(CL_PIN, LOW);
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      inputBuffer[inputIndex] = 0;
      int val = atoi(inputBuffer);
      if (val >= 0 && val <= 255) {
        targetDutyCycle = val;
        Serial.print("Set duty: ");
        Serial.println(val);
      }
      inputIndex = 0;
    } else {
      if (inputIndex < sizeof(inputBuffer) - 1) {
        inputBuffer[inputIndex++] = c;
      }
    }
  }
}

// zc detection
ISR(ADC_vect) {
  static uint8_t currentPhase = 0;
  int16_t adcVal = ADC;
  if (!calibrationDone) return;

  uint16_t threshold = 512;
  uint8_t sense = getFloatingPhaseADC(step);
  switch (sense) {
    case PHASE_A_ADC: threshold = zeroCrossThresholdA; break;
    case PHASE_B_ADC: threshold = zeroCrossThresholdB; break;
    case PHASE_C_ADC: threshold = zeroCrossThresholdC; break;
  }

  if ((lastADCValue < threshold && adcVal >= threshold) ||
      (lastADCValue > threshold && adcVal <= threshold)) {
    zeroCrossDetected = true;
  }

  lastADCValue = adcVal;
  ADMUX = (ADMUX & 0xF8) | (sense & 0x07);
}

uint8_t getFloatingPhaseADC(uint8_t s) {
  switch (s) {
    case 0: return PHASE_C_ADC;
    case 1: return PHASE_B_ADC;
    case 2: return PHASE_A_ADC;
    case 3: return PHASE_C_ADC;
    case 4: return PHASE_B_ADC;
    case 5: return PHASE_A_ADC;
    default: return PHASE_A_ADC;
  }
}