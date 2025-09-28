// Dual Fan Controller with RPM Synchronization
// Fan 1: Scythe Flex II Slim Perforomance model :Max 1800 RPM, PWM on Pin 9, Tachometer on Pin 18
// Fan 2: Scythe Flex II Performance model       :Max 2000 RPM, PWM on Pin 11, Tachometer on Pin 3
// Fan 2 is throttled to match Fan 1's actual RPM (max 1800)
// Potentiometer 10K connected between GND and 5V, wiper to analog input A0
// Notice something weird reading this code? Well thats because this was 99% of this is vibe coded [claude AI]. It's a Personal Project, does what I want, and I hate coding.
// You can just buy the 1500 RPM versions of these fans and never have to use this code if you want. But I had all the hardware and no idea on how to code so alas. 
// Had issues with interference of PWM and RPM pulses, idk im not that smart, moved the pins away from each other. 

// Fan specifications
// Fan RPM was measured via the arduino with out load invidually, RPM of the fans were higher than stated documentation. 
// Recomend testing your fans and updating their reported max speed here.
// Target RPM is lowered to accomendated varriance and load - 50 RPM under stated speed of the Flex II Slim
const int FAN1_MAX_RPM = 1950;
const int FAN2_MAX_RPM = 2120;
const int TARGET_MAX_RPM = 1750;  // Sync both fans to this max

// Pin definitions
int PWMPin1 = 9;   // Fan 1 PWM (Timer1 OC1A)
int PWMPin2 = 11;  // Fan 2 PWM (Timer2 OC2A) - separated from Timer1
int TachoPin1 = 18; // Fan 1 Tachometer (Pin A4) - physically separated from Pin 3
int TachoPin2 = 3; // Fan 2 Tachometer (Interrupt 1)

// PWM control variables
int icr = 639;   // For 25kHz PWM
int Frequency;
int TargetRPM;
int DutyCycle1, DutyCycle2;
int PotentiometerValue;

// Adaptive control for Fan 2
int Fan2_PWM_Adjustment = 0;  // Dynamic adjustment for Fan 2
int LastValidRPM2 = 1800;     // Store last reasonable reading

// Tachometer variables
volatile unsigned int TachoPulse1 = 0;
volatile unsigned int TachoPulse2 = 0;
int RPM1, RPM2;
unsigned long LastUpdate = 0;
const unsigned long UPDATE_INTERVAL = 1500;  // 1.5 second measurement
unsigned long LastInterruptTime1 = 0, LastInterruptTime2 = 0;
const unsigned long DEBOUNCE_TIME = 2;  // 2ms debounce

// RPM history for averaging
int RPM1History[3] = {1800, 1800, 1800};
int RPM2History[3] = {1800, 1800, 1800};
int HistoryIndex = 0;

void setup() {
  Serial.begin(9600);
  
  // Setup PWM pins
  pinMode(PWMPin1, OUTPUT);
  pinMode(PWMPin2, OUTPUT);
  
  // Setup tachometer pins with external pull-up resistors
  pinMode(TachoPin1, INPUT);  // Pin A4 (18) - External 10k pull-up on 5v
  pinMode(TachoPin2, INPUT);  // Pin 3 - External 10k pull-up on 5v
  
  // Pin A4 (18) requires pin change interrupt
  PCICR |= (1 << PCIE1);     // Enable PCINT for pins A0-A5
  PCMSK1 |= (1 << PCINT12);  // Enable PCINT12 (corresponds to A4)
  
  // Pin 3 uses external interrupt
  attachInterrupt(digitalPinToInterrupt(TachoPin2), counter2, RISING);
  
  // Timer/Counter 1 initialization for 25kHz PWM on Pin 9 (Fan 1)
  TCCR1A = 0x82;  // Fast PWM mode, clear OC1A on compare match
  TCCR1B = 0x19;  // Fast PWM mode, no prescaler
  TCNT1H = 0x00;
  TCNT1L = 0x00;
  
  // Set ICR1 for 25kHz frequency
  ICR1H = icr >> 8;
  ICR1L = icr & 0x00ff;
  
  // Timer/Counter 2 initialization for PWM on Pin 11 (Fan 2)
  // Using normal PWM mode for Timer2 (different frequency is OK)
  TCCR2A = 0x83;  // Fast PWM mode, clear OC2A on compare match
  TCCR2B = 0x01;  // No prescaler
  
  // Calculate and display frequency
  Frequency = 16000000 / (1 + icr);
  Serial.print("PWM Frequency: ");
  Serial.print(Frequency);
  Serial.println(" Hz");
  Serial.println("Target RPM | Fan1 RPM (PWM%) | Fan2 RPM (PWM%)");
  Serial.println("------------------------------------------------");
}

void loop() {
  // Update measurements every 1.5 seconds
  if (millis() - LastUpdate >= UPDATE_INTERVAL) {
    // Read potentiometer to set target RPM
    PotentiometerValue = analogRead(A0);
    TargetRPM = map(PotentiometerValue, 0, 1023, 0, TARGET_MAX_RPM);
    
    // Calculate Fan 1 duty cycle (linear scaling)
    if (TargetRPM < 200) {
      DutyCycle1 = 0;  // Below minimum speed
    } else {
      DutyCycle1 = map(TargetRPM, 200, FAN1_MAX_RPM, 20, 100);
      DutyCycle1 = constrain(DutyCycle1, 0, 100);
    }
    
    // Calculate Fan 2 duty cycle with adaptive control
    if (TargetRPM < 200) {
      DutyCycle2 = 0;  // Below minimum speed
    } else {
      // Start with base scaling
      DutyCycle2 = map(TargetRPM, 200, TARGET_MAX_RPM, 18, 90);
      DutyCycle2 = constrain(DutyCycle2, 0, 100);
      
      // Apply adaptive adjustment based on previous readings
      DutyCycle2 += Fan2_PWM_Adjustment;
      DutyCycle2 = constrain(DutyCycle2, 0, 100);
    }
    
    // Set PWM for both fans
    OCR1A = (long)icr * DutyCycle1 / 100;  // Fan 1 PWM (Timer1, Pin 9)
    OCR2A = (255 * DutyCycle2) / 100;      // Fan 2 PWM (Timer2, Pin 11)
    
    // Read tachometer pulses for both fans
    noInterrupts();
    unsigned int pulses1 = TachoPulse1;
    unsigned int pulses2 = TachoPulse2;
    TachoPulse1 = 0;
    TachoPulse2 = 0;
    interrupts();
    
    // Calculate raw RPM for both fans (1 pulse per revolution for these models)
    int rawRPM1 = (pulses1 * 60) / (1 * (UPDATE_INTERVAL / 1000.0));  // 1 pulse per rev
    int rawRPM2 = (pulses2 * 60) / (1 * (UPDATE_INTERVAL / 1000.0));  // 1 pulse per rev
    
    // Apply intelligent filtering based on expected RPM ranges
    // If readings are way too high, they're likely PWM noise
    if (rawRPM1 > TargetRPM * 1.3) {  // If >30% higher than target, probably noise
      rawRPM1 = TargetRPM;  // Use target as estimate
    }
    if (rawRPM2 > TargetRPM * 1.3) {  // If >30% higher than target, probably noise
      rawRPM2 = TargetRPM;  // Use target as estimate
    }
    
    // Store in history for averaging
    RPM1History[HistoryIndex] = rawRPM1;
    RPM2History[HistoryIndex] = rawRPM2;
    HistoryIndex = (HistoryIndex + 1) % 3;
    
    // Calculate average RPM
    int totalRPM1 = 0, totalRPM2 = 0;
    for (int i = 0; i < 3; i++) {
      totalRPM1 += RPM1History[i];
      totalRPM2 += RPM2History[i];
    }
    RPM1 = totalRPM1 / 3;
    RPM2 = totalRPM2 / 3;
    
    // Additional filtering: if RPM is still way too high after averaging, cap it
    if (RPM1 > TargetRPM * 1.2) {
      RPM1 = TargetRPM * 1.1;  // Allow 10% overage maximum
    }
    if (RPM2 > TargetRPM * 1.2) {
      RPM2 = TargetRPM * 1.1;  // Allow 10% overage maximum
    }
    
    // Filter out obviously wrong readings for Fan 2 (likely PWM noise)
    if (RPM2 > TargetRPM * 1.5) {  // If reading is >150% of target, it's probably noise
      RPM2 = LastValidRPM2;  // Use last good reading
    } else {
      LastValidRPM2 = RPM2;  // Store as good reading
    }
    
    // Adaptive control: adjust Fan 2's PWM based on RPM error
    if (TargetRPM > 200) {  // Only adjust when fan should be running
      int rpmError = TargetRPM - RPM2;
      if (abs(rpmError) > 100) {  // Only adjust if error is significant
        // Small adjustment based on error (positive error = need more PWM)
        Fan2_PWM_Adjustment += (rpmError > 0) ? 1 : -1;
        Fan2_PWM_Adjustment = constrain(Fan2_PWM_Adjustment, -20, 20);  // Limit adjustment range
      }
    }
    
    // Print synchronized status with raw pulse debugging
    Serial.print("    ");
    Serial.print(TargetRPM);
    Serial.print("    |   ");
    Serial.print(RPM1);
    Serial.print(" (");
    Serial.print(DutyCycle1);
    Serial.print("%, ");
    Serial.print(pulses1);
    Serial.print("p)   |   ");
    Serial.print(RPM2);
    Serial.print(" (");
    Serial.print(DutyCycle2);
    Serial.print("%, ");
    Serial.print(pulses2);
    Serial.print("p, adj:");
    Serial.print(Fan2_PWM_Adjustment);
    Serial.print(")");
    
    // Show sync status
    int rpmDifference = abs(RPM1 - RPM2);
    if (rpmDifference < 100 && RPM2 > 0) {
      Serial.print(" ✓SYNC");
    } else {
      Serial.print(" DIFF:");
      Serial.print(rpmDifference);
    }
    
    // Show filter status if applied
    if (pulses1 > TargetRPM / 20 || pulses2 > TargetRPM / 20) {  // High pulse count
      Serial.print(" [FILTERED]");
    }
    Serial.println("");
    
    LastUpdate = millis();
  }
  
  delay(10);
}

// Interrupt handlers
ISR(PCINT1_vect) {
  // Pin change interrupt for A4 (Pin 18) - Fan 1 tachometer
  static bool lastState = HIGH;
  bool currentState = digitalRead(TachoPin1);
  
  if (lastState == HIGH && currentState == LOW) {  // Falling edge detection
    unsigned long currentTime = micros();
    if (currentTime - LastInterruptTime1 > DEBOUNCE_TIME * 1000) {
      TachoPulse1++;
      LastInterruptTime1 = currentTime;
    }
  }
  lastState = currentState;
}

void counter2() {
  unsigned long currentTime = micros();
  if (currentTime - LastInterruptTime2 > DEBOUNCE_TIME * 1000) {
    TachoPulse2++;
    LastInterruptTime2 = currentTime;
  }
}
