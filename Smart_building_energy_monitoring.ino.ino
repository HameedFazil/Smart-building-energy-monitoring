

#include <LiquidCrystal.h>
#include <Servo.h>
liquidCrystal lcd(12, 11, 5, 4, 3, 2);

#define PIR_PIN 7           // PIR Motion Sensor
#define TRIG_PIN 9          // Ultrasonic Trigger
#define ECHO_PIN 8          // Ultrasonic Echo

// Outputs
#define BUZZER_PIN 6        // Buzzer/Alarm
#define FAN_PIN 10          // Fan (via relay or direct)
#define SERVO_PIN A2        // Servo Motor

// Servo
Servo doorServo;

// ===== VARIABLES =====
bool systemArmed = true;
bool motionDetected = false;
long distance = 0;
unsigned long lastMotionTime = 0;
int servoPosition = 0;

// Thresholds
#define DISTANCE_THRESHOLD 50  // cm - trigger if object closer than this
#define MOTION_COOLDOWN 5000   // ms - time between motion alerts

void setup() {
  // Initialize Serial
  Serial.begin(9600);
  
  // Initialize LCD
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Security System");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  
  // Initialize Pins
  pinMode(PIR_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  
  // Initialize Servo
  doorServo.attach(SERVO_PIN);
  doorServo.write(0);
  
  // Startup sequence
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(FAN_PIN, LOW);
  
  delay(2000);
  lcd.clear();
  lcd.print("System Ready");
  Serial.println("System Initialized!");
}

void loop() {
  // Read sensors
  motionDetected = digitalRead(PIR_PIN);
  distance = getDistance();
  
  // Update LCD display
  updateDisplay();
  
  // Check for intrusion
  if (systemArmed) {
    checkSecurity();
  }
  
  // Auto fan control based on activity
  autoFanControl();
  
  // Check serial commands (from NodeMCU or Bluetooth)
  if (Serial.available() > 0) {
    handleSerialCommand();
  }
  
  delay(100);
}

// ===== ULTRASONIC DISTANCE MEASUREMENT =====
long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  long dist = duration * 0.034 / 2;
  
  return (dist > 0 && dist < 400) ? dist : 999;
}

// ===== SECURITY CHECK =====
void checkSecurity() {
  unsigned long currentTime = millis();
  
  // Motion detected
  if (motionDetected && (currentTime - lastMotionTime > MOTION_COOLDOWN)) {
    triggerAlarm("Motion Detected!");
    lastMotionTime = currentTime;
  }
  
  // Object too close
  if (distance < DISTANCE_THRESHOLD && distance > 0) {
    triggerAlarm("Intruder Alert!");
    lastMotionTime = currentTime;
  }
}

// ===== TRIGGER ALARM =====
void triggerAlarm(String message) {
  Serial.println("ALERT: " + message);
  
  // Sound alarm
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
    delay(200);
  }
  
  // Servo reaction (simulate door lock or camera movement)
  doorServo.write(90);
  delay(1000);
  doorServo.write(0);
}

// ===== AUTO FAN CONTROL =====
void autoFanControl() {
  // Turn on fan if motion detected (ventilation)
  if (motionDetected) {
    digitalWrite(FAN_PIN, HIGH);
  } else {
    digitalWrite(FAN_PIN, LOW);
  }
}

// ===== UPDATE LCD DISPLAY =====
void updateDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  
  if (systemArmed) {
    lcd.print("ARMED  Dist:");
    lcd.print(distance);
    lcd.print("cm");
  } else {
    lcd.print("DISARMED");
  }
  
  lcd.setCursor(0, 1);
  if (motionDetected) {
    lcd.print("Motion: YES");
  } else {
    lcd.print("Motion: NO");
  }
}

// ===== HANDLE SERIAL COMMANDS =====
void handleSerialCommand() {
  String command = Serial.readStringUntil('\n');
  command.trim();
  
  Serial.println("Received: " + command);
  
  // ARM/DISARM system
  if (command == "ARM") {
    systemArmed = true;
    Serial.println("System ARMED");
    lcd.clear();
    lcd.print("System ARMED");
    delay(1000);
  }
  else if (command == "DISARM") {
    systemArmed = false;
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("System DISARMED");
    lcd.clear();
    lcd.print("System DISARMED");
    delay(1000);
  }
  
  // Manual control
  else if (command == "FAN_ON") {
    digitalWrite(FAN_PIN, HIGH);
    Serial.println("Fan ON");
  }
  else if (command == "FAN_OFF") {
    digitalWrite(FAN_PIN, LOW);
    Serial.println("Fan OFF");
  }
  else if (command == "ALARM_ON") {
    digitalWrite(BUZZER_PIN, HIGH);
    Serial.println("Alarm ON");
  }
  else if (command == "ALARM_OFF") {
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("Alarm OFF");
  }
  else if (command == "DOOR_OPEN") {
    doorServo.write(90);
    Serial.println("Door Opened");
  }
  else if (command == "DOOR_CLOSE") {
    doorServo.write(0);
    Serial.println("Door Closed");
  }
  
  // Status request
  else if (command == "STATUS") {
    Serial.print("Armed:");
    Serial.print(systemArmed);
    Serial.print(",Motion:");
    Serial.print(motionDetected);
    Serial.print(",Distance:");
    Serial.println(distance);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
