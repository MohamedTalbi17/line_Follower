/**
 * Line Follower Robot - Complete ESP32 Code
 * Hardware: ESP32 + 11 IR sensors via MUX + MPU6050 + LCD 16x2 + TB6612 motor driver
 * Features: Menu system, sensor calibration, PID line following, gyro-based turning
 * 
 * IMPORTANT: Sensors output Black=1, White=0 (inverted from raw readings)
 * Motor PWM limited to 200 (78%) to protect TB6612 from RS-380 current
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MPU6050.h>
#include <EEPROM.h>

// ============== PIN DEFINITIONS ==============
// MUX pins for 11 IR sensors
#define S0 13
#define S1 12
#define S2 14
#define S3 27
#define SIG 36

// Motor A pins (Left motor)
#define PWMA 32
#define AIN1 33
#define AIN2 25

// Motor B pins (Right motor)
#define PWMB 26
#define BIN1 2
#define BIN2 4

// Standby and buttons
#define STBY 15
#define BTN_MENU 17
#define BTN_SELECT 16

// ============== CONSTANTS ==============
#define NUM_SENSORS 11
#define MAX_PWM 200  // Motor protection: max 78% power
#define SENSOR_THRESHOLD 10  // Black threshold: below 10% of max = black (1)
#define ANGLE_TOLERANCE 5.0  // ±5 degrees acceptable for angle turning

// ============== OBJECTS ==============
LiquidCrystal_I2C lcd(0x27, 16, 2);
MPU6050 mpu;

// ============== SENSOR VARIABLES ==============
int sensorRaw[NUM_SENSORS];           // Raw analog values from sensors
int sensorMin[NUM_SENSORS];           // Calibrated minimums (white surface)
int sensorMax[NUM_SENSORS];           // Calibrated maximums (black surface)
bool sensorBinary[NUM_SENSORS];       // Processed binary (1=black, 0=white)
int redThresholdPercent = 50;         // Red detection threshold (% between black/white)

// ============== GYRO VARIABLES ==============
float currentAngle = 0;
float gyroXOffset = 0;
int16_t gyroX, gyroY, gyroZ;
unsigned long lastGyroTime = 0;

// Kalman filter variables
float Q_angle = 0.001;
float Q_bias = 0.003;
float R_measure = 0.03;
float angle = 0;
float bias = 0;
float P[2][2] = {{0, 0}, {0, 0}};

// ============== PID VARIABLES ==============
// Line following PID
float Kp = 0.6, Ki = 0.0, Kd = 0.2;
int lastError = 0;
float integral = 0;

// Angle turning PID
float Kp_angle = 2.0, Ki_angle = 0.05, Kd_angle = 0.8;
float lastAngleError = 0;
float angleIntegral = 0;

// ============== ROBOT STATE ==============
int currentStage = 2;
int baseSpeed = 180;
bool inMenuMode = true;
int menuSelection = 0;
const int MENU_ITEMS = 7;
const char* menuOptions[] = {
  "1.Calibrate Sens",
  "2.View Sens B/W",
  "3.View Sens Red",
  "4.Set Red Thresh",
  "5.Calibrate Gyro",
  "6.Select Stage",
  "7.Run Robot"
};

// Button debouncing
unsigned long lastMenuPress = 0;
unsigned long lastSelectPress = 0;
const int DEBOUNCE_DELAY = 200;

// Forward motion variables
int currentLeftSpeed = 0;
int currentRightSpeed = 0;
int lastValidPosition = 500;  // Store last known line position

// ============== SETUP ==============
void setup() {
  Serial.begin(115200);
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Line Follower");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  
  // Setup pins
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(SIG, INPUT);
  
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  
  pinMode(BTN_MENU, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);
  
  digitalWrite(STBY, HIGH);  // Enable motor driver
  
  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    lcd.clear();
    lcd.print("    103 V2.0");
    delay(2000);
  }
  
  // Initialize EEPROM
  EEPROM.begin(512);
  
  // Load calibration if exists
  loadCalibration();
  
  // Set PWM properties (1kHz frequency, 8-bit resolution)
  ledcAttach(PWMA, 1000, 8);
  ledcAttach(PWMB, 1000, 8);
  
  delay(1000);
  displayMenu();
}

// ============== MAIN LOOP ==============
void loop() {
  // Handle menu navigation
  if (inMenuMode) {
    handleMenu();
    return;
  }
  
  // Read sensors and convert to binary
  readAllSensors();
  processSensors();
  
  // Read gyro and update angle
  readGyro();
  updateAngle();
  
  // Stage-based logic
  switch(currentStage) {
    case 1:
      pidLine(baseSpeed);
      // User adds: if (sensorBinary[0] && sensorBinary[10]) { currentStage = 2; }
      break;
      
    case 2:
      pidLine(baseSpeed);
      break;
      
    case 3:
      pidLine(baseSpeed);
      break;
      
    default:
      pidLine(baseSpeed);
      break;
  }
  
  delay(10);  
}

// ============== MENU FUNCTIONS ==============
void displayMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(">");
  lcd.print(menuOptions[menuSelection]);
  
  if (menuSelection < MENU_ITEMS - 1) {
    lcd.setCursor(1, 1);
    lcd.print(menuOptions[menuSelection + 1]);
  }
}

void handleMenu() {
  if (digitalRead(BTN_MENU) == LOW && millis() - lastMenuPress > DEBOUNCE_DELAY) {
    lastMenuPress = millis();
    handleMenuButton();
  }
  
  if (digitalRead(BTN_SELECT) == LOW && millis() - lastSelectPress > DEBOUNCE_DELAY) {
    lastSelectPress = millis();
    handleSelectButton();
  }
}

void handleMenuButton() {
  menuSelection = (menuSelection + 1) % MENU_ITEMS;
  displayMenu();
}

void handleSelectButton() {
  switch(menuSelection) {
    case 0:  // Calibrate Sensors
      calibrateSensors();
      break;
    case 1:  // View Sensors B/W
      viewSensorsBW();
      break;
    case 2:  // View Sensors Red
      viewSensorsRed();
      break;
    case 3:  // Set Red Threshold
      setRedThreshold();
      break;
    case 4:  // Calibrate Gyro
      calibrateGyro();
      viewGyroAngle();
      break;
    case 5:  // Select Stage
      selectStage();
      break;
    case 6:  // Run Robot
      inMenuMode = false;
      resetAngle();
      lcd.clear();
      lcd.print("Running Stage ");
      lcd.print(currentStage);
      break;
  }
}

// ============== SENSOR FUNCTIONS ==============
int readMux(int channel) {
  digitalWrite(S0, channel & 0x01);
  digitalWrite(S1, (channel >> 1) & 0x01);
  digitalWrite(S2, (channel >> 2) & 0x01);
  digitalWrite(S3, (channel >> 3) & 0x01);
  
  delayMicroseconds(50);
  return analogRead(SIG);
}

void readAllSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorRaw[i] = readMux(i);
  }
}

void processSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    // Prevent division by zero
    if (sensorMax[i] == 0) {
      sensorBinary[i] = 0;
      continue;
    }
    
    int percentOfMax = (sensorRaw[i] * 100) / sensorMax[i];
    
    sensorBinary[i] = (percentOfMax < SENSOR_THRESHOLD) ? 1 : 0;
  }
}

bool isRed(int sensorIndex) {
  // Check if specific sensor sees red
  if (sensorIndex < 0 || sensorIndex >= NUM_SENSORS) return false;
  
  int normalized = map(sensorRaw[sensorIndex], sensorMin[sensorIndex], 
                      sensorMax[sensorIndex], 100, 0);
  normalized = constrain(normalized, 0, 100);
  
  // Red is between threshold and 50% (between white and black)
  return (normalized >= redThresholdPercent && normalized <= 50);
}

int getLinePosition() {
  // Calculate weighted position (0-1000 scale, 500=center)
  int sum = 0;
  int weightedSum = 0;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorBinary[i]) {
      sum += 1;
      weightedSum += i * 100;  // Weight by position (0-1000)
    }
  }
  
  if (sum == 0) {
    // No line detected - return last valid position to maintain turn direction
    return lastValidPosition;
  }
  
  // Line detected - update and return position
  lastValidPosition = weightedSum / sum;
  return lastValidPosition;
}

// ============== CALIBRATION FUNCTIONS ==============
void calibrateSensors() {
  lcd.clear();
  lcd.print("Calibrating...");
  lcd.setCursor(0, 1);
  lcd.print("Move over B&W");
  
  // Initialize min/max arrays
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 4095;  // Start with max ADC value
    sensorMax[i] = 0;     // Start with min ADC value
  }
  
  // Calibrate for 5 seconds
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    readAllSensors();
    
    // Update min/max for each sensor
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (sensorRaw[i] < sensorMin[i]) sensorMin[i] = sensorRaw[i];
      if (sensorRaw[i] > sensorMax[i]) sensorMax[i] = sensorRaw[i];
    }
    
    // Show progress
    int progress = (millis() - startTime) / 50;  // 0-100
    lcd.setCursor(14, 1);
    lcd.print(progress / 10);
    
    delay(10);
  }
  
  saveCalibration();
  
  lcd.clear();
  lcd.print("Calibration");
  lcd.setCursor(0, 1);
  lcd.print("Complete!");
  delay(1500);
  displayMenu();
}

void saveCalibration() {
  // Save calibration to EEPROM
  int addr = 0;
  
  // Save magic number to check if calibration exists
  EEPROM.write(addr++, 0xAB);
  EEPROM.write(addr++, 0xCD);
  
  // Save min/max values
  for (int i = 0; i < NUM_SENSORS; i++) {
    EEPROM.write(addr++, sensorMin[i] & 0xFF);
    EEPROM.write(addr++, (sensorMin[i] >> 8) & 0xFF);
    EEPROM.write(addr++, sensorMax[i] & 0xFF);
    EEPROM.write(addr++, (sensorMax[i] >> 8) & 0xFF);
  }
  
  // Save red threshold
  EEPROM.write(addr++, redThresholdPercent);
  
  EEPROM.commit();
}

void loadCalibration() {
  // Load calibration from EEPROM if exists
  int addr = 0;
  
  // Check magic number
  if (EEPROM.read(addr++) != 0xAB || EEPROM.read(addr++) != 0xCD) {
    // No calibration found, use defaults
    for (int i = 0; i < NUM_SENSORS; i++) {
      sensorMin[i] = 100;   // Default white value
      sensorMax[i] = 3000;  // Default black value
    }
    return;
  }
  
  // Load min/max values
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = EEPROM.read(addr++) | (EEPROM.read(addr++) << 8);
    sensorMax[i] = EEPROM.read(addr++) | (EEPROM.read(addr++) << 8);
  }
  
  // Load red threshold
  redThresholdPercent = EEPROM.read(addr++);
}

// ============== SENSOR VIEW FUNCTIONS ==============
void viewSensorsBW() {
  lcd.clear();
  lcd.print("B/W (Menu=Exit)");
  
  while (digitalRead(BTN_MENU) == HIGH) {
    readAllSensors();
    processSensors();
    
    // Display binary sensors on LCD
    lcd.setCursor(0, 1);
    for (int i = 0; i < NUM_SENSORS; i++) {
      lcd.print(sensorBinary[i] ? "1" : "0");
    }
    
    delay(50);
  }
  
  delay(DEBOUNCE_DELAY);
  displayMenu();
}

void viewSensorsRed() {
  lcd.clear();
  lcd.print("Red (Menu=Exit)");
  
  while (digitalRead(BTN_MENU) == HIGH) {
    readAllSensors();
    
    // Display red detection on LCD
    lcd.setCursor(0, 1);
    for (int i = 0; i < NUM_SENSORS; i++) {
      lcd.print(isRed(i) ? "R" : "-");
    }
    
    delay(100);
  }
  
  delay(DEBOUNCE_DELAY);
  displayMenu();
}

void setRedThreshold() {
  lcd.clear();
  lcd.print("Red Threshold");
  
  while (digitalRead(BTN_MENU) == HIGH) {
    if (digitalRead(BTN_SELECT) == LOW) {
      redThresholdPercent += 10;
      if (redThresholdPercent > 90) redThresholdPercent = 10;
      delay(DEBOUNCE_DELAY);
    }
    
    lcd.setCursor(0, 1);
    lcd.print("Value: ");
    lcd.print(redThresholdPercent);
    lcd.print("%  ");
    
    delay(50);
  }
  
  saveCalibration();
  delay(DEBOUNCE_DELAY);
  displayMenu();
}

// ============== GYRO FUNCTIONS ==============
void calibrateGyro() {
  lcd.clear();
  lcd.print("Gyro Calibrate");
  lcd.setCursor(0, 1);
  lcd.print("Keep still...");
  
  long sumX = 0;
  const int samples = 100;
  
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az;
    mpu.getRotation(&gyroX, &gyroY, &gyroZ);
    sumX += gyroX;
    delay(10);
  }
  
  gyroXOffset = sumX / samples;
  
  resetAngle();
  
  lcd.clear();
  lcd.print("Gyro Ready!");
  lcd.setCursor(0, 1);
  lcd.print("Offset: ");
  lcd.print(gyroXOffset);
  delay(1500);
}

void viewGyroAngle() {
  lcd.clear();
  lcd.print("Angle (Menu=Exit)");
  
  while (digitalRead(BTN_MENU) == HIGH) {
    readGyro();
    updateAngle();
    
    lcd.setCursor(0, 1);
    lcd.print("Angle: ");
    lcd.print(currentAngle, 1);
    lcd.print(" deg  ");
    
    delay(50);
  }
  
  delay(DEBOUNCE_DELAY);
  displayMenu();
}

void readGyro() {
  int16_t ax, ay, az;
  mpu.getRotation(&gyroX, &gyroY, &gyroZ);
}

void kalmanFilter(float gyroRate, float dt) {
  // Kalman filter implementation for smooth angle
  angle += dt * (gyroRate - bias);
  
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;
  
  // Small drift correction when not rotating
  if (abs(gyroRate) < 0.3) {
    float innovation = -angle * 0.0001;
    bias += innovation * P[1][0] / (P[0][0] + R_measure);
  }
  
  // Prevent covariance from growing too large
  if (P[0][0] > 10) P[0][0] = 10;
  if (P[1][1] > 10) P[1][1] = 10;
}

void updateAngle() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastGyroTime) / 1000.0;
  if (dt > 0.5) dt = 0.01;
  lastGyroTime = currentTime;
  
  // X-axis gyro measures turning (negative because of mounting)
  float gyroRate = -((gyroX - gyroXOffset) / 131.0);
  kalmanFilter(gyroRate, dt);
  currentAngle = angle;
}

void resetAngle() {
  angle = 0;
  bias = 0;
  currentAngle = 0;
  P[0][0] = 0;
  P[0][1] = 0;
  P[1][0] = 0;
  P[1][1] = 0;
}

float getAngle() {
  return currentAngle;
}

// ============== STAGE SELECTION ==============
void selectStage() {
  lcd.clear();
  lcd.print("Select Stage");
  
  while (digitalRead(BTN_MENU) == HIGH) {
    if (digitalRead(BTN_SELECT) == LOW) {
      currentStage++;
      if (currentStage > 10) currentStage = 1;
      delay(DEBOUNCE_DELAY);
    }
    
    lcd.setCursor(0, 1);
    lcd.print("Stage: ");
    lcd.print(currentStage);
    lcd.print("  ");
    
    delay(50);
  }
  
  delay(DEBOUNCE_DELAY);
  displayMenu();
}

// ============== MOTOR FUNCTIONS ==============
void setMotors(int leftSpeed, int rightSpeed) {
  // Constrain speeds to protect motor driver
  leftSpeed = constrain(leftSpeed, 0, MAX_PWM);
  rightSpeed = constrain(rightSpeed, 0, MAX_PWM);
  
  // Store current speeds
  currentLeftSpeed = leftSpeed;
  currentRightSpeed = rightSpeed;
  
  ledcWrite(PWMB, leftSpeed);
  
  
  ledcWrite(PWMA, rightSpeed);
}

void stopMotors() {
  setMotors(0, 0);
}

// ============== PID LINE FOLLOWING ==============
void pidLine(int baseSpeed) {
  int position = getLinePosition();
  
  int error = position - 500;
  
  integral += error;
  integral = constrain(integral, -10000, 10000);
  
  int derivative = error - lastError;
  
  float output = Kp * error + Ki * integral + Kd * derivative;
  
  lastError = error;
  
  int leftSpeed = baseSpeed + output;
  int rightSpeed = baseSpeed - output;
  
  setMotors(leftSpeed, rightSpeed);
}

// ============== PID ANGLE TURNING ==============
void pidAngle(float relativeAngle, int baseSpeed) {
  // RELATIVE angle turning - adds to current angle
  float targetAngle = currentAngle + relativeAngle;
  
  // Reset PID terms for new turn
  angleIntegral = 0;
  lastAngleError = 0;
  
  // Turn until target angle reached (within tolerance)
  while (true) {
    // Read sensors and gyro
    readAllSensors();
    processSensors();
    readGyro();
    updateAngle();
    
    // Calculate angle error
    float angleError = targetAngle - currentAngle;
    
    // Check if within tolerance
    if (abs(angleError) < ANGLE_TOLERANCE) {
      stopMotors();
      delay(100);
      
      // Double check after settling
      readGyro();
      updateAngle();
      angleError = targetAngle - currentAngle;
      
      if (abs(angleError) < ANGLE_TOLERANCE) {
        break;
      }
    }
    
    // PID calculation for angle
    angleIntegral += angleError;
    angleIntegral = constrain(angleIntegral, -100, 100);
    
    float angleDerivative = angleError - lastAngleError;
    
    float turnOutput = Kp_angle * angleError + Ki_angle * angleIntegral + Kd_angle * angleDerivative;
    
    lastAngleError = angleError;
    
    // Apply turning with optional forward motion
    int leftSpeed = baseSpeed + turnOutput;
    int rightSpeed = baseSpeed - turnOutput;
    
    setMotors(leftSpeed, rightSpeed);
    
    delay(10);
  }
  
  stopMotors();
}

// ============== FORWARD WITH STRAIGHT CORRECTION ==============
void forward(int acceleration, int duration_ms) {
  // Move forward with acceleration/deceleration and straight-line correction
  
  unsigned long startTime = millis();
  unsigned long accelTime = duration_ms * 0.8;
  unsigned long decelTime = duration_ms * 0.2;
  
  int startSpeed = currentLeftSpeed;
  int maxSpeed = startSpeed;
  int stepInterval = 120;
  unsigned long lastStep = millis();
  
  // Store initial angle for straight-line correction
  float startAngle = currentAngle;
  
  while (millis() - startTime < duration_ms) {
    // Read gyro for straight-line correction
    readGyro();
    updateAngle();
    
    unsigned long elapsed = millis() - startTime;
    
    // Acceleration/deceleration logic
    if (elapsed < accelTime) {
      if (millis() - lastStep > stepInterval) {
        maxSpeed += acceleration;
        maxSpeed = constrain(maxSpeed, 0, MAX_PWM);
        lastStep = millis();
      }
    } else {
      if (millis() - lastStep > stepInterval) {
        maxSpeed -= acceleration;
        maxSpeed = constrain(maxSpeed, startSpeed, MAX_PWM);
        lastStep = millis();
      }
    }
    
    // Straight-line correction using angle
    float angleError = startAngle - currentAngle;
    float correction = angleError * Kp_angle * 2;
    
    // Apply speed with correction
    int leftSpeed = maxSpeed + correction;
    int rightSpeed = maxSpeed - correction;
    
    setMotors(leftSpeed, rightSpeed);
    
    delay(10);
  }
  
  // Return to original speed
  setMotors(startSpeed, startSpeed);
}

// ============== UTILITY FUNCTIONS ==============
void printDebug() {
  // Serial debug output (optional)
  Serial.print("Sensors: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorBinary[i]);
    Serial.print(" ");
  }
  Serial.print(" | Position: ");
  Serial.print(getLinePosition());
  Serial.print(" | Angle: ");
  Serial.println(currentAngle);
}

// ============== EMERGENCY STOP ==============
void emergencyStop() {
  // Check for both buttons pressed (emergency stop)
  if (digitalRead(BTN_MENU) == LOW && digitalRead(BTN_SELECT) == LOW) {
    stopMotors();
    inMenuMode = true;
    lcd.clear();
    lcd.print("EMERGENCY STOP!");
    delay(2000);
    displayMenu();
  }
}