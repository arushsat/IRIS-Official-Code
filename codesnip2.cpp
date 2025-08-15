#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <math.h>

// Pin definitions
#define SOLAR_VOLTAGE_PIN A0
#define SOLAR_CURRENT_PIN A1
#define BATTERY_VOLTAGE_PIN A2
#define BATTERY_CURRENT_PIN A3
#define CHARGE_CONTROL_PIN 3
#define LOAD_CONTROL_PIN 4
#define STATUS_LED_PIN 13
#define BUZZER_PIN 5
#define BUTTON_PIN 6

// Constants
#define SOLAR_VOLTAGE_DIVIDER_RATIO 11.0  // Voltage divider for 0-50V range
#define CURRENT_SENSOR_OFFSET 2.5  // ACS712 current sensor offset
#define CURRENT_SENSOR_SENSITIVITY 0.185  // ACS712 30A sensor sensitivity
#define BATTERY_FULL_VOLTAGE 54.0  // 48V LiFePO4 battery (13S)
#define BATTERY_EMPTY_VOLTAGE 39.0
#define BATTERY_CRITICAL_VOLTAGE 35.0
#define SOLAR_MIN_VOLTAGE 15.0
#define SOLAR_MIN_POWER 10.0  // 10W minimum to start charging
#define CHARGE_CURRENT_LIMIT 20.0  // 20A maximum charge current

// System states
enum SystemState {
  STANDBY,
  CHARGING,
  DISCHARGING,
  ENERGY_SAVING,
  ERROR
};

// Global variables
LiquidCrystal_I2C lcd(0x27, 16, 2);
SystemState currentState = STANDBY;
unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL = 5000;  // 5 seconds

// Power measurements
float solarVoltage = 0.0;
float solarCurrent = 0.0;
float solarPower = 0.0;
float batteryVoltage = 0.0;
float batteryCurrent = 0.0;
float batteryPower = 0.0;
int batteryPercentage = 0;

// Statistics
struct PowerStats {
  float totalSolarEnergy = 0.0;  // Wh
  float totalBatteryEnergy = 0.0;  // Wh
  unsigned long chargeCycles = 0;
  unsigned long uptime = 0;
  float efficiency = 0.0;
} stats;

// Energy saving mode settings
bool energySavingMode = false;
unsigned long lastEnergySavingCheck = 0;
const unsigned long ENERGY_SAVING_CHECK_INTERVAL = 30000;  // 30 seconds

void setup() {
  Serial.begin(9600);
  Serial.println("IRIS Solar Energy Management System Starting...");
  
  // Initialize pins
  pinMode(CHARGE_CONTROL_PIN, OUTPUT);
  pinMode(LOAD_CONTROL_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Initialize analog pins
  analogReference(DEFAULT);
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  
  // Load statistics from EEPROM
  loadStats();
  
  // Display startup message
  lcd.setCursor(0, 0);
  lcd.print("IRIS Solar System");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  
  delay(2000);
  
  // Initial system check
  performSystemCheck();
  
  Serial.println("System ready!");
}

void loop() {
  // Check for button press to toggle energy saving mode
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(50);  // Debounce
    if (digitalRead(BUTTON_PIN) == LOW) {
      energySavingMode = !energySavingMode;
      Serial.println(energySavingMode ? "Energy saving mode ON" : "Energy saving mode OFF");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Energy Save:");
      lcd.setCursor(0, 1);
      lcd.print(energySavingMode ? "ON" : "OFF");
      delay(1000);
    }
  }
  
  // Update measurements and system state
  if (millis() - lastUpdateTime >= UPDATE_INTERVAL) {
    updateMeasurements();
    updateSystemState();
    updateStatistics();
    lastUpdateTime = millis();
  }
  
  // Check energy saving mode periodically
  if (millis() - lastEnergySavingCheck >= ENERGY_SAVING_CHECK_INTERVAL) {
    checkEnergySavingMode();
    lastEnergySavingCheck = millis();
  }
  
  // Update display
  updateDisplay();
  
  // Handle system state
  handleSystemState();
  
  delay(100);
}

void updateMeasurements() {
  // Read solar panel measurements
  solarVoltage = readSolarVoltage();
  solarCurrent = readSolarCurrent();
  solarPower = solarVoltage * solarCurrent;
  
  // Read battery measurements
  batteryVoltage = readBatteryVoltage();
  batteryCurrent = readBatteryCurrent();
  batteryPower = batteryVoltage * batteryCurrent;
  
  // Calculate battery percentage
  batteryPercentage = calculateBatteryPercentage(batteryVoltage);
  
  // Validate readings
  if (solarVoltage < 0 || solarCurrent < 0 || batteryVoltage < 0) {
    Serial.println("Error: Invalid sensor readings!");
    currentState = ERROR;
    return;
  }
  
  Serial.print("Solar: "); Serial.print(solarVoltage, 1); Serial.print("V, ");
  Serial.print(solarCurrent, 2); Serial.print("A, "); Serial.print(solarPower, 1); Serial.println("W");
  Serial.print("Battery: "); Serial.print(batteryVoltage, 1); Serial.print("V, ");
  Serial.print(batteryCurrent, 2); Serial.print("A, "); Serial.print(batteryPower, 1); Serial.println("W");
  Serial.print("Battery: "); Serial.print(batteryPercentage); Serial.println("%");
}

float readSolarVoltage() {
  int rawValue = analogRead(SOLAR_VOLTAGE_PIN);
  float voltage = (rawValue * 5.0 / 1024.0) * SOLAR_VOLTAGE_DIVIDER_RATIO;
  return voltage;
}

float readSolarCurrent() {
  int rawValue = analogRead(SOLAR_CURRENT_PIN);
  float voltage = (rawValue * 5.0 / 1024.0);
  float current = (voltage - CURRENT_SENSOR_OFFSET) / CURRENT_SENSOR_SENSITIVITY;
  return max(0, current);  // Current cannot be negative
}

float readBatteryVoltage() {
  int rawValue = analogRead(BATTERY_VOLTAGE_PIN);
  float voltage = (rawValue * 5.0 / 1024.0) * SOLAR_VOLTAGE_DIVIDER_RATIO;
  return voltage;
}

float readBatteryCurrent() {
  int rawValue = analogRead(BATTERY_CURRENT_PIN);
  float voltage = (rawValue * 5.0 / 1024.0);
  float current = (voltage - CURRENT_SENSOR_OFFSET) / CURRENT_SENSOR_SENSITIVITY;
  return current;  // Can be positive (charging) or negative (discharging)
}

int calculateBatteryPercentage(float voltage) {
  if (voltage >= BATTERY_FULL_VOLTAGE) return 100;
  if (voltage <= BATTERY_EMPTY_VOLTAGE) return 0;
  
  float percentage = ((voltage - BATTERY_EMPTY_VOLTAGE) / 
                     (BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE)) * 100;
  return (int)percentage;
}

void updateSystemState() {
  // Check for critical battery level
  if (batteryVoltage <= BATTERY_CRITICAL_VOLTAGE) {
    currentState = ERROR;
    triggerAlarm();
    return;
  }
  
  // Determine system state based on conditions
  if (solarPower >= SOLAR_MIN_POWER && batteryVoltage < BATTERY_FULL_VOLTAGE) {
    currentState = CHARGING;
  } else if (batteryPercentage <= 20 || energySavingMode) {
    currentState = ENERGY_SAVING;
  } else if (batteryPercentage > 20 && !energySavingMode) {
    currentState = DISCHARGING;
  } else {
    currentState = STANDBY;
  }
}

void handleSystemState() {
  switch (currentState) {
    case CHARGING:
      handleCharging();
      break;
    case DISCHARGING:
      handleDischarging();
      break;
    case ENERGY_SAVING:
      handleEnergySaving();
      break;
    case ERROR:
      handleError();
      break;
    default:
      handleStandby();
      break;
  }
}

void handleCharging() {
  // Calculate optimal charge current
  float chargeCurrent = min(solarPower / batteryVoltage, CHARGE_CURRENT_LIMIT);
  
  // Enable charging
  digitalWrite(CHARGE_CONTROL_PIN, HIGH);
  digitalWrite(LOAD_CONTROL_PIN, LOW);
  
  // PWM control for charge current (simplified)
  int pwmValue = map(chargeCurrent, 0, CHARGE_CURRENT_LIMIT, 0, 255);
  analogWrite(CHARGE_CONTROL_PIN, pwmValue);
  
  digitalWrite(STATUS_LED_PIN, HIGH);
  
  Serial.print("Charging at "); Serial.print(chargeCurrent, 2); Serial.println("A");
}

void handleDischarging() {
  // Normal operation mode
  digitalWrite(CHARGE_CONTROL_PIN, LOW);
  digitalWrite(LOAD_CONTROL_PIN, HIGH);
  digitalWrite(STATUS_LED_PIN, LOW);
  
  Serial.println("Operating normally");
}

void handleEnergySaving() {
  // Reduce power consumption
  digitalWrite(CHARGE_CONTROL_PIN, LOW);
  digitalWrite(LOAD_CONTROL_PIN, LOW);
  
  // Blink LED to indicate energy saving mode
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink >= 1000) {
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    lastBlink = millis();
  }
  
  Serial.println("Energy saving mode active");
}

void handleError() {
  // Disable all outputs
  digitalWrite(CHARGE_CONTROL_PIN, LOW);
  digitalWrite(LOAD_CONTROL_PIN, LOW);
  digitalWrite(STATUS_LED_PIN, LOW);
  
  // Continuous alarm
  digitalWrite(BUZZER_PIN, HIGH);
  
  Serial.println("ERROR: Critical battery level!");
}

void handleStandby() {
  // Minimal power consumption
  digitalWrite(CHARGE_CONTROL_PIN, LOW);
  digitalWrite(LOAD_CONTROL_PIN, LOW);
  digitalWrite(STATUS_LED_PIN, LOW);
  
  Serial.println("Standby mode");
}

void checkEnergySavingMode() {
  // Auto-enable energy saving if battery is low
  if (batteryPercentage <= 15 && !energySavingMode) {
    energySavingMode = true;
    Serial.println("Auto-enabling energy saving mode");
  }
  
  // Auto-disable energy saving if battery is well-charged
  if (batteryPercentage >= 80 && energySavingMode) {
    energySavingMode = false;
    Serial.println("Auto-disabling energy saving mode");
  }
}

void updateStatistics() {
  // Update solar energy (Wh)
  stats.totalSolarEnergy += (solarPower * UPDATE_INTERVAL / 3600000.0);
  
  // Update battery energy (Wh)
  stats.totalBatteryEnergy += (batteryPower * UPDATE_INTERVAL / 3600000.0);
  
  // Update charge cycles
  if (currentState == CHARGING && batteryCurrent > 0.1) {
    stats.chargeCycles++;
  }
  
  // Update uptime
  stats.uptime += UPDATE_INTERVAL;
  
  // Calculate efficiency
  if (stats.totalSolarEnergy > 0) {
    stats.efficiency = (stats.totalBatteryEnergy / stats.totalSolarEnergy) * 100;
  }
  
  // Save to EEPROM every 100 updates
  static int updateCounter = 0;
  if (++updateCounter >= 100) {
    saveStats();
    updateCounter = 0;
  }
}

void updateDisplay() {
  lcd.clear();
  
  // First line: Solar power and battery voltage
  lcd.setCursor(0, 0);
  lcd.print("S:");
  lcd.print((int)solarPower);
  lcd.print("W B:");
  lcd.print((int)batteryVoltage);
  lcd.print("V");
  
  // Second line: Battery percentage and state
  lcd.setCursor(0, 1);
  lcd.print("B:");
  lcd.print(batteryPercentage);
  lcd.print("% ");
  
  switch (currentState) {
    case CHARGING:
      lcd.print("CHG");
      break;
    case DISCHARGING:
      lcd.print("RUN");
      break;
    case ENERGY_SAVING:
      lcd.print("SAVE");
      break;
    case ERROR:
      lcd.print("ERR");
      break;
    default:
      lcd.print("STBY");
      break;
  }
}

void performSystemCheck() {
  Serial.println("Performing system check...");
  
  // Check sensors
  float testVoltage = readSolarVoltage();
  if (testVoltage < 0 || testVoltage > 60) {
    Serial.println("Warning: Solar voltage sensor may be faulty");
  }
  
  // Check battery
  if (batteryVoltage < BATTERY_EMPTY_VOLTAGE) {
    Serial.println("Warning: Battery voltage critically low");
    currentState = ERROR;
  }
  
  Serial.println("System check complete");
}

void triggerAlarm() {
  // Sound alarm for critical conditions
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);
    delay(500);
  }
}

void loadStats() {
  EEPROM.get(0, stats);
}

void saveStats() {
  EEPROM.put(0, stats);
}
