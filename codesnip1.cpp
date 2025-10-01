#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

// Pin definitions
#define DHTPIN 2
#define DHTTYPE DHT22
#define COOLING_PIN 3
#define PUMP_PIN 4
#define LED_PIN 13
#define BUTTON_PIN 5

// Constants
#define HUMIDITY_THRESHOLD 60.0
#define TEMP_THRESHOLD 25.0
#define COOLING_POWER_HIGH 255
#define COOLING_POWER_LOW 128
#define WATER_LEVEL_SENSOR_PIN A0
#define MAX_WATER_CAPACITY 5000  // 5 liters in ml

// Global variables
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C LCD display

float humidity = 0.0;
float temperature = 0.0;
int waterLevel = 0;
unsigned long lastCycleTime = 0;
const unsigned long CYCLE_INTERVAL = 10000;  // 10 seconds
bool systemActive = false;

// Water collection statistics
struct WaterStats {
  float totalCollected = 0.0;
  unsigned long cycles = 0;
  float efficiency = 0.0;
} stats;

void setup() {
  Serial.begin(9600);
  Serial.println("IRIS Water Extraction System Starting...");
  
  // Initialize pins
  pinMode(COOLING_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Initialize sensors
  dht.begin();
  lcd.init();
  lcd.backlight();
  
  // Load statistics from EEPROM
  loadStats();
  
  // Display startup message
  lcd.setCursor(0, 0);
  lcd.print("IRIS Water System");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  
  delay(2000);
  
  Serial.println("System ready!");
}

void loop() {
  // Check for button press to toggle system
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(50);  // Debounce
    if (digitalRead(BUTTON_PIN) == LOW) {
      systemActive = !systemActive;
      Serial.println(systemActive ? "System activated" : "System deactivated");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(systemActive ? "ACTIVE" : "STANDBY");
      delay(1000);
    }
  }
  
  if (!systemActive) {
    digitalWrite(LED_PIN, LOW);
    delay(1000);
    return;
  }
  
  // Check if it's time for a new cycle
  if (millis() - lastCycleTime >= CYCLE_INTERVAL) {
    waterExtractionCycle();
    lastCycleTime = millis();
  }
  
  // Update display
  updateDisplay();
  
  // Small delay to prevent overwhelming the system
  delay(100);
}

void waterExtractionCycle() {
  Serial.println("\n--- Starting Water Extraction Cycle ---");
  
  // Read sensor data
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
  
  // Validate sensor readings
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Error: Failed to read from DHT sensor!");
    lcd.clear();
    lcd.print("Sensor Error!");
    return;
  }
  
  Serial.print("Humidity: "); Serial.print(humidity); Serial.println("%");
  Serial.print("Temperature: "); Serial.print(temperature); Serial.println("°C");
  
  // Check water storage capacity
  waterLevel = analogRead(WATER_LEVEL_SENSOR_PIN);
  int waterPercentage = map(waterLevel, 0, 1023, 0, 100);
  
  if (waterPercentage >= 95) {
    Serial.println("Warning: Water storage nearly full!");
    lcd.clear();
    lcd.print("Storage Full!");
    return;
  }
  
  // Calculate optimal cooling power based on environmental conditions
  int coolingPower = calculateOptimalCooling(humidity, temperature);
  
  // Activate cooling system
  analogWrite(COOLING_PIN, coolingPower);
  digitalWrite(LED_PIN, HIGH);
  
  Serial.print("Cooling power set to: "); Serial.println(coolingPower);
  
  // Simulate water condensation (in real implementation, this would be actual condensation)
  float waterCollected = simulateWaterCondensation(humidity, temperature, coolingPower);
  
  // Store water
  if (waterCollected > 0) {
    storeWater(waterCollected);
    updateStatistics(waterCollected);
    
    Serial.print("Water collected this cycle: "); Serial.print(waterCollected); Serial.println(" ml");
    Serial.print("Total water collected: "); Serial.print(stats.totalCollected); Serial.println(" ml");
  }
  
  // Deactivate cooling system
  analogWrite(COOLING_PIN, 0);
  digitalWrite(LED_PIN, LOW);
  
  Serial.println("--- Cycle Complete ---\n");
}

int calculateOptimalCooling(float humidity, float temperature) {
  int basePower = COOLING_POWER_LOW;
  
  // Increase cooling power for higher humidity
  if (humidity > HUMIDITY_THRESHOLD) {
    basePower = COOLING_POWER_HIGH;
  }
  
  // Adjust for temperature (more cooling needed at higher temps)
  if (temperature > TEMP_THRESHOLD) {
    basePower = min(255, basePower + 50);
  }
  
  return basePower;
}

float simulateWaterCondensation(float humidity, float temperature, int coolingPower) {
  // This is a simplified model - in reality, actual condensation would occur
  float baseEfficiency = 0.1;  // 10% base efficiency
  
  // Humidity factor (higher humidity = more water available)
  float humidityFactor = humidity / 100.0;
  
  // Temperature factor (optimal around 20-30°C)
  float tempFactor = 1.0;
  if (temperature >= 20 && temperature <= 30) {
    tempFactor = 1.2;
  } else if (temperature < 10 || temperature > 40) {
    tempFactor = 0.5;
  }
  
  // Cooling power factor
  float coolingFactor = coolingPower / 255.0;
  
  // Calculate water collected (ml per cycle)
  float waterCollected = baseEfficiency * humidityFactor * tempFactor * coolingFactor * 100;
  
  // Add some randomness to simulate real-world conditions
  waterCollected += random(-5, 5);
  
  return max(0, waterCollected);
}

void storeWater(float amount) {
  // In real implementation, this would control actual water storage mechanisms
  // For now, we'll just update our tracking
  waterLevel = min(1023, waterLevel + map(amount, 0, 100, 0, 50));
  
  // Activate pump briefly to simulate water storage
  digitalWrite(PUMP_PIN, HIGH);
  delay(100);
  digitalWrite(PUMP_PIN, LOW);
}

void updateStatistics(float waterCollected) {
  stats.totalCollected += waterCollected;
  stats.cycles++;
  
  // Calculate efficiency (ml per cycle)
  stats.efficiency = stats.totalCollected / stats.cycles;
  
  // Save to EEPROM every 10 cycles
  if (stats.cycles % 10 == 0) {
    saveStats();
  }
}

void updateDisplay() {
  lcd.clear();
  
  // First line: Status and humidity
  lcd.setCursor(0, 0);
  lcd.print("H:");
  lcd.print((int)humidity);
  lcd.print("% T:");
  lcd.print((int)temperature);
  lcd.print("C");
  
  // Second line: Water collected and storage
  lcd.setCursor(0, 1);
  lcd.print("W:");
  lcd.print((int)stats.totalCollected);
  lcd.print("ml S:");
  lcd.print(map(waterLevel, 0, 1023, 0, 100));
  lcd.print("%");
}

void loadStats() {
  // Load statistics from EEPROM
  EEPROM.get(0, stats);
}

void saveStats() {
  // Save statistics to EEPROM
  EEPROM.put(0, stats);
}
