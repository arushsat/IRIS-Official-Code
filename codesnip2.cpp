#include <SolarPanel.h>
#include <BatteryManagement.h>

float solarPower;
int batteryLevel;

void setup() {
  Serial.begin(9600);
  SolarPanel.begin();
  BatteryManagement.begin();
}

void loop() {
  // Get current power output from solar panel and battery level
  solarPower = SolarPanel.getPowerOutput();
  batteryLevel = BatteryManagement.getBatteryLevel();

  // If solar power is sufficient then charge the battery
  if (solarPower > 50) {
    BatteryManagement.chargeBattery(solarPower);
    Serial.println("Charging battery...");
  } else if (batteryLevel < 20) {
    // If battery is low then switch to energy-saving mode
    BatteryManagement.switchToEnergySavingMode();
    Serial.println("Switching to energy-saving mode...");
  } else {
    // Operate normally if sufficient power is available
    Serial.println("Operating normally with solar power.");
  }
  
  delay(10000);  // Delay for 10 seconds before checking again
}
