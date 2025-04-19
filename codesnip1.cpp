#include <DHT.h>
#include <CoolingSystem.h>
#include <WaterStorage.h>

#define DHTPIN 2
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

float humidity;
float temperature;

void setup() {
  Serial.begin(9600);
  dht.begin();
  CoolingSystem.begin();
  WaterStorage.begin();
}

void loop() {
  // get the current humidity and temperature
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();

  // check if the readings are valid
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Adjust the cooling system for optimal condensation
  if (humidity > 60) {
    CoolingSystem.setPowerMode("HIGH");
  } else {
    CoolingSystem.setPowerMode("LOW");
  }

  // Condense and store the water
  float waterCollected = CoolingSystem.condense(temperature, humidity);
  WaterStorage.store(waterCollected);

  // print the water collected to serial monitor
  Serial.print("Water collected: ");
  Serial.println(waterCollected);
  delay(10000);  // Delay for 10 seconds before next cycle
}
