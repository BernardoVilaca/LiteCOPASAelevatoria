#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// Instância do sensor
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

void setup() {
  // O ESP32 aguenta velocidades de serial bem maiores (115200 é o padrão)
  Serial.begin(115200);
  Serial.println("--- Teste ADXL345 no ESP32 ---");

  // No ESP32, o Wire.begin() usa os pinos 21 e 22 por padrão
  if(!accel.begin()) {
    Serial.println("Erro: Sensor ADXL345 não encontrado! Verifique a fiação.");
    while(1);
  }

  Wire.setClock(4000);

  // Define o range para 2G (mais sensível para movimentos leves)
  accel.setRange(ADXL345_RANGE_2_G);
  
  Serial.println("Sensor ativo. Lendo dados...");
}

void loop() {
  sensors_event_t event; 
  accel.getEvent(&event);

  // Exibição dos dados formatada para o Serial Plotter do Arduino IDE
  Serial.print("X:"); Serial.print(event.acceleration.x); Serial.print(" ");
  Serial.print("Y:"); Serial.print(event.acceleration.y); Serial.print(" ");
  Serial.print("Z:"); Serial.println(event.acceleration.z);

  delay(100); // 10 leituras por segundo
}