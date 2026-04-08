#include "max6675.h"

// Definicao dos pinos conforme montagem
int thermoSO = 19;  // Fio Azul
int thermoCS = 5;   // Fio Cinza
int thermoSCK = 18; // Fio Verde

MAX6675 thermocouple(thermoSCK, thermoCS, thermoSO);

void setup() {
  Serial.begin(115200);
  Serial.println("--- Teste de Termopar MAX6675 ---");
  delay(500);
}


void loop() {
  float celsius = thermocouple.readCelsius();
  
  if (isnan(celsius)) {
    Serial.println("Erro na leitura do termopar!");
  } else {
    Serial.print("Temperatura: ");
    Serial.print(celsius);
    Serial.println("C");
  }
  delay(1000);
}
