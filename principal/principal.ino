#include <SPI.h>                
#include <Adafruit_ADS1X15.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>       
#include "esp_sleep.h"
#include "Connectar_Desconectar.h"
#include "SIFE_LIB.h"


// Define os pinos do Modem
#define MODEM_RX     16  
#define MODEM_TX     17  
#define MODEM_PWRKEY 4   
#define WDT_TIMEOUT_MS 120000 // Dois minutos

TwoWire I2C_2 = TwoWire(1);   // Destinado aos sensores

// Cria os objetos dos acelerômetros
Adafruit_ADXL345_Unified accel1(12345); 
Adafruit_ADXL345_Unified accel2(12346); 
Adafruit_ADXL345_Unified accel3(12347);
Adafruit_ADS1115 ads;                   

const int thermoSO = 19;
const int thermoCS = 5;   
const int thermoSCK = 18; 

AmostraAcelerometro bufferSensor1[NUM_AMOSTRAS];
AmostraAcelerometro bufferSensor2[NUM_AMOSTRAS];
AmostraAcelerometro bufferSensor3[NUM_AMOSTRAS];

float medidaPressaoAtual = 0.0;
float medidaTemperaturaAtual = 0.0;

unsigned long timer_ciclo = 0;
bool primeiro_ciclo = true;

unsigned long timer_sife = 0;


/******************************************************************************************************************/
// Faz a leitura da temperatura.
float lerTemperaturaSPI() {
  uint16_t dados;
  digitalWrite(thermoCS, LOW);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  dados = SPI.transfer16(0x00);
  SPI.endTransaction();
  digitalWrite(thermoCS, HIGH);
 
  if (dados & 0x4) return -1.0; 
  return (dados >> 3) * 0.25;
}

/******************************************************************************************************************/
// Faz o filtro da leitura das temperaturas. Coleta 5 amostras e exclui as amostras nos extremos.
float lerTemperaturaFiltrada() {
  const int amostras = 5;
  float leituras[amostras];
  for (int i = 0; i < amostras; i++) {
    esp_task_wdt_reset(); 
    leituras[i] = lerTemperaturaSPI();
    delay(5);
  }
  for (int i = 0; i < amostras - 1; i++) {
    for (int j = i + 1; j < amostras; j++) {
      esp_task_wdt_reset();
      if (leituras[i] > leituras[j]) {
        float temp = leituras[i];
        leituras[i] = leituras[j];
        leituras[j] = temp;
      }
    }
  }

  return leituras[amostras / 2];
}

/****************************************************************************************************/
// Seleciona qual barramento de I2C vai ser usado. 
void selectBus(TwoWire *bus) {
  if (bus == &I2C_2) { 
    Wire.begin(25, 26);
    Wire.setClock(10000);
  } else { 
    Wire.begin(21, 22); 
    Wire.setClock(2000);
  }
}

/***************************************************************************************************************/
void collectSensorSamples(Adafruit_ADXL345_Unified &accel, AmostraAcelerometro *buffer, TwoWire *bus) {
  for (int i = 0; i < NUM_AMOSTRAS; ++i) {
    esp_task_wdt_reset();
    sensors_event_t e;
    if (bus) selectBus(bus);
    accel.getEvent(&e);
    if (bus) selectBus(nullptr);
    
    buffer[i].x = (int16_t)(e.acceleration.x * 100);
    buffer[i].y = (int16_t)(e.acceleration.y * 100);
    buffer[i].z = (int16_t)(e.acceleration.z * 100);
  }
}

/******************************************************************************************************************/
void processarLeituraEnvio() 
{
  esp_task_wdt_reset(); 
  Serial.println("\n==================================================");
  Serial.println("[PASSO 1] Lendo Sensores de Planta (Vibracao/Pressao/Temperatura)...");
  
  // Faz a inicialização dos sensores
  accel1.begin(0x53);
  accel2.begin(0x1D);
  selectBus(&I2C_2);
  accel3.begin(0x53);
  selectBus(nullptr);
  ads.begin();
  
  int16_t adc = ads.readADC_SingleEnded(0);
  medidaPressaoAtual = ads.computeVolts(adc) - 0.0024;
  if (medidaPressaoAtual < 0) medidaPressaoAtual = 0.0;
  
  // Coleta os dados dos sensores
  collectSensorSamples(accel1, bufferSensor1, nullptr);
  collectSensorSamples(accel2, bufferSensor2, nullptr);
  collectSensorSamples(accel3, bufferSensor3, &I2C_2);
  
  medidaTemperaturaAtual = lerTemperaturaFiltrada();
  if (medidaTemperaturaAtual < 0) medidaTemperaturaAtual = 0.0;

  esp_task_wdt_reset();
  Serial.println("[PASSO 2] Leitura concluida. Iniciando comunicacao com a Nuvem...");
  
  bool conexãoSucedida = conectarRedeEbroker();
  if (conexãoSucedida) {
      Serial.println("[PASSO 3] Enviando pacote de dados via MQTT...");
      enviarPressaoRealTime(medidaPressaoAtual);
      enviarTemperaturaRealTime(medidaTemperaturaAtual);
      enviarEnergiaRealTime(loadvoltage2, loadvoltage1, realCurrent1, SoC, fonte, erro_ina1, erro_ina2);
      enviarVibracaoRealTimeChunked(1, bufferSensor1);
      enviarVibracaoRealTimeChunked(2, bufferSensor2);
      enviarVibracaoRealTimeChunked(3, bufferSensor3);
      desconectarRede();
      Serial.println("[SUCESSO] Todos os dados foram transmitidos e salvos no servidor.");
  } else {
      Serial.println("[FALHA] Nao foi possivel transmitir os dados neste ciclo.");
  }
  Serial.println("==================================================\n");
  esp_task_wdt_reset();
}

/******************************************************************************************************************/
void setup() 
{
  pinMode(thermoCS, OUTPUT);
  digitalWrite(thermoCS, HIGH);
  SPI.begin(thermoSCK, thermoSO, -1, thermoCS);                  
  SIFE_Setup();
  Serial.begin(115200);
  
  // Dá um pulso no PWRKEY do modem.
  pinMode(MODEM_PWRKEY, OUTPUT);
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(100);
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(1000);
  digitalWrite(MODEM_PWRKEY, LOW);
  Serial.println("[SETUP] PULSO NO PWRKEY DADO.");
  Serial1.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  
  Wire.begin(21, 22); 
  Wire.setClock(2000); 
  Wire.setTimeOut(150); 
  I2C_2.begin(25, 26); 
  I2C_2.setClock(10000);
  I2C_2.setTimeOut(150); 
  
  delay(100); 

  esp_task_wdt_config_t cfg = { 
    .timeout_ms = WDT_TIMEOUT_MS, 
    .idle_core_mask = (1 << 0) | (1 << 1), 
    .trigger_panic = true 
  };
  esp_task_wdt_reconfigure(&cfg);
  esp_task_wdt_add(NULL); 
}

/******************************************************************************************************************/
void loop() 
{
  esp_task_wdt_reset();

  if(millis() - timer_sife >= 1000) {
    timer_sife = millis();
    Gerenciamento_Carga();
  }

  if(primeiro_ciclo && millis() > 8000) {
    primeiro_ciclo = false;
    Gerenciamento_Carga();
    processarLeituraEnvio();
    timer_ciclo = millis();
  }

  else if (!primeiro_ciclo && (millis() - timer_ciclo >= (TEMPO_ENVIO_AC * 1000UL))) {
    processarLeituraEnvio();
    timer_ciclo = millis(); 
  } 
}