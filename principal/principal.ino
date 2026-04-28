/******************************************************************
 *  @brief
 *  @authors
 *  @warning
 *****************************************************************/

// Inclusão de bibliotecas ******************************************
#include <SPI.h>                
#include <Adafruit_ADS1X15.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>       
#include "esp_sleep.h"
#include "Connectar_Desconectar.h"
#include "SIFE_LIB.h"


//#define WDT_TIMEOUT_MS  2 * MINUTES_FACTOR    // Dois minutos
#define WDT_TIMEOUT_MS  40000 // 40 segundos

TwoWire I2C_2 = TwoWire(1);   // Destinado aos sensores

// Cria os objetos dos acelerômetros ******************************************
Adafruit_ADXL345_Unified accel1(12345); 
Adafruit_ADXL345_Unified accel2(12346); 
Adafruit_ADXL345_Unified accel3(12347);
Adafruit_ADS1115 ads;                   

// Pinagem termopar ******************************************
const int thermoSO = 19;
const int thermoCS = 5;   
const int thermoSCK = 18; 

// Buffers acelerômetros ******************************************
AmostraAcelerometro bufferSensor1[NUM_AMOSTRAS];
AmostraAcelerometro bufferSensor2[NUM_AMOSTRAS];
AmostraAcelerometro bufferSensor3[NUM_AMOSTRAS];

// Variáveis de controle ******************************************
float medidaPressaoAtual = 0.0;
float medidaTemperaturaAtual = 0.0;
unsigned long timer_ciclo = 0;
bool primeiro_ciclo = true;
unsigned long timer_sife = 0;


/******************************************************************************************************************/
// Faz a leitura da temperatura.
float lerTemperaturaSPI() 
{
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
  for (int i = 0; i < NUM_AMOSTRAS; ++i) 
  {
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
  // ----------------------------------
  esp_task_wdt_reset(); 
  Serial.println("\n==================================================");
  Serial.println("[PASSO 1] Lendo Sensores de Planta (Vibracao/Pressao/Temperatura)...");

  // Cria variáveis de controle
  String JsonTemperatura, JsonPressao, JsonSife;
  String jsonsVibracao[NUM_AMOSTRAS/CHUNK_SIZE];
  
  // inicialização dos sensores
  accel1.begin(0x53);
  accel2.begin(0x1D);
  selectBus(&I2C_2);
  accel3.begin(0x53);
  selectBus(nullptr);
  ads.begin();
  
  // Obtém dados de pressão
  int16_t adc = ads.readADC_SingleEnded(0);
  medidaPressaoAtual = ads.computeVolts(adc) - 0.0024;
  if (medidaPressaoAtual < 0) medidaPressaoAtual = 0.0;
  JsonPressao = getMedida(medidaPressaoAtual);

  // Obtém Json temperatura
  medidaTemperaturaAtual = lerTemperaturaFiltrada();
  if (medidaTemperaturaAtual < 0) medidaTemperaturaAtual = 0.0;
  JsonTemperatura = getMedida(medidaTemperaturaAtual);
  
  // Coleta os dados dos sensores
  collectSensorSamples(accel1, bufferSensor1, nullptr);
  collectSensorSamples(accel2, bufferSensor2, nullptr);
  collectSensorSamples(accel3, bufferSensor3, &I2C_2);

  // Obtém dados estruturados dos acelerômetros (criptografado):
  getVibracao(1, bufferSensor1, jsonsVibracao);
  getVibracao(2, bufferSensor2, jsonsVibracao);
  getVibracao(3, bufferSensor3, jsonsVibracao);

  // Obtém medidas do SIFE
  JsonSife = getEnergiaSife(loadvoltage2, loadvoltage1, realCurrent1, SoC, fonte, erro_ina1, erro_ina2);

  // evitar bugs
  jsonSmall.clear();
  jsonLarge.clear();

  // ----------------------------------
  esp_task_wdt_reset();
  Serial.println("[PASSO 2] Leitura concluida. Iniciando comunicacao com a Nuvem...");

  if(!conectarRedeEbroker()) {
    Serial.println("[FALHA] Nao foi possivel transmitir os dados neste ciclo.");
    return;
  }

  // ----------------------------------
  esp_task_wdt_reset();
  client.loop();

  Serial.println("[PASSO 3] Enviando pacote de dados via MQTT...");

  if( !client.publish(TOPIC_ENERGIA_REAL, JsonSife.c_str()) ) {
    Serial.println("[SIFE] Não foi publicado corretamente.");
  }
  waitingTime(300);

  if( !client.publish(TOPIC_PRESSAO_REAL, JsonPressao.c_str()) ) {
    Serial.println("[PRESSAO] Não foi publicado corretamente.");
  }
  waitingTime(300);

  if( !client.publish(TOPIC_TEMP_REAL, JsonTemperatura.c_str()) ) {
    Serial.println("[TEMPERATURA] Não foi publicado corretamente.");
  }
  waitingTime(300);
  // Publicação dos dados dos acelerômetros

  int sensorId = 1;
  for (int i = 0; i < NUM_AMOSTRAS/CHUNK_SIZE; i++) 
  {
    const char* topic = (sensorId == 1) ? TOPIC_VIBRA_S1_REAL : (sensorId == 2 ? TOPIC_VIBRA_S2_REAL : TOPIC_VIBRA_S3_REAL);
    if( !client.publish(topic, jsonsVibracao[i].c_str())  ) {
      Serial.println("[ACCEL] Não foi publicado corretamente: " + String(sensorId));
    }
    sensorId++;
    waitingTime(500);
  }

  desconectarRede();
  Serial.println("==================================================\n");
  esp_task_wdt_reset();
}

/******************************************************************************************************************/
void setup() 
{
  // Configurações de pinagem Termopar
  pinMode(thermoCS, OUTPUT);
  digitalWrite(thermoCS, HIGH);
  SPI.begin(thermoSCK, thermoSO, -1, thermoCS);     

  // Inicialização da Serial
  Serial.begin(BAUD_RATE);

  // Setup Sistema de Fornecimento de Energia (SIFE)
  SIFE_Setup();

  // Setup Modem GSM
  Serial1.begin(BAUD_RATE, SERIAL_8N1, MODEM_RX, MODEM_TX);
  pinMode(MODEM_PWRKEY, OUTPUT);      // Configura Pinagem do modem GSM
  digitalWrite(MODEM_PWRKEY, LOW);    // Garante estado conhecido
  offModem();                         // Garante que modem está desconectado
  Serial.println("[SETUP] Modem inicializado em estado off");
  
  // Inicialização barramento I2C
  Wire.begin(21, 22); 
  Wire.setClock(2000); 
  Wire.setTimeOut(150); 
  I2C_2.begin(25, 26); 
  I2C_2.setClock(10000);
  I2C_2.setTimeOut(150); 
  delay(100); 

  // Configuração Watchdog
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
  Gerenciamento_Carga();

  if(primeiro_ciclo && millis() > 8000) {
    primeiro_ciclo = false;

    processarLeituraEnvio();
    timer_ciclo = millis();
  }

  else if (!primeiro_ciclo && (millis() - timer_ciclo >= (TEMPO_ENVIO_AC * 1000UL))) {
    processarLeituraEnvio();
    timer_ciclo = millis(); 
  } 
}