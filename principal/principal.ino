// Inclusão de bibliotecas ******************************************
#include <SPI.h>                
#include <Adafruit_ADS1X15.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>       
#include "esp_sleep.h"
#include "Connectar_Desconectar.h"
#include "SIFE_LIB.h"
#include "backup.h"

#define WDT_TIMEOUT_MS  120000 // 2 minutos

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
float lerTemperaturaSPI() {
  uint16_t dados;
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(thermoCS, LOW);
  dados = SPI.transfer16(0x00);
  digitalWrite(thermoCS, HIGH);
  SPI.endTransaction();
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
    waitingTime(250); 
  }
  
  // Ordenação das amostras
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

/***************************************************************************************************************/
void clearI2C(int sda, int scl) {
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, OUTPUT);
  
  for (int i = 0; i < 10; i++) {
    digitalWrite(scl, LOW);
    waitingTime(1);
    digitalWrite(scl, HIGH);
    waitingTime(1);
  }
  pinMode(sda, OUTPUT);
  digitalWrite(sda, LOW);
  waitingTime(1);
  digitalWrite(scl, HIGH);
  waitingTime(1);
  digitalWrite(sda, HIGH);
}

/***************************************************************************************************************/
void collectSensorSamples(Adafruit_ADXL345_Unified &accel, AmostraAcelerometro *buffer) {
  for (int i = 0; i < NUM_AMOSTRAS; ++i) 
  {
    esp_task_wdt_reset();
    sensors_event_t e;
    accel.getEvent(&e);
    
    buffer[i].x = (int16_t)(e.acceleration.x * 100);
    buffer[i].y = (int16_t)(e.acceleration.y * 100);
    buffer[i].z = (int16_t)(e.acceleration.z * 100);
  }
}

/******************************************************************************************************************/
void processarLeituraEnvio() 
{
  Backup_AtualizarCiclo();
  // ----------------------------------
  esp_task_wdt_reset(); 
  Serial.println("\n==================================================");
  Serial.println("[PASSO 1] Lendo Sensores de Planta (Vibracao/Pressao/Temperatura)...");

  String JsonTemperatura, JsonPressao, JsonSife;

  // Obtém dados de pressão
  int16_t adc = ads.readADC_SingleEnded(0);
  medidaPressaoAtual = ads.computeVolts(adc) - 0.0024;
  if (medidaPressaoAtual < 0) medidaPressaoAtual = 0.0;
  JsonPressao = getMedida(medidaPressaoAtual, "REALTIME");

  // Obtém Json temperatura 
  medidaTemperaturaAtual = lerTemperaturaFiltrada();
  if (medidaTemperaturaAtual < 0) medidaTemperaturaAtual = 0.0;
  JsonTemperatura = getMedida(medidaTemperaturaAtual, "REALTIME");
 
  collectSensorSamples(accel1, bufferSensor1);
  collectSensorSamples(accel2, bufferSensor2);

  Wire.begin(25, 26);
  collectSensorSamples(accel3, bufferSensor3);

  Wire.begin(21, 22);

  // Obtém medidas do SIFE 
  JsonSife = getEnergiaSife(loadvoltage2, loadvoltage1, realCurrent1, SoC, fonte, erro_ina1, erro_ina2, "REALTIME");

  jsonSmall.clear();
  jsonLarge.clear();

  // ----------------------------------
  esp_task_wdt_reset();
  Serial.println("[PASSO 2] Leitura concluida. Iniciando comunicacao com a Nuvem...");

  bool conexaoEstabelecida = conectarRedeEbroker();

  // VALIDAÇÃO DE IP E DEAD SOCKET (1 Tentativa de Reconexão Forçada)
  if (conexaoEstabelecida) {
      if (!checkIP() || !client.loop()) {
          Serial.println("  -> [REDE] IP Invalido ou Dead Socket (MQTT) detectado.");
          Serial.println("  -> [REDE] Tentando reconexao forcada (1x)...");
          
          desconectarRede();
          modem.gprsDisconnect(); // Força a queda do PDP Context
          waitingTime(2000);
          
          conexaoEstabelecida = conectarRedeEbroker();
          
          if (conexaoEstabelecida) {
              if (!checkIP() || !client.loop()) {
                  Serial.println("  -> [REDE] Falha persistente de IP/Socket apos reconexao. Abortando.");
                  conexaoEstabelecida = false;
              }
          }
      }
  }

  if(!conexaoEstabelecida) {
    Serial.println("[FALHA] Nao foi possivel transmitir os dados neste ciclo.");
    Backup_MarcarFalha();
    Backup_ArmazenarSeNecessario(
      medidaTemperaturaAtual,
      medidaPressaoAtual,
      loadvoltage2,
      loadvoltage1,
      realCurrent1,
      SoC,
      fonte,
      erro_ina1,
      erro_ina2,
      bufferSensor1,
      bufferSensor2,
      bufferSensor3
    );
    desconectarRede();
    return;
  }

  // ----------------------------------
  esp_task_wdt_reset();

  Backup_EnviarPendentes();
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
  Serial.println("[PASSO 4] Enviando dados dos Acelerômetros...");

  if( !enviarDadosAcelerometro(1, bufferSensor1, TOPIC_VIBRA_S1_REAL, "REALTIME") ) {
    Serial.println("[ACCEL S1] Não foi publicado corretamente.");
  } 
  
  if( !enviarDadosAcelerometro(2, bufferSensor2, TOPIC_VIBRA_S2_REAL, "REALTIME") ) {     
      Serial.println("[ACCEL S2] Não foi publicado corretamente."); 
  }

  if( !enviarDadosAcelerometro(3, bufferSensor3, TOPIC_VIBRA_S3_REAL, "REALTIME") ) { 
      Serial.println("[ACCEL S3] Não foi publicado corretamente."); 
  }

  desconectarRede();
  Serial.println("==================================================\n");
  esp_task_wdt_reset();
}

/******************************************************************************************************************/
void setup() 
{
  Serial.begin(BAUD_RATE);

clearI2C(21, 22); 
  clearI2C(25, 26); 

  Wire.begin(21, 22); 
  Wire.setClock(100000); 
  Wire.setTimeOut(150); 

  accel1.begin(0x53);
  accel2.begin(0x1D);

  Wire.begin(25, 26);
  accel3.begin(0x1D);

  Wire.begin(21, 22);
  delay(100);

  Backup_Begin();
  SIFE_Setup();

  pinMode(thermoCS, OUTPUT);
  digitalWrite(thermoCS, HIGH);
  SPI.begin(thermoSCK, thermoSO, -1, thermoCS);     

  ads.begin();

  Serial1.begin(BAUD_RATE, SERIAL_8N1, MODEM_RX, MODEM_TX);
  pinMode(MODEM_PWRKEY, OUTPUT);
  digitalWrite(MODEM_PWRKEY, HIGH);

  pinMode(MODEM_SLEEP, OUTPUT);
  digitalWrite(MODEM_SLEEP, DTR_SET_WAKE);

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

    if (fonte == FonteOFF) {
        Serial.println("[ENERGIA] Tarefa concluida na bateria. A entrar em Deep Sleep imediato!");
        iniciarDeepSleep();
    }
  }
  else if (!primeiro_ciclo && (millis() - timer_ciclo >= (TEMPO_ENVIO_AC * 1000UL))) {
    processarLeituraEnvio();
    timer_ciclo = millis(); 

    if (fonte == FonteOFF) {
        Serial.println("[ENERGIA] Tarefa concluida na bateria. A entrar em Deep Sleep imediato!");
        iniciarDeepSleep();
    }
  } 
}
