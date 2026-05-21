#ifndef CONNECTAR_DESCONECTAR_H
#define CONNECTAR_DESCONECTAR_H

// Definições gerais *************************************************************************************************
#define NUM_AMOSTRAS            512     // Nº amostras transformada rápida de Fourier 
#define TAMANHO_BACKUP          64     // Nº backups
#define TINY_GSM_MODEM_A7670            // Modelo SIMGSM
#define BUFFER_MQTT_GSM         4096
#define KEEP_ALIVE_S            20      // Keep Alive para PINGREQ/PINGRESP
#define SOCKET_TIMEOUT_S        30      // Socket Timeout Broker MQTT

// Bibliotecas *************************************************************************************************
#include <Arduino.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h> 
#include "ESPEncrypt.h"

// GSM - GPRS LTE *********************************************************************************************
const char apn[]      = "zap.vivo.com.br"; 
const char gprsUser[] = "vivo";
const char gprsPass[] = "vivo";

#define SerialAT Serial1 
static TinyGsm modem(SerialAT);
static TinyGsmClient gsmClient(modem);
static PubSubClient client(gsmClient);
// Define os pinos do Modem ******************************************
#define MODEM_RX        16  
#define MODEM_TX        17  
#define MODEM_PWRKEY    4
#define MODEM_SLEEP     14         // Pino Sleep/DTR A7670 (controla UART sleep)
#define DTR_SET_SLEEP   0           // DTR LOW  -> modem pode dormir (CSCLK ativo)
#define DTR_SET_WAKE    1           // DTR HIGH -> acorda a UART do modem
#define MINUTES_FACTOR  6000
#define BAUD_RATE       115200
// Broker MQTT *************************************************************************************************
#define MQTT_SERVER "broker.hivemq.com" 
const int MQTT_PORT = 1883; 
static char mqttMsgBuffer[BUFFER_MQTT_GSM];
const int CHUNK_SIZE = 64;

// Tópicos MQTT *************************************************************************************************
#define TOPIC_VIBRA_S1_REAL   "sife_felipe/vibracao/sensor1/dados"
#define TOPIC_VIBRA_S2_REAL   "sife_felipe/vibracao/sensor2/dados"
#define TOPIC_VIBRA_S3_REAL   "sife_felipe/vibracao/sensor3/dados"
#define TOPIC_PRESSAO_REAL    "sife_felipe/pressao/dados"
#define TOPIC_TEMP_REAL       "sife_felipe/temperatura/dados"
#define TOPIC_ENERGIA_REAL    "sife_felipe/energia/dados"

#define TOPIC_VIBRA_S1_BACKUP "sife_felipe/vibracao/sensor1/backup"
#define TOPIC_VIBRA_S2_BACKUP "sife_felipe/vibracao/sensor2/backup"
#define TOPIC_VIBRA_S3_BACKUP "sife_felipe/vibracao/sensor3/backup"
#define TOPIC_PRESSAO_BACKUP  "sife_felipe/pressao/backup"
#define TOPIC_TEMP_BACKUP     "sife_felipe/temperatura/backup"
#define TOPIC_ENERGIA_BACKUP  "sife_felipe/energia/backup"

static StaticJsonDocument<512> jsonSmall;
static DynamicJsonDocument jsonLarge(BUFFER_MQTT_GSM);

typedef struct { int16_t x, y, z; } AmostraAcelerometro;

// Criptografia embarcada *************************************************************************************************
#define AES_KEY "6cc18720aed8cb60ceed9fb679495144"
ESPEncrypt crypto(AES_KEY);

int counterErrorTcp = 0;
int counterErrorModemInit = 0;

/*******************************************************************************************************************************/
// Coloca o modem em sleep de baixo consumo via CSCLK + pino DTR.
// O modem mantém o registro de rede e retoma muito mais rápido do que um boot completo.
// Chamar sleepModem() é preferível a offModem() quando o ESP32 ficará acordado entre ciclos.
bool sleepModem()
{
    Serial.print("[MODEM] Entrando em sleep (CSCLK + DTR): ");

    // Habilita o modo sleep por software no modem (UART entra em baixo consumo)
    modem.sendAT("+CSCLK=1");
    if (modem.waitResponse(2000L) != 1) {
        Serial.println("CSCLK=1 falhou — modem pode nao suportar sleep agora.");
        return false;
    }

    // Puxa DTR para LOW: autoriza o hardware do modem a dormir a UART
    digitalWrite(MODEM_SLEEP, DTR_SET_SLEEP);
    delay(100);

    Serial.println("OK");
    return true;
}

/*******************************************************************************************************************************/
// Acorda o modem de um sleep anterior (CSCLK/DTR).
// Deve ser chamado antes de qualquer comunicação AT após sleepModem().
bool wakeModem()
{
    Serial.print("[MODEM] Acordando (DTR wake): ");

    // Puxa DTR para HIGH para sinalizar ao modem que a UART voltará
    digitalWrite(MODEM_SLEEP, DTR_SET_WAKE);
    delay(50);

    // Descarta bytes residuais que possam ter chegado durante o sleep
    while (SerialAT.available()) SerialAT.read();

    // Confirma que a UART responde
    if (modem.testAT(1500)) {
        // Desativa o modo sleep por software para liberar a UART completamente
        modem.sendAT("+CSCLK=0");
        modem.waitResponse(1000L);
        Serial.println("OK");
        return true;
    }

    // Se não respondeu, o modem pode estar desligado — powerModem() cuidará disso
    Serial.println("sem resposta (pode estar desligado).");
    return false;
}

// Envia Comandos AT *************************************************************************************************
void sendATCommand(String cmd) {
  Serial.print("Enviando: ");
  Serial.println(cmd);
  
  modem.sendAT(cmd); // Envia o comando com \r\n automaticamente
  
  // Aguarda a resposta por até 2 segundos
  String response = "";
  if (modem.waitResponse(2000L, response) == 1) { // 1 = OK
    Serial.print("Resposta: ");
    Serial.println(response);
  } else {
    Serial.println("Sem resposta ou erro!");
  }
}
// Liga o modem **********************************************************************************************************
bool powerModem() {
    Serial.println("Testing AT... ");
  if (!modem.testAT()) {
    Serial.println("Failed to connect to modem");
  } else {
    Serial.println("Modem is responding!");
    return true;
  }

    // Se não respondeu, aí sim damos o pulso
    digitalWrite(MODEM_PWRKEY, LOW);
    delay(1500); 
    digitalWrite(MODEM_PWRKEY, HIGH);
    
    // Aguarda o boot e limpa o buffer
    delay(5000);
    while(SerialAT.available()) SerialAT.read();
    return true;
}

// Desliga o modem **********************************************************************************************************
bool offModem()
{
    modem.gprsDisconnect();
    modem.sendAT("+CPOF");      // Comando AT para desligar modem

    if (modem.waitResponse(8000L) == 1) {
        Serial.println("[MODEM] Power OFF via AT");
        return true;
    }
    Serial.println("[OFF MODEM] AT+CPOF falhou");
    Serial.println("[OFF MODEM] Fallback PWRKEY");

    digitalWrite(MODEM_PWRKEY, HIGH); // Garante que começa em HIGH
    delay(100);
    digitalWrite(MODEM_PWRKEY, LOW);  // Puxa para LOW
    delay(3000);                      // Segura por 3 segundos
    digitalWrite(MODEM_PWRKEY, HIGH); // Solta para HIGH (repouso)
    return false;
}

// adakhsgauisd **********************************************************************************************************
void waitingTime(const int wait_ms) 
{
    esp_task_wdt_reset();
    unsigned long aux = millis();
    while(millis() - aux <= (unsigned long) wait_ms) {
        client.loop();
        delay(1);
    }
}

/*******************************************************************************************************************************/
// Função que inicializa o modem e faz a ligação com o Broker MQTT.
bool conectarRedeEbroker() 
{
    esp_task_wdt_reset(); 
    Serial.println("  -> [REDE] Ligando a interface do chip 4G...");

    // Tenta acordar o modem caso esteja em sleep (CSCLK/DTR) do ciclo anterior.
    // Se o modem estiver desligado, wakeModem() apenas avisa e powerModem() completa o boot.
    wakeModem();
    powerModem();
    SerialAT.println("AT");
    delay(3000);
    
    // Verifica inicialização correta do GSM
    if (!modem.init()) {
        sendATCommand("+CPMS?");
        sendATCommand("+CSCLK?");
        sendATCommand("+CFUN?");
        Serial.println("  -> [REDE] ERRO: O modem nao respondeu. Verifique a alimentacao de energia.");
        Serial.println("[MODEM] Restart forçado (init não correspondeu)");
        
        
        counterErrorModemInit++; 
        Serial.printf("[MODEM] Tentativa de inicializacao %d de 3 falhou.\n", counterErrorModemInit);
        
        if (counterErrorModemInit >= 3) {
            Serial.println("[CRÍTICO] Falha no modem 3 vezes seguidas. Reiniciando o ESP32 em 2 segundos...");
            delay(2000); 
            ESP.restart();
        }
       

        return false;
    }

    // Se o modem iniciou com sucesso, zera o contador para não acumular falsos positivos no futuro
    counterErrorModemInit = 0;

    // Apenas LTE
    if (!modem.setNetworkMode((NetworkMode)38)) {
        Serial.println("  -> [REDE] ERRO: Não foi setado Network mode corretamente");
    }

    // Conexão com operadora
    Serial.println("  -> [REDE] Buscando sinal da antena da operadora (Isso pode levar alguns minutos)...");
    esp_task_wdt_reset();
    if (!modem.waitForNetwork(600000L)) {
        Serial.println("  -> [REDE] ERRO: Antena nao encontrada. Verifique o cartao SIM ou a area de cobertura.");
        return false;
    }
    
    // Conexão LTE/4G 
    Serial.println("  -> [REDE] Autenticando com a Vivo (GPRS)...");
    esp_task_wdt_reset();
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        Serial.println("  -> [REDE] ERRO: Falha ao estabelecer dados moveis.");
        return false;
    }
    
    // Configuração de servidor, porta, keep alive e timeout do socket 
    client.setServer(MQTT_SERVER, MQTT_PORT);
    client.setBufferSize(BUFFER_MQTT_GSM);
    client.setKeepAlive(KEEP_ALIVE_S);
    client.setSocketTimeout(SOCKET_TIMEOUT_S);

    // Cria Client ID aleatório
    String clientId = "SIFE_PLANTA_" + String(esp_random(), HEX);

    Serial.println("  -> [REDE] Conectando ao Servidor MQTT na nuvem...");
    esp_task_wdt_reset();

    // Conexão MQTT com clientId aleatório
    if (client.connect(clientId.c_str())) {
        Serial.println("  -> [REDE] SUCESSO! Link com o servidor estabelecido.");
        return true;
    } else {
        Serial.printf("Erro ao conectar ao servidor MQTT. Código: %i\n", client.state());
        if(client.state() == -2) {
            counterErrorTcp++;
        }
        if(counterErrorTcp >= 2) {
            Serial.println("[MODEM] Reiniciando Modem (contador de falhas -2 >= 2)");
            modem.restart();            // restarta o modem GSM
            counterErrorTcp = 0;
            conectarRedeEbroker();
        }
    }
    return false;
}

/*******************************************************************************************************************************/
// Função que desconecta o client MQTT e o modem do GPRS.
void desconectarRede() 
{
    if (client.connected()) client.disconnect();
    sleepModem();   // Mantém contexto de rede; acorda muito mais rápido no próximo ciclo
    Serial.println("  -> [REDE] Modem em sleep. Conexao 4G suspensa com seguranca.");
}

/*******************************************************************************************************************************/
// Função que processa Json de vibração (acelerômetro)
void getVibracao(const int sensorID, AmostraAcelerometro* bufferRaw, String outJsons[])
{
    for (int parte = 0; parte < NUM_AMOSTRAS / CHUNK_SIZE; parte++)
    {
        esp_task_wdt_reset(); 
        jsonLarge.clear();
        jsonLarge["s"] = sensorID;
        jsonLarge["p"] = parte + 1;

        JsonArray dataX = jsonLarge.createNestedArray("x");
        JsonArray dataY = jsonLarge.createNestedArray("y");
        JsonArray dataZ = jsonLarge.createNestedArray("z");
        
        for (int i = parte * CHUNK_SIZE; i < (parte * CHUNK_SIZE) + CHUNK_SIZE; i++) {
            dataX.add(bufferRaw[i].x);
            dataY.add(bufferRaw[i].y);
            dataZ.add(bufferRaw[i].z);
        }
        String jsonString;
        serializeJson(jsonLarge, jsonString);
        String cipher = crypto.encryptString(jsonString);
        outJsons[parte] = cipher;
    }
}


/******************************************************************************************************************/
// Função dedicada para formatar e enviar os dados de um acelerômetro específico
bool enviarDadosAcelerometro(int sensorId, AmostraAcelerometro *buffer, const char* topic) {
  // Cria o array de Strings apenas para o tempo de vida desta função
  String jsonsVibracaoTemp[NUM_AMOSTRAS/CHUNK_SIZE];
  
  // Obtém dados estruturados do acelerômetro (criptografado)
  getVibracao(sensorId, buffer, jsonsVibracaoTemp);
  
  bool sucessoTotal = true;

  // Envia todas as partes (chunks) deste sensor para o seu respectivo tópico
  for (int i = 0; i < NUM_AMOSTRAS/CHUNK_SIZE; i++) {
    esp_task_wdt_reset(); // Alimenta o watchdog durante o envio
    
    // Se a publicação falhar, marcamos o sucessoTotal como false
    if( !client.publish(topic, jsonsVibracaoTemp[i].c_str()) ) {
      sucessoTotal = false; 
    }
    
    waitingTime(500); // Aguarda para não afogar o broker MQTT
  }

  return sucessoTotal;
}

/********************************************************************************************************/
// Obtém medida criptografada
String getMedida(float medida)
{
    jsonSmall.clear();
    jsonSmall["tipo"] = "REALTIME";
    jsonSmall["val"] = medida;
    String jsonString;
    serializeJson(jsonSmall, jsonString);
    String cipher = crypto.encryptString(jsonString);
    return cipher;
}

/************************************************************************************************************************ */
String getEnergiaSife(float v_f, float v_b, float i_b, float s, int r, bool e1, bool e2)
{
    jsonSmall.clear();
    jsonSmall["tipo"] = "ENERGIA";
    jsonSmall["V_Fonte"] = v_f;
    jsonSmall["V_Bat"] = v_b;
    jsonSmall["I_Bat"] = i_b; 
    jsonSmall["SoC"] = s;
    jsonSmall["RedeAC"] = r;
    jsonSmall["INA1_Err"] = e1; 
    jsonSmall["INA2_Err"] = e2;
    String jsonString;
    serializeJson(jsonSmall, jsonString);
    String cipher = crypto.encryptString(jsonString);
    return cipher;
}
#endif
