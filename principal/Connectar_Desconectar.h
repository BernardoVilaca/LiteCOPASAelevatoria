#ifndef CONNECTAR_DESCONECTAR_H
#define CONNECTAR_DESCONECTAR_H

#define NUM_AMOSTRAS 512      
#define TAMANHO_BACKUP 64     
#define TINY_GSM_MODEM_A7670

#include <Arduino.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h> 

const char apn[]      = "zap.vivo.com.br"; 
const char gprsUser[] = "vivo";
const char gprsPass[] = "vivo";

#define MQTT_SERVER "broker.hivemq.com" 
const int MQTT_PORT = 1883; 

#define TOPIC_VIBRA_S1_REAL   "sife_felipe/vibracao/sensor1/dados"
#define TOPIC_VIBRA_S2_REAL   "sife_felipe/vibracao/sensor2/dados"
#define TOPIC_VIBRA_S3_REAL   "sife_felipe/vibracao/sensor3/dados"
#define TOPIC_PRESSAO_REAL    "sife_felipe/pressao/dados"
#define TOPIC_TEMP_REAL       "sife_felipe/temperatura/dados"
#define TOPIC_ENERGIA_REAL    "sife_felipe/energia/dados"

#define SerialAT Serial1 
static TinyGsm modem(SerialAT);
static TinyGsmClient gsmClient(modem);
static PubSubClient client(gsmClient);

static char mqttMsgBuffer[4096];
static StaticJsonDocument<512> jsonSmall;
static DynamicJsonDocument jsonLarge(4096);

typedef struct { int16_t x, y, z; } AmostraAcelerometro;

bool conectarRedeEbroker() {
    Serial.println("  -> [REDE] Ligando a interface do chip 4G...");
    esp_task_wdt_reset(); 
    
    
    SerialAT.println("AT");
    delay(3000);
    
    
    if (!modem.init()) {
        Serial.println("  -> [REDE] ERRO: O modem nao respondeu. Verifique a alimentacao de energia.");
        return false;
    }

    Serial.println("  -> [REDE] Buscando sinal da antena da operadora (Isso pode levar alguns minutos)...");
    esp_task_wdt_reset();
    if (!modem.waitForNetwork(600000L)) {
        Serial.println("  -> [REDE] ERRO: Antena nao encontrada. Verifique o cartao SIM ou a area de cobertura.");
        return false;
    }

    Serial.println("  -> [REDE] Autenticando com a Vivo (GPRS)...");
    esp_task_wdt_reset();
   
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        Serial.println("  -> [REDE] ERRO: Falha ao estabelecer dados moveis.");
        return false;
    }
    
    client.setServer(MQTT_SERVER, MQTT_PORT);
    client.setBufferSize(4096);
    String clientId = "SIFE_PLANTA_" + String(esp_random(), HEX);

    Serial.println("  -> [REDE] Conectando ao Servidor MQTT na nuvem...");
    esp_task_wdt_reset();
    if (client.connect(clientId.c_str())) {
        Serial.println("  -> [REDE] SUCESSO! Link com o servidor estabelecido.");
        return true;
    }
    return false;
}

void desconectarRede() {
    if (client.connected()) client.disconnect();
    modem.gprsDisconnect();
    Serial.println("  -> [REDE] Conexao 4G encerrada em seguranca.");
}

bool enviarVibracaoRealTimeChunked(int sensorID, AmostraAcelerometro* bufferRaw) {
    const int CHUNK_SIZE = 64;
    if (modem.isGprsConnected() && client.connected()){
    for (int parte = 0; parte < NUM_AMOSTRAS / CHUNK_SIZE; parte++) {
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
        size_t n = serializeJson(jsonLarge, mqttMsgBuffer, sizeof(mqttMsgBuffer));
        const char* topic = (sensorID == 1) ? TOPIC_VIBRA_S1_REAL : (sensorID == 2 ? TOPIC_VIBRA_S2_REAL : TOPIC_VIBRA_S3_REAL);
        if (!client.publish(topic, mqttMsgBuffer, n)) return false;
        client.loop();
        
        delay(200); 
        }
    

    return true;
    }

    Serial.println("O Cliente desconectou no envio dos dados do acelerômetros.");
    return false;
}

bool enviarPressaoRealTime(float medidaVolts) {
    jsonSmall.clear();
    if (modem.isGprsConnected() && client.connected()){
    jsonSmall["tipo"] = "REALTIME";
    jsonSmall["val"] = medidaVolts;
    size_t n = serializeJson(jsonSmall, mqttMsgBuffer, sizeof(mqttMsgBuffer));
    bool sucesso = client.publish(TOPIC_PRESSAO_REAL, mqttMsgBuffer, n);
    client.loop();
    return sucesso;
    }
    Serial.println("O Cliente desconectou no envio da pressão.");
    return false;
}

bool enviarTemperaturaRealTime(float temperatura) {
    jsonSmall.clear();
    if (modem.isGprsConnected() && client.connected()){
    jsonSmall["tipo"] = "REALTIME";
    jsonSmall["val"] = temperatura;
    size_t n = serializeJson(jsonSmall, mqttMsgBuffer, sizeof(mqttMsgBuffer));
    bool sucesso = client.publish(TOPIC_TEMP_REAL, mqttMsgBuffer, n);
    client.loop();
    return sucesso;
    }
    Serial.println("O Cliente desconectou no envio da temperatura.");
    return false;
}

bool enviarEnergiaRealTime(float v_f, float v_b, float i_b, float s, int r, bool e1, bool e2) {
    jsonSmall.clear();
    if (modem.isGprsConnected() && client.connected()) { 
    jsonSmall["tipo"] = "ENERGIA";
    jsonSmall["V_Fonte"] = v_f;
    jsonSmall["V_Bat"] = v_b;
    jsonSmall["I_Bat"] = i_b; 
    jsonSmall["SoC"] = s;
    jsonSmall["RedeAC"] = r;
    jsonSmall["INA1_Err"] = e1; 
    jsonSmall["INA2_Err"] = e2; 
    size_t n = serializeJson(jsonSmall, mqttMsgBuffer, sizeof(mqttMsgBuffer));
    bool sucesso = client.publish(TOPIC_ENERGIA_REAL, mqttMsgBuffer, n);
    client.loop();
    return sucesso;
    }
    Serial.println("O Cliente desconectou no envio da energia.");
    return false;
}
#endif