/*********************************************************************************************************
 * @brief Biblioteca de controle e gerenciamento do Sistema de Fornecimento de Energia (SIFE)
 * @details Implementa algoritmos de carga CC/CV, Coulomb Counting e gerenciamento de Deep Sleep.
 *********************************************************************************************************/
#ifndef SIFE_LIB_H 
#define SIFE_LIB_H  

// Inclusão de bibliotecas
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "esp_task_wdt.h"
#include <rom/rtc.h>

// ========================================================================================================
// --- Mapeamento de Hardware ---
// ========================================================================================================
#define MOS_PIM 15             // Pino de controle do MOSFET (vai ligado em E, de enable)
#define PWM_PIN 12             // Pino de saída do PWM (vai ligado em P, de PWM)
#define WAKEUP_PIN GPIO_NUM_34 // Pino que detecta o retorno da rede AC

// Endereços I2C dos sensores INA219
Adafruit_INA219 ina219_1(0x41); // INA de saída do conversor (Bateria)
Adafruit_INA219 ina219_2(0x44); // INA de entrada (Fonte)

// ========================================================================================================
// --- Configurações Deep Sleep ---
// ========================================================================================================
#define SEGUNDOS_PARA_MICROSEGUNDOS 1000000ULL 
#define TEMPO_DE_SONO_LOADED 900 
#define TEMPO_ENVIO_AC 180
const double C = 0.036; // Constante de corrente em Deep Sleep (Amperes)

// ========================================================================================================
// --- Flags e Limiares ---
// ========================================================================================================
// Relacionadas à fonte chaveada ou painel
#define TensaoFonteOFF 1.0   // Limiar analógico (V) para detectar desconexão da fonte
#define CorrenteFonteOFF 10.0 // Limiar analógico (mA) para detectar desconexão da fonte
#define FonteON 1 
#define FonteOFF 0 
#define PRECHARGE 0          // Estado de pré-carga da bateria
#define CONST_CURRENT 1      // Estado de corrente constante (CC)
#define CONST_VOLTAGE 2      // Estado de tensão constante (CV)

// Relacionadas ao controlador de carga
#define ModoInicial 0 
#define ModoFonteDireta 1 
#define ModoCarga 2 
#define ModoBateria 3 

// Relacionadas ao MOSFET
#define LIGA_MOS 1 
#define DESLIGA_MOS 0 

// Relacionadas à máquina de estados
#define DeleiON 1 
#define DeleiOFF 0 
#define MarkerON 1 
#define MarkerOFF 0 
#define CheckpointON 1 
#define CheckpointOFF 0 
#define CarregouON 1 
#define CarregouOFF 0 
#define CaiuON 1 
#define CaiuOFF 0 

// ========================================================================================================
// --- Parâmetros de PWM (LEDC ESP32) ---
// ========================================================================================================
const int PWM_CHANNEL = 0; 
const int PWM_RESOLUTION = 10; 
const int PWM_FREQ = 50000; // 50kHz para evitar ruído no indutor
int Bit_Carga = 1023; 

// ========================================================================================================
// --- Variáveis de Controle e Estado ---
// ======================================================================================================== 
int delei = DeleiOFF; 
int aumenta = 0, diminui = 0; 
int Marker = MarkerOFF; 
int checkpoint = CheckpointOFF;

RTC_NOINIT_ATTR int controlador;
RTC_NOINIT_ATTR int fonte;

RTC_DATA_ATTR int carregou = CarregouOFF;
RTC_DATA_ATTR int caiu = CaiuOFF;
RTC_DATA_ATTR bool erro_ina1 = false;
RTC_DATA_ATTR bool erro_ina2 = false;
RTC_DATA_ATTR int tentativas_restart_ina = 0;  

// ========================================================================================================
// --- Parâmetros de Carga e Segurança ---
// ======================================================================================================== 
float Load_Corrente = 750.0; // Corrente alvo (mA) para CC
float tolerancia_UP = 100.0; 
float safeLimit = Load_Corrente + tolerancia_UP; 
float setpoint_inf = 100.0;  // Corrente mínima (mA) para fim de carga em CV
float prechargeVoltage = 13.0; 
float cvVoltage = 14.4; 
int chargeState = PRECHARGE; 

// Shunts e Leituras Reais
float R_Shunt1 = 0.122; 
float R_Shunt2 = 0.129; 
float realCurrent1 = 0; 
float realCurrent2 = 0; 
float loadvoltage1 = 0; 
float loadvoltage2 = 0; 
float tensaoShutdown = 12.3; 
float tensaoReinicioCarga = 13.2; 
float safeBatteryV = 14.80;

RTC_NOINIT_ATTR float SoC;  

// ========================================================================================================
// --- Gestão de Energia (Coulomb Counting) ---
// ======================================================================================================== 
float capNAmph = 1500.0; 
float Coulomb_Bat = capNAmph * 3.6;

RTC_NOINIT_ATTR double coulombs;  

// ========================================================================================================
// --- Temporização e Intervalos ---
// ========================================================================================================
unsigned long tempoAtual = 0;  
unsigned long tempoAnterior = 0;  
unsigned long tempoAnterior2 = 0;  
unsigned long lastMillis = 0;  
unsigned long now = 0;  
unsigned long Timer = 0;  
unsigned long timer_inicio_carga = 0;  
unsigned long dormir = 0;  
unsigned long t_estabiliza = 0; 

// Intervalos
unsigned long intervalo = 700;  
unsigned long intervalo2 = 500; 
unsigned long Inter_Timer = 5000; 
unsigned long Sleep_Timer = 70000; 
const unsigned long TEMPO_CARENCIA = 5000; 

// Protótipos de Funções Auxiliares
float Arredonda(float v);
void ControleMosfet(int comandoMosfet);
void Monitora();
void Pot_dig();
void ContaCoulomb();
void comandos();
void iniciarDeepSleep();
extern bool offModem();
/*********************************************************************************************************
* @brief Estima o SoC inicial com base na curva de tensão de circuito aberto(OCV) via interpolação linear por partes
*********************************************************************************************************/ 
float calcularSoCInicial(float v) {
  if (v >= 14.0f) return 100.0f;
  if (v <= 11.2f) return 0.0f;
  if (v >= 13.5f) {
    return 90.0f + ((v - 13.5f) * (10.0f / (14.0f - 13.5f)));
  }
  if (v >= 12.8f) {
    return 70.0f + ((v - 12.8f) * (20.0f / (13.5f - 12.8f)));
  }
  if (v >= 11.5f) {
    return 10.0f + ((v - 11.5f) * (60.0f / (12.8f - 11.5f)));
  }
  if (v >= 11.2f) {
    return 0.0f + ((v - 11.2f) * (10.0f / (11.5f - 11.2f)));
  }
  return 0.0f;
}  

/*********************************************************************************************************
* @brief Função que inicializa parâmetros de hardware e firmware do SIFE. Deve ser colocada no Setup()
********************************************************************************************************/
void SIFE_Setup() {
  // 1. Captura o motivo do reset logo no boot
  RESET_REASON razao = rtc_get_reset_reason(0);
  
  // 2. Configuração inicial de pinagem e periféricos
  pinMode(MOS_PIM, OUTPUT);
  digitalWrite(MOS_PIM, LOW);
  ledcAttachChannel(PWM_PIN, PWM_FREQ, PWM_RESOLUTION, PWM_CHANNEL);
  delay(3000);
  Serial.println("INICIALIZANDO O SIFE 2.0");
  
  // Inicializa barramento e sensores INA219
  bool status_ina1 = ina219_1.begin();
  bool status_ina2 = ina219_2.begin();
  
  if (!status_ina1 || !status_ina2) {
    Serial.println("[SIFE - ERRO] Sensores INA219 de Energia nao detectados.");
    if (tentativas_restart_ina < 2) {
      tentativas_restart_ina++;
      Serial.printf("[SIFE] Tentativa automatica de reparo: %d de 2. Reiniciando...\n", tentativas_restart_ina);
      delay(3000);
      ESP.restart();
    } else {
      Serial.println("[SIFE - ALERTA] Limite de reinicios atingido. Prosseguindo com erros ativos.");
      erro_ina1 = !status_ina1;
      erro_ina2 = !status_ina2;
    }
  } else {
    Serial.println("[SIFE - OK] Sensores de Energia INA219 operacionais!");
    erro_ina1 = false;
    erro_ina2 = false;
    ina219_1.setCalibration_32V_1A();
    ina219_2.setCalibration_32V_1A();
  }
  
  // 3. Força uma leitura imediata dos INAs para alimentar as variáveis de tensão antes do cálculo do SoC
  if (!erro_ina1 && !erro_ina2) {
    float shuntvoltage1 = ina219_1.getShuntVoltage_mV();
    float busvoltage1 = ina219_1.getBusVoltage_V();
    realCurrent1 = shuntvoltage1 / R_Shunt1;
    float perdas = ((realCurrent1 / 1000.0f) * 0.1f);
    loadvoltage1 = Arredonda(busvoltage1 + (shuntvoltage1 / 1000.0f) - perdas);
    
    float shuntvoltage2 = ina219_2.getShuntVoltage_mV();
    float busvoltage2 = ina219_2.getBusVoltage_V();
    realCurrent2 = shuntvoltage2 / R_Shunt2;
    loadvoltage2 = busvoltage2 + (shuntvoltage2 / 1000.0f);
    
    if (realCurrent2 < CorrenteFonteOFF && loadvoltage2 < TensaoFonteOFF) {
      fonte = FonteOFF;
    } else {
      fonte = FonteON;
    }
  }
  
  // 4. Aplicação do SoC Dinâmico baseado no motivo do Reset
  if (razao == POWERON_RESET || razao == EXT_CPU_RESET) {
    Serial.println("[BOOT] Reset por Hardware/Power-On. Calculando SoC dinamico por OCV...");
    SoC = calcularSoCInicial(loadvoltage1);
    coulombs = (SoC / 100.0) * Coulomb_Bat;
    controlador = ModoInicial;
    tentativas_restart_ina = 0;
    Serial.printf("[BOOT] Tensao OCV lida: %.2fV -> SoC Inicial Ajustado: %.2f%%\n", loadvoltage1, SoC);
  } else if (razao == RTCWDT_SYS_RESET || razao == TG0WDT_SYS_RESET || razao == SW_CPU_RESET || razao == DEEPSLEEP_RESET) {
    Serial.println("[BOOT] Recuperado de interrupcao ou Deep Sleep. Retendo dados anteriores de SoC.");
  } else {
    Serial.println("[BOOT] Condicao atipica detectada. Reconfigurando SoC Dinamico...");
    SoC = calcularSoCInicial(loadvoltage1);
    coulombs = (SoC / 100.0) * Coulomb_Bat;
    controlador = ModoInicial;
    tentativas_restart_ina = 0;
  }
  
  // 5. Configuração de temporizadores e buffers restantes
  dormir = millis();
  lastMillis = millis();
  delei = DeleiON;
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  
  switch (cause) {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Acordei pela Rede AC!");
      caiu = CaiuOFF;
      controlador = ModoCarga;
      timer_inicio_carga = millis();
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Acordei pelo timer");
      coulombs -= (C * TEMPO_DE_SONO_LOADED);
      if (coulombs < 0) coulombs = 0;
      SoC = (coulombs / Coulomb_Bat) * 100.0;
      break;
    default:
      break;
  }
}  

/*********************************************************************************************************
* @brief Inicia o processo de Deep Sleep configurando as fontes de despertar do sistema
*********************************************************************************************************/
void iniciarDeepSleep() {
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  if (loadvoltage1 <= tensaoShutdown) {
    esp_sleep_enable_ext0_wakeup(WAKEUP_PIN, 1);
    Serial.println("Configurando o ESP32 para dormir até a rede voltar. Bateria com carga crítica");

    Serial.println("Colocando o modem em sleep devido a bateria critica...");
    offModem();
  } else {
    esp_sleep_enable_timer_wakeup(TEMPO_DE_SONO_LOADED * SEGUNDOS_PARA_MICROSEGUNDOS);
    Serial.println("Configurando o ESP32 para dormir por " + String(TEMPO_DE_SONO_LOADED) + " segundos. Bateria com carga OK");
  }
  Serial.println("Indo dormir agora...");
  Serial.flush();
  esp_deep_sleep_start();
}  

/*********************************************************************************************************
* @brief Função de arredontamento customizada para estabilização de leitura de tensão
*********************************************************************************************************/
float Arredonda(float v) {
  int dezena = int(v * 10.0f);
  int centesima = (int)(v * 100.0f) % 10;
  if (centesima == 9) {
    dezena += 1;
  }
  else if (centesima == 1) {
    dezena -= 1;
  }
  return dezena / 10.0f;
}  

/*********************************************************************************************************
* @brief Executa o acionamento lógico do MOSFET de isolamento da bateria
*********************************************************************************************************/
void ControleMosfet(int comandoMosfet) {
  if (comandoMosfet == LIGA_MOS) {
    digitalWrite(MOS_PIM, HIGH);
  } else if (comandoMosfet == DESLIGA_MOS) {
    digitalWrite(MOS_PIM, LOW);
  } else {
    digitalWrite(MOS_PIM, LOW);
  }
}  

/*********************************************************************************************************
* @brief Realiza o monitoramento periódico de corrente e tensão através dos sensores INA219
*********************************************************************************************************/
void Monitora() {
  float shuntvoltage1 = 0, shuntvoltage2 = 0;
  float busvoltage1 = 0, busvoltage2 = 0;
  float power_mW1 = 0, power_mW2 = 0;
  float Rshunt = 0.1, perdas = 0;
  tempoAtual = millis();
  
  if (tempoAtual - tempoAnterior2 >= intervalo2) {
    tempoAnterior2 = tempoAtual;
    shuntvoltage1 = ina219_1.getShuntVoltage_mV();
    busvoltage1 = ina219_1.getBusVoltage_V();
    realCurrent1 = shuntvoltage1 / R_Shunt1;
    perdas = ((realCurrent1 / 1000.0f) * (Rshunt));
    loadvoltage1 = busvoltage1 + (shuntvoltage1 / 1000.0f) - perdas;
    power_mW1 = realCurrent1 * loadvoltage1;
    
    shuntvoltage2 = ina219_2.getShuntVoltage_mV();
    busvoltage2 = ina219_2.getBusVoltage_V();
    realCurrent2 = shuntvoltage2 / R_Shunt2;
    loadvoltage2 = busvoltage2 + (shuntvoltage2 / 1000.0f);
    power_mW2 = realCurrent2 * loadvoltage2;
    
    if (realCurrent2 < CorrenteFonteOFF && loadvoltage2 < TensaoFonteOFF) {
      fonte = FonteOFF;
    } else {
      fonte = FonteON;
    }
    loadvoltage1 = Arredonda(loadvoltage1);
  }
}  

/*********************************************************************************************************
* @brief Ajusta dinamicamente a tensão de saída atuando sobre a malha de realimentação via PWM (CC/CV)
*********************************************************************************************************/
void Pot_dig() {
  tempoAtual = millis();
  if (tempoAtual - tempoAnterior >= intervalo && Bit_Carga <= 1023) {
    tempoAnterior = tempoAtual;
    if (loadvoltage1 < prechargeVoltage - 0.5) {
      chargeState = PRECHARGE;
    }
    
    switch (chargeState) {
      case PRECHARGE:
        intervalo2 = 200;
        intervalo = 400;
        if (loadvoltage1 < prechargeVoltage) {
          if (realCurrent1 > safeLimit) {
            Bit_Carga += 2;
          } else {
            Bit_Carga -= 2;
          }
        } else {
          chargeState = CONST_CURRENT;
          checkpoint = CheckpointOFF;
        }
        break;
        
      case CONST_CURRENT:
        intervalo = 600;
        intervalo2 = 400;
        if (loadvoltage1 < cvVoltage) {
          if (realCurrent1 < Load_Corrente && Bit_Carga > 0) {
            Bit_Carga -= 2;
          } else if (realCurrent1 > Load_Corrente) {
            Bit_Carga += 2;
          }
        } else {
          chargeState = CONST_VOLTAGE;
          checkpoint = CheckpointOFF;
        }
        break;
        
      case CONST_VOLTAGE:
        intervalo = 3000;
        intervalo2 = 1500;
        if (loadvoltage1 >= cvVoltage) {
          if (loadvoltage1 > cvVoltage && Bit_Carga > 0) {
            Bit_Carga += 1;
          } else if (loadvoltage1 < cvVoltage) {
            Bit_Carga -= 1;
          }
        }
        if (realCurrent1 < setpoint_inf) {
          checkpoint = CheckpointON;
        }
        break;
    }
  }
}  

/*********************************************************************************************************
* @brief Integração contínua da corrente no tempo para cálculo de carga real acumulada (Coulomb Counting)
********************************************************************************************************/
void ContaCoulomb() {
  unsigned long now = millis();
  unsigned long elapsed_ms = now - lastMillis;
  lastMillis = now;
  
  if (elapsed_ms > 2000 || elapsed_ms == 0) return;
  
  float shuntV = ina219_1.getShuntVoltage_mV();
  float Corrente_mA = shuntV / R_Shunt1;
  double current_A = (double)Corrente_mA / 1000.0;
  double dt_s = (double)elapsed_ms / 1000.0;
  double deltaCoulomb = abs(current_A * dt_s);
  
  if (Corrente_mA < 0 || fonte == FonteOFF) {
    coulombs -= deltaCoulomb;
  } else {
    coulombs += deltaCoulomb;
  }
  
  if (coulombs < 0) coulombs = 0;
  if (coulombs > Coulomb_Bat) coulombs = Coulomb_Bat;
  
  SoC = (coulombs / Coulomb_Bat) * 100.0;
  double mAh_restante = (coulombs / 3600.0) * 1000.0;
  
  if (millis() - Timer >= Inter_Timer) {
    Timer = millis();
  }
}  

/*********************************************************************************************************
* @brief Controla as transições lógicas e ações associadas a cada estado do sistema
********************************************************************************************************/
void comandos() {
  switch (controlador) {
    case ModoInicial:
      ControleMosfet(DESLIGA_MOS);
      break;
      
    case ModoFonteDireta:
      ControleMosfet(DESLIGA_MOS);
      Bit_Carga = 0;
      intervalo2 = 5000;
      Monitora();
      break;
      
    case ModoCarga:
      if (delei == DeleiON) {
        if (millis() - timer_inicio_carga < 3000) {
          ControleMosfet(DESLIGA_MOS);
          break;
        } else {
          delei = DeleiOFF;
          timer_inicio_carga = millis();
          Serial.println(">>> Transiente da fonte estabilizado. Ligando SEPIC.");
        }
      }
      if (Marker == MarkerOFF) {
        Bit_Carga = 1020;
        checkpoint = CheckpointOFF;
        Marker = MarkerON;
      }
      Pot_dig();
      Monitora();
      ContaCoulomb();
      ControleMosfet(LIGA_MOS);
      
      if (loadvoltage1 > safeBatteryV && Bit_Carga < 800) {
        ControleMosfet(DESLIGA_MOS);
        Serial.println("Perigo: Excesso de tensão, protegendo bateria!");
        coulombs = Coulomb_Bat;
        carregou = CarregouON;
        controlador = ModoFonteDireta;
        Marker = MarkerOFF;
        checkpoint = CheckpointOFF;
        SoC = 100.0;
        Serial.println(">>> Contador resetado para 100% (Bateria Cheia)");
      }
      
      if (millis() - timer_inicio_carga > TEMPO_CARENCIA) {
        if (realCurrent1 <= setpoint_inf && realCurrent1 > 0.0 && fonte == FonteON && checkpoint == CheckpointON) {
          Serial.println(">>> Bateria 100%. Mudando para modo Fonte.");
          coulombs = Coulomb_Bat;
          carregou = CarregouON;
          controlador = ModoFonteDireta;
          Marker = MarkerOFF;
          checkpoint = CheckpointOFF;
          SoC = 100.0;
          Serial.println(">>> Contador resetado para 100% (Bateria Cheia)");
        }
      }
      break;
      
    case ModoBateria:
      ControleMosfet(DESLIGA_MOS);
      intervalo2 = 5000;
      Bit_Carga = 1023;
      Monitora();
      if (millis() - dormir > Sleep_Timer && loadvoltage1 > tensaoShutdown && fonte == FonteOFF) {
        dormir = millis();
        iniciarDeepSleep();
      } else if (millis() - dormir > Sleep_Timer && loadvoltage1 <= tensaoShutdown && fonte == FonteOFF) {
        iniciarDeepSleep();
      }
      break;
  }
}  

/*********************************************************************************************************
* @brief Orquestração geral do chaveamento de energia com base no monitoramento dinâmico
*********************************************************************************************************/
void Gerenciamento_Carga() {
  comandos();
  Bit_Carga = constrain(Bit_Carga, 0, 1023);
  ledcWriteChannel(PWM_CHANNEL, Bit_Carga);
  
  if (fonte == FonteOFF) {
    if (controlador != ModoBateria) {
      Serial.println("ALERTA: Fonte OFF. Usando Bateria!");
      caiu = CaiuON;
      carregou = CarregouOFF;
      controlador = ModoBateria;
      lastMillis = millis();
    }
    ContaCoulomb();
  }
  else if (fonte == FonteON && caiu == CaiuON) {
    Serial.println(">>> Fonte VOLTOU. Analisando estado da bateria...");
    caiu = CaiuOFF;
    if (loadvoltage1 > tensaoReinicioCarga) {
      Serial.println("-----------Bateria ja estava cheia. Pulando carga!-----------");
      carregou = CarregouON;
      controlador = ModoFonteDireta;
      SoC = 100.0;
      coulombs = Coulomb_Bat;
    } else {
      Serial.println("-----------Bateria descarregada. Iniciando recarga...-----------");
      delei = DeleiON;
      Marker = MarkerOFF;
      checkpoint = CheckpointOFF;
      timer_inicio_carga = millis();
      controlador = ModoCarga;
    }
  }
  else if (fonte == FonteON && carregou == CarregouON && controlador == ModoFonteDireta) {
    if (loadvoltage1 < tensaoReinicioCarga) {
      if (t_estabiliza == 0) {
        t_estabiliza = millis();
      }
      if (millis() - t_estabiliza > 15000) {
        Serial.println(">>> Queda de tensao confirmada (Auto-descarga). Reiniciando carga...");
        carregou = CarregouOFF;
        controlador = ModoCarga;
        Marker = MarkerOFF;
        checkpoint = CheckpointOFF;
        delei = DeleiON;
        timer_inicio_carga = millis();
        t_estabiliza = 0;
        if (SoC > 95.0) SoC = 95.0;
      }
    } else {
      t_estabiliza = 0;
    }
  }
  else if (fonte == FonteON && carregou == CarregouOFF && controlador != ModoCarga) {
    if (loadvoltage1 > tensaoReinicioCarga) {
      Serial.println("-----------Sistema Reiniciado: Bateria detectada como cheia.-----------");
      carregou = CarregouON;
      controlador = ModoFonteDireta;
      SoC = 100.0;
      coulombs = Coulomb_Bat;
    } else {
      Serial.println("-----------Sistema Reiniciado: Iniciando carga de ciclo completo.-----------");
      timer_inicio_carga = millis();
      controlador = ModoCarga;
      Bit_Carga = 1023;
      delei = DeleiON;
    }
  }
}
#endif
