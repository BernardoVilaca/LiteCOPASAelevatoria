/*********************************************************************************************************
 * @brief descrição do que o header faz no geral
 * @authors Autores e colaboradores.
 * @attention Pontos de atenção
 * @details -- 
 * @date --
 *********************************************************************************************************/

 // Inclusão de bibs (melhor deixar aq do que no .ino)
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "esp_task_wdt.h"

// ========================================================================================================
// --- Mapeamento de Hardware ---
// ========================================================================================================
#define MOS_PIM      15             // Pino de controle do MOSFET (vai ligado em E, de enable)
#define PWM_PIN      12             // Pino de saída do PWM (vai ligado em P, de PWM)
#define WAKEUP_PIN GPIO_NUM_34      // Pino que detecta o retorno da rede AC
//*PS: Wakeu_Pin já está ligado no lugar que precisa automaticamente na SIPCBAN, é um recurso de hardware já disponivel

// Endereços I2C dos sensores INA219
Adafruit_INA219 ina219_1(0x41);         // INA de saída do conversor
Adafruit_INA219 ina219_2(0x44);         // INA de entrada/bateria

// ========================================================================================================
//____________________________________Configurações Deep sleep_____________________________________________
#define SEGUNDOS_PARA_MICROSEGUNDOS 1000000ULL  /* Fator de conversão */
#define TEMPO_DE_SONO_LOADED  180                       /* Tempo que ele vai dormir com a bateria em estado ok(em segundos) */ 
#define TEMPO_ENVIO_AC 120

//_________________________________________________________________________________________________________
//--------------------------------------Flags--------------------------------------------------------------
//_________________________________________________________________________________________________________
//Flags relacionadas a fonte chaveada ou painel
#define TensaoFonteOFF    1.0     // Limiar analógico (V) para detectar se a fonte foi desconectada
#define CorrenteFonteOFF  10.0    // Limiar analógico (mA) para detectar se a fonte foi desconectada
#define FonteON           1       // Estado lógico: Há energia entrando no sistema
#define FonteOFF          0       // Estado lógico: Não há energia na entrada
#define PRECHARGE         0       // Pré carga caso a bateria esteja demasiadamente descarregada
#define CONST_CURRENT     1       // Estado de corrente constante
#define CONST_VOLTAGE     2       // Estado de tensão constante
//_________________________________________________________________________________________________________
//Flags relacionadas ao controlador de carga
#define ModoInicial 0          // Boot/Segurança: Tudo desligado enquanto o sistema inicializa
#define ModoFonteDireta 1      // Bateria a 100%: PWM desligado, energia passa direto para a carga
#define ModoCarga 2            // Carregando: Algoritmo CC/CV atuando no controle de PWM
#define ModoBateria 3          // Falta de AC (Apagão): Sistema operando apenas pela bateria
//_________________________________________________________________________________________________________
//Flags MOSFET ControleMosfet()
#define LIGA_MOS      1           // Fecha o circuito do MOSFET (conecta a bateria)
#define DESLIGA_MOS   0           // Abre o circuito do MOSFET (desconecta a bateria)
//_________________________________________________________________________________________________________
//Flags relacionadas a maquina de estados 
#define DeleiON 1              // Ativa carência de tempo (espera estabilizar transientes da fonte)
#define DeleiOFF 0             // Desativa a carência de tempo
#define MarkerON 1             // Trava de execução acionada (garante que uma ação ocorra só 1x)
#define MarkerOFF 0            // Trava de execução liberada
#define CheckpointON 1         // Sinaliza que o ciclo CV terminou com sucesso (corrente baixa)
#define CheckpointOFF 0        // Sinaliza que o ciclo de carga ainda está em andamento
#define CarregouON 1           // Memória: Bateria já bateu 100% recentemente (evita efeito sanfona)
#define CarregouOFF 0          // Memória: Bateria precisa ou está recebendo um novo ciclo de carga
#define CaiuON 1               // Flag de evento: Marca o exato momento em que a rede AC caiu
#define CaiuOFF 0              // Flag de evento: Sistema normal ou queda já processada
//_________________________________________________________________________________________________________

// ========================================================================================================
// --- Parâmetros de PWM (LEDC ESP32) ---
// ========================================================================================================
const int PWM_CHANNEL    = 0;          // Canal interno do ESP32 usado para gerar o sinal PWM
const int PWM_RESOLUTION = 10;         // Resolução do PWM (10 bits = valores de 0 a 1023)
const int PWM_FREQ       = 50000;      // Frequência do PWM em Hz (50kHz para silenciar o indutor)
int Bit_Carga            = 1023;       // Valor atual do ciclo de trabalho (Duty Cycle) do PWM

// ========================================================================================================
// --- Variáveis de Controle e Estado ---
// ========================================================================================================   
int delei = DeleiOFF;                         // Controla o estado de carência/espera inicial do sistema
int aumenta = 0, diminui = 0;                 // Auxiliares para lógica de incremento/decremento do PWM(ajuste fino)
int Marker = MarkerOFF;                       // Garante que uma configuração de estado ocorra apenas uma vez
int checkpoint  = CheckpointOFF;              // Indica se a bateria atingiu os critérios de carga completa
RTC_DATA_ATTR int controlador = ModoInicial;  // Estado atual do sistema (Salvo durante o sono)
RTC_DATA_ATTR int fonte = FonteOFF;           // Indica se a fonte AC está presente (Salvo durante o sono)
RTC_DATA_ATTR int carregou = CarregouOFF;     // Memória se o ciclo de 100% foi concluído (Salvo no sono)
RTC_DATA_ATTR int caiu = CaiuOFF;             // Flag que registra se houve queda de energia (Salvo no sono)
RTC_DATA_ATTR bool erro_ina1 = false;         // Indica se houve erro ou não no INA1 para avaliação no Node-red
RTC_DATA_ATTR bool erro_ina2 = false;         // Indica se houve erro ou não no INA1 para avaliação no Node-red
RTC_DATA_ATTR int tentativas_restart_ina = 0; // Auxiliar que determina a quantidade de tentativas que houveram para reiniciar o INA
// ========================================================================================================
// --- Parâmetros de Carga e Segurança (float) ---
// ======================================================================================================== 
float Load_Corrente    = 750.0;        // Corrente alvo (mA) para a fase de Corrente Constante (CC)
float tolerancia_UP    = 100.0;        // Margem de erro permitida acima da corrente alvo
float safeLimit = Load_Corrente + tolerancia_UP;    // Limite máximo de corrente antes de atuar proteção
float setpoint_inf     = 100.0;        // Corrente mínima (mA) que indica fim de carga na fase CV
float prechargeVoltage = 13.0;         // Tensão limite para sair do modo de carga lenta (Pré-carga)
float cvVoltage = 14.4;                // Tensão alvo para a bateria (fase de Tensão Constante)
int chargeState = PRECHARGE;           // Estado atual do algoritmo de carga (0=Pre, 1=CC, 2=CV)

// Shunts e Leituras Reais
float R_Shunt1 = 0.122;                 // Valor Resistor Shunt 1
float R_Shunt2 = 0.129;                 // Valor Resistor Shunt 2
float realCurrent1 = 0;                 // Corrente instantânea medida na bateria (mA)
float realCurrent2 = 0;                 // Corrente instantânea medida na entrada/fonte (mA)
float loadvoltage1 = 0;                 // Tensão corrigida lida nos terminais da bateria (V)
float loadvoltage2 = 0;                 // Tensão lida na entrada do conversor (V)
float tensaoShutdown = 11.2;            // Tensão mínima de segurança antes de desligar tudo (Deep Sleep até a rede voltar)
float tensaoReinicioCarga = 13.2;       // Tensão de histerese para disparar nova carga após flutuação
float safeBatteryV = 14.80;             // Proteção de sobretensão: desliga o MOSFET se atingir este valor
RTC_DATA_ATTR float SoC = 0;            // Estado de Carga da bateria em % (State of Charge)

// ========================================================================================================
// --- Gestão de Energia (Coulomb Counting) ---
// ======================================================================================================== 
float capNAmph                = 1500.0;             // Capacidade nominal da bateria em mAh
float Coulomb_Bat             = capNAmph * 3.6;     // Capacidade total convertida para Coulombs (As)
RTC_DATA_ATTR double coulombs = 0.0;                // Acumulador de carga atual (Salvo durante o sono)

// ========================================================================================================
// --- Temporização e Intervalos (unsigned long) ---
// ========================================================================================================
unsigned long tempoAtual            = 0;    // Armazena o millis() atual do ciclo de loop
unsigned long tempoAnterior         = 0;    // Controle de tempo para a função Pot_dig()
unsigned long tempoAnterior2        = 0;    // Controle de tempo para a função Monitora()
unsigned long lastMillis            = 0;    // Último tempo registrado para o cálculo do delta de Coulombs
unsigned long now                   = 0;    // Auxiliar de tempo instantâneo
unsigned long Timer                 = 0;    // Controle de tempo para exibição de mensagens no Serial
unsigned long timer_inicio_carga    = 0;    // Marca o início de um novo ciclo para aplicar a carência
unsigned long dormir  = 0;    // Cronômetro para o tempo de inatividade antes do Deep Sleep
unsigned long t_estabiliza          = 0;    // Timer de confirmação para evitar triggers falsos de carga

// Intervalos
unsigned long intervalo             = 700;      // Frequência de atualização do potenciômetro digital (ms)
unsigned long intervalo2            = 500;      // Frequência de leitura dos sensores INA219 (ms)
unsigned long Inter_Timer           = 5000;     // Intervalo de print do log de bateria no Serial (ms)
unsigned long Sleep_Timer           = 70000;    // Tempo de espera (ms) antes de entrar em economia de energia
const unsigned long TEMPO_CARENCIA  = 5000;     // Tempo de espera (ms) para estabilizar a fonte antes de medir


/*******************************************************************************************************************************/
// Função que inicializa parâmetros de hardware e firmware do SIFE. Deve ser colocada no Setup()
void SIFE_Setup()
{ 
  // Configuração da piangem MOSFET e PWM de controle.
  pinMode(MOS_PIM, OUTPUT);  
  digitalWrite(MOS_PIM, LOW);
  ledcAttachChannel(PWM_PIN, PWM_FREQ, PWM_RESOLUTION, PWM_CHANNEL); 

  delay(3000);    //Delay para estabilização inicial
  Serial.println("INICIALIZANDO O SIFE 2.0"); 

  //Busca pelos sensores INA219 no barramento I2C:
  bool status_ina1 = ina219_1.begin();
  bool status_ina2 = ina219_2.begin();

  if (!status_ina1 || !status_ina2) {
    Serial.println("[SIFE - ERRO] Sensores INA219 de Energia nao detectados.");
    // Tenta novamente buscar os inas 
    if (tentativas_restart_ina < 2) {
      tentativas_restart_ina++;
      Serial.printf("[SIFE] Tentativa automatica de reparo: %d de 2. Reiniciando...\n", tentativas_restart_ina);
      delay(3000);
      ESP.restart();  
    } else {
      Serial.println("[SIFE - ALERTA] Limite de reinicios atingido. O sistema prosseguira com as marcacoes de erro ativas.");
      erro_ina1 = !status_ina1;
      erro_ina2 = !status_ina2;
    }
  } else {
    Serial.println("[SIFE - OK] Sensores de Energia INA219 operacionais!");
    tentativas_restart_ina = 0;
    erro_ina1 = false;
    erro_ina2 = false;
    ina219_1.setCalibration_32V_1A();
    ina219_2.setCalibration_32V_1A();
  }
  
  dormir = millis();
  lastMillis = millis();
  delei = DeleiON;        // Ativa a carência inicial

  // Configura o sistema após despertar do ESP32 (deepsleep)
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  switch(cause) {
  case ESP_SLEEP_WAKEUP_EXT0:
    Serial.println("Acordei pela Rede AC!");
    caiu = CaiuOFF;                            // A rede está de vola
    controlador = ModoCarga;                   // Habilita o modo de carga para a máquina de estados
    timer_inicio_carga = millis();
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.println("Acordei pelo timer");
    break;
  default:
    Serial.println("Boot geral");
    break;
}
}

/*****************************************************************************************************************************/
//Função que inicia o deep sleep
void iniciarDeepSleep() 
{ 
  // Desabilita as fontes de despertar configuradas para evitar bugs internos da API
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

  // Configura despertar
  if(loadvoltage1 <= tensaoShutdown){
    // 1. Configura o sleep até que a rede AC volte
    esp_sleep_enable_ext0_wakeup(WAKEUP_PIN, 1);
    Serial.println("Configurando o ESP32 para dormir até a rede voltar. Bateria com carga crítica");
  }else{
    // 1. Configura o despertar por timer para a bateria com carga(enuanto a tensão da bateria é maior que a de shutdown)
    esp_sleep_enable_timer_wakeup(TEMPO_DE_SONO_LOADED * SEGUNDOS_PARA_MICROSEGUNDOS);
    Serial.println("Configurando o ESP32 para dormir por " + String(TEMPO_DE_SONO_LOADED) + " segundos. Bateria com carga OK");
  }

  Serial.println("Indo dormir agora...");
  Serial.flush();

  // 2. Entra no modo Deep Sleep
  esp_deep_sleep_start();
}

/*****************************************************************************************************************************/
//Função de arredontamento de float
float Arredonda(float v) {
  // Inteiro de déciMOS (ex.: 13.66 → 136)
  int dezena = int(v * 10.0f);
  // Dígito das centésimas (ex.: 13.66 → 1366 % 10 → 6)
  int centesima = (int)(v * 100.0f) % 10;
  
  if (centesima == 9) {
    // 13.9 → 139 → +1 → 140 → 14.0
    dezena += 1;
  }
  else if (centesima == 1) {
    // 13.1 → 131 → −1 → 130 → 13.0
    dezena -= 1;
  }
// Retorna v truncado para uma casa decimal,
// mas arredonda para cima se o centésimo for 9,
// e arredonda para baixo se o centésimo for 1. 
  return dezena / 10.0f;
}

//Função para controle do MOSFET na placa do SIFE e para controle do distribuidor de energia
void ControleMosfet(int comandoMosfet){
  if(comandoMosfet == LIGA_MOS){
    digitalWrite(MOS_PIM , HIGH);             // Ativa a carga da bateria
  }else if(comandoMosfet == DESLIGA_MOS){  
    digitalWrite(MOS_PIM , LOW);              // Desativa a carga da bateria
  } else {
    digitalWrite(MOS_PIM , LOW);              // Padrão: desativa carga da bateria
  }
}

/************************************************************************************************************************* */
//Função que faz o monitoramento da potência de entrada e de carga da bateria
void Monitora()
{
  float shuntvoltage1 = 0, shuntvoltage2 = 0;
  float busvoltage1 = 0, busvoltage2 = 0; 
  float power_mW1 = 0, power_mW2 = 0;
  float Rshunt = 0.1, perdas = 0; 
  tempoAtual = millis();

  if(tempoAtual - tempoAnterior2 >= intervalo2) 
  {
    tempoAnterior2 = tempoAtual;         // Atualiza o tempo da última execução
    
    // Medições do INA219-1 (mede a carga)
    shuntvoltage1 = ina219_1.getShuntVoltage_mV();
    busvoltage1 = ina219_1.getBusVoltage_V();
    realCurrent1 = shuntvoltage1/ R_Shunt1;
    perdas = ((realCurrent1/1000)*(Rshunt));
    loadvoltage1 = busvoltage1 + (shuntvoltage1 / 1000) - perdas; // Calibra a tensão lida sobre a carga contando com as perdas
    power_mW1 = realCurrent1 * loadvoltage1;
    
    // Medições do INA219-2 (mede a entrada do conversor)
    shuntvoltage2 = ina219_2.getShuntVoltage_mV();
    busvoltage2 = ina219_2.getBusVoltage_V(); 
    realCurrent2 = shuntvoltage2/ R_Shunt2;
    loadvoltage2 = busvoltage2 + (shuntvoltage2 / 1000);
    power_mW2 = realCurrent2 * loadvoltage2;

    if(realCurrent2 < CorrenteFonteOFF && loadvoltage2 < TensaoFonteOFF){ 
      fonte = FonteOFF;
    }else{
      fonte = FonteON;
    }

    Serial.print("Queda no Shunt 2 :");
    Serial.print(shuntvoltage2); // Exibe as perdas(importante para calibração, após calibrar pode comentar)
    Serial.println("mV");
    Serial.print("Tensao da FONTE:  "); Serial.print(loadvoltage2); Serial.println(" V");
    Serial.print("Corrente da FONTE:       "); Serial.print(realCurrent2); Serial.println(" mA");
    Serial.print("Potencia da FONTE:         "); Serial.print(power_mW2); Serial.println(" mW");
    Serial.println("");

    Serial.print("Queda no Shunt  1:");
    Serial.print(shuntvoltage1); // Exibe  as perdas(importante para calibração, após calibrar pode comentar)
    Serial.println("mV");
    Serial.print("Tensao na BATERIA:  "); Serial.print(loadvoltage1); Serial.println(" V");
    Serial.print("Corrente na BATERIA:       "); Serial.print(realCurrent1); Serial.println(" mA"); 
    Serial.print("Potencia na BATERIA:         "); Serial.print(power_mW1); Serial.println(" mW");
    Serial.println(""); 
    loadvoltage1 = Arredonda(loadvoltage1);  
  }
}

//Função que atua no controle da tensão de saída do conversor DC/DC, de forma similar a um potenciometro digital
void Pot_dig(){  
  tempoAtual = millis(); 
  if (tempoAtual - tempoAnterior >= intervalo && Bit_Carga <= 1023) {  
    tempoAnterior = tempoAtual; 
    if(loadvoltage1 < prechargeVoltage - 0.5){
      chargeState = PRECHARGE;
    } 
    switch(chargeState){

      // --- Fase 0: Pré-carga ---
      case PRECHARGE:
        intervalo2 = 200;   //Tempo em ms
        intervalo = 400;
        if (loadvoltage1 < prechargeVoltage) {
          // Ajusta para manter tensão limitada e corrente segura
          if (realCurrent1 > safeLimit) {
            Bit_Carga += 2;
          }
          else {
            Bit_Carga -= 2;
          }
        }else {
          // Transição para CC
          chargeState = CONST_CURRENT;
          checkpoint = CheckpointOFF;
        }
        break;

      // --- Fase 1: Corrente constante ---
      case CONST_CURRENT:
        intervalo = 600;
        intervalo2 = 400;
        if (loadvoltage1 < cvVoltage) {
          // Ajusta para manter corrente próxima do alvo
          if (realCurrent1 < Load_Corrente && Bit_Carga > 0){
            Bit_Carga -= 2;
          }
          else if (realCurrent1 > Load_Corrente) {
            Bit_Carga += 2;
          }
        } else {
          // Transição para CV
          chargeState = CONST_VOLTAGE;
          checkpoint = CheckpointOFF;
        }
        break;

      // --- Fase 2: Tensão constante ---
      case CONST_VOLTAGE:
        intervalo = 3000;
        intervalo2 = 1500;

        // Se já atingiu a tensão alvo, não aumenta mais
        if (loadvoltage1 >= cvVoltage) {
          // Mantém Bit_Carga estável ou ajusta apenas para segurar em 14.4V
          if (loadvoltage1 > cvVoltage && Bit_Carga > 0) {
            Bit_Carga += 1; // reduz a tensão se passar do limite
          }
          else if (loadvoltage1 < cvVoltage) {
            Bit_Carga -= 1; // aumenta se cair abaixo do alvo
          }
          // Se estiver exatamente em 14.4V, não mexe
        }

        // Critério de fim de carga: corrente muito baixa (ex.: C/20)
        if (realCurrent1 < setpoint_inf) {
          checkpoint = CheckpointON; // Finaliza carga
        }
        break;
    }
    Serial.print("Estado:");
    Serial.print(chargeState);
    Serial.print("  Bit:");
    Serial.println(Bit_Carga);
  }
}  

/*******************************************************************************************************************************/
// Função que realiza a integração da corrente no tempo para estimar a carga real da bateria (Coulomb Counting)
void ContaCoulomb() 
{
  unsigned long now = millis();
  unsigned long elapsed_ms = now - lastMillis; // Calcula o tempo decorrido desde a última integração
  lastMillis = now;

  // --- Proteção contra falhas de tempo ---
  // Evita erros de cálculo caso o loop trave ou o sistema acorde de um Deep Sleep (pula variações > 2s)
  if (elapsed_ms > 2000 || elapsed_ms == 0) return;

  // --- Aquisição de Dados Analógicos ---
  // Mede a queda de tensão no resistor shunt para calcular a corrente via Lei de Ohm
  float shuntV = ina219_1.getShuntVoltage_mV();     
  float Corrente_mA = shuntV / R_Shunt1; 
  
  // Conversão de unidades para o Sistema Internacional (SI)
  double current_A = (double)Corrente_mA / 1000.0; // Converte mA para Ampere
  double dt_s = (double)elapsed_ms / 1000.0;       // Converte milissegundos para segundos (delta tempo)
  double deltaCoulomb = abs(current_A * dt_s);     // Calcula a variação de carga em Coulombs (Q = I * t)

  // --- Lógica de Fluxo de Energia ---
  // Determina se a bateria está perdendo ou ganhando elétrons
  if (Corrente_mA < 0 || fonte == FonteOFF) {
    coulombs -= deltaCoulomb; // Modo Descarga: subtrai a carga consumida
  } else {
    coulombs += deltaCoulomb; // Modo Carga: soma a carga injetada pela fonte
  }

  // --- Travas de Segurança (Clamping) ---
  // Garante que o contador nunca ultrapasse os limites físicos da bateria (0% a 100%)
  if (coulombs < 0){
    coulombs = 0;
  }
  if (coulombs > Coulomb_Bat){
    coulombs = Coulomb_Bat;
  }

  // --- Cálculos de Estado de Carga (SoC) ---
  SoC = (coulombs / Coulomb_Bat) * 100.0;              // Percentual relativo à capacidade total
  double mAh_restante = (coulombs / 3600.0) * 1000.0; // Converte Coulombs de volta para mAh para exibição

  // --- Bloco de Telemetria Serial ---
  // Exibe o status do sistema em intervalos definidos por 'Inter_Timer'
  if (millis() - Timer >= Inter_Timer) {
    Timer = millis();

    Serial.print("Modo: ");
    if (fonte == FonteOFF) {
      Serial.print("DESCARGA");
    } else {
      Serial.print("CARGA");
    }

    Serial.print(" | Corrente: "); 
    Serial.print(Corrente_mA); 
    Serial.print("mA");
    
    Serial.print(" | Restante: "); 
    Serial.print(mAh_restante, 2); 
    Serial.print("mAh");
    
    Serial.print(" | Bateria: ");  
    Serial.print(SoC, 2); 
    Serial.println("%");
  }
}

/*******************************************************************************************************************************/
// Função que implementa uma máquina de estados, comutando entre 3 modos: Carga da bateria, modo fonte DC e modo backup (Energia caiu)
void comandos() 
{
  // Máquina de Estados
  switch(controlador) 
  {
    case ModoInicial: // DESLIGADO
      ControleMosfet(DESLIGA_MOS);        
      break;

    case ModoFonteDireta: // FONTE DIRETA (BATERIA CHEIA) 
      ControleMosfet(DESLIGA_MOS); 
      Bit_Carga = 0;                //Coloca PWM em LOW, o que zera a tensão injetada no feedback do conversor DC/DC e coloca tensão máxima no VA (aprox: 17-18V)
      intervalo2 = 5000;            //Define um intervaloi de 5 segundos para a leitura dos dados do INA
      Monitora();                   //Chama a função que monitora os INAs
      break;

    case ModoCarga: // CARREGANDO (CC/CV)
       
      if (delei == DeleiON) {
        // Verifica se já se passaram 3000ms desde o timer_inicio_carga
        if (millis() - timer_inicio_carga < 3000) {
          ControleMosfet(DESLIGA_MOS); // Garante que o MOSFET da bateria fique desligado
          break;       // SAI DO SWITCH: O processador continua o loop, alimenta o Watchdog e não trava!
        } else {
          // Passaram os 3 segundos! Destrava o sistema.
          delei = DeleiOFF;   
          timer_inicio_carga = millis(); // Reseta o timer para os 5s de carência começarem a contar do zero
          Serial.println(">>> Transiente da fonte estabilizado. Ligando SEPIC.");
        }
      } 
      if(Marker == MarkerOFF){        //Marker reinicializa o valor de duty cycle do PWM toda vez que se inicia um ciclo de carga
        Bit_Carga = 1020;             //Começa com a tensão mínima de carga(quanto mais tensão na realimentação negativa do conversor DC/DC, menor a saída)
        checkpoint = CheckpointOFF;   //Checkpoint é usado como flag para sinalizar que a carga foi finalizada
        Marker = MarkerON;
      }  
      Pot_dig();                  // Realiza o controle da tensão de carga através do PWM
      Monitora();                 // Monitora parâmetro de carga com os INA219
      ContaCoulomb();             // Conta os coulombs
      ControleMosfet(LIGA_MOS);   // Essa função deve vir após as anteriores, para que o mosfet só seja acionado quando a tensão de carga seja a mínima inicialmente
      
      if(loadvoltage1 > safeBatteryV && Bit_Carga < 800){   //Caso a tensão suba demais, considera que a bateria está com alta resistência interna e está carregada
        ControleMosfet(DESLIGA_MOS);                                  //Desliga a carga
        Serial.println("Perigo: Excesso de tensão, protegendo bateria!");                
        coulombs = Coulomb_Bat;                             // RESET: Balde cheio 
        carregou = CarregouON;                              // Flag que confirma que a carga foi finalizada
        controlador = ModoFonteDireta;                      //Entra no modo de uso da energia da fonte DC ligada em rede
        Marker = MarkerOFF;                                 //Coloca marker em LOW para garantir recarga posteriormente
        checkpoint = CheckpointOFF;                         //Coloca o checpoint em off para ser usado posteriormente
        SoC = 100.0;                                        //Considera a bateria carregada (aproximação básica)
        Serial.println(">>> Contador resetado para 100% (Bateria Cheia)");
      }
      
      // Só verifica se acabou a carga se já passou o TEMPO DE CARENCIA (5s)
      if (millis() - timer_inicio_carga > TEMPO_CARENCIA) 
      { 
        // Verifica se a corrente caiu (bateria cheia) 
        if (realCurrent1 <= setpoint_inf && realCurrent1 > 0.0 && fonte == FonteON && checkpoint == CheckpointON) {
          Serial.println(">>> Bateria 100%. Mudando para modo Fonte.");
          coulombs = Coulomb_Bat;         //RESET: Balde cheio 
          carregou = CarregouON;          //Flag que confirma que a carga foi finalizada 
          controlador = ModoFonteDireta;  //Entra no modo de uso da energia da fonte DC ligada em rede
          Marker = MarkerOFF;             //Coloca marker em LOW para garantir recarga posteriormente
          checkpoint = CheckpointOFF;     //Coloca o checpoint em off para ser usado posteriormente
          SoC = 100.0;                    //Considera a bateria carregada (aproximação básica)
          Serial.println(">>> Contador resetado para 100% (Bateria Cheia)");
        }
      }
      break;

    case ModoBateria: // MODO BATERIA (SEM REDE)
       ControleMosfet(DESLIGA_MOS);   //Mantém o Mosfet desligado para que a energia, ao voltar, não seja aplicada diretamente na bateria sem devido controle
       intervalo2 = 5000;   // Intervalo de leitura dos INA219
       Bit_Carga = 1023;       // Coloca o PWM no máximo para que caso a rede AC volte, o sistmea não inicializae com tensão máxima 
       Monitora();          // Importante: Monitora continua rodando para detectar se a fonte voltou  
       
       if (millis() - dormir > Sleep_Timer && loadvoltage1 > tensaoShutdown && fonte == FonteOFF) {
          //Sleep periódico (bateria com capacidade ok)
          dormir = millis();
          iniciarDeepSleep();
       } else if(millis() - dormir > Sleep_Timer && loadvoltage1 <= tensaoShutdown && fonte == FonteOFF){
          //Sleep definitivo até a rede voltar para preservar saúde da bateria 
          iniciarDeepSleep();
       }
       break;
  }
}

/************************************************************************************************************************* */
// Função principal que orquestra a inteligência do sistema, decidindo o modo de operação baseado na fonte e bateria.
void Gerenciamento_Carga()
{
  // Executa a máquina de estados base (ModoCarga, ModoFonteDireta ou ModoBateria)
  comandos();
  
  // --- Segurança de Hardware ---
  Bit_Carga = constrain(Bit_Carga, 0, 1023);  // Garante que o Duty Cycle do PWM nunca saia da faixa de 10 bits (0 a 1023)
  ledcWriteChannel(PWM_CHANNEL, Bit_Carga);   // Aplica fisicamente o valor ao driver do conversor

  // ========================================================================================================
  // 1. CENÁRIO: FONTE OFF (Operação via Bateria)
  // Ocorre quando o INA219 de entrada detecta queda de tensão/corrente da rede AC ou painel
  // ========================================================================================================
  if (fonte == FonteOFF) {
      if (controlador != ModoBateria) { 
        Serial.println("ALERTA: Fonte OFF. Usando Bateria!");  
        caiu = CaiuON;                    // Sinaliza para o sistema que houve um evento de interrupção de energia
        carregou = CarregouOFF;           // Reseta a memória de carga completa (precisará reavaliar ao voltar)
        controlador = ModoBateria;        // Altera a lógica para modo backup/descarga
        lastMillis = millis();            // Referência de tempo para o cálculo de Coulombs na descarga
      }
      ContaCoulomb();                     // Monitoramento contínuo da autonomia restante
  }

  // ========================================================================================================
  // 2. CENÁRIO: FONTE VOLTOU (Recuperação de Energia)
  // Bloco executado uma única vez após a restauração da fonte de entrada
  // ========================================================================================================
  else if (fonte == FonteON && caiu == CaiuON) {
    Serial.println(">>> Fonte VOLTOU. Analisando estado da bateria...");
    
    // Reseta a detecção de queda para evitar que este bloco se repita no próximo loop
    caiu = CaiuOFF; 

    // Avalia se a bateria ainda está cheia (acima do limiar de reinício) para poupar ciclos
    if (loadvoltage1 > tensaoReinicioCarga) {
      Serial.println("-----------Bateria ja estava cheia. Pulando carga!-----------");
      carregou = CarregouON;              // Define estado como carregado
      controlador = ModoFonteDireta;      // Vai direto para o modo de bypass
      SoC = 100.0;                        // Sincroniza o contador de Coulombs para "cheio"
      coulombs = Coulomb_Bat; 
    } else {
      // Caso contrário, inicia o protocolo de carga segura
      Serial.println("-----------Bateria descarregada. Iniciando recarga...-----------");
      delei = DeleiON;                    // Ativa carência para estabilização de transientes
      Marker = MarkerOFF;                 // Libera marcador para configurações iniciais do modo carga
      checkpoint = CheckpointOFF;         // Reseta confirmação de término de carga
      timer_inicio_carga = millis();      // Inicia cronômetro de segurança/carência
      controlador = ModoCarga;            // Entra no algoritmo CC/CV
    }
  } 

  // ========================================================================================================
  // 3. CENÁRIO: MANUTENÇÃO / HISTERESE (Proteção contra Auto-descarga)
  // Evita que o carregador fique ligando/desligando por oscilações rápidas (ruído)
  // ========================================================================================================
  else if (fonte == FonteON && carregou == CarregouON && controlador == ModoFonteDireta) {
    // Verifica se a tensão caiu abaixo do ponto de reinício (Histerese)
    if (loadvoltage1 < tensaoReinicioCarga) {   
      if (t_estabiliza == 0) {
        t_estabiliza = millis();          // Inicia contagem de confirmação de queda real
      }
      // Janela de 15 segundos: Confirma se a queda é persistente e não um pico de carga momentâneo
      if (millis() - t_estabiliza > 15000) { 
        Serial.println(">>> Queda de tensao confirmada (Auto-descarga). Reiniciando carga...");
        carregou = CarregouOFF;           // Reseta memória de carga
        controlador = ModoCarga;          // Reativa algoritmo de carga
        Marker = MarkerOFF; 
        checkpoint = CheckpointOFF; 
        delei = DeleiON; 
        timer_inicio_carga = millis();
        t_estabiliza = 0;
        if (SoC > 95.0) SoC = 95.0;       // Ajusta SoC para refletir que houve perda de carga
      }
    } else {
      t_estabiliza = 0;                   // Reseta timer se a tensão normalizar antes dos 15s
    }
  }

  // ========================================================================================================
  // 4. CENÁRIO: POWER-UP / RESET (Cold Boot)
  // Gerencia o comportamento do sistema quando ele é ligado ou sofre um reset inesperado
  // ========================================================================================================
  else if (fonte == FonteON && carregou == CarregouOFF && controlador != ModoCarga) { 
    // Proteção de inicialização: Verifica se a bateria conectada já está carregada
    if (loadvoltage1 > tensaoReinicioCarga) {
      Serial.println("-----------Sistema Reiniciado: Bateria detectada como cheia.-----------");
      carregou = CarregouON;
      controlador = ModoFonteDireta;
      SoC = 100.0;
      coulombs = Coulomb_Bat; 
    } else {
      // Se a bateria estiver com carga baixa no boot, inicia carga completa por segurança
      Serial.println("-----------Sistema Reiniciado: Iniciando carga de ciclo completo.-----------");
      timer_inicio_carga = millis(); 
      controlador = ModoCarga;
      Bit_Carga = 1023;                   // Inicia com Duty Cycle seguro (tensão mínima no SEPIC)
      delei = DeleiON;                    // Aplica carência inicial
    }
  }
}