#ifndef BACKUP_H
#define BACKUP_H

#include <Arduino.h>
#include <string.h>
#include "Connectar_Desconectar.h"

#define BACKUP_MAX_REGISTROS 4
#define BACKUP_INTERVALO_HORAS 6
#define BACKUP_NUM_AMOSTRAS 64


// Registro persistente com todas as medidas do ciclo (versao reduzida para RTC).
struct BackupRecord {
	float temperatura;
	float pressao;
	float v_fonte;
	float v_bat;
	float i_bat;
	float soc;
	int16_t fonte;
	uint8_t erro_ina1;
	uint8_t erro_ina2;
	AmostraAcelerometro vib1[BACKUP_NUM_AMOSTRAS];
	AmostraAcelerometro vib2[BACKUP_NUM_AMOSTRAS];
	AmostraAcelerometro vib3[BACKUP_NUM_AMOSTRAS];
};

RTC_DATA_ATTR uint16_t contBackup = 0;
RTC_DATA_ATTR uint8_t conteudoBackup = 0;
RTC_DATA_ATTR bool medidaImportante = false;
RTC_DATA_ATTR BackupRecord backupRegistros[BACKUP_MAX_REGISTROS];
static unsigned long ciclosBackup = 0;

static unsigned long calcularCiclosBackup() {
	const unsigned long ciclo_s = TEMPO_ENVIO_AC;
	const unsigned long intervalo_s = BACKUP_INTERVALO_HORAS * 3600UL;
	if (ciclo_s == 0) return 1;
	unsigned long ciclos = intervalo_s / ciclo_s;
	if (ciclos == 0) ciclos = 1;
	return ciclos;
}

// Limpa o buffer da RTC (reseta conteudo e zera a memoria).
static void limparRegistrosBackup() {
	conteudoBackup = 0;
	for (uint8_t i = 0; i < BACKUP_MAX_REGISTROS; i++) {
		memset(&backupRegistros[i], 0, sizeof(BackupRecord));
	}
	Serial.println("[BACKUP] Buffer RTC limpo.");
}

// Reduz o vetor de vibracao para caber na RTC (amostragem uniforme).
static void reduzirAmostras(AmostraAcelerometro *destino, const AmostraAcelerometro *origem) {
	const int passo = NUM_AMOSTRAS / BACKUP_NUM_AMOSTRAS;
	for (int i = 0; i < BACKUP_NUM_AMOSTRAS; i++) {
		destino[i] = origem[i * passo];
	}
}

// Envia vibracao de backup em um unico pacote (amostras reduzidas).
static bool enviarVibracaoBackup(int sensorId, const AmostraAcelerometro *buffer, const char *topic) {
	jsonLarge.clear();
	jsonLarge["s"] = sensorId;
	jsonLarge["p"] = 1;
	jsonLarge["n"] = BACKUP_NUM_AMOSTRAS;

	JsonArray dataX = jsonLarge.createNestedArray("x");
	JsonArray dataY = jsonLarge.createNestedArray("y");
	JsonArray dataZ = jsonLarge.createNestedArray("z");

	for (int i = 0; i < BACKUP_NUM_AMOSTRAS; i++) {
		dataX.add(buffer[i].x);
		dataY.add(buffer[i].y);
		dataZ.add(buffer[i].z);
	}

	String jsonString;
	serializeJson(jsonLarge, jsonString);
	String cipher = crypto.encryptString(jsonString);
	bool ok = client.publish(topic, cipher.c_str());
	if (!ok) {
		Serial.println("[BACKUP] Falha ao publicar vibracao reduzida.");
	}
	return ok;
}

static void backupPreencherRegistro(
	BackupRecord &record,
	float temperatura,
	float pressao,
	float v_fonte,
	float v_bat,
	float i_bat,
	float soc,
	int16_t fonte,
	bool erroIna1,
	bool erroIna2,
	const AmostraAcelerometro *vib1,
	const AmostraAcelerometro *vib2,
	const AmostraAcelerometro *vib3
) {
	record.temperatura = temperatura;
	record.pressao = pressao;
	record.v_fonte = v_fonte;
	record.v_bat = v_bat;
	record.i_bat = i_bat;
	record.soc = soc;
	record.fonte = fonte;
	record.erro_ina1 = erroIna1 ? 1 : 0;
	record.erro_ina2 = erroIna2 ? 1 : 0;
	reduzirAmostras(record.vib1, vib1);
	reduzirAmostras(record.vib2, vib2);
	reduzirAmostras(record.vib3, vib3);
}

// Calcula o numero de ciclos para atingir 6h sem depender de RTC/RTCClock.
void Backup_Begin() {
	if (ciclosBackup == 0) {
		ciclosBackup = calcularCiclosBackup();
	}
	Serial.println("[BACKUP] Sistema iniciado. Ciclos por janela = " + String(ciclosBackup));
}

// Incrementa o contador de ciclos e reinicia quando atingir a janela de backup.
void Backup_AtualizarCiclo() {
	if (ciclosBackup == 0) {
		ciclosBackup = calcularCiclosBackup();
	}

	if (contBackup == ciclosBackup) {
		contBackup = 0;
	}
	contBackup = contBackup + 1;
	Serial.println("[BACKUP] Ciclo atual: " + String(contBackup) + "/" + String(ciclosBackup));
	if (contBackup == ciclosBackup) {
		Serial.println("[BACKUP] Ciclo importante: se falhar, salva na RTC.");
	}
}

// Indica se este ciclo e o ciclo de coleta obrigatoria (6h).
bool Backup_CicloImportante() {
	return contBackup == ciclosBackup;
}

// Marca a medida como relevante caso falhe a comunicacao no ciclo importante.
void Backup_MarcarFalha() {
	if (Backup_CicloImportante()) {
		medidaImportante = true;
		Serial.println("[BACKUP] Falha no ciclo importante. Marcado para armazenar.");
	}
}

void Backup_ArmazenarSeNecessario(
	float temperatura,
	float pressao,
	float v_fonte,
	float v_bat,
	float i_bat,
	float soc,
	int16_t fonte,
	bool erroIna1,
	bool erroIna2,
	const AmostraAcelerometro *vib1,
	const AmostraAcelerometro *vib2,
	const AmostraAcelerometro *vib3
) {
	// Armazena apenas se for um ciclo importante que falhou.
	if (!medidaImportante) {
		Serial.println("[BACKUP] Falha nao era ciclo importante. Nada sera salvo.");
		return;
	}

	medidaImportante = false;
	conteudoBackup = conteudoBackup + 1;

	uint8_t index = 0;
	if (conteudoBackup <= BACKUP_MAX_REGISTROS) {
		index = conteudoBackup - 1;
	} else {
		limparRegistrosBackup();
		conteudoBackup = 1;
		index = 0;
	}

	backupPreencherRegistro(backupRegistros[index], temperatura, pressao, v_fonte, v_bat, i_bat, soc, fonte, erroIna1, erroIna2, vib1, vib2, vib3);
	Serial.println("[BACKUP] Registro salvo na RTC. Posicao = " + String(index + 1) + "/" + String(BACKUP_MAX_REGISTROS));
}

// Envia os pendentes primeiro; se algum enviar, limpa o buffer todo.
void Backup_EnviarPendentes() {
	if (conteudoBackup == 0) {
		Serial.println("[BACKUP] Sem pendencias para envio.");
		return;
	}

	bool deletarBackup = false;
	Serial.println("[BACKUP] Enviando pendencias. Total = " + String(conteudoBackup));
	for (uint8_t i = 0; i < conteudoBackup; i++) {
		bool sucesso = true;
		BackupRecord &record = backupRegistros[i];
		String jsonSife = getEnergiaSife(record.v_fonte, record.v_bat, record.i_bat, record.soc, record.fonte, record.erro_ina1, record.erro_ina2);
		String jsonPressao = getMedida(record.pressao);
		String jsonTemperatura = getMedida(record.temperatura);

		if (!client.publish(TOPIC_ENERGIA_BACKUP, jsonSife.c_str())) {
			sucesso = false;
		}
		waitingTime(300);

		if (!client.publish(TOPIC_PRESSAO_BACKUP, jsonPressao.c_str())) {
			sucesso = false;
		}
		waitingTime(300);

		if (!client.publish(TOPIC_TEMP_BACKUP, jsonTemperatura.c_str())) {
			sucesso = false;
		}
		waitingTime(300);

		if (!enviarVibracaoBackup(1, record.vib1, TOPIC_VIBRA_S1_BACKUP)) {
			sucesso = false;
		}

		if (!enviarVibracaoBackup(2, record.vib2, TOPIC_VIBRA_S2_BACKUP)) {
			sucesso = false;
		}

		if (!enviarVibracaoBackup(3, record.vib3, TOPIC_VIBRA_S3_BACKUP)) {
			sucesso = false;
		}

		if (sucesso) {
			deletarBackup = true;
			Serial.println("[BACKUP] Registro enviado com sucesso.");
		} else {
			Serial.println("[BACKUP] Falha ao enviar registro pendente.");
		}
	}

	if (deletarBackup) {
		limparRegistrosBackup();
		Serial.println("[BACKUP] Pendencias limpas apos envio.");
	}
}

#endif
