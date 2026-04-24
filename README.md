# LiteCOPASAelevatoria

## Descrição do Projeto
O **LiteCOPASAelevatoria** é um sistema de monitoramento remoto desenvolvido para sistemas de elevação de água. O projeto baseia-se em um microcontrolador ESP32 que realiza a leitura de parâmetros físicos por meio de conversores de sensores industriais, e leitura de sensores dedicados do projeto, e transmite os dados via telemetria utilizando o protocolo MQTT sobre uma rede móvel.

## Descrição da planta de instalação
Disponível em "COMO_CONSTRUIDO_ELEVATÓRIA_COPASA_MORRO_DOS_PINTOS (3).pdf"

## Funcionalidades Principais
* **Monitoramento Contínuo:** Leitura de temperatura, parâmetros elétricos e vibracionais em tempo real.
* **Autogerenciamento de Energia** Atuação em conjunto com Sistema Inteligente de Fornecimento de Energia.
* **Telemetria MQTT:** Transmissão de dados via protocolo MQTT, permitindo integração com painéis de visualização e bancos de dados, a partir do Node-Red.
* **Conectividade Via telefonia móvel:** Autenticação e conexão em redes 4G.
* **Eficiência Energética:** Implementação de rotinas de *Deep Sleep* no ESP32 para conservação de energia.

## Versão atual do código
Disponível na pasta "principal", nele estará subdividido em: principal/principal.ino, que é a main do projeto presente atualmennte no ESP32, a principal/SIFE_LIB.h, que é a versão do LiteCOPASAelevatoria para a biblioteca do Sistema Inteligente de Fornecimento de Energia integrado ao projeto. E ainda, principal/Connectar_Desconectar.h, que tem as configurações de uso do envio de dados via rede móvel.

## Resultados e relatórios
Disponível em "Testes&Resultados"

## Referências Teóricas
Disponível em "Referências", e "ReferênciasInternasLITE", estas sendo as usadas pelo Orientador do projeto, Adriano Borges, em suas aulas no ensino técnico.

## Autores
* **Bernardo Batista Vilaça**
* **Felipe Augusto Ramos Diniz**
