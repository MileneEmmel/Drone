# Autopiloto de Drone Didático com ESP32 e FreeRTOS

## Visão Geral do Projeto

Este projeto foi desenvolvido como parte da avaliação M1 da disciplina de **Sistemas em Tempo Real** do curso de Engenharia de Computação da Universidade do Vale do Itajaí (UNIVALI), sob a orientação do Prof. Felipe Viel.

O objetivo é modelar e analisar o comportamento de um sistema de controle de drone didático em uma placa ESP32 com o sistema operacional FreeRTOS. O foco principal é a investigação de como diferentes políticas de escalonamento e configurações de hardware afetam a previsibilidade e a robustez de tarefas com requisitos temporais rígidos (*hard real-time*) e brandos (*soft real-time*).

Para garantir a reprodutibilidade em laboratório, todos os estímulos externos (comandos de navegação, telemetria, perturbação de carga e emergência) são simulados através dos sensores de toque capacitivos da própria placa.

## Arquitetura do Sistema

O sistema é composto por cinco tarefas principais que competem por um único núcleo do processador (configuração *unicore*):

* **`task_fus_imu` (Fusão Sensorial):**
    * **Tipo:** Periódica (T=5ms) e *Hard Real-Time*.
    * **Função:** Simula o "coração" do drone, atualizando o estado inercial (`roll`, `pitch`, `yaw`) a cada 5ms. Ao final, notifica a `task_ctrl_att`.

* **`task_ctrl_att` (Controle de Atitude):**
    * **Tipo:** Encadeada e *Hard Real-Time*.
    * **Função:** Acordada pela `task_fus_imu`, simula os cálculos de controle (PID) que seriam enviados aos motores para manter a estabilidade.

* **`task_nav_plan` (Navegação e Telemetria):**
    * **Tipo:** Dirigida a Evento e *Soft Real-time*.
    * **Função:** Gerencia os comandos do usuário. É acionada pelo **Touch B** (para atualizar rota) ou pelo **Touch C** (para solicitar um relatório de telemetria).

* **`task_fail_safe` (Segurança):**
    * **Tipo:** Dirigida a Evento e *Hard Real-Time*.
    * **Função:** A tarefa mais crítica do sistema, acionada pelo **Touch D** (emergência). Possui a maior prioridade na política customizada para garantir a resposta mais rápida possível.

* **`task_perturbation` (Perturbação):**
    * **Tipo:** Dirigida a Evento (Opcional).
    * **Função:** Acionada pelo **Touch A**, injeta uma carga de trabalho extra e imprevisível na CPU para aumentar o estresse do sistema durante os testes.

## Funcionalidades Implementadas

* **Análise Temporal Completa:** O código mede e reporta `misses de deadline` e a latência entre a interrupção (toque) e a ativação da tarefa.
* **Políticas de Escalonamento Configuráveis:** Permite alternar entre as políticas **Rate Monotonic (RM)**, **Deadline Monotonic (DM)** e **Custom Criticality** através de macros no código, facilitando a execução da metodologia de testes.
* **Debounce Robusto:** Implementa uma técnica de debounce que desativa a interrupção de toque na ISR e a reativa na tarefa correspondente, garantindo que um único toque gere um único evento.
* **Relatório Automático:** Ao final de 60 segundos de teste, uma tarefa dedicada gera um sumário completo no monitor serial, com a contagem de *misses* e estatísticas de latência, automatizando a coleta de dados.

## Como Compilar e Testar

Este projeto foi desenvolvido com o **ESP-IDF v5.5.1**.

### 1. Configuração do Projeto

As principais configurações do projeto são feitas através do `idf.py menuconfig`.

* **Frequência da CPU:** `Component config` -> `ESP System Settings` -> `CPU frequency`. (Use `80 MHz` para os testes de estresse).
* **Modo Unicore:** `Component config` -> `FreeRTOS` -> `Kernel` -> Desmarcar `Enable native FreeRTOS SMP PREEMPTION`.

### 2. Seleção da Política de Escalonamento

A política de prioridades é selecionada diretamente no arquivo `main/main.c`, ativando a macro desejada:

```c
// Alterne entre as políticas comentando/descomentando as linhas
#define POLICY_RATE_MONOTONIC
// #define POLICY_DEADLINE_MONOTONIC
// #define POLICY_CUSTOM_CRITICALITY
```

### 3. Compilação e Gravação

Para compilar, gravar na placa e abrir o monitor serial, execute o seguinte comando no terminal do ESP-IDF:

```bash
idf.py flash monitor
```

### 4. Execução dos Testes

1.  Após a inicialização, o programa informará o início do teste de 60 segundos.
2.  Durante este tempo, execute o "teste de estresse" pressionando os sensores de toque (A, B, C, D) em sequências rápidas e simultâneas.
3.  Observe os logs em tempo real (ativação de tarefas, *deadline misses*, etc.).
4.  Ao final dos 60 segundos, um relatório completo com as estatísticas do teste será impresso no monitor. Copie esses dados para a tabela do seu relatório.

## Autores

* **Milene Emmel Rovedder**
* **Daniel Henrique da Silva**
