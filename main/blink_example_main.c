/*
 * Trabalho M1 - Sistemas em Tempo Real
 * Temática 1: Drone
 * Autores: Daniel Henrique da Silva e Milene Emmel Rovedder
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/touch_pad.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOSConfig.h"

#define TAG "DRONE" // Define a tag para os logs do sistema

// Mapeamento dos touch pads para eventos do sistema.
#define TOUCH_A TOUCH_PAD_NUM3 // (GPIO 15) -> Perturbação/Sobrecarga
#define TOUCH_B TOUCH_PAD_NUM4 // (GPIO 13) -> Controle de rota / Navegação
#define TOUCH_C TOUCH_PAD_NUM7 // (GPIO 27) -> Telemetria
#define TOUCH_D TOUCH_PAD_NUM9 // (GPIO 32) -> Fail-safe / Emergência

#define DEBOUNCE_DELAY_MS 200 // Constante para o tempo de debounce em milissegundos

// Parâmetros de tempo e stack para as tarefas
#define FUS_T_MS 5    // Período da FUS_IMU em milissegundos
#define STK      3072 // Tamanho da stack para as tarefas

// Deadlines para cada tarefa em milissegundos
#define FUS_IMU_DEADLINE_MS  5
#define CTRL_ATT_DEADLINE_MS 5
#define NAV_PLAN_DEADLINE_MS 20
#define FS_TASK_DEADLINE_MS  10

// Carga de trabalho simulada (WCET) em microssegundos
#define WCET_FUS_IMU_US      1000
#define WCET_CTRL_ATT_US     800
#define WCET_NAV_PLAN_US     3500
#define WCET_FS_TASK_US      900
#define WCET_TELEMETRY_US    500
#define WCET_PERTURBATION_US 2000 // Carga extra para a perturbação

// Política de escalonamento
//#define POLICY_RATE_MONOTONIC
// #define POLICY_DEADLINE_MONOTONIC
 #define POLICY_CUSTOM_CRITICALITY

// Prioridades com base na política escolhida.
#ifdef POLICY_RATE_MONOTONIC
    #define PRIO_FUS_IMU      5
    #define PRIO_FS_TASK      4
    #define PRIO_PERTURBATION 4
    #define PRIO_NAV_PLAN     3
    #define PRIO_CTRL_ATT     2
    #define POLICY_NAME "Rate Monotonic (RM)"
#endif
#ifdef POLICY_DEADLINE_MONOTONIC
    #define PRIO_FUS_IMU      5
    #define PRIO_CTRL_ATT     5
    #define PRIO_FS_TASK      4
    #define PRIO_PERTURBATION 4
    #define PRIO_NAV_PLAN     3
     #define POLICY_NAME "Deadline Monotonic (DM)"
#endif
#ifdef POLICY_CUSTOM_CRITICALITY
    #define PRIO_FS_TASK      6 // Emergência tem a maior prioridade
    #define PRIO_FUS_IMU      5
    #define PRIO_PERTURBATION 5
    #define PRIO_CTRL_ATT     4
    #define PRIO_NAV_PLAN     3
    #define POLICY_NAME "Custom Criticality"
#endif

static TaskHandle_t hFUS = NULL, hCTRL = NULL, hNAV = NULL, hFS = NULL; // Handles para referenciar as tarefas
typedef enum { EV_NAV = 1, EV_TEL = 2 } nav_evt_t; // Eventos para NAV_PLAN (Navegação e Telemetria)
static QueueHandle_t qNav = NULL; // Fila para NAV_PLAN - Escolhida por ser flexível e poder enviar dados (o tipo de evento).
static SemaphoreHandle_t semFS = NULL; // Semáforo para o Fail-Safe - Escolhido por ser o mecanismo mais rápido para sinalizar de uma ISR.
static SemaphoreHandle_t semPerturbation = NULL; // Semáforo para a tarefa de perturbação.

// Struct para simular o estado (orientação) do drone.
typedef struct {
    float roll, pitch, yaw;
} state_t;
static state_t g_state = {0};

// Timestamps para cálculo de latência.
volatile int64_t isr_event_timestamp = 0, isr_fs_timestamp = 0;
// Contadores de deadline misses para o relatório.
static int fus_imu_misses = 0, ctrl_att_misses = 0, nav_plan_misses = 0, fs_task_misses = 0; 

// Variáveis para as estatísticas do relatório automático.
volatile uint64_t nav_event_count = 0;
volatile uint64_t nav_latency_sum = 0;
volatile int64_t  nav_latency_max = 0;
volatile uint64_t fs_event_count  = 0;
volatile uint64_t fs_latency_sum  = 0;
volatile int64_t  fs_latency_max  = 0;

// Variável para controlar o tempo do debounce.
volatile int64_t last_isr_time = 0;

// Função para simular carga de trabalho (WCET) de forma previsível.
static inline void cpu_tight_loop_us(uint32_t us) {
    int64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < us) { 
        __asm__ __volatile__("nop");
    }
}

// Fusão de sensores - Periódica (T = 5ms, D = 5ms, C = 0,8ms)
static void task_fus_imu(void *arg) {
    TickType_t next = xTaskGetTickCount();
    const TickType_t T = pdMS_TO_TICKS(FUS_T_MS);
    for (;;) {
        vTaskDelayUntil(&next, T); // vTaskDelayUntil garante uma periodicidade precisa (determinística), corrigindo jitters.
        int64_t start_time = esp_timer_get_time();
        
        // Simulação rápida de filtro (Madgwick/Complementar "fake") + "carga" de 1ms
        g_state.roll  *= 0.98f;  g_state.pitch *= 0.98f;  g_state.yaw *= 0.98f;
        g_state.roll  += 0.1f;   g_state.yaw   += 0.05f;
        cpu_tight_loop_us(WCET_FUS_IMU_US);

        if (hCTRL) xTaskNotifyGive(hCTRL); // Notifica a tarefa de controle que um novo dado está pronto.
        
        // Verifica se a tarefa cumpriu seu deadline.
        int64_t exec_time = (esp_timer_get_time() - start_time) / 1000;
        if (exec_time > FUS_IMU_DEADLINE_MS) {
            fus_imu_misses++;
            ESP_LOGE(TAG, "DEADLINE MISS: FUS_IMU (Exec: %lld ms)", exec_time);
        }
    }
}

// Controle de altitude - Ativada pela FUS_IMU (D = 5ms, C = 0,8ms)
static void task_ctrl_att(void *arg) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Bloqueia a tarefa até que a FUS_IMU a notifique.
        int64_t start_time = esp_timer_get_time();

        cpu_tight_loop_us(WCET_CTRL_ATT_US); // PID simulado com carga de 0,8ms

        // Verifica se a tarefa cumpriu seu deadline.
        int64_t exec_time = (esp_timer_get_time() - start_time) / 1000;
        if (exec_time > CTRL_ATT_DEADLINE_MS) {
            ctrl_att_misses++;
            ESP_LOGE(TAG, "DEADLINE MISS: CTRL_ATT (Exec: %lld ms)", exec_time);
        }
    }
}

// Controle de rota/Navegação (Touch B) (D = 20ms, C = 3,5ms) || Telemetria (Touch C) (D = 20ms, C = 0,5ms)
static void task_nav_plan(void *arg) {
    nav_evt_t ev;
    for (;;) {
        if (xQueueReceive(qNav, &ev, portMAX_DELAY) == pdTRUE) { // Bloqueia até receber um evento (Touch B ou C) na Fila.
            int64_t task_start_time = esp_timer_get_time();
            int64_t latency = (task_start_time - isr_event_timestamp) / 1000;
            
            if (ev == EV_NAV) { // Controle de rota / Navegação (Touch B)
                ESP_LOGW(TAG, "NAV_PLAN: Touch B (Navegação) ativado. Latência: %lld ms", latency);
                cpu_tight_loop_us(WCET_NAV_PLAN_US); // ~3,5ms
                
            } else if (ev == EV_TEL) { // Telemetria (Touch C)
                ESP_LOGI(TAG, "NAV_PLAN: Touch C (Telemetria) ativado. Latência: %lld ms", latency);
                printf("TEL: roll=%.2f pitch=%.2f yaw=%.2f\n", g_state.roll, g_state.pitch, g_state.yaw);
                cpu_tight_loop_us(WCET_TELEMETRY_US); // ~0,5ms
            }

            // Verifica se a tarefa cumpriu seu deadline.
            int64_t exec_time = (esp_timer_get_time() - task_start_time) / 1000;
            if ((latency + exec_time) > NAV_PLAN_DEADLINE_MS) {
                nav_plan_misses++;
                ESP_LOGE(TAG, "DEADLINE MISS: NAV_PLAN (Total: %lld ms)", latency + exec_time);
            }

            nav_event_count++;
            nav_latency_sum += latency;
            if (latency > nav_latency_max) { 
                nav_latency_max = latency; 
            }
            
        }
    }
}

// Fail-safe / Emergência (Touch D) (D = 10ms, C = 0,9ms)
static void task_fail_safe(void *arg) {
    for (;;) {
        if (xSemaphoreTake(semFS, portMAX_DELAY) == pdTRUE) { // Bloqueia até receber o sinal de emergência do ISR
            int64_t task_start_time = esp_timer_get_time();
            int64_t latency = (task_start_time - isr_fs_timestamp) / 1000;
            
            ESP_LOGE(TAG, "FAIL-SAFE: Touch D ativado! Latência: %lld ms", latency);

            // Simula a rotina de pouso seguro.
            cpu_tight_loop_us(WCET_FS_TASK_US);

            // Verifica se a tarefa cumpriu seu deadline.
            int64_t exec_time = (esp_timer_get_time() - task_start_time) / 1000;
            if ((latency + exec_time) > FS_TASK_DEADLINE_MS) {
                fs_task_misses++;
                ESP_LOGE(TAG, "DEADLINE MISS: FS_TASK (Total: %lld ms)", latency + exec_time);
            }

            fs_event_count++;
            fs_latency_sum += latency;
            if (latency > fs_latency_max) {
                fs_latency_max = latency;
            }
        }
    }
}

// Perturbação/Sobrecarga (Touch A)
static void task_perturbation(void *arg) {
    for (;;) {
        if (xSemaphoreTake(semPerturbation, portMAX_DELAY) == pdTRUE) { // Bloqueia até receber o sinal do Touch A.
            ESP_LOGI(TAG, "PERTURBAÇÃO: Touch A ativado. Injetando %d us de carga extra na CPU.", WCET_PERTURBATION_US);
            cpu_tight_loop_us(WCET_PERTURBATION_US);

            if (hCTRL) xTaskNotifyGive(hCTRL); // Notifica a tarefa de controle que um novo dado está pronto, gerando sobrecarga
        }
    }
}

// Relatório automático ao final do teste
void task_report(void *pvParameters) {
    const int test_duration_ms = 60000; // Duração do teste em milissegundos (60 segundos)
    ESP_LOGI(TAG, "Teste de 60 segundos iniciado. Pressione os botões...");
    vTaskDelay(pdMS_TO_TICKS(test_duration_ms)); // Dorme pelo tempo do teste

    int64_t nav_latency_avg = 0;
    if (nav_event_count > 0) {
        nav_latency_avg = nav_latency_sum / nav_event_count;
    }

    int64_t fs_latency_avg = 0;
    if (fs_event_count > 0) {
        fs_latency_avg = fs_latency_sum / fs_event_count;
    }

    printf("\n\nRELATÓRIO FINAL DO TESTE\n");
    printf("==============================================\n");
    printf("Política de Escalonamento: %s\n", POLICY_NAME);

    printf("----------------------------------------------\n");
    printf("DEADLINE MISSES:\n");
    printf("  - FUS_IMU  (Hard RT): %d\n", fus_imu_misses);
    printf("  - CTRL_ATT (Hard RT): %d\n", ctrl_att_misses);
    printf("  - FS_TASK  (Hard RT): %d\n", fs_task_misses);
    printf("  - NAV_PLAN (Soft RT): %d\n", nav_plan_misses);
    printf("----------------------------------------------\n");
    printf("LATÊNCIA DE EVENTOS (ISR -> Tarefa):\n");
    printf("  - Navegação/Telemetria (Touch B/C):\n");
    printf("      Eventos Registrados: %llu\n",    nav_event_count);
    printf("      Latência Média:      %lld ms\n", nav_latency_avg);
    printf("      Latência Máxima:     %lld ms\n", nav_latency_max);
    printf("  - Fail-Safe (Touch D):\n");
    printf("      Eventos Registrados: %llu\n",    fs_event_count);
    printf("      Latência Média:      %lld ms\n", fs_latency_avg);
    printf("      Latência Máxima:     %lld ms\n", fs_latency_max);
    printf("==============================================\n");
    printf("Fim do teste.\n\n");
    vTaskDelete(NULL);
}

// ISR (Rotina de Serviço de Interrupção) - Detecção capacitiva de toque
static void touch_isr_handler(void *arg) {
    uint32_t pad_intr = touch_pad_get_status();
    touch_pad_clear_status();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Debounce
    int64_t current_time = esp_timer_get_time();
    if ((current_time - last_isr_time) < (DEBOUNCE_DELAY_MS * 1000)) {
        touch_pad_clear_status();
        return;
    }
    last_isr_time = current_time;

    // Detecção capacitiva de toque e envio de eventos para a tarefa correspondente
    if (pad_intr & (1 << TOUCH_B)) {
        isr_event_timestamp = esp_timer_get_time();
        nav_evt_t ev = EV_NAV;
        xQueueSendFromISR(qNav, &ev, &xHigherPriorityTaskWoken);
    }
    if (pad_intr & (1 << TOUCH_C)) {
        isr_event_timestamp = esp_timer_get_time();
        nav_evt_t ev = EV_TEL;
        xQueueSendFromISR(qNav, &ev, &xHigherPriorityTaskWoken);
    }
    if (pad_intr & (1 << TOUCH_D)) {
        isr_fs_timestamp = esp_timer_get_time();
        xSemaphoreGiveFromISR(semFS, &xHigherPriorityTaskWoken);
    }
    if (pad_intr & (1 << TOUCH_A)) {
        xSemaphoreGiveFromISR(semPerturbation, &xHigherPriorityTaskWoken);
    }

    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

// Função principal: executada apenas uma vez para configurar e iniciar o sistema.
void app_main(void) {
    ESP_LOGI(TAG, "Inicializando Autopiloto Didático STR...");
    ESP_LOGI(TAG, "Política de Escalonamento: %s", POLICY_NAME);

    // Inicializa os mecanismos de comunicação (IPC)
    qNav  = xQueueCreate(8, sizeof(nav_evt_t));
    semFS = xSemaphoreCreateBinary();
    semPerturbation = xSemaphoreCreateBinary();

    // Configuração dos touch pads
    touch_pad_init();
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    touch_pad_config(TOUCH_B, 0);
    touch_pad_config(TOUCH_C, 0);
    touch_pad_config(TOUCH_D, 0);
    touch_pad_config(TOUCH_A, 0);
    touch_pad_filter_start(10);
    uint16_t touch_value;
    touch_pad_read_filtered(TOUCH_B, &touch_value);
    touch_pad_set_thresh(TOUCH_B, touch_value * 0.4);
    touch_pad_read_filtered(TOUCH_C, &touch_value);
    touch_pad_set_thresh(TOUCH_C, touch_value * 0.4);
    touch_pad_read_filtered(TOUCH_D, &touch_value);
    touch_pad_set_thresh(TOUCH_D, touch_value * 0.4);
    touch_pad_read_filtered(TOUCH_A, &touch_value);
    touch_pad_set_thresh(TOUCH_A, touch_value * 0.4);
    touch_pad_isr_register(touch_isr_handler, NULL);
    touch_pad_intr_enable();

    // Cria as tarefas e entrega o controle ao escalonador do FreeRTOS.
    xTaskCreatePinnedToCore(task_fail_safe, "FS_TASK",  STK, NULL, PRIO_FS_TASK,  &hFS,   0);
    xTaskCreatePinnedToCore(task_fus_imu,   "FUS_IMU",  STK, NULL, PRIO_FUS_IMU,  &hFUS,  0);
    xTaskCreatePinnedToCore(task_ctrl_att,  "CTRL_ATT", STK, NULL, PRIO_CTRL_ATT, &hCTRL, 0);
    xTaskCreatePinnedToCore(task_nav_plan,  "NAV_PLAN", STK, NULL, PRIO_NAV_PLAN, &hNAV,  0);
    xTaskCreatePinnedToCore(task_perturbation, "PERTURB_TASK", STK, NULL, PRIO_PERTURBATION, NULL, 0);
    
    // Relatório
    xTaskCreatePinnedToCore(task_report, "REPORT_TASK", 4096, NULL, 2, NULL, 0);

    ESP_LOGI(TAG, "Sistema inicializado. Tarefas criadas e em execução no Core 0.");
    
    vTaskDelete(NULL);
}