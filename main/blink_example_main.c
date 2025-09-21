/*
 * Trabalho M1 - Sistemas em Tempo Real (Versão Refatorada)
 * Temática 1: Autopiloto de Drone Didático
 * Estrutura baseada no exemplo do professor, com funcionalidades completas para o trabalho.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/touch_pad.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"

// --- CONFIGURAÇÃO DOS TESTES E DO SISTEMA ---

static const char *TAG = "DRONE_STR";

// Mapeamento dos Sensores Touch (conforme pinout) aos eventos
#define TOUCH_PAD_NAV_PLAN      TOUCH_PAD_NUM4  // GPIO 13 (Touch B - Navegação)
#define TOUCH_PAD_TELEMETRY     TOUCH_PAD_NUM6  // GPIO 14 (Touch C - Telemetria)
#define TOUCH_PAD_FAIL_SAFE     TOUCH_PAD_NUM5  // GPIO 12 (Touch D - Emergência)

// Política de Prioridades (escolha uma e recompile)
#define POLICY_RATE_MONOTONIC
// #define POLICY_DEADLINE_MONOTONIC
// #define POLICY_CUSTOM_CRITICALITY

// Definição das Prioridades
#ifdef POLICY_RATE_MONOTONIC
    #define PRIO_FUS_IMU    5
    #define PRIO_FS_TASK    4
    #define PRIO_NAV_PLAN   3
    #define PRIO_CTRL_ATT   2
#endif
#ifdef POLICY_DEADLINE_MONOTONIC
    #define PRIO_FUS_IMU    5
    #define PRIO_CTRL_ATT   5
    #define PRIO_FS_TASK    4
    #define PRIO_NAV_PLAN   3
#endif
#ifdef POLICY_CUSTOM_CRITICALITY
    #define PRIO_FS_TASK    6
    #define PRIO_FUS_IMU    5
    #define PRIO_CTRL_ATT   4
    #define PRIO_NAV_PLAN   3
#endif

// Parâmetros Temporais e de Stack
#define FUS_IMU_PERIOD_MS       5
#define FUS_IMU_DEADLINE_MS     5
#define CTRL_ATT_DEADLINE_MS    5
#define NAV_PLAN_DEADLINE_MS    20
#define FS_TASK_DEADLINE_MS     10
#define STACK_SIZE              3072

// Carga de trabalho simulada (WCET) em microsegundos
#define WCET_FUS_IMU_US         1000
#define WCET_CTRL_ATT_US        800
#define WCET_NAV_PLAN_US        3500
#define WCET_FS_TASK_US         900
#define WCET_TELEMETRY_US       500

// --- COMUNICAÇÃO ENTRE TAREFAS (IPC) E DADOS GLOBAIS ---

// Handles das tarefas
static TaskHandle_t xCtrlAttHandle = NULL;
// Semáforo para a tarefa de emergência
static SemaphoreHandle_t xFailSafeSemaphore;
// PADRÃO DO PROFESSOR: Fila para eventos de navegação/telemetria
typedef enum { EV_NAV = 1, EV_TEL = 2 } nav_event_t;
static QueueHandle_t xNavQueue;

// PADRÃO DO PROFESSOR: Struct para o estado simulado do drone
typedef struct { float roll, pitch, yaw; } state_t;
static state_t g_state = {0};

// Variáveis para medição de latência e deadlines
volatile int64_t isr_nav_plan_timestamp = 0;
volatile int64_t isr_fail_safe_timestamp = 0;
static int fus_imu_misses = 0, ctrl_att_misses = 0, nav_plan_misses = 0, fs_task_misses = 0;

// PADRÃO DO PROFESSOR: Função para simular carga de forma previsível
static inline void cpu_tight_loop_us(uint32_t us) {
    int64_t start = esp_timer_get_time();
    while ((esp_timer_get_time() - start) < us) {
        __asm__ __volatile__("nop");
    }
}


// --- FUNÇÕES DAS TAREFAS ---

void fus_imu_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(FUS_IMU_PERIOD_MS);
    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        int64_t start_time = esp_timer_get_time();
        
        // Simula filtro e atualização do estado global
        g_state.roll  += 0.1f;
        g_state.yaw   += 0.05f;
        cpu_tight_loop_us(WCET_FUS_IMU_US);

        if (xCtrlAttHandle != NULL) xTaskNotifyGive(xCtrlAttHandle);
        
        int64_t execution_time = (esp_timer_get_time() - start_time) / 1000;
        if (execution_time > FUS_IMU_DEADLINE_MS) {
            fus_imu_misses++;
            ESP_LOGE(TAG, "DEADLINE MISS: FUS_IMU (Exec: %lld ms)", execution_time);
        }
    }
}

void ctrl_att_task(void *pvParameters) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        int64_t start_time = esp_timer_get_time();

        cpu_tight_loop_us(WCET_CTRL_ATT_US); // Simula PID

        int64_t execution_time = (esp_timer_get_time() - start_time) / 1000;
        if (execution_time > CTRL_ATT_DEADLINE_MS) {
            ctrl_att_misses++;
            ESP_LOGE(TAG, "DEADLINE MISS: CTRL_ATT (Exec: %lld ms)", execution_time);
        }
    }
}

// Usa Fila para receber eventos de Navegação OU Telemetria
void nav_plan_task(void *pvParameters) {
    nav_event_t event;
    while (1) {
        if (xQueueReceive(xNavQueue, &event, portMAX_DELAY) == pdTRUE) {
            int64_t task_start_time = esp_timer_get_time();
            int64_t latency = (task_start_time - isr_nav_plan_timestamp) / 1000;
            
            if (event == EV_NAV) {
                ESP_LOGW(TAG, "NAV_PLAN: Touch B (Navegação) ativado. Latência: %lld ms", latency);
                cpu_tight_loop_us(WCET_NAV_PLAN_US);
                int64_t execution_time = (esp_timer_get_time() - task_start_time) / 1000;
                if ((latency + execution_time) > NAV_PLAN_DEADLINE_MS) {
                    nav_plan_misses++;
                    ESP_LOGE(TAG, "DEADLINE MISS: NAV_PLAN (Total: %lld ms)", latency + execution_time);
                }
            } else if (event == EV_TEL) {
                ESP_LOGI(TAG, "NAV_PLAN: Touch C (Telemetria) ativado. Latência: %lld ms", latency); // <-- ALTERE ESTA LINHA
                printf("TEL: roll=%.2f pitch=%.2f yaw=%.2f\n", g_state.roll, g_state.pitch, g_state.yaw);
                cpu_tight_loop_us(WCET_TELEMETRY_US);
            }
        }
    }
}

void fs_task(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(xFailSafeSemaphore, portMAX_DELAY) == pdTRUE) {
            int64_t task_start_time = esp_timer_get_time();
            int64_t latency = (task_start_time - isr_fail_safe_timestamp) / 1000;
            ESP_LOGE(TAG, "FAIL-SAFE: Touch D ativado! Latência: %lld ms", latency);

            cpu_tight_loop_us(WCET_FS_TASK_US);

            int64_t execution_time = (esp_timer_get_time() - task_start_time) / 1000;
            if ((latency + execution_time) > FS_TASK_DEADLINE_MS) {
                fs_task_misses++;
                ESP_LOGE(TAG, "DEADLINE MISS: FS_TASK (Total: %lld ms)", latency + execution_time);
            }
        }
    }
}


// --- INTERRUPÇÕES E INICIALIZAÇÃO ---

static void touch_isr_handler(void *arg) {
    uint32_t pad_intr = touch_pad_get_status();
    touch_pad_clear_status();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Envia eventos diferentes para a mesma Fila
    if (pad_intr & (1 << TOUCH_PAD_NAV_PLAN)) {
        isr_nav_plan_timestamp = esp_timer_get_time();
        nav_event_t ev = EV_NAV;
        xQueueSendFromISR(xNavQueue, &ev, &xHigherPriorityTaskWoken);
    }
    if (pad_intr & (1 << TOUCH_PAD_TELEMETRY)) {
        // reusa o timestamp da NAV_PLAN para simplificar, já que a deadline é soft
        isr_nav_plan_timestamp = esp_timer_get_time(); 
        nav_event_t ev = EV_TEL;
        xQueueSendFromISR(xNavQueue, &ev, &xHigherPriorityTaskWoken);
    }

    // Usa semáforo por ser um evento único e de alta prioridade
    if (pad_intr & (1 << TOUCH_PAD_FAIL_SAFE)) {
        isr_fail_safe_timestamp = esp_timer_get_time();
        xSemaphoreGiveFromISR(xFailSafeSemaphore, &xHigherPriorityTaskWoken);
    }

    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

void app_main(void) {
    ESP_LOGI(TAG, "Inicializando Autopiloto Didático STR...");
    ESP_LOGI(TAG, "Política de Escalonamento: %s",
    #ifdef POLICY_RATE_MONOTONIC
        "Rate Monotonic (RM)"
    #elif defined(POLICY_DEADLINE_MONOTONIC)
        "Deadline Monotonic (DM)"
    #else
        "Custom Criticality"
    #endif
    );

    // Criar Fila e Semáforo
    xNavQueue = xQueueCreate(8, sizeof(nav_event_t));
    xFailSafeSemaphore = xSemaphoreCreateBinary();

    // Inicializar e configurar os Touch Pads
    touch_pad_init();
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    touch_pad_config(TOUCH_PAD_NAV_PLAN, 0);
    touch_pad_config(TOUCH_PAD_TELEMETRY, 0);
    touch_pad_config(TOUCH_PAD_FAIL_SAFE, 0);
    touch_pad_filter_start(10);

    // Definir o threshold (limiar) para o toque
    uint16_t touch_value;
    touch_pad_read_filtered(TOUCH_PAD_NAV_PLAN, &touch_value);
    touch_pad_set_thresh(TOUCH_PAD_NAV_PLAN, touch_value * 0.2);
    touch_pad_read_filtered(TOUCH_PAD_TELEMETRY, &touch_value);
    touch_pad_set_thresh(TOUCH_PAD_TELEMETRY, touch_value * 0.2);
    touch_pad_read_filtered(TOUCH_PAD_FAIL_SAFE, &touch_value);
    touch_pad_set_thresh(TOUCH_PAD_FAIL_SAFE, touch_value * 0.2);

    // Registrar a ISR
    touch_pad_isr_register(touch_isr_handler, NULL);
    touch_pad_intr_enable();

    // Criar as tarefas com as prioridades definidas
    xTaskCreatePinnedToCore(
        fus_imu_task, "FUS_IMU", STACK_SIZE, NULL, PRIO_FUS_IMU, NULL, 0
    );
    xTaskCreatePinnedToCore(
        ctrl_att_task, "CTRL_ATT", STACK_SIZE, NULL, PRIO_CTRL_ATT, &xCtrlAttHandle, 0
    );
    xTaskCreatePinnedToCore(
        nav_plan_task, "NAV_PLAN", STACK_SIZE, NULL, PRIO_NAV_PLAN, NULL, 0
    );
    xTaskCreatePinnedToCore(
        fs_task, "FS_TASK", STACK_SIZE, NULL, PRIO_FS_TASK, NULL, 0
    );

    ESP_LOGI(TAG, "Sistema inicializado. Tarefas criadas e em execução no Core 0.");
    ESP_LOGI(TAG, "Pressione os Touch Pads para gerar eventos.");
}