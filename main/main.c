#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "data_structures.h"
#include "hardware_bib.h"
#include "lap_processing.h"
#include "utils.h"

// Definição dos checkpoints
VirtualLine checkpoints[NUM_CHECKPOINTS] = {
    // Linha de Chegada/Partida
    {.start = {.lat = -23.701944, .lon = -46.699444}, .end = {.lat = -23.701389, .lon = -46.699167}},
    // Fim do Setor 1
    {.start = {.lat = -23.704167, .lon = -46.700556}, .end = {.lat = -23.704722, .lon = -46.700278}},
    // Fim do Setor 2
    {.start = {.lat = -23.707222, .lon = -46.694722}, .end = {.lat = -23.706667, .lon = -46.694167}}
};

// Inicialização da struct LapState que você sentiu falta!
LapState lap_state = {
    .racing = false, 
    .next_checkpoint_index = 0,
    .lap_start_time = 0,
    .last_checkpoint_time = 0
};

void app_main(void){

    init_uart(); //Chama função para inicializar a porta UART

    // Core 0: Task1 Leitura e processamento da IMU - Task2 Leitura e processamento do GPS
    // Core 1: Exibição dos dados
    xTaskCreate(rx_task, "uart_rx_task", 4096, NULL, configMAX_PRIORITIES - 1, NULL); // Cria a task que lê a porta UART    
};