#include <stdio.h>
#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "string.h"
#include "esp_timer.h"
#include "TimeNode.h"

static const int RX_BUF_SIZE = 1024;    // Utilizado para o buffer do pino RX da porta UART

#define TXD_PIN (GPIO_NUM_17)           // Pino 17 da ESP definido como TX
#define RXD_PIN (GPIO_NUM_16)           // Pino 16 da ESP definido como RX

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

//__________________________________________________________________________________________
// FUNÇÕES AUXILIARES
// Função para facilitar os prints de tempo
void print_time(const char *label, int64_t start_time, int64_t end_time){
    int64_t elapsed_time_ms = (end_time - start_time) / 1000; // Tempo em milissegundos
    int minutes = elapsed_time_ms / (60 * 1000);             // Minutos
    int seconds = (elapsed_time_ms % (60 * 1000)) / 1000;    // Segundos
    int milliseconds = elapsed_time_ms % 1000;              // Milissegundos

    printf("%s: %02d:%02d:%03d (min:seg:ms)\n", label, minutes, seconds, milliseconds);
}

// Inicialização da porta UART
void init_uart(void){
    const uart_config_t uart_config = {
        .baud_rate = 9600,                      // Taxa de transmissão padrão do GPS
        .data_bits = UART_DATA_8_BITS,          // 8 bits de dados
        .parity = UART_PARITY_DISABLE,          // Sem paridade
        .stop_bits = UART_STOP_BITS_1,          // 1 bit de parada
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,  // Sem controle de fluxo
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Instala o driver da UART com buffer de recepção
    esp_err_t ret = uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, RX_BUF_SIZE * 2, 10, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE("UART", "Failed to install UART driver: %s", esp_err_to_name(ret));
        return;
    }

    // Configurando UART
    ret = uart_param_config(UART_NUM_2, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE("UART", "Failed to configure UART parameters: %s", esp_err_to_name(ret));
        return;
    }

    // Configurando os pinos de TX e RX
    ret = uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE("UART", "Failed to set UART pins: %s", esp_err_to_name(ret));
    }
}

// Função para converter coordenadas NMEA para decimal
double nmea_to_decimal(const char *coord, char direction){
    double degrees = 0.0;
    double minutes = 0.0;

    // Verifica o formato: latitude tem 2 dígitos para graus, longitude tem 3 dígitos
    if (strlen(coord) > 7) {
        if (coord[4] == '.') { // Exemplo: 2655.48542 (latitude)
            sscanf(coord, "%2lf%lf", &degrees, &minutes); // 2 dígitos para latitude
        } else { // Exemplo: 04856.61815 (longitude)
            sscanf(coord, "%3lf%lf", &degrees, &minutes); // 3 dígitos para longitude
        }
    }

    // Converte para decimal
    double decimal = degrees + (minutes / 60.0);

    // Aplica sinal baseado na direção
    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }

    return decimal;
}

// Função auxiliar para calcular a orientação de um trio de pontos (p, q, r)
int calc_orientation(Point p, Point q, Point r) {
    // Retorna:
    // 0 --> p, q e r são colineares
    // 1 --> Sentido horário (direita)
    // 2 --> Sentido anti-horário (esquerda)
    double val = (q.lon - p.lon) * (r.lat - q.lat) - (q.lat - p.lat) * (r.lon - q.lon);
    if (val == 0) return 0;  // Colinear
    return (val > 0) ? 1 : 2; // Horário ou Anti-horário
}

// Função principal que verifica se o segmento de reta 'car_last_poscar_current_pos' cruza 'line_start_pointline_end_point'
bool check_line_crossing(Point car_last_pos, Point car_current_pos, Point line_start_point, Point line_end_point) {
    // Encontra as quatro orientações necessárias
    int o1 = calc_orientation(car_last_pos, car_current_pos, line_start_point);
    int o2 = calc_orientation(car_last_pos, car_current_pos, line_end_point);
    int o3 = calc_orientation(line_start_point, line_end_point, car_last_pos);
    int o4 = calc_orientation(line_start_point, line_end_point, car_current_pos);

    // Caso geral: se as orientações indicam uma interseção
    if (o1 != o2 && o3 != o4) {
        return true;
    }

    return false;
}
//__________________________________________________________________________________________

// Compara a posição atual com os checkpoints
void process_virtual_lap(Point current_pos, Point last_pos) {
    // Pega o checkpoint que estamos esperando cruzar
    VirtualLine target_line = checkpoints[lap_state.next_checkpoint_index];

    // Verifica se cruzamos a linha do checkpoint alvo
    if (check_line_crossing(last_pos, current_pos, target_line.start, target_line.end)) {
        
        int64_t current_time = esp_timer_get_time();

        // Se o checkpoint cruzado for a linha de chegada (índice 0)
        if (lap_state.next_checkpoint_index == 0) {
            if (!lap_state.racing) {
                // INÍCIO DA VOLTA
                printf("Volta iniciada!\n");
                lap_state.racing = true;
                lap_state.lap_start_time = current_time;
                lap_state.last_checkpoint_time = current_time;
            } else {
                // FIM DA VOLTA
                printf("VOLTA COMPLETADA!\n");
                // Aqui você calcula e salva o tempo total da volta e do último setor
                // print_time("Tempo Total:", lap_state.lap_start_time, current_time);
                
                // Reinicia para a próxima volta
                lap_state.lap_start_time = current_time;
                lap_state.last_checkpoint_time = current_time;
                lap_state.next_checkpoint_index = 0; // Reseta para a linha de chegada
            }
        } else {
            // Se for qualquer outro checkpoint (Setor 1, Setor 2, etc.)
            if (lap_state.racing) {
                printf("Passou pelo Checkpoint %d!\n", lap_state.next_checkpoint_index);
                // Aqui você calcula e salva o tempo do setor
                // print_time("Tempo Setor", lap_state.last_checkpoint_time, current_time);
                lap_state.last_checkpoint_time = current_time;
            }
        }

        // Avança para o próximo checkpoint
        lap_state.next_checkpoint_index++;
        // Se passamos pelo último checkpoint, o próximo alvo é a linha de chegada (índice 0)
        if (lap_state.next_checkpoint_index >= NUM_CHECKPOINTS) {
            lap_state.next_checkpoint_index = 0;
        }
    }
}

// Função para processar mensagens NMEA e extrair dados do $GNRMC
void process_nmea_line(const char *line){

    static Point last_known_position;
    static bool has_last_position = false;

    // Verifica se a linha começa com $GNRMC
    if (strncmp(line, "$GNRMC", 6) == 0) {
        // Tokens da linha separados por vírgulas
        char tokens[20][20];
        int token_count = 0;
        const char *ptr = line;

        // Divide a linha em tokens separados por vírgulas
        while (*ptr) {
            char *comma = strchr(ptr, ',');
            if (comma == NULL) {
                strcpy(tokens[token_count++], ptr);
                break;
            } else {
                strncpy(tokens[token_count], ptr, comma - ptr);
                tokens[token_count][comma - ptr] = '\0';
                token_count++;
                ptr = comma + 1;
            }
        }
        
        // Status (V = inválido, A = ativo)
        if (strcmp(tokens[2], "A") == 0) {

            Point current_position = {.lat = nmea_to_decimal(tokens[3],tokens[4][0]), 
                                      .lon = nmea_to_decimal(tokens[5], tokens[6][0]),
                                      .speed = (atof(tokens[7]) * 1.852)};

            if (has_last_position) {
                process_virtual_lap(current_position, last_known_position);
            }

            last_known_position = current_position;
            has_last_position = true;

            // Imprime resultados
            /*
            printf("Latitude: %.8f\n", latitude);
            printf("Longitude: %.8f\n", longitude);
            printf("Velocidade: %.2f km/h\n\n", speed_kmh);
            */

        } else {
            has_last_position = false;
            printf("Sem sinal de GPS, aguarde.\n");
        }
    }
}

// Função para ler a porta UART, armazenar cada linha em buffer e enviar para process_nmea_line()
static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);

    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1); // Buffer de recepção
    if (!data) {
        ESP_LOGE(RX_TASK_TAG, "Failed to allocate memory for RX buffer");
        vTaskDelete(NULL);
        return;
    }

    char *line_buffer = (char *)malloc(RX_BUF_SIZE); // Aloca dinamicamente o buffer de linha
    if (!line_buffer) {
        ESP_LOGE(RX_TASK_TAG, "Failed to allocate memory for line buffer");
        free(data);
        vTaskDelete(NULL);
        return;
    }
    memset(line_buffer, 0, RX_BUF_SIZE); // Preenche com zeros

    int line_pos = 0; // Posição atual no buffer de linha

    while (1) {
        // Lê os dados recebidos na UART
        int rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 100 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = '\0'; // Finaliza a string recebida
            //ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data); // Mostra o log do que foi lido na porta UART

            for (int i = 0; i < rxBytes; i++) {
                char c = data[i];

                // Se for um caractere de nova linha, processa a linha
                if (c == '\n') {
                    line_buffer[line_pos] = '\0'; // Finaliza a linha
                    ESP_LOGI(RX_TASK_TAG, "Processing line: %s", line_buffer); // Mostra o log de qual linha esta sendo enviada para processamento

                    //NOVO: Deveria jogar a linha para uma 

                    // Processa a linha NMEA
                    process_nmea_line(line_buffer);

                    // Reseta o buffer de linha
                    memset(line_buffer, 0, RX_BUF_SIZE);
                    line_pos = 0;

                } else if (c != '\r' && line_pos < RX_BUF_SIZE - 1) {
                    // Adiciona o caractere ao buffer de linha, ignorando '\r'
                    line_buffer[line_pos++] = c;
                }
            }
        } else if (rxBytes < 0) {
            ESP_LOGE(RX_TASK_TAG, "UART read error");
        }
    }

    free(data);
    free(line_buffer); // Libera a memória alocada para o buffer de linha
    vTaskDelete(NULL);
}

void app_main(void){

    init_uart(); //Chama função para inicializar a porta UART

    // Core 0: Task1 Leitura e processamento da IMU - Task2 Leitura e processamento do GPS
    // Core 1: Exibição dos dados
    xTaskCreate(rx_task, "uart_rx_task", 4096, NULL, configMAX_PRIORITIES - 1, NULL); // Cria a task que lê a porta UART    
};