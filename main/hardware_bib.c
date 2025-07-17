#include "hardware_bib.h"

static const int RX_BUF_SIZE = 1024;    // Utilizado para o buffer do pino RX da porta UART

#define TXD_PIN (GPIO_NUM_17)           // Pino 17 da ESP definido como TX
#define RXD_PIN (GPIO_NUM_16)           // Pino 16 da ESP definido como RX

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