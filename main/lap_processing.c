#include "lap_processing.h"

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
