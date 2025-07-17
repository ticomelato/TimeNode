#include "utils.h"

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

// Função para facilitar os prints de tempo
void print_time(const char *label, int64_t start_time, int64_t end_time){
    int64_t elapsed_time_ms = (end_time - start_time) / 1000; // Tempo em milissegundos
    int minutes = elapsed_time_ms / (60 * 1000);             // Minutos
    int seconds = (elapsed_time_ms % (60 * 1000)) / 1000;    // Segundos
    int milliseconds = elapsed_time_ms % 1000;              // Milissegundos

    printf("%s: %02d:%02d:%03d (min:seg:ms)\n", label, minutes, seconds, milliseconds);
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

