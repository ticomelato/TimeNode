#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

// --- CONSTANTES E DEFINIÇÕES DE TIPO ---

#define NUM_CHECKPOINTS 3

// Estrutura para representar um ponto geográfico
typedef struct {
    double lat;
    double lon;
    float speed;
} Point;

// Estrutura para representar uma linha virtual (definida por dois pontos)
typedef struct {
    Point start;
    Point end;
} VirtualLine;

// Estrutura para armazenar o estado atual da volta
typedef struct {
    bool racing;                   // Indica se uma volta está em andamento
    int next_checkpoint_index;     // Índice do próximo checkpoint (0 para linha de chegada)
    int64_t lap_start_time;        // Timestamp do início da volta
    int64_t last_checkpoint_time;  // Timestamp do último checkpoint cruzado
} LapState;


// --- DECLARAÇÕES DE VARIÁVEIS GLOBAIS ---
// Apenas declaramos que as variáveis existem. A definição delas deve estar em UM ÚNICO arquivo .c

extern VirtualLine checkpoints[NUM_CHECKPOINTS];
extern LapState lap_state;

#endif // DATA_STRUCTURES_H