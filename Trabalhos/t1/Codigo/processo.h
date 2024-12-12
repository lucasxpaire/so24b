typedef enum {
    MORTO, 
    PRONTO, 
    BLOQUEADO, 
    EXECUTANDO,
    BLOQUEADO_ES,
    BLOQUEADO_PROC
} estado_t;

typedef enum {
    BLOQUEIO_ES,
    BLOQUEIO_LE,
    BLOQUEIO_ESPERA
} motivo_bloqueio_t;

typedef struct porta_t {
    bool ocupada;
    int estadoTeclado;
    int teclado;
    int estadoTela;
    int tela;
    struct porta_t *proxima; // Ponteiro para a próxima porta livre
} porta_t;

typedef struct processo_t {
    int pid;
    int reg_PC;
    int reg_A;
    int reg_X;
    int reg_erro;
    int reg_complemento;
    int modo;
    estado_t estado;
    motivo_bloqueio_t motivo_bloqueio; // Adicionado campo para motivo de bloqueio
    porta_t *porta;
    bool chamada_sistema; // Adicionada variável para diferenciar chamada de sistema
    double prioridade; // Adicionada variável para prioridade
    struct processo_t *prox_processo; // Adicionada variável para próximo processo na fila
} processo_t;
