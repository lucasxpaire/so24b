#ifndef PROCESSO_H
#define PROCESSO_H

typedef struct processo_t processo_t;
typedef struct estado_t estado_t;
typedef struct porta_t porta_t;

typedef enum { 
    MORTO = 1,
    PRONTO = 2,
    BLOQUEADO = 3,
    EXECUTANDO = 4
} estado_t;

typedef enum{
    BLOQUEIO_LE = 1,
    BLOQUEIO_ESC = 2,
    BLOQUEIO_ESPERA = 3
} bloqueio_t;

struct porta_t {
    bool porta_ocupada;
    int teclado_estado;
    int teclado;
    int tela_estado;
    int tela;
};

typedef struct {
    int pid;                   
    estado_t estado;
    int PC;                 
    int A;  
    int X;
    int reg_erro;           //registrador de erro
    int reg_complemento;    //registrador complemento
    int modo;               //modo de execução(usuário ou supervisor)  
    porta_t *porta_processo; //porta do processo 
    bloqueio_t bloqueio;     // Motivo do bloqueio
    int pid_esperado;       // PID do processo esperado (para bloqueio de espera)
    struct processo_t *proximo;

} processo_t;

#endif
