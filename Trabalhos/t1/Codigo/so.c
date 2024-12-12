// so.c
// sistema operacional
// simulador de computador
// so24b

// INCLUDES {{{1
#include "so.h"
#include "dispositivos.h"
#include "irq.h"
#include "programa.h"
#include "instrucao.h"
#include "processo.h"

#include <stdlib.h>
#include <stdbool.h>

#define QUANTUM 5


// CONSTANTES E TIPOS {{{1
// intervalo entre interrupções do relógio
#define INTERVALO_INTERRUPCAO 30   
#define MAX_PROCESSOS 10

int PID_GERAL = 0; 

struct so_t {
  cpu_t *cpu;
  mem_t *mem;
  es_t *es;
  console_t *console;
  bool erro_interno;
  
  // t1: tabela de processos, processo corrente, pendências, etc
  processo_t tabela_processos[MAX_PROCESSOS];
  processo_t *processo_corrente;  
  porta_t tabela_portas[MAX_PROCESSOS];
  porta_t *portas_livres; // Lista de portas livres
  processo_t *fila_processos; // Fila de processos para round-robin
  int quantum; // Quantum para round-robin
};

static void muda_estado(processo_t *processo, estado_t estado){
    processo->estado = estado;
}

static void inicializa_portas(so_t *self) {
    for (int i = 0; i < MAX_PROCESSOS; i++) {
        self->tabela_portas[i].ocupada = false;
        self->tabela_portas[i].estadoTeclado = D_TERM_A_TECLADO_OK + (i * 4);
        self->tabela_portas[i].teclado = D_TERM_A_TECLADO + (i * 4);
        self->tabela_portas[i].estadoTela = D_TERM_A_TELA_OK + (i * 4);
        self->tabela_portas[i].tela = D_TERM_A_TELA + (i * 4);
        self->tabela_portas[i].proxima = (i < MAX_PROCESSOS - 1) ? &self->tabela_portas[i + 1] : NULL;
    }
    self->portas_livres = &self->tabela_portas[0];
}

static porta_t* atribuir_porta(so_t *self) {
    if (self->portas_livres == NULL) {
        return NULL; 
    }
    porta_t *porta = self->portas_livres;
    self->portas_livres = porta->proxima;
    porta->ocupada = true;
    porta->proxima = NULL;
    return porta;
}

static void liberar_porta(so_t *self, porta_t *porta) {
    porta->ocupada = false;
    porta->proxima = self->portas_livres;
    self->portas_livres = porta;
}

static void inicializa_processo(so_t *self, processo_t *processo, int ender) {
    processo->pid = PID_GERAL++; 
    processo->reg_PC = ender;
    processo->reg_A = 0;
    processo->reg_X = 0;
    processo->reg_erro = 0;
    processo->reg_complemento = 0;
    processo->modo = usuario;
    processo->porta = atribuir_porta(self);
    processo->prioridade = 0.5;
    processo->prox_processo = NULL; 
    if (processo->porta == NULL) {
        console_printf("SO: Erro ao atribuir porta ao processo");
        self->processo_corrente->reg_A = -1;
        self->erro_interno = true;
    }
    muda_estado(processo, PRONTO);
}

err_t inicializa_tabela_processos(so_t *self){
    for(int i = 0; i < MAX_PROCESSOS; i++)
    {
        self->tabela_processos[i].estado = MORTO;
    }
    return ERR_OK;
}

// função de tratamento de interrupção (entrada no SO)
static int so_trata_interrupcao(void *argC, int reg_A);

// funções auxiliares
// carrega o programa contido no arquivo na memória do processador; retorna end. inicial
static int so_carrega_programa(so_t *self, char *nome_do_executavel);
// copia para str da memória do processador, até copiar um 0 (retorna true) ou tam bytes
static bool copia_str_da_mem(int tam, char str[tam], mem_t *mem, int ender);

// Função para salvar o estado do processador na tabela de processos
static bool salva_estado_processo(so_t *self) {
    bool deu_erro = false;
    if (mem_le(self->mem, IRQ_END_A, &self->processo_corrente->reg_A) != ERR_OK)
    {
      deu_erro = true;
    }
    if (mem_le(self->mem, IRQ_END_X, &self->processo_corrente->reg_X) != ERR_OK)
    {
      deu_erro = true;
    }
    if (mem_le(self->mem, IRQ_END_PC, &self->processo_corrente->reg_PC) != ERR_OK)
    {
      deu_erro = true;
    }
    if (mem_le(self->mem, IRQ_END_erro, &self->processo_corrente->reg_erro) != ERR_OK)
    {
      deu_erro = true;
    }
    if (mem_le(self->mem, IRQ_END_complemento, &self->processo_corrente->reg_complemento) != ERR_OK)
    {
      deu_erro = true;
    }
    if (mem_le(self->mem, IRQ_END_modo, &self->processo_corrente->modo) != ERR_OK)
    {
      deu_erro = true;
    }
    return deu_erro;
}


// Função para recuperar o estado do processador a partir da tabela de processos
static bool recupera_estado_processo(so_t *self) {
    bool deu_erro = false;
    if (mem_escreve(self->mem, IRQ_END_A, self->processo_corrente->reg_A) != ERR_OK)
    {
      deu_erro = true;
    }
    if (mem_escreve(self->mem, IRQ_END_X, self->processo_corrente->reg_X) != ERR_OK)
    {
      deu_erro = true;
    }
    if (mem_escreve(self->mem, IRQ_END_PC, self->processo_corrente->reg_PC) != ERR_OK)
    {
      deu_erro = true;
    }
    if (mem_escreve(self->mem, IRQ_END_erro, self->processo_corrente->reg_erro) != ERR_OK)
    {
      deu_erro = true;
    }
    if (mem_escreve(self->mem, IRQ_END_complemento, self->processo_corrente->reg_complemento) != ERR_OK)
    {
      deu_erro = true;
    }
    if (mem_escreve(self->mem, IRQ_END_modo, self->processo_corrente->modo) != ERR_OK)
    {
      deu_erro = true;
    }
    return deu_erro;
}

// CRIAÇÃO {{{1

so_t *so_cria(cpu_t *cpu, mem_t *mem, es_t *es, console_t *console)
{
  so_t *self = malloc(sizeof(*self));
  if (self == NULL) return NULL;

  self->cpu = cpu;
  self->mem = mem;
  self->es = es;
  self->console = console;
  self->erro_interno = false;
  self->fila_processos = NULL; 
  self->quantum = QUANTUM; 

  inicializa_portas(self);

  if(inicializa_tabela_processos(self) != ERR_OK){
    console_printf("SO: Erro ao inicializar a tabela de processos");
    self->erro_interno = true;
  }

  // quando a CPU executar uma instrução CHAMAC, deve chamar a função
  //   so_trata_interrupcao, com primeiro argumento um ptr para o SO
  cpu_define_chamaC(self->cpu, so_trata_interrupcao, self);

  // coloca o tratador de interrupção na memória
  // quando a CPU aceita uma interrupção, passa para modo supervisor, 
  //   salva seu estado à partir do endereço 0, e desvia para o endereço
  //   IRQ_END_TRATADOR
  // colocamos no endereço IRQ_END_TRATADOR o programa de tratamento
  //   de interrupção (escrito em asm). esse programa deve conter a 
  //   instrução CHAMAC, que vai chamar so_trata_interrupcao (como
  //   foi definido acima)
  int ender = so_carrega_programa(self, "trata_int.maq");
  if (ender != IRQ_END_TRATADOR) {
    console_printf("SO: problema na carga do programa de tratamento de interrupção");
    self->erro_interno = true;
  }

  // programa o relógio para gerar uma interrupção após INTERVALO_INTERRUPCAO
  if (es_escreve(self->es, D_RELOGIO_TIMER, INTERVALO_INTERRUPCAO) != ERR_OK) {
    console_printf("SO: problema na programação do timer");
    self->erro_interno = true;
  }

  return self;
}

void so_destroi(so_t *self)
{
  cpu_define_chamaC(self->cpu, NULL, NULL);
  free(self);
}


// TRATAMENTO DE INTERRUPÇÃO {{{1

// funções auxiliares para o tratamento de interrupção
static void so_salva_estado_da_cpu(so_t *self);
static void so_trata_irq(so_t *self, int irq);
static void so_trata_pendencias(so_t *self);
static void so_escalona(so_t *self);
static int so_despacha(so_t *self);
static void so_pendencia_de_escrita(so_t *self, processo_t *processo);
static void so_pendencia_de_leitura(so_t *self, processo_t *processo);
static void so_pendencia_de_espera(so_t *self, processo_t *processo);
static void so_desbloqueia_processo(so_t *self, processo_t *processo);
static void retira_processo_fila(so_t *self, processo_t *processo);
static void adiciona_processo_fila(so_t *self, processo_t *processo);
static void coloca_processo_fila(so_t *self, processo_t *processo);
static void so_escalona_round_robin(so_t *self);
static void so_escalona_prioridade(so_t *self);

// função a ser chamada pela CPU quando executa a instrução CHAMAC, no tratador de
// interrupção em assembly
// essa é a única forma de entrada no SO depois da inicialização
// na inicialização do SO, a CPU foi programada para chamar esta função para executar
// a instrução CHAMAC
// a instrução CHAMAC só deve ser executada pelo tratador de interrupção
//
// o primeiro argumento é um ponteiro para o SO, o segundo é a identificação
// da interrupção
// o valor retornado por esta função é colocado no registrador A, e pode ser
// testado pelo código que está após o CHAMAC. No tratador de interrupção em
// assembly esse valor é usado para decidir se a CPU deve retornar da interrupção
// (e executar o código de usuário) ou executar PARA e ficar suspensa até receber
// outra interrupção
static int so_trata_interrupcao(void *argC, int reg_A)
{
  so_t *self = argC;
  irq_t irq = reg_A;
  // esse print polui bastante, recomendo tirar quando estiver com mais confiança
  console_printf("SO: recebi IRQ %d (%s)", irq, irq_nome(irq));
  // salva o estado da cpu no descritor do processo que foi interrompido
  so_salva_estado_da_cpu(self);
  // faz o atendimento da interrupção
  so_trata_irq(self, irq);
  // faz o processamento independente da interrupção
  so_trata_pendencias(self);
  // escolhe o próximo processo a executar
  so_escalona(self);
  // recupera o estado do processo escolhido
  return so_despacha(self);
}

static void so_salva_estado_da_cpu(so_t *self)
{
  // t1: salva os registradores que compõem o estado da cpu no descritor do
  // processo corrente. os valores dos registradores foram colocados pela
  // CPU na memória, nos endereços IRQ_END_*
  // se não houver processo corrente, não faz nada
  bool deu_erro;
  if(self->processo_corrente == NULL || self->processo_corrente->estado != EXECUTANDO)
  {
    return;
  }
  else
  {
    deu_erro = salva_estado_processo(self);
  }

  if(deu_erro)
  {
    console_printf("SO: Erro ao salvar o estado da CPU");
    self->erro_interno = true;
  }
}

static void so_trata_pendencias(so_t *self) {
    for (int i = 0; i < MAX_PROCESSOS; i++) {
        processo_t *proc = &self->tabela_processos[i];
        if (proc->estado == BLOQUEADO) {
            switch (proc->motivo_bloqueio) {
                case BLOQUEIO_ES:
                    so_pendencia_de_escrita(self, proc);
                    break;
                case BLOQUEIO_LE:
                    so_pendencia_de_leitura(self, proc);
                    break;
                case BLOQUEIO_ESPERA:
                    so_pendencia_de_espera(self, proc);
                    break;
                default:
                    console_printf("SO: motivo de bloqueio desconhecido");
                    self->erro_interno = true;
                    break;
            }
        }
    }
}

static void so_escalona(so_t *self) {
    // Escolhe o próximo processo a executar, que passa a ser o processo
    // corrente; pode continuar sendo o mesmo de antes ou não
    // Verifica se o processo corrente ainda está pronto para continuar
    if (self->processo_corrente != NULL && self->processo_corrente->estado == PRONTO) {
        return; // O processo corrente continua sendo executado
    }

    // Atualiza a fila de processos prontos
    self->fila_processos = NULL;
    for (int i = 0; i < MAX_PROCESSOS; i++) {
        if (self->tabela_processos[i].estado == PRONTO) {
            coloca_processo_fila(self, &self->tabela_processos[i]);
        }
    }

    // Escolhe o próximo processo a executar com base no escalonador
    if (self->fila_processos != NULL) {
        if (self->quantum <= 0) {
            so_escalona_round_robin(self);
        } else {
            so_escalona_prioridade(self);
        }
    } else {
        self->processo_corrente = NULL;
    }
}

static void so_escalona_round_robin(so_t *self) {
    if (self->fila_processos != NULL) {
        self->processo_corrente = self->fila_processos;
        retira_processo_fila(self, self->processo_corrente);
        self->quantum = QUANTUM;
    } else {
        self->processo_corrente = NULL;
    }
}

static void so_escalona_prioridade(so_t *self) {
    processo_t *processo_maior_prioridade = NULL;
    double maior_prioridade = -1.0;

    for (int i = 0; i < MAX_PROCESSOS; i++) {
        processo_t *proc = &self->tabela_processos[i];
        if (proc->estado == PRONTO && proc->prioridade > maior_prioridade) {
            processo_maior_prioridade = proc;
            maior_prioridade = proc->prioridade;
        }
    }

    if (processo_maior_prioridade != NULL) {
        self->processo_corrente = processo_maior_prioridade;
        retira_processo_fila(self, self->processo_corrente);
    } else {
        self->processo_corrente = NULL;
    }
}

static int so_despacha(so_t *self)
{
  // t1: se houver processo corrente, coloca o estado desse processo onde ele
  // será recuperado pela CPU (em IRQ_END_*) e retorna 0, senão retorna 1
  // o valor retornado será o valor de retorno de CHAMAC
  console_printf("quantum = %d", self->quantum);

  bool deu_erro;
  if(self->erro_interno || self->processo_corrente == NULL)
  {
    console_printf("SO: deu ruim na 1 verificacao do despacha, erro interno %d", self->erro_interno);
    if(self->processo_corrente != NULL)
    {
      console_printf("SO: processo corrente %d estado %d", self->processo_corrente->pid, self->processo_corrente->estado);
    }
    return 1;
  }
  else
  {
    deu_erro = recupera_estado_processo(self);
  }

  if(deu_erro){
    console_printf("SO: Erro ao despachar o processo");
    self->erro_interno = true;
    return 1;
  }
  else
  {
    self->processo_corrente->estado = EXECUTANDO;
    console_printf("SO: processo %d despachado com sucesso", self->processo_corrente->pid);
    return 0;
  }
}

// TRATAMENTO DE UMA IRQ {{{1

// funções auxiliares para tratar cada tipo de interrupção
static void so_trata_irq_reset(so_t *self);
static void so_trata_irq_chamada_sistema(so_t *self);
static void so_trata_irq_err_cpu(so_t *self);
static void so_trata_irq_relogio(so_t *self);
static void so_trata_irq_desconhecida(so_t *self, int irq);

static void so_trata_irq(so_t *self, int irq)
{
  // verifica o tipo de interrupção que está acontecendo, e atende de acordo
  switch (irq) {
    case IRQ_RESET:
      so_trata_irq_reset(self);
      break;
    case IRQ_SISTEMA:
      so_trata_irq_chamada_sistema(self);
      break;
    case IRQ_ERR_CPU:
      so_trata_irq_err_cpu(self);
      break;
    case IRQ_RELOGIO:
      so_trata_irq_relogio(self);
      break;
    default:
      so_trata_irq_desconhecida(self, irq);
  }
}

// interrupção gerada uma única vez, quando a CPU inicializa
static void so_trata_irq_reset(so_t *self)
{
  // t1: deveria criar um processo para o init, e inicializar o estado do
  // processador para esse processo com os registradores zerados, exceto
  // o PC e o modo.
  // como não tem suporte a processos, está carregando os valores dos
  // registradores diretamente para a memória, de onde a CPU vai carregar
  // para os seus registradores quando executar a instrução RETI

  // coloca o programa init na memória
  int ender = so_carrega_programa(self, "init.maq");
  if (ender != 100) {
    console_printf("SO: problema na carga do programa inicial");
    self->erro_interno = true;
    return;
  }

  for(int i = 0; i< MAX_PROCESSOS; i++){
    processo_t *processo = &self->tabela_processos[i];
    if(processo->estado == MORTO)
    {
      inicializa_processo(self, processo, ender);
      return;
    }
  }
}

// interrupção gerada quando a CPU identifica um erro
static void so_trata_irq_err_cpu(so_t *self)
{
  // Ocorreu um erro interno na CPU
  // O erro está codificado em IRQ_END_erro
  // Em geral, causa a morte do processo que causou o erro
  // Ainda não temos processos, causa a parada da CPU
  int err_int;
  // t1: com suporte a processos, deveria pegar o valor do registrador erro
  // no descritor do processo corrente, e reagir de acordo com esse erro
  // (em geral, matando o processo)
  mem_le(self->mem, IRQ_END_erro, &err_int);
  err_t err = err_int;
  console_printf("SO: IRQ não tratada -- erro na CPU: %s", err_nome(err));
  self->erro_interno = true;

  // Trata a morte do processo corrente
  if (self->processo_corrente != NULL) {
      console_printf("SO: Matando o processo PID %d", self->processo_corrente->pid);
      liberar_porta(self, self->processo_corrente->porta);
      muda_estado(self->processo_corrente, MORTO);
  }
}

// interrupção gerada quando o timer expira
static void so_trata_irq_relogio(so_t *self)
{
  // rearma o interruptor do relógio e reinicializa o timer para a próxima interrupção
  err_t e1, e2;
  e1 = es_escreve(self->es, D_RELOGIO_INTERRUPCAO, 0); // desliga o sinalizador de interrupção
  e2 = es_escreve(self->es, D_RELOGIO_TIMER, INTERVALO_INTERRUPCAO);
  if (e1 != ERR_OK || e2 != ERR_OK) {
    console_printf("SO: problema da reinicialização do timer");
    self->erro_interno = true;
  }

  // Decrementa o quantum do processo corrente
  self->quantum--;

  if (self->quantum <= 0) {
    // Preempção: coloca o processo corrente no final da fila
    if (self->processo_corrente != NULL) {
        adiciona_processo_fila(self, self->processo_corrente);
        muda_estado(self->processo_corrente, PRONTO);
    }
    so_escalona_round_robin(self);
    self->quantum = QUANTUM;
  }
}

// foi gerada uma interrupção para a qual o SO não está preparado
static void so_trata_irq_desconhecida(so_t *self, int irq)
{
  console_printf("SO: não sei tratar IRQ %d (%s)", irq, irq_nome(irq));
  self->erro_interno = true;
}

// CHAMADAS DE SISTEMA {{{1

// funções auxiliares para cada chamada de sistema
static void so_chamada_le(so_t *self);
static void so_chamada_escr(so_t *self);
static void so_chamada_cria_proc(so_t *self);
static void so_chamada_mata_proc(so_t *self);
static void so_chamada_espera_proc(so_t *self);

static void so_bloqueia_processo(so_t *self, processo_t *processo, estado_t estado, motivo_bloqueio_t motivo) {
    if (processo->estado != MORTO && processo->estado != BLOQUEADO) {
        processo->estado = estado;
        processo->motivo_bloqueio = motivo;
    }
}

static void so_desbloqueia_processo(so_t *self, processo_t *processo) {
    if (processo->estado == BLOQUEADO) {
        processo->estado = PRONTO;
    }
}

static void so_tentativa_leitura(so_t *self, processo_t *proc) {
    porta_t *porta = proc->porta;

    int estado;
    if (es_le(self->es, porta->estadoTeclado, &estado) != ERR_OK) {
        console_printf("SO: problema no acesso ao estado do teclado");
        self->erro_interno = true;
        return;
    }

    if (estado == 0 && proc->chamada_sistema) {
        // Bloqueia o processo se o teclado não estiver pronto e for uma chamada de sistema
        so_bloqueia_processo(self, proc, BLOQUEADO, BLOQUEIO_LE);
        return;
    }

    int dado;
    if (es_le(self->es, porta->teclado, &dado) != ERR_OK) {
        console_printf("SO: problema no acesso ao teclado");
        self->erro_interno = true;
        return;
    }
    proc->reg_A = dado;
    so_desbloqueia_processo(self, proc);
}

static void so_tentativa_escrita(so_t *self, processo_t *proc) {
    porta_t *porta = proc->porta;

    int estado;
    if (es_le(self->es, porta->estadoTela, &estado) != ERR_OK) {
        console_printf("SO: problema no acesso ao estado da tela");
        self->erro_interno = true;
        return;
    }

    if (estado == 0 && proc->chamada_sistema) {
        // Bloqueia o processo se a tela não estiver pronta e for uma chamada de sistema
        so_bloqueia_processo(self, proc, BLOQUEADO, BLOQUEIO_ES);
        return;
    }

    int dado = proc->reg_X;
    if (es_escreve(self->es, porta->tela, dado) != ERR_OK) {
        console_printf("SO: problema no acesso à tela");
        self->erro_interno = true;
        return;
    }
    proc->reg_A = 0;
    so_desbloqueia_processo(self, proc);
}

static void so_pendencia_de_escrita(so_t *self, processo_t *processo)
{
  so_tentativa_escrita(self, processo);
}

static void so_pendencia_de_leitura(so_t *self, processo_t *processo)
{
  so_tentativa_leitura(self, processo);
}

static void so_pendencia_de_espera(so_t *self, processo_t *processo) {
    for (int i = 0; i < MAX_PROCESSOS; i++) {
        processo_t *processo_esperado = &self->tabela_processos[i];
        if (processo_esperado->pid == processo->reg_X && processo_esperado->estado == MORTO) {
            so_desbloqueia_processo(self, processo);
            console_printf("Desbloquando processo porque o esperado de PID %d morreu.", processo_esperado->pid);
        }
    }
}

// implementação da chamada se sistema SO_LE
// faz a leitura de um dado da entrada corrente do processo, coloca o dado no reg A
static void so_chamada_le(so_t *self) {
    self->processo_corrente->chamada_sistema = true;
    so_tentativa_leitura(self, self->processo_corrente);
    if (self->processo_corrente->estado == PRONTO) {
        mem_escreve(self->mem, IRQ_END_A, self->processo_corrente->reg_A);
    }
    self->processo_corrente->chamada_sistema = false;
}

// implementação da chamada se sistema SO_ESCR
// escreve o valor do reg X na saída corrente do processo
static void so_chamada_escr(so_t *self) {
    self->processo_corrente->chamada_sistema = true;
    so_tentativa_escrita(self, self->processo_corrente);
    if (self->processo_corrente->estado == PRONTO) {
        mem_escreve(self->mem, IRQ_END_A, 0);
    }
    self->processo_corrente->chamada_sistema = false;
}

// implementação da chamada de sistema SO_CRIA_PROC
// cria um processo
static void so_chamada_cria_proc(so_t *self)
{
  // em X está o endereço onde está o nome do arquivo
  int ender_proc = self->processo_corrente->reg_X;
  char nome[100];
  if (copia_str_da_mem(100, nome, self->mem, ender_proc)) 
  {
    int ender_carga = so_carrega_programa(self, nome);
    if (ender_carga > 0) 
    {
      // Procura uma entrada livre na tabela de processos
      for (int i = 0; i < MAX_PROCESSOS; i++) 
      {
        if (self->tabela_processos[i].estado == MORTO) 
        {     
          inicializa_processo(self, &self->tabela_processos[i], ender_carga);
          if(self->processo_corrente->reg_A == -1)
          {
            return;
          }
          self->processo_corrente->reg_A = self->tabela_processos[i].pid;
          return;
        }
      }
    }
  }
  // Se houve erro, escreve -1 no registrador A do processo corrente
  self->processo_corrente->reg_A = -1;
}

// implementação da chamada de sistema SO_MATA_PROC
// mata o processo com pid X (ou o processo corrente se X é 0)
static void so_chamada_mata_proc(so_t *self)
{
  int pid = self->processo_corrente->reg_X;

  if (pid == 0) {
    // Mata o processo corrente
    console_printf("SO: Matando o proprio proceso PID %d.", self->processo_corrente->pid);
    liberar_porta(self, self->processo_corrente->porta);
    muda_estado(self->processo_corrente, MORTO);
    return;
  } else {
    // Procura o processo na tabela de processos
    console_printf("SO: Matando o processo PID %d", pid);
    for (int i = 0; i < MAX_PROCESSOS; i++) {
      if (self->tabela_processos[i].pid == pid) {
        liberar_porta(self, self->tabela_processos[i].porta);
        muda_estado(&self->tabela_processos[i], MORTO);
        return;
      }
    }
  }

  // Se não encontrou o processo, escreve -1 no registrador A do processo corrente
  self->processo_corrente->reg_A = -1;
}

// implementação da chamada se sistema SO_ESPERA_PROC
// espera o fim do processo com pid X
static void so_chamada_espera_proc(so_t *self) {
    int pid_esperado = self->processo_corrente->reg_X;

    if (pid_esperado == 0 || pid_esperado == self->processo_corrente->pid) {
        // Não pode esperar por si mesmo ou por um processo inexistente
        self->processo_corrente->reg_A = -1;
        return;
    }

    for (int i = 0; i < MAX_PROCESSOS; i++) {
        if (self->tabela_processos[i].pid == pid_esperado && self->tabela_processos[i].estado != MORTO) {
            // Bloqueia o processo chamador até que o processo esperado termine
            so_bloqueia_processo(self, self->processo_corrente, BLOQUEADO, BLOQUEIO_ESPERA);
            return;
        }
    }

    // Se o processo esperado não existe, retorna erro
    self->processo_corrente->reg_A = -1;
}

// CARGA DE PROGRAMA {{{1

// carrega o programa na memória
// retorna o endereço de carga ou -1
static int so_carrega_programa(so_t *self, char *nome_do_executavel)
{
  // programa para executar na nossa CPU
  programa_t *prog = prog_cria(nome_do_executavel);
  if (prog == NULL) {
    console_printf("Erro na leitura do programa '%s'\n", nome_do_executavel);
    return -1;
  }

  int end_ini = prog_end_carga(prog);
  int end_fim = end_ini + prog_tamanho(prog);

  for (int end = end_ini; end < end_fim; end++) {
    if (mem_escreve(self->mem, end, prog_dado(prog, end)) != ERR_OK) {
      console_printf("Erro na carga da memória, endereco %d\n", end);
      return -1;
    }
  }

  prog_destroi(prog);
  console_printf("SO: carga de '%s' em %d-%d", nome_do_executavel, end_ini, end_fim);
  return end_ini;
}

// ACESSO À MEMÓRIA DOS PROCESSOS {{{1

// copia uma string da memória do simulador para o vetor str.
// retorna false se erro (string maior que vetor, valor não char na memória,
// erro de acesso à memória)
// T1: deveria verificar se a memória pertence ao processo
static bool copia_str_da_mem(int tam, char str[tam], mem_t *mem, int ender)
{
  for (int indice_str = 0; indice_str < tam; indice_str++) {
    int caractere;
    if (mem_le(mem, ender + indice_str, &caractere) != ERR_OK) {
      return false;
    }
    if (caractere < 0 || caractere > 255) {
      return false;
    }
    str[indice_str] = caractere;
    if (caractere == 0) {
      return true;
    }
  }
  // estourou o tamanho de str
  return false;
}

static void coloca_processo_fila(so_t *self, processo_t *processo) {
    if (self->fila_processos == NULL) {
        self->fila_processos = processo;
        processo->prox_processo = NULL;
    } else {
        processo_t *ultimo_processo = self->fila_processos;
        while (ultimo_processo->prox_processo != NULL) {
            ultimo_processo = ultimo_processo->prox_processo;
        }
        ultimo_processo->prox_processo = processo;
        processo->prox_processo = NULL;
    }
}



static void adiciona_processo_fila(so_t *self, processo_t *processo) {
    if (self->fila_processos == NULL) {
        self->fila_processos = processo;
        processo->prox_processo = NULL;
    } else {
        processo_t *ultimo_processo = self->fila_processos;
        while (ultimo_processo->prox_processo != NULL) {
            ultimo_processo = ultimo_processo->prox_processo;
        }
        ultimo_processo->prox_processo = processo;
        processo->prox_processo = NULL;
    }
}

static void retira_processo_fila(so_t *self, processo_t *processo) {
    if (self->fila_processos == processo) {
        self->fila_processos = processo->prox_processo;
    } else {
        processo_t *processo_anterior = self->fila_processos;
        while (processo_anterior->prox_processo != processo) {
            processo_anterior = processo_anterior->prox_processo;
        }
        processo_anterior->prox_processo = processo->prox_processo;
    }
    processo->prox_processo = NULL;
}

static void so_trata_irq_chamada_sistema(so_t *self) {
    // a identificação da chamada está no registrador A
    int id_chamada = self->processo_corrente->reg_A;

    switch (id_chamada) {
        case SO_LE:
            so_chamada_le(self);
            break;
        case SO_ESCR:
            so_chamada_escr(self);
            break;
        case SO_CRIA_PROC:
            so_chamada_cria_proc(self);
            break;
        case SO_MATA_PROC:
            so_chamada_mata_proc(self);
            break;
        case SO_ESPERA_PROC:
            so_chamada_espera_proc(self);
            break;
        default:
            console_printf("SO: chamada de sistema desconhecida (%d)", id_chamada);
            muda_estado(self->processo_corrente, MORTO);
    }

}

// vim: foldmethod=marker