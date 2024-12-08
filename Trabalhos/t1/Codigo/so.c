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

// CONSTANTES E TIPOS {{{1
// intervalo entre interrupções do relógio
#define INTERVALO_INTERRUPCAO 50   // em instruções executadas

#define MAX_PROCESSOS 4
int pid_inicial = 0;
int processos_criados = 0;

struct so_t {
  cpu_t *cpu;
  mem_t *mem;
  es_t *es;
  console_t *console;
  bool erro_interno;

  // t1: tabela de processos, processo corrente, pendências, etc  
  processo_t tabela_processos[MAX_PROCESSOS];   //tabela de processos
  porta_t tabela_portas[MAX_PROCESSOS];         //tabela de portas
  processo_t *processo_corrente;                //proceso em execucao
};

// função de tratamento de interrupção (entrada no SO)
static int so_trata_interrupcao(void *argC, int reg_A);

// funções auxiliares
// carrega o programa contido no arquivo na memória do processador; retorna end. inicial
static int so_carrega_programa(so_t *self, char *nome_do_executavel);
// copia para str da memória do processador, até copiar um 0 (retorna true) ou tam bytes
static bool copia_str_da_mem(int tam, char str[tam], mem_t *mem, int ender);

// CRIAÇÃO {{{1

// Inicializa a tabela de processos
err_t so_cria_tabela_processos(so_t *self) {
    for (int i = 0; i < MAX_PROCESSOS; i++) {
        self->tabela_processos[i].estado = MORTO;
        self->tabela_processos[i].porta_processo = NULL;
    }
    return ERR_OK;
}

err_t so_cria_tabela_portas(so_t *self) {
    for (int i = 0; i < MAX_PROCESSOS; i++) {
        self->tabela_portas[i].porta_ocupada = false;
    }
    return ERR_OK;
}

static void so_inicializa_processo(so_t *self, processo_t *processo, int ender_carga)
{
    processo->PC = ender_carga;
    processo->modo = usuario; // Modo usuário
    processo->pid = pid_inicial++;
    processo->A = 0;
    processo->X = 0;
    processo->reg_erro = 0;
    processo->reg_complemento = 0;
    processo->estado = PRONTO;
    processo->porta_processo = atribuir_porta(self);
    self->processo_corrente = processo;
}

// Inicializa as portas de E/S e atribui aos processos
static void inicializa_portas(so_t *self)
{
    for (int i = 0; i < MAX_PROCESSOS; i++)
    {
        self->tabela_portas[i].porta_ocupada = false;
        self->tabela_portas[i].teclado_estado = D_TERM_A_TECLADO_OK + i * 2;
        self->tabela_portas[i].teclado = D_TERM_A_TECLADO + i * 2;
        self->tabela_portas[i].tela_estado = D_TERM_A_TELA_OK + i * 2;
        self->tabela_portas[i].tela = D_TERM_A_TELA + i * 2;
    }
}

// Atribui uma porta disponível a um processo
static porta_t *atribuir_porta(so_t *self)
{
    for (int i = 0; i < MAX_PROCESSOS; i++)
    {
        if (!self->tabela_portas[i].porta_ocupada)
        {
            self->tabela_portas[i].porta_ocupada = true;
            return &self->tabela_portas[i];
        }
    }
    return NULL;
}


so_t *so_cria(cpu_t *cpu, mem_t *mem, es_t *es, console_t *console)
{
  so_t *self = malloc(sizeof(*self));
  if (self == NULL) return NULL;

  self->cpu = cpu;
  self->mem = mem;
  self->es = es;
  self->console = console;
  self->erro_interno = false;

  self->processo_corrente = NULL;

  //inicializa tabela processos, se der um erro interno do so ao tentar criar, muda erro_interno = true
  if(so_cria_tabela_processos(self) != ERR_OK){
    self->erro_interno = true;
    console_printf("SO: Erro ao criar a tabela de processos");
  }

  if(so_cria_trabela_portas(self) != ERR_OK){
    self->erro_interno = true;
    console_printf("SO: Erro ao criar a tabela de portas");
  }
  inicializa_portas(self);

  ///////////////////////////////////
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

// função a ser chamada pela CPU quando executa a instrução CHAMAC, no tratador de
//   interrupção em assembly
// essa é a única forma de entrada no SO depois da inicialização
// na inicialização do SO, a CPU foi programada para chamar esta função para executar
//   a instrução CHAMAC
// a instrução CHAMAC só deve ser executada pelo tratador de interrupção
//
// o primeiro argumento é um ponteiro para o SO, o segundo é a identificação
//   da interrupção
// o valor retornado por esta função é colocado no registrador A, e pode ser
//   testado pelo código que está após o CHAMAC. No tratador de interrupção em
//   assembly esse valor é usado para decidir se a CPU deve retornar da interrupção
//   (e executar o código de usuário) ou executar PARA e ficar suspensa até receber
//   outra interrupção
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

// Função auxiliar para ler um registrador e definir erro interno se falhar
static bool le_registrador(mem_t *mem, int endereco, int *registrador, const char *nome_registrador)
{
    if (mem_le(mem, endereco, registrador) != ERR_OK)
    {
        console_printf("Erro ao ler o registrador %s", nome_registrador);
        return false;
    }
    return true;
}

static void so_salva_estado_da_cpu(so_t *self)
{
    // Verifica se há um processo corrente e se ele está em execução
    if (self->processo_corrente == NULL || self->processo_corrente->estado != EXECUTANDO)
    {
        return; // Se não houver processo corrente ou se não estiver em execução, não faz nada
    }

    // Tenta ler os valores dos registradores da CPU e salvar no descritor do processo corrente
    if (!le_registrador(self->mem, IRQ_END_A, &self->processo_corrente->A, "A") ||
        !le_registrador(self->mem, IRQ_END_X, &self->processo_corrente->X, "X") ||
        !le_registrador(self->mem, IRQ_END_PC, &self->processo_corrente->PC, "PC") ||
        !le_registrador(self->mem, IRQ_END_modo, &self->processo_corrente->modo, "modo") ||
        !le_registrador(self->mem, IRQ_END_erro, &self->processo_corrente->reg_erro, "erro") ||
        !le_registrador(self->mem, IRQ_END_complemento, &self->processo_corrente->reg_complemento, "complemento"))
    {
        self->erro_interno = true;
        return;
    }
}

static void so_trata_pendencias(so_t *self)
{
  // t1: realiza ações que não são diretamente ligadas com a interrupção que
  //   está sendo atendida:
  // - E/S pendente
  // - desbloqueio de processos
  // - contabilidades
}

static void so_escalona(so_t *self)
{
  // escolhe o próximo processo a executar, que passa a ser o processo
  //   corrente; pode continuar sendo o mesmo de antes ou não
  // t1: na primeira versão, escolhe um processo caso o processo corrente não possa continuar
  //   executando. depois, implementar escalonador melhor

  // Verifica se o processo corrente ainda está pronto para ser executado
    if (self->processo_corrente != NULL && self->processo_corrente->estado == PRONTO)
    {
        return; // O processo corrente continua sendo executado
    }

    // Procura o primeiro processo na tabela de processos que esteja pronto
    for (int i = 0; i < MAX_PROCESSOS; i++)
    {
        if (self->tabela_processos[i].estado == PRONTO)
        {
            self->processo_corrente = &self->tabela_processos[i];
            console_printf("Processo %d escalonado para execução.", self->processo_corrente->pid);
            return;
        }
    }

    // Se nenhum processo estiver pronto, não há processo corrente
    self->processo_corrente = NULL;
    console_printf("Nenhum processo pronto para execução.");
}

static int so_despacha(so_t *self)
{
  // t1: se houver processo corrente, coloca o estado desse processo onde ele
  //   será recuperado pela CPU (em IRQ_END_*) e retorna 0, senão retorna 1
  // o valor retornado será o valor de retorno de CHAMAC
  // Verifica se há um erro interno
  // Verifica se há um erro interno
    if (self->erro_interno)
    {
        console_printf("Erro interno detectado. Não é possível despachar o processo.");
        return 1; // Indica que não pode despachar o processo devido a um erro interno
    }

    // Verifica se há um processo corrente
    if (self->processo_corrente == NULL)
    {
        console_printf("Nenhum processo corrente para despachar.");
        return 1; // Indica que não há processo corrente para despachar
    }

    // Tenta escrever os valores dos registradores do processo corrente na memória
    if (mem_escreve(self->mem, IRQ_END_A, self->processo_corrente->A) != ERR_OK)
    {
        console_printf("Erro ao escrever o registrador A na memória.");
        self->erro_interno = true; // Define erro interno se a escrita falhar
        return 1; // Indica que houve um erro ao despachar o processo
    }
    if (mem_escreve(self->mem, IRQ_END_X, self->processo_corrente->X) != ERR_OK)
    {
        console_printf("Erro ao escrever o registrador X na memória.");
        self->erro_interno = true; // Define erro interno se a escrita falhar
        return 1; // Indica que houve um erro ao despachar o processo
    }
    if (mem_escreve(self->mem, IRQ_END_PC, self->processo_corrente->PC) != ERR_OK)
    {
        console_printf("Erro ao escrever o registrador PC na memória.");
        self->erro_interno = true; // Define erro interno se a escrita falhar
        return 1; // Indica que houve um erro ao despachar o processo
    }
    if (mem_escreve(self->mem, IRQ_END_modo, self->processo_corrente->modo) != ERR_OK)
    {
        console_printf("Erro ao escrever o registrador modo na memória.");
        self->erro_interno = true; // Define erro interno se a escrita falhar
        return 1; // Indica que houve um erro ao despachar o processo
    }
    if (mem_escreve(self->mem, IRQ_END_erro, self->processo_corrente->reg_erro) != ERR_OK)
    {
        console_printf("Erro ao escrever o registrador erro na memória.");
        self->erro_interno = true; // Define erro interno se a escrita falhar
        return 1; // Indica que houve um erro ao despachar o processo
    }
    if (mem_escreve(self->mem, IRQ_END_complemento, self->processo_corrente->reg_complemento) != ERR_OK)
    {
        console_printf("Erro ao escrever o registrador complemento na memória.");
        self->erro_interno = true; // Define erro interno se a escrita falhar
        return 1; // Indica que houve um erro ao despachar o processo
    }

    self->processo_corrente->estado = EXECUTANDO;
    console_printf("Processo %d despachado com sucesso.", self->processo_corrente->pid);
    return 0; // Indica que o processo foi despachado com sucesso
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
  //   processador para esse processo com os registradores zerados, exceto
  //   o PC e o modo.
  // como não tem suporte a processos, está carregando os valores dos
  //   registradores diretamente para a memória, de onde a CPU vai carregar
  //   para os seus registradores quando executar a instrução RETI

  // coloca o programa init na memória
  int ender = so_carrega_programa(self, "init.maq");
  if (ender != 100) {
    console_printf("SO: problema na carga do programa inicial");
    self->erro_interno = true;
    return;
  }

  for (int i = 0; i < MAX_PROCESSOS; i++) {
    processo_t *processo = &self->tabela_processos[i];
    if (processo->estado == MORTO) {
        so_inicializa_processo(self, processo, ender);
        processos_criados++;
        return;
    }
  }

  // altera o PC para o endereço de carga
  mem_escreve(self->mem, IRQ_END_PC, ender);
  // passa o processador para modo usuário
  mem_escreve(self->mem, IRQ_END_modo, usuario);
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
  //   no descritor do processo corrente, e reagir de acordo com esse erro
  //   (em geral, matando o processo)
  mem_le(self->mem, IRQ_END_erro, &err_int);
  err_t err = err_int;
  console_printf("SO: IRQ não tratada -- erro na CPU: %s", err_nome(err));
  self->erro_interno = true;
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
  // t1: deveria tratar a interrupção
  //   por exemplo, decrementa o quantum do processo corrente, quando se tem
  //   um escalonador com quantum
  console_printf("SO: interrupção do relógio (não tratada)");
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

static void so_bloqueia_proc(so_t *self, processo_t *processo, bloqueio_t motivo, int pid_esperado)
{
    if (processo->estado != BLOQUEADO && processo->estado != MORTO)
    {
        processo->estado = BLOQUEADO;
        processo->bloqueio = motivo;

        if (motivo == BLOQUEIO_ESPERA)
        {
            processo->pid_esperado = pid_esperado;
        }
    }
}

static void so_desbloqueia_proc(so_t *self, processo_t *processo)
{
    if (processo->estado == BLOQUEADO)
    {
        processo->estado = PRONTO;
    }
}

// Função auxiliar para tentar ler um dado da entrada corrente do processo
static void so_tentativa_leitura(so_t *self, processo_t *processo, bool chamada_sistema)
{
    porta_t *porta = processo->porta_processo;

    int estado;
    if (es_le(self->es, porta->teclado_estado, &estado) != ERR_OK)
    {
        console_printf("SO: problema no acesso ao estado do teclado");
        self->erro_interno = true;
        return;
    }

    if (estado == 0 && chamada_sistema == 1)
    {
        // Bloqueia o processo se a entrada não estiver disponível
        so_bloqueia_proc(self, processo, BLOQUEIO_LE, 0);
        return;
    }

    int dado;
    if (es_le(self->es, porta->teclado, &dado) != ERR_OK)
    {
        console_printf("SO: problema no acesso ao teclado");
        self->erro_interno = true;
        return;
    }

    // Escreve no reg A do processo corrente
    processo->A = dado;

    // Desbloqueia o processo após a leitura
    so_desbloqueia_proc(self, processo);
}

// Função auxiliar para tentar escrever um dado na saída corrente do processo
static void so_tentativa_escrita(so_t *self, processo_t *processo, bool chamada_sistema)
{
    porta_t *porta = processo->porta_processo;

    int estado;
    if (es_le(self->es, porta->tela_estado, &estado) != ERR_OK)
    {
        console_printf("SO: problema no acesso ao estado da tela");
        self->erro_interno = true;
        return;
    }

    if (estado == 0 && chamada_sistema)
    {
        // Bloqueia o processo se a saída não estiver disponível
        so_bloqueia_proc(self, processo, BLOQUEIO_ESC, 0);
        return;
    }

    int dado = processo->X;
    if (mem_le(self->mem, porta->tela, dado) != ERR_OK)
    {
        console_printf("SO: problema no acesso ao registrador X");
        self->erro_interno = true;
        return;
    }

    if (es_escreve(self->es, porta->tela, dado) != ERR_OK)
    {
        console_printf("SO: problema no acesso à tela");
        self->erro_interno = true;
        return;
    }

    // Escreve 0 no reg A do processo corrente para indicar sucesso
    processo->A = 0;

    // Desbloqueia o processo após a escrita
    so_desbloqueia_proc(self, processo);
}

static void so_trata_irq_chamada_sistema(so_t *self)
{
  // a identificação da chamada está no registrador A
  // t1: com processos, o reg A tá no descritor do processo corrente
  int id_chamada;
  if (mem_le(self->mem, IRQ_END_A, &id_chamada) != ERR_OK) {
    console_printf("SO: erro no acesso ao id da chamada de sistema");
    self->erro_interno = true;
    return;
  }
  console_printf("SO: chamada de sistema %d", id_chamada);
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
      // t1: deveria matar o processo
      self->erro_interno = true;
  }
}

// implementação da chamada se sistema SO_LE
// faz a leitura de um dado da entrada corrente do processo, coloca o dado no reg A
static void so_chamada_le(so_t *self)
{
    so_tentativa_leitura(self, self->processo_corrente, true);
}

// implementação da chamada se sistema SO_ESCR
// escreve o valor do reg X na saída corrente do processo
static void so_chamada_escr(so_t *self)
{
    so_tentativa_escrita(self, self->processo_corrente, true);
}

// implementação da chamada se sistema SO_CRIA_PROC
// cria um processo
static void so_chamada_cria_proc(so_t *self)
{
  // ainda sem suporte a processos, carrega programa e passa a executar ele
  // quem chamou o sistema não vai mais ser executado, coitado!
  // T1: deveria criar um novo processo

  // em X está o endereço onde está o nome do arquivo
  int ender_proc = self->processo_corrente->X;
  // t1: deveria ler o X do descritor do processo criador
  char nome[100];
  if (copia_str_da_mem(100, nome, self->mem, ender_proc)) {
    int ender_carga = so_carrega_programa(self, nome);
    if (ender_carga > 0) {
        // t1: deveria escrever no PC do descritor do processo criado
        //mem_escreve(self->mem, IRQ_END_PC, ender_carga);
        //return;
      for (int i = 0; i < MAX_PROCESSOS; i++)
      {
        processo_t *processo = &self->tabela_processos[i];
        if (processo->estado == MORTO)
        {
          so_inicializa_processo(self, processo, ender_carga);

          if(processo->porta_processo == NULL){
            console_printf("Erro ao atribuir porta ao processo.");
            self->processo_corrente->A = -1;
            return;
          }

          console_printf("Criando novo processo com PID %d", processo->pid);
          self->processo_corrente->A = processo->pid;
          processos_criados++;
          return;
        }
      }
    } // else?
    else
    {
      console_printf("Nao tem como criar processo, escrevendo -1 no reg A do criador.");
      self->processo_corrente->A = -1;
    }
  }
  else
  {
    console_printf("Erro ao copiar o nome do arquivo da memória.");
    self->processo_corrente->A = -1;
  }
}
// implementação da chamada se sistema SO_MATA_PROC
// mata o processo com pid X (ou o processo corrente se X é 0)
static void so_chamada_mata_proc(so_t *self)
{
    int pid = self->processo_corrente->X;

    if(pid != 0)
    {
      console_printf("Mantando processo de PID %d", pid);
      for (int i = 0; i < MAX_PROCESSOS; i++)
      {
        if (self->tabela_processos[i].pid == pid)
        {
          self->tabela_processos[i].estado = MORTO;
          if (self->tabela_processos[i].porta_processo != NULL)
          {
            self->tabela_processos[i].porta_processo->porta_ocupada = false;
          }
          self->processo_corrente->A = 0;
          return;
        }
        console_printf("Erro: Processo %d não foi achado.", pid);
      }
    }
    else if(pid==0)
    {
        // Matar o próprio processo corrente
        console_printf("Matando o próprio processo de PID %d.", self->processo_corrente->pid);
        self->processo_corrente->estado = MORTO;
        if (self->processo_corrente->porta_processo != NULL)
        {
            self->processo_corrente->porta_processo->porta_ocupada = false;
        }
        self->processo_corrente->A = 0;
        return;
    }

    self->processo_corrente->A = -1;
}

// implementação da chamada se sistema SO_ESPERA_PROC
// espera o fim do processo com pid X
static void so_chamada_espera_proc(so_t *self)
{
  // T1: deveria bloquear o processo se for o caso (e desbloquear na morte do esperado)
  // ainda sem suporte a processos, retorna erro -1
  //console_printf("SO: SO_ESPERA_PROC não implementada");
  //mem_escreve(self->mem, IRQ_END_A, -1);
  int pid_esperado = self->processo_corrente->X;

  // Verifica se o processo esperado existe e está em execução
  bool processo_encontrado = false;
  for (int i = 0; i < MAX_PROCESSOS; i++)
  {
    if (self->tabela_processos[i].pid == pid_esperado && self->tabela_processos[i].estado != MORTO)
    {
     processo_encontrado = true;
    break;
    }
  }

  if (processo_encontrado)    
  {
    // Bloqueia o processo corrente até que o processo esperado termine
    so_bloqueia_proc(self, self->processo_corrente, BLOQUEIO_ESPERA, pid_esperado);
    console_printf("Bloqueando processo %d esperando pelo processo %d", self->processo_corrente->pid, pid_esperado);
  }
  else
  {
    // Se o processo esperado não existe ou já terminou, retorna erro -1
    console_printf("Erro: Processo %d não encontrado ou já terminou.", pid_esperado);
    self->processo_corrente->A = -1;
  }
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
//   erro de acesso à memória)
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

// vim: foldmethod=marker
