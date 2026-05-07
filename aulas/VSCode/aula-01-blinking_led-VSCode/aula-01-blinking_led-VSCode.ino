/*
 * Curso de Formação de Professores – Atividades Experimentais de Física
 * Local: Parque da Ciência Newton Freire Maia (PR, Brasil)
 * Plataforma: Arduino
 * Ambiente alvo: VS Code
 * Autores do curso: Aron da Rocha Battistella; Marcos Rocha; Alan Henrique Abreu Dias
 * Colaboradores do curso: Letícia Trzaskos Abbeg; Gabriel Cordeiro Chileider
 * Autoria dos códigos: Aron da Rocha Battistella e Marcos Rocha
 * Colaboração nos códigos: Letícia Trzaskos Abbeg, Gabriel Cordeiro Chileider e Alan Henrique Abreu Dias
 * Repositório: https://github.com/parquedaciencia/curso-arduino-parque-da-ciencia
 * Caminho no repositório: aulas/VSCode/aula-01-blinking_led-VSCode/aula-01-blinking_led-VSCode.ino
 * Data da última revisão: 29/04/2026
 *
 * Descrição:
 *   ============================================================
 *   Projeto   : Aula 01 - Blinking LED
 *   Arquivo   : aula-01-blinking_led-VSCode.ino
 *   Pasta     : aula-01-blinking_led-VSCode
 *   Este sketch realiza o acionamento cíclico de um LED conectado
 *   a uma saída digital do Arduino, alternando entre os estados
 *   ligado e desligado em intervalos fixos.
 *
 *   O objetivo é introduzir o uso de portas digitais, a estrutura
 *   básica de um programa para Arduino e o controle temporal simples
 *   por meio da função delay().
 *   ============================================================
 */
static constexpr uint8_t PINO_LED = 2;
static constexpr unsigned long TEMPO_ACESO_MS = 1000;
static constexpr unsigned long TEMPO_APAGADO_MS = 1000;

/*!
 * @brief Configura os recursos iniciais do Arduino.
 *
 * Esta função é executada uma única vez após a energização ou o reset
 * da placa. Neste programa, ela configura o pino utilizado pelo LED
 * como saída digital para permitir seu acionamento pelo microcontrolador.
 */
void setup()
{
    // Define o pino do LED como saída digital.
    pinMode(PINO_LED, OUTPUT); // Configura o pino 2 como saída para controlar o LED
}

/*!
 * @brief Executa continuamente o ciclo de acionamento do LED.
 *
 * Esta função implementa o comportamento principal do experimento.
 * O LED é ligado por um intervalo fixo e, em seguida, desligado por
 * outro intervalo fixo, produzindo o efeito visual de piscar de forma
 * periódica e contínua.
 */
void loop()
{
    // Liga o LED.
    digitalWrite(PINO_LED, HIGH); // Define o pino como HIGH (5V) para acender o LED
    delay(TEMPO_ACESO_MS); // Aguarda 1000 ms (1 segundo) com o LED aceso

    // Desliga o LED.
    digitalWrite(PINO_LED, LOW); // Define o pino como LOW (0V) para apagar o LED
    delay(TEMPO_APAGADO_MS); // Aguarda 1000 ms (1 segundo) com o LED apagado
}
