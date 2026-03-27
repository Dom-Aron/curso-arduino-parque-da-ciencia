// ----------------------------------------------
// Parque da Ciência Newton Freire Maia
// Curso de Formação de Professores
// Atividades Experimentais de Física
// Utilizando a Plataforma Arduino
// ----------------------------------------------
// Projeto: Piscar LED
// Descrição:
// Acende e apaga um LED conectado ao pino digital 2
// em intervalos de 1 segundo.
// ----------------------------------------------

static constexpr uint8_t PINO_LED = 2;
static constexpr unsigned long TEMPO_ACESO_MS = 1000;
static constexpr unsigned long TEMPO_APAGADO_MS = 1000;

/*
    Função de configuração inicial do Arduino.

    Esta função é executada apenas uma vez, logo após
    a placa ser energizada ou reiniciada.
*/
void setup()
{
    // Define o pino do LED como saída digital.
    pinMode(PINO_LED, OUTPUT);
}

/*
    Função principal do Arduino.

    Esta função é executada continuamente em loop.
    Nela, o LED é ligado e desligado em intervalos fixos.
*/
void loop()
{
    // Liga o LED.
    digitalWrite(PINO_LED, HIGH);
    delay(TEMPO_ACESO_MS);

    // Desliga o LED.
    digitalWrite(PINO_LED, LOW);
    delay(TEMPO_APAGADO_MS);
}