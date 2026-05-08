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
 * Caminho no repositório: aulas/VSCode/aula-07-speed_of_sound-VSCode/aula-07-speed_of_sound-VSCode.ino
 * Data da última revisão: 29/04/2026
 *
 * Descrição:
 *   ============================================================
 *   Projeto   : Aula 07 - Speed of Sound
 *   Arquivo   : aula-07-speed_of_sound-VSCode.ino
 *   Pasta     : aula-07-speed_of_sound-VSCode
 *   Este sketch utiliza um sensor ultrassônico HC-SR04 para estimar a
 *   velocidade do som no ar a partir do tempo de voo do pulso e da
 *   distância informada pelo usuário no Monitor Serial.
 *
 *   O programa realiza múltiplas leituras, calcula médias válidas e
 *   apresenta um erro relativo simples em relação a um valor de
 *   referência.
 *   ============================================================
 */
// Definição dos pinos do sensor HC-SR04.
const int trigPin = 9;
const int echoPin = 10;

// Constantes de configuração da rotina de aquisição.
const int NUM_LEITURAS = 200;   // Quantidade de leituras usadas na média final.
const long TIMEOUT = 30000;     // Tempo máximo de espera do eco, em microssegundos.

// Variáveis globais de controle do experimento.
float distanciaAtual_cm = 0.0;  // Distância do anteparo selecionada pelo usuário, em centímetros.
bool medindo = false;           // Indica se uma medição deve ser executada.

/*! 
 * @brief Inicializa o experimento e prepara a interface serial.
 *
 * Configura os pinos do sensor ultrassônico, garante que o pino
 * Trig comece em nível lógico baixo, inicializa a comunicação
 * serial e apresenta ao usuário o cabeçalho do experimento e o
 * menu de seleção de distância.
 */
void setup() {
  // Configura os pinos do sensor.
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Garante que o Trig inicie em nível baixo.
  digitalWrite(trigPin, LOW);

  // Inicializa a comunicação serial.
  Serial.begin(9600);

  // Aguarda a serial ficar disponível em placas que exigem isso.
  while (!Serial) {
    ;
  }

  imprimirCabecalho();
  mostrarMenu();
}

/*! 
 * @brief Controla a interação com o usuário e executa as medições.
 *
 * Verifica continuamente se o usuário enviou uma opção pelo
 * Monitor Serial. Quando uma distância válida é escolhida, o
 * sistema chama a rotina principal de medição e, ao final,
 * retorna ao menu principal para uma nova coleta.
 */
void loop() {
  // Verifica se há entrada do usuário no Monitor Serial.
  if (Serial.available() > 0) {
    char opcao = Serial.read();

    // Limpa o buffer da serial para remover caracteres residuais.
    while (Serial.available() > 0) {
      Serial.read();
    }

    configurarDistancia(opcao);
  }

  // Se uma distância válida foi selecionada, realiza a medição.
  if (medindo) {
    realizarMedicao(distanciaAtual_cm);

    // Após a medição, retorna ao menu principal.
    medindo = false;
    Serial.println("\n--------------------------------------------------");
    mostrarMenu();
  }
}

/*! 
 * @brief Exibe o cabeçalho inicial do experimento no Monitor Serial.
 *
 * Apresenta uma breve introdução ao experimento e orientações para
 * posicionamento do anteparo e do sensor ultrassônico antes do
 * início das medições.
 */
void imprimirCabecalho() {
  Serial.println("\n==================================================");
  Serial.println("  EXPERIMENTO: MEDICAO DA VELOCIDADE DO SOM");
  Serial.println("==================================================");
  Serial.println("Certifique-se de posicionar o anteparo corretamente.");
  Serial.println("O sensor deve estar perpendicular ao anteparo.");
  Serial.println("==================================================\n");
}

/*! 
 * @brief Exibe o menu de seleção de distância no Monitor Serial.
 *
 * Mostra ao usuário as opções disponíveis de distância para o
 * posicionamento do anteparo. A distância escolhida será usada no
 * cálculo da velocidade do som após a coleta das leituras.
 */
void mostrarMenu() {
  Serial.println("Selecione a distancia atual do anteparo:");
  Serial.println("[1] - 20 cm (0.2 m)");
  Serial.println("[2] - 30 cm (0.3 m)");
  Serial.println("[3] - 40 cm (0.4 m)");
  Serial.println("[4] - 50 cm (0.5 m)");
  Serial.println("[5] - 60 cm (0.6 m)");
  Serial.print("Digite o numero correspondente: ");
}

/*! 
 * @brief Define a distância do anteparo a partir da opção escolhida.
 *
 * Interpreta o caractere recebido pelo Monitor Serial, associa a
 * opção a uma distância pré-definida e habilita a execução da
 * medição. Caso a opção seja inválida, o sistema informa o erro e
 * exibe novamente o menu.
 *
 * @param opcao Caractere digitado pelo usuário no Monitor Serial.
 */
void configurarDistancia(char opcao) {
  switch (opcao) {
    case '1':
      distanciaAtual_cm = 20.0;
      medindo = true;
      break;
    case '2':
      distanciaAtual_cm = 30.0;
      medindo = true;
      break;
    case '3':
      distanciaAtual_cm = 40.0;
      medindo = true;
      break;
    case '4':
      distanciaAtual_cm = 50.0;
      medindo = true;
      break;
    case '5':
      distanciaAtual_cm = 60.0;
      medindo = true;
      break;
    default:
      // Ignora caracteres de quebra de linha e trata outras entradas como inválidas.
      if (opcao != '\n' && opcao != '\r') {
        Serial.println("\n[ERRO] Opcao invalida. Tente novamente.\n");
        mostrarMenu();
      }
      medindo = false;
      break;
  }

  if (medindo) {
    Serial.println(opcao);  // Ecoa a escolha do usuário.
    Serial.print("\nDistancia configurada para: ");
    Serial.print(distanciaAtual_cm);
    Serial.println(" cm");
    Serial.println("Iniciando medicoes (Aguarde)...\n");
  }
}

/*! 
 * @brief Executa a rotina principal de medição e calcula a velocidade do som.
 *
 * Realiza múltiplas leituras com o sensor HC-SR04, descarta falhas,
 * calcula o tempo médio de voo do pulso ultrassônico e estima a
 * velocidade do som a partir da distância total percorrida pelo
 * pulso, considerando o trajeto de ida e volta entre sensor e
 * anteparo. Também apresenta no Monitor Serial um resumo dos
 * resultados e um erro relativo simples em relação ao valor de
 * referência de 343 m/s.
 *
 * @param distancia_cm Distância entre o sensor e o anteparo, em centímetros.
 */
void realizarMedicao(float distancia_cm) {
  long somaTempos = 0;
  int leiturasValidas = 0;
  float distancia_metros = distancia_cm / 100.0;

  // A distância total percorrida pelo som corresponde à ida e à volta.
  float distanciaTotal_metros = distancia_metros * 2.0;

  Serial.println("Lendo sensor...");

  // Realiza múltiplas leituras para calcular a média final.
  for (int i = 0; i < NUM_LEITURAS; i++) {
    long tempo_us = dispararSensor();

    // Considera válida apenas a leitura que retornou um tempo positivo.
    if (tempo_us > 0) {
      somaTempos += tempo_us;
      leiturasValidas++;

      // Exibe cada leitura individual para fins de acompanhamento e depuração.
      Serial.print("  Leitura ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(tempo_us);
      Serial.println(" us");
    } else {
      Serial.print("  Leitura ");
      Serial.print(i + 1);
      Serial.println(": FALHA (Timeout/Sem eco)");
    }

    // Pequeno atraso para evitar interferência do eco da leitura anterior.
    delay(100);
  }

  // Processa os resultados apenas se houver leituras válidas.
  if (leiturasValidas > 0) {
    // Calcula o tempo médio em microssegundos.
    float tempoMedio_us = (float)somaTempos / leiturasValidas;

    // Converte o tempo médio para segundos.
    float tempoMedio_s = tempoMedio_us / 1000000.0;

    // Calcula a velocidade do som: v = delta_s / delta_t.
    float velocidade_m_s = distanciaTotal_metros / tempoMedio_s;

    // Exibe os resultados finais formatados.
    Serial.println("\n--- RESULTADOS ---");
    Serial.print("Distancia ate o anteparo : ");
    Serial.print(distancia_metros, 2);
    Serial.println(" m");

    Serial.print("Distancia total percorrida (ida e volta): ");
    Serial.print(distanciaTotal_metros, 2);
    Serial.println(" m");

    Serial.print("Tempo medio de voo       : ");
    Serial.print(tempoMedio_s, 6);
    Serial.println(" s");

    Serial.print("VELOCIDADE DO SOM MEDIDA : ");
    Serial.print(velocidade_m_s, 2);
    Serial.println(" m/s");

    // Calcula um erro relativo simples usando 343 m/s como referência aproximada.
    float erroRelativo = abs(velocidade_m_s - 343.0) / 343.0 * 100.0;
    Serial.print("Erro relativo (ref 343 m/s): ");
    Serial.print(erroRelativo, 1);
    Serial.println(" %");
  } else {
    Serial.println("\n[ERRO] Nao foi possivel obter leituras validas.");
    Serial.println("Verifique as conexoes do sensor e se o anteparo esta no alcance.");
  }
}

/*! 
 * @brief Dispara o sensor ultrassônico e mede o tempo do eco retornado.
 *
 * Gera o pulso de disparo no pino Trig do HC-SR04, aguarda o
 * retorno do pulso no pino Echo e devolve a duração medida em
 * microssegundos. Um timeout é utilizado para evitar que o código
 * fique bloqueado caso não haja eco detectado.
 *
 * @return Duração do pulso de eco, em microssegundos. Retorna 0 em caso de timeout.
 */
long dispararSensor() {
  // Garante que o pino Trig esteja em nível baixo antes do disparo.
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Envia um pulso de 10 microssegundos para iniciar a medição.
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Lê a duração do pulso retornado no pino Echo.
  long duracao = pulseIn(echoPin, HIGH, TIMEOUT);

  return duracao;
}
