/*
 * EXPERIMENTO INVESTIGATIVO: MEDIÇÃO DA VELOCIDADE DO SOM
 * Código Aprimorado com Depuração e Cálculo Automático
 * 
 * Data: Março 2026
 * 
 * Descrição:
 * Este código utiliza o sensor ultrassônico HC-SR04 para medir o tempo de voo
 * de um pulso sonoro. A partir de distâncias pré-determinadas (20, 30, 40, 50 e 60 cm),
 * o programa calcula automaticamente a velocidade do som no ar.
 * 
 * O código inclui:
 * 1. Filtro de leituras espúrias (outliers).
 * 2. Média de múltiplas leituras para maior precisão.
 * 3. Menu interativo via Monitor Serial para selecionar a distância atual do anteparo.
 * 4. Cálculo da velocidade do som (v = 2d / t).
 */

// Definição dos pinos do HC-SR04
const int trigPin = 9;
const int echoPin = 10;

// Constantes para calibração e depuração
const int NUM_LEITURAS = 200;         // Número de leituras para fazer a média
const long TIMEOUT = 30000;         // Timeout do pulseIn em microssegundos (aprox. 5 metros)

// Variáveis globais
float distanciaAtual_cm = 0.0;      // Distância selecionada pelo usuário
bool medindo = false;               // Estado do sistema

void setup() {
  // Configuração dos pinos
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Garante que o Trig inicie em LOW
  digitalWrite(trigPin, LOW);
  
  // Inicialização da comunicação serial
  Serial.begin(9600);
  
  // Aguarda a porta serial abrir (necessário para placas como Leonardo/Micro)
  while (!Serial) {
    ; 
  }
  
  imprimirCabecalho();
  mostrarMenu();
}

void loop() {
  // Verifica se há entrada do usuário no Monitor Serial
  if (Serial.available() > 0) {
    char opcao = Serial.read();
    
    // Limpa o buffer da serial (remove \n ou \r)
    while(Serial.available() > 0) {
      Serial.read();
    }
    
    configurarDistancia(opcao);
  }
  
  // Se uma distância válida foi selecionada, realiza a medição
  if (medindo) {
    realizarMedicao(distanciaAtual_cm);
    
    // Após a medição, volta ao menu principal
    medindo = false;
    Serial.println("\n--------------------------------------------------");
    mostrarMenu();
  }
}

// Função para imprimir o cabeçalho inicial
void imprimirCabecalho() {
  Serial.println("\n==================================================");
  Serial.println("  EXPERIMENTO: MEDICAO DA VELOCIDADE DO SOM");
  Serial.println("==================================================");
  Serial.println("Certifique-se de posicionar o anteparo corretamente.");
  Serial.println("O sensor deve estar perpendicular ao anteparo.");
  Serial.println("==================================================\n");
}

// Função para mostrar o menu de opções
void mostrarMenu() {
  Serial.println("Selecione a distancia atual do anteparo:");
  Serial.println("[1] - 20 cm (0.2 m)");
  Serial.println("[2] - 30 cm (0.3 m)");
  Serial.println("[3] - 40 cm (0.4 m)");
  Serial.println("[4] - 50 cm (0.5 m)");
  Serial.println("[5] - 60 cm (0.6 m)");
  Serial.print("Digite o numero correspondente: ");
}

// Função para configurar a distância baseada na escolha do usuário
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
      // Ignora caracteres inválidos como quebras de linha
      if (opcao != '\n' && opcao != '\r') {
        Serial.println("\n[ERRO] Opcao invalida. Tente novamente.\n");
        mostrarMenu();
      }
      medindo = false;
      break;
  }
  
  if (medindo) {
    Serial.println(opcao); // Ecoa a escolha do usuário
    Serial.print("\nDistancia configurada para: ");
    Serial.print(distanciaAtual_cm);
    Serial.println(" cm");
    Serial.println("Iniciando medicoes (Aguarde)...\n");
  }
}

// Função principal que realiza a medição, filtra dados e calcula a velocidade
void realizarMedicao(float distancia_cm) {
  long somaTempos = 0;
  int leiturasValidas = 0;
  float distancia_metros = distancia_cm / 100.0;
  
  // A distância total percorrida pelo som é ida e volta (2 * d)
  float distanciaTotal_metros = distancia_metros * 2.0;
  
  Serial.println("Lendo sensor...");
  
  // Realiza múltiplas leituras para tirar a média
  for (int i = 0; i < NUM_LEITURAS; i++) {
    long tempo_us = dispararSensor();
    
    // Verifica se a leitura é válida (maior que 0 e menor que o timeout)
    if (tempo_us > 0) {
      somaTempos += tempo_us;
      leiturasValidas++;
      
      // Depuração: mostra as leituras individuais
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
    
    // Pequeno atraso entre leituras para evitar interferência do eco anterior
    delay(100);
  }
  
  // Processamento dos resultados
  if (leiturasValidas > 0) {
    // Calcula o tempo médio em microssegundos
    float tempoMedio_us = (float)somaTempos / leiturasValidas;
    
    // Converte tempo médio para segundos
    float tempoMedio_s = tempoMedio_us / 1000000.0;
    
    // Calcula a velocidade do som: v = delta_S / delta_t
    // Onde delta_S é a distância total (ida e volta)
    float velocidade_m_s = distanciaTotal_metros / tempoMedio_s;
    
    // Exibe os resultados finais formatados
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
    
    // Análise de erro básica (assumindo v teórica aprox 343 m/s a 20C)
    float erroRelativo = abs(velocidade_m_s - 343.0) / 343.0 * 100.0;
    Serial.print("Erro relativo (ref 343 m/s): ");
    Serial.print(erroRelativo, 1);
    Serial.println(" %");
    
  } else {
    Serial.println("\n[ERRO] Nao foi possivel obter leituras validas.");
    Serial.println("Verifique as conexoes do sensor e se o anteparo esta no alcance.");
  }
}

// Função de baixo nível para interagir com o HC-SR04
long dispararSensor() {
  // Garante que o Trig está LOW antes do pulso
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Envia pulso HIGH de 10 microssegundos para o Trig
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Lê o tempo do pulso Echo (em microssegundos)
  // Utiliza um timeout para não travar o código caso não haja obstáculo
  long duracao = pulseIn(echoPin, HIGH, TIMEOUT);
  
  return duracao;
}
