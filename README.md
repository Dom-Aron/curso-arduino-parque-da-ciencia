# Curso de Arduino — Parque da Ciência Newton Freire Maia

<p align="center">
  <strong>Formação de professores em atividades experimentais de Física com Arduino</strong>
</p>

<p align="center">
  <a href="README.en.md">English version</a>
</p>

<p align="center">
  <a href="https://github.com/Parque-da-Ciencia/curso-arduino-parque-da-ciencia">
    <img src="https://img.shields.io/badge/GitHub-Parque--da--Ci%C3%AAncia-181717?style=for-the-badge&logo=github" alt="GitHub do Parque da Ciência">
  </a>
  <img src="https://img.shields.io/badge/Arduino-Experimenta%C3%A7%C3%A3o-00979D?style=for-the-badge&logo=arduino&logoColor=white" alt="Arduino">
  <img src="https://img.shields.io/badge/F%C3%ADsica-Ensino%20Experimental-blue?style=for-the-badge" alt="Física Experimental">
</p>

---

## Sobre o projeto

Este repositório reúne os códigos, materiais de apoio e exemplos utilizados no curso de formação de professores do **Parque da Ciência Newton Freire Maia**, com foco no uso da plataforma **Arduino** em atividades experimentais de Física.

A proposta do curso é aproximar professores da Educação Básica de práticas acessíveis de instrumentação, aquisição de dados, automação e visualização de fenômenos físicos, utilizando componentes eletrônicos simples e programação em C/C++ para Arduino.

O material foi organizado para servir tanto como apoio durante a formação quanto como referência posterior para planejamento de aulas, oficinas, demonstrações experimentais e projetos escolares.

---

## Objetivos

- Introduzir o uso do Arduino como ferramenta de ensino experimental.
- Relacionar programação, eletrônica básica e conceitos de Física.
- Desenvolver atividades práticas com sensores, atuadores e displays.
- Estimular a construção de experimentos de baixo custo para a sala de aula.
- Apoiar professores na criação, adaptação e documentação de práticas investigativas.

---

## Estrutura do repositório

O repositório está organizado em duas versões principais dos códigos:

```text
curso-arduino-parque-da-ciencia/
├── ArduinoIDE/          # Códigos preparados para uso direto na Arduino IDE
├── VSCode/              # Códigos preparados para uso no VS Code / PlatformIO ou fluxo similar
├── docs/                # Documentação, materiais auxiliares e arquivos LaTeX
├── tools/               # Scripts utilitários para imagens, documentação e processamento
└── README.md
```

A separação entre **ArduinoIDE** e **VSCode** permite que o mesmo conteúdo seja usado em diferentes ambientes de desenvolvimento, respeitando o perfil e a familiaridade dos participantes.

---

## Aulas e experimentos

| Aula | Pasta | Tema | Conceitos trabalhados |
|---|---|---|---|
| Aula 01 | `aula-01-blinking_led-ArduinoIDE` | LED piscante | Saídas digitais, temporização e estrutura básica de um sketch |
| Aula 02 | `aula-02-potentiometer-ArduinoIDE` | Potenciômetro | Entradas analógicas, leitura ADC e controle de variáveis |
| Aula 03 | `aula-03-ohm_law` | Lei de Ohm | Tensão, corrente, resistência, aquisição de dados e gráficos |
| Aula 04 | `aula-04-thermodynamic_law-ArduinoIDE` | Termodinâmica sem LCD | Sensor LM35, temperatura, leitura analógica e Monitor/Plotter Serial |
| Aula 04 | `aula-04-thermodynamic_law+LCD-ArduinoIDE` | Termodinâmica com LCD | Sensor LM35, temperatura, histerese e visualização em display LCD I2C |
| Aula 05 | `aula-05-hall_effect_latch_sensor-ArduinoIDE` | Sensor Hall latch | Campo magnético, sensor Hall, detecção digital e contagem de eventos |
| Aula 06 | `aula-06-LM35-ArduinoIDE` | Sensor LM35 | Temperatura, calibração simples, histerese e visualização serial |
| Aula 07 | `aula-07-lcd_display-ArduinoIDE` | Display LCD | Interface I2C, textos, caracteres customizados e animações |
| Aula 08 | `aula-08-speed_of_sound-ArduinoIDE` | Velocidade do som | Medidas de tempo, distância, propagação sonora e estimativa experimental |
| Aula 09 | `aula-09-driver-ArduinoIDE` | Driver com transistor e relé | Acionamento de cargas, transistor BC548, relé, diodo de proteção e corrente de base |
| Aula 10 | `aula-10-stopwatch-ArduinoIDE` | Cronômetro | Medidas de tempo, botões, LCD I2C e organização de estados |
| Aula 11 | `aula-11-gravitational_acceleration-ArduinoIDE` | Aceleração gravitacional | Queda livre, sensores, medidas de tempo e estimativa de g |

---

## Componentes utilizados

Ao longo das aulas, são utilizados componentes como:

- Arduino Uno ou compatível;
- LEDs e resistores;
- potenciômetro;
- sensor de temperatura LM35;
- display LCD 16×2 com módulo I2C;
- botão tátil;
- sensor Hall;
- fototransistores infravermelhos;
- relé 5 V;
- transistor BC548;
- diodo 1N4007;
- jumpers e protoboard.

A lista exata de componentes pode variar de acordo com a aula e com a adaptação feita pelo professor.

---

## Como usar

### 1. Clonar o repositório

```bash
git clone https://github.com/Parque-da-Ciencia/curso-arduino-parque-da-ciencia.git
cd curso-arduino-parque-da-ciencia
```

### 2. Escolher o ambiente

Para usar com a **Arduino IDE**, acesse:

```text
ArduinoIDE/
```

Para usar com **VS Code** ou outro fluxo de desenvolvimento, acesse:

```text
VSCode/
```

### 3. Abrir a aula desejada

Cada aula possui sua própria pasta, contendo o código principal e, quando necessário, arquivos auxiliares de documentação, scripts ou coleta de dados.

### 4. Enviar o código para o Arduino

Conecte a placa Arduino ao computador, selecione a placa e a porta correta no ambiente de desenvolvimento e faça o upload do código.

---

## Requisitos de software

Para acompanhar o curso, recomenda-se ter instalado:

| Software | Uso no curso | Download |
|---|---|---|
| Arduino IDE | Abrir, compilar e enviar os códigos para a placa Arduino | [arduino.cc/en/software](https://www.arduino.cc/en/software) |
| VS Code | Ambiente alternativo para edição dos códigos e organização do projeto | [code.visualstudio.com](https://code.visualstudio.com/) |
| Python 3 | Coleta, processamento e visualização de dados em algumas aulas | [python.org/downloads](https://www.python.org/downloads/) |
| Git | Clonar o repositório e acompanhar atualizações do projeto | [git-scm.com/downloads](https://git-scm.com/downloads) |

Também pode ser necessário instalar o driver da placa Arduino utilizada, especialmente em placas compatíveis que usam conversores USB-serial como CH340 ou CP2102.

Algumas aulas utilizam bibliotecas adicionais, como suporte a display LCD I2C. Quando necessário, a biblioteca esperada é indicada no cabeçalho do código.

---

## Organização das branches

Este repositório utiliza duas branches principais:

| Branch | Função |
|---|---|
| `main` | Versão estável e pública do material |
| `develop` | Versão de desenvolvimento e preparação das próximas atualizações |

O fluxo recomendado é desenvolver novas aulas, correções e melhorias em `develop` e enviar para `main` apenas quando o material estiver revisado.

---

## Padrão de documentação

Os códigos do curso seguem um padrão de documentação em português, incluindo:

- cabeçalho institucional;
- descrição do experimento;
- identificação dos componentes esperados;
- comentários explicativos no código;
- documentação de funções em estilo Doxygen quando aplicável.

Esse cuidado busca tornar o material mais acessível para professores, estudantes e colaboradores que desejem estudar, adaptar ou ampliar as atividades.

---

## Sobre o Parque da Ciência Newton Freire Maia

O **Parque da Ciência Newton Freire Maia**, localizado em Pinhais, Paraná, é um espaço dedicado à divulgação científica, à educação e à aproximação entre ciência, tecnologia, cultura e sociedade.

Este repositório faz parte das ações de formação, experimentação e produção de materiais educacionais associados ao uso de tecnologias abertas no ensino de Ciências e Física.

---

## Créditos

| Categoria | Informação |
|---|---|
| Instituição | Parque da Ciência Newton Freire Maia |
| Curso | Formação de Professores — Atividades Experimentais de Física com Arduino |
| Autor dos códigos originais | Aron da Rocha Battistella |
| GitHub do autor | [Dom-Aron](https://github.com/Dom-Aron) |
| Organização GitHub | [Parque-da-Ciencia](https://github.com/Parque-da-Ciencia) |
| Repositório | [curso-arduino-parque-da-ciencia](https://github.com/Parque-da-Ciencia/curso-arduino-parque-da-ciencia) |

---

## Contribuições

Contribuições são bem-vindas, especialmente para:

- correção de erros;
- melhoria dos comentários e da documentação;
- adaptação das atividades para diferentes contextos escolares;
- inclusão de novos experimentos;
- criação de materiais de apoio;
- padronização dos códigos e exemplos.

Antes de contribuir, recomenda-se abrir uma issue ou discutir a proposta com os responsáveis pelo repositório.

---

## Licença

Este repositório utiliza uma política de licenciamento separada para código-fonte e materiais educacionais:

| Tipo de conteúdo | Licença |
|---|---|
| Códigos-fonte, scripts e exemplos de programação | MIT License |
| Textos, roteiros, imagens, documentação e materiais didáticos | Creative Commons Attribution 4.0 International — CC BY 4.0 |

Em resumo: os códigos podem ser reutilizados, modificados e distribuídos com atribuição da autoria; os materiais educacionais podem ser compartilhados e adaptados, desde que seja dado o devido crédito ao Parque da Ciência Newton Freire Maia e aos autores.

Consulte os arquivos de licença do repositório para os termos completos:

- [`LICENSE`](LICENSE) — política geral de licenciamento;
- [`LICENSE-CODE`](LICENSE-CODE) — termos da MIT License para código;
- [`LICENSE-DOCS`](LICENSE-DOCS) — termos e referência da CC BY 4.0 para materiais educacionais.

---

<p align="center">
  <strong>Parque da Ciência Newton Freire Maia</strong><br>
  Ciência, tecnologia e educação pública em ação.
</p>
