# Materiais e ferramentas — Driver de relé com BC548

Este arquivo reúne os materiais de montagem, instrumentos de medição, ferramentas e cuidados recomendados para montar e testar o circuito de acionamento de relé com transistor BC548.

A lista complementa o arquivo [`lista-componentes.md`](./lista-componentes.md), que descreve os componentes eletrônicos do circuito.

## Materiais de montagem

| Material | Finalidade |
|---|---|
| Protoboard | Montar o circuito sem solda |
| Jumpers macho-macho | Fazer ligações na protoboard |
| Jumpers macho-fêmea | Conectar módulos, sensores ou placas externas, quando necessário |
| Fonte de 5 V | Alimentar a bobina do relé e/ou a placa de controle, conforme a montagem |
| Cabo USB | Programar e alimentar a placa Arduino, quando aplicável |
| Etiquetas ou fita adesiva | Identificar fios, terminais do relé e pontos de medição |
| Folha de anotações ou planilha | Registrar valores medidos, hipóteses de cálculo e resultados |

## Ferramentas recomendadas

| Ferramenta | Uso |
|---|---|
| Multímetro digital | Medir resistência da bobina, tensão de alimentação, continuidade e possíveis erros de ligação |
| Alicate de corte | Ajustar tamanho de fios e jumpers |
| Alicate de bico | Posicionar fios e componentes na protoboard |
| Chave de fenda pequena | Apertar bornes, quando forem usados conectores para a carga |
| Computador com Arduino IDE ou VS Code | Carregar o código de teste na placa de controle |
| Datasheets dos componentes | Conferir limites elétricos, pinagem e correntes máximas |

## Arquivos de apoio

| Arquivo | Função |
|---|---|
| `calculo_resistor_base_bc548_rele_5v.tex` | Documento técnico principal, com dedução e exemplo numérico |
| `README.md` | Visão geral da documentação da pasta |
| `lista-componentes.md` | Lista dos componentes eletrônicos usados na montagem |
| `materais.md` | Lista de materiais, ferramentas e cuidados de montagem |
| `img/malha_base.png` | Figura da malha da base |
| `img/malha_coletor.png` | Figura da malha do coletor |
| `img/diagrama_driver_rele.png` | Diagrama geral do circuito driver |
| `img/grafico_bc548_datasheet_curvas_ib_rb10k.png` | Gráfico didático das curvas do BC548 e ponto de operação |

## Preparação antes da montagem

Antes de energizar o circuito, recomenda-se:

1. identificar corretamente os terminais do transistor BC548;
2. identificar os terminais da bobina do relé;
3. identificar os contatos comum, normalmente aberto e normalmente fechado do relé;
4. medir a resistência da bobina com o multímetro;
5. comparar a resistência medida com o valor usado no cálculo;
6. conferir a polaridade do diodo de proteção;
7. conferir se o GND da fonte da bobina está em comum com o GND do circuito de controle, quando forem usadas fontes separadas;
8. verificar se o resistor de base está em série com a base do transistor;
9. verificar se o resistor de *pull-down* está ligado entre base e GND.

## Pontos de medição sugeridos

Durante os testes, os seguintes pontos podem ser medidos com o multímetro:

| Ponto de medição | O que verificar |
|---|---|
| Tensão da fonte | Se a alimentação está próxima de 5 V |
| Tensão no pino digital | Se o sinal de comando alterna entre nível baixo e nível alto |
| Tensão base-emissor | Se a junção base-emissor fica próxima de 0,7 V quando o transistor conduz |
| Tensão na bobina | Se o relé recebe tensão suficiente para acionar |
| Tensão coletor-emissor | Se o transistor entra em saturação quando o relé está acionado |
| Continuidade dos contatos do relé | Se os contatos estão comutando corretamente |

## Cuidados de montagem

- Não ligue a bobina do relé diretamente ao pino digital do microcontrolador.
- Não esqueça o resistor de base.
- Não esqueça o diodo de proteção em paralelo com a bobina.
- Confira a orientação do diodo antes de ligar a alimentação.
- Confira a pinagem real do transistor usado, pois encapsulamentos parecidos podem ter pinagens diferentes.
- Evite alimentar cargas de maior potência diretamente pela protoboard.
- Use fonte externa adequada quando a bobina ou a carga exigir corrente maior que a fornecida pela placa de controle.
- Não use a rede elétrica diretamente em montagens didáticas na protoboard.

## Cuidados didáticos e de segurança

Para atividades com estudantes, recomenda-se que a primeira montagem acione apenas a bobina do relé e, no máximo, uma carga de baixa tensão. A etapa de comutação de cargas externas deve ser feita somente com supervisão e com isolamento adequado.

O objetivo principal desta montagem é compreender:

- a função do transistor como chave;
- a diferença entre corrente de base e corrente de coletor;
- o papel da saturação no acionamento de cargas;
- a necessidade de proteger o transistor contra cargas indutivas;
- a função do resistor de *pull-down* para evitar acionamentos indesejados.

## Checklist rápido

Antes de ligar:

- [ ] O transistor está na orientação correta.
- [ ] O resistor de base está em série com a base.
- [ ] O resistor de *pull-down* está entre base e GND.
- [ ] O relé está ligado no coletor do transistor.
- [ ] O emissor do transistor está ligado ao GND.
- [ ] O diodo está em paralelo com a bobina.
- [ ] O catodo do diodo está no lado positivo da bobina.
- [ ] O anodo do diodo está no lado do coletor.
- [ ] A fonte de alimentação está correta.
- [ ] Os GNDs estão em comum, quando houver mais de uma fonte.
