# Cálculo do resistor de base para acionamento de relé com BC548

Este diretório reúne a documentação técnica, em LaTeX e Markdown, do circuito de *driver* para acionamento de um relé de 5 V usando um transistor NPN BC548 como chave eletrônica.

O material foi escrito para apoiar uma montagem didática de eletrônica aplicada ao Arduino, destacando a diferença entre o cálculo ideal baseado no ganho nominal do transistor e o dimensionamento prático necessário para favorecer a saturação.

## Arquivo principal

- [`calculo_resistor_base_bc548_rele_5v.tex`](./calculo_resistor_base_bc548_rele_5v.tex): documento LaTeX principal, com dedução simbólica, exemplo numérico, discussão física e referências.

Quando compilado, o arquivo gera uma nota técnica sobre o cálculo do resistor de base de um transistor BC548 para acionar a bobina de um relé comum de 5 V.

## Objetivos da documentação

Esta documentação tem como objetivos:

1. explicar por que um pino digital de controle, como o de um Arduino, não deve acionar diretamente a bobina de um relé;
2. apresentar o transistor BC548 como interface de potência entre o circuito lógico e a bobina do relé;
3. deduzir a expressão geral para o resistor de base $R_B$;
4. aplicar a dedução a um relé de 5 V com bobina de aproximadamente $70\,\Omega$;
5. comparar o cálculo usando ganho típico com o cálculo usando margem de segurança;
6. explicar a função do diodo de proteção em paralelo com a bobina;
7. justificar o uso de um resistor de *pull-down* na base do transistor.

## Estrutura sugerida da pasta

```text
.
├── README.md
├── lista-componentes.md
├── materais.md
├── calculo_resistor_base_bc548_rele_5v.tex
└── figuras/
    ├── malha_base.png
    ├── malha_coletor.png
    ├── diagrama_driver_rele.png
    └── grafico_bc548_datasheet_curvas_ib_rb10k.png
```

> Observação: se o repositório usar o nome `materiais.md` em vez de `materais.md`, mantenha o nome já padronizado no projeto.

## Ideia física do circuito

A bobina do relé exige uma corrente maior do que aquela que um pino digital deve fornecer diretamente. Por isso, o pino de controle aciona apenas a base do transistor, enquanto a corrente principal da bobina circula pela malha do coletor.

Na montagem analisada:

- o pino digital fornece a tensão de comando $V_{BB}$;
- o resistor $R_B$ limita a corrente de base $I_B$;
- o transistor BC548 conduz a corrente da bobina quando entra em saturação;
- o relé é alimentado por uma fonte de 5 V;
- o diodo 1N4007 protege o transistor contra a sobretensão gerada no desligamento da bobina;
- o resistor de *pull-down* evita que a base fique flutuante.

## Equações principais

A corrente exigida pela bobina do relé pode ser estimada por:

$$
I_L = \frac{V_{\text{relé}}}{R_L}
$$

Como a corrente de coletor deve alimentar a bobina:

$$
I_C = I_L
$$

Usando o modelo simplificado do transistor bipolar:

$$
I_C = \beta I_B
$$

logo:

$$
I_B = \frac{I_L}{\beta}
$$

Na malha da base:

$$
V_{BB} - V_{BE} = R_B I_B
$$

Portanto, a expressão geral para o resistor de base é:

$$
R_B = \frac{\beta\,(V_{BB} - V_{BE})}{I_L}
$$

ou, substituindo a corrente da bobina:

$$
R_B = \beta\,(V_{BB} - V_{BE})\,\frac{R_L}{V_{\text{relé}}}
$$

## Exemplo numérico documentado

Para a montagem didática considerada:

| Grandeza | Valor adotado |
|---|---:|
| Tensão de comando, $V_{BB}$ | 5 V |
| Tensão da bobina, $V_{\text{relé}}$ | 5 V |
| Queda base-emissor, $V_{BE}$ | aproximadamente 0,7 V |
| Resistência da bobina, $R_L$ | aproximadamente 70 Ω |
| Corrente da bobina, $I_L$ | aproximadamente 71,4 mA |

Resultados principais:

| Hipótese de cálculo | Corrente de base | Resistência de base calculada |
|---|---:|---:|
| $\beta = 200$ | aproximadamente 0,357 mA | aproximadamente 12,0 kΩ |
| $\beta = 100$ | aproximadamente 0,714 mA | aproximadamente 6,0 kΩ |

O cálculo com $\beta = 200$ representa uma primeira estimativa usando o ganho típico adotado. O cálculo com $\beta = 100$ introduz uma margem de segurança equivalente a considerar 50% menos ganho, aumentando a corrente de base e favorecendo a saturação do transistor.

## Observação sobre saturação

Em chaveamento de relé, o transistor não deve ser pensado como um amplificador linear, mas como uma chave eletrônica. A escolha de $R_B$ deve garantir corrente de base suficiente para manter o transistor saturado mesmo com variações reais de ganho, temperatura, tensão de alimentação e tolerâncias dos componentes.

Por isso, o valor de $R_B$ não deve ser escolhido apenas pelo ganho nominal de catálogo. O documento LaTeX discute essa diferença e mostra por que uma margem de projeto é importante.

## Diodo de proteção

A bobina do relé é uma carga indutiva. Quando o transistor desliga, a corrente da bobina não cai instantaneamente, e a tensão induzida pode atingir valores capazes de danificar o transistor.

O diodo 1N4007 é ligado em paralelo com a bobina, em polarização reversa durante o funcionamento normal. No desligamento, ele entra em condução e cria um caminho de recirculação para a corrente da bobina, limitando a tensão sobre o transistor.

## Resistor de pull-down

O resistor de *pull-down* é ligado entre a base do transistor e o GND. Ele garante que a base permaneça em nível lógico baixo quando o pino de controle estiver em alta impedância, desconectado ou durante a inicialização do microcontrolador.

Esse resistor deve ter valor bem maior que o resistor de base para não desviar corrente significativa do acionamento da base.

## Arquivos auxiliares

- [`lista-componentes.md`](./lista-componentes.md): lista objetiva dos componentes eletrônicos usados na montagem.
- [`materais.md`](./materais.md): materiais, ferramentas, instrumentos e cuidados necessários para montar, testar e documentar o circuito.

## Referências usadas no documento LaTeX

- BOYLESTAD, Robert L.; NASHELSKY, Louis. *Dispositivos eletrônicos e teoria de circuitos*. 11. ed. São Paulo: Pearson, 2013.
- ON Semiconductor. *BC546/547/548/549/550 — 30 V, 100 mA NPN general-purpose transistors*. Datasheet.
- SONGLE Relay. *SRD series relay datasheet*. Dados típicos de bobina para a família SRD-05VDC-SL-C.
