# Lista de componentes — Driver de relé com BC548

Esta lista reúne os componentes eletrônicos usados na montagem do circuito de acionamento de um relé de 5 V com transistor NPN BC548.

A montagem tem finalidade didática: mostrar como um sinal lógico de baixa potência pode controlar uma carga que exige corrente maior, usando um transistor como chave eletrônica.

## Componentes principais

| Quantidade | Componente | Especificação sugerida | Função no circuito |
|---:|---|---|---|
| 1 | Microcontrolador ou placa de controle | Arduino Uno, Arduino Nano ou equivalente de 5 V | Fornecer o sinal digital de comando para a base do transistor |
| 1 | Transistor NPN | BC548, preferencialmente BC548B ou equivalente compatível | Atuar como chave eletrônica para energizar a bobina do relé |
| 1 | Relé eletromecânico | Relé 5 V, família SRD-05VDC-SL-C ou equivalente | Comutar a carga externa por meio dos contatos do relé |
| 1 | Diodo de proteção | 1N4007, 1N4004, 1N4148 ou diodo equivalente adequado à corrente da bobina | Proteger o transistor contra a tensão induzida no desligamento da bobina |
| 1 | Resistor de base, $R_B$ | Valor calculado conforme a margem de projeto; exemplo: entre 6,0 kΩ e 12 kΩ | Limitar a corrente de base do transistor |
| 1 | Resistor de *pull-down*, $R_{PD}$ | Dezenas de kΩ; exemplo: 47 kΩ, 68 kΩ ou 100 kΩ | Manter a base em nível baixo quando o pino de controle estiver flutuante |
| 1 | Fonte de alimentação | 5 V com corrente suficiente para a bobina do relé | Alimentar a bobina do relé |
| 1 | Protoboard | Protoboard comum | Montagem temporária do circuito |
| Diversos | Jumpers | Macho-macho ou macho-fêmea, conforme a montagem | Interligações elétricas |

## Componentes opcionais

| Quantidade | Componente | Especificação sugerida | Função |
|---:|---|---|---|
| 1 | LED indicador | LED comum de 3 mm ou 5 mm | Indicar visualmente o estado de acionamento |
| 1 | Resistor para LED | 220 Ω a 1 kΩ | Limitar a corrente no LED indicador |
| 1 | Bornier ou conector | 2 ou 3 vias | Facilitar a conexão da carga externa |
| 1 | Módulo de relé pronto | Módulo de relé 5 V | Alternativa pronta para comparação com o circuito discreto |

## Valores calculados no documento

Para o exemplo analisado no arquivo LaTeX, foram adotados:

| Grandeza | Valor |
|---|---:|
| Tensão de comando, $V_{BB}$ | 5 V |
| Tensão da bobina, $V_{\text{relé}}$ | 5 V |
| Queda base-emissor, $V_{BE}$ | aproximadamente 0,7 V |
| Resistência da bobina, $R_L$ | aproximadamente 70 Ω |
| Corrente da bobina, $I_L$ | aproximadamente 71,4 mA |

A partir desses valores:

| Hipótese | Resultado para $R_B$ | Interpretação |
|---|---:|---|
| $\beta = 200$ | aproximadamente 12,0 kΩ | Estimativa inicial usando o ganho típico adotado |
| $\beta = 100$ | aproximadamente 6,0 kΩ | Cálculo com margem de segurança para favorecer saturação |

## Observações importantes sobre a escolha do resistor de base

O valor de $R_B$ deve ser escolhido considerando simultaneamente:

1. a corrente exigida pela bobina do relé;
2. a corrente máxima segura que o pino do microcontrolador pode fornecer;
3. a necessidade de levar o transistor à saturação;
4. a variação real do ganho $\beta$ entre transistores diferentes;
5. o aquecimento, as tolerâncias dos componentes e as variações da fonte.

No documento LaTeX, a dedução mostra que:

$$
R_B = \frac{\beta\,(V_{BB} - V_{BE})}{I_L}
$$

Como o transistor é usado como chave, a escolha prática deve privilegiar uma margem segura de saturação, e não apenas o ganho nominal do datasheet.

## Observações sobre o relé

Antes da montagem, confira os dados reais do relé usado:

- tensão nominal da bobina;
- resistência elétrica da bobina;
- corrente nominal da bobina;
- corrente máxima suportada pelos contatos;
- tensão máxima suportada pelos contatos;
- identificação dos terminais da bobina, comum, normalmente aberto e normalmente fechado.

Relés visualmente parecidos podem ter bobinas com correntes diferentes. Por isso, a resistência da bobina deve ser medida ou retirada do datasheet sempre que possível.

## Observações sobre o diodo

O diodo deve ser ligado em paralelo com a bobina do relé, em polarização reversa durante o funcionamento normal.

Na prática, isso significa:

- catodo do diodo no lado positivo da bobina;
- anodo do diodo no lado ligado ao coletor do transistor.

Se o diodo for invertido, ele pode provocar um curto-circuito durante o acionamento do relé.

## Observações sobre o pull-down

O resistor de *pull-down* deve ser ligado entre a base do transistor e o GND. Ele evita acionamentos indesejados quando o pino digital ainda não foi configurado como saída ou quando está em estado de alta impedância.

Como regra prática, seu valor deve ser bem maior que $R_B$, para não roubar corrente significativa da base.
