# Aula 04 — Cronômetro com sensores (desafios e soluções)

Este projeto implementa um cronômetro para experimentos (ex.: queda de uma bolinha) usando um display LCD 16x2 via I2C, um botão único com múltiplos comandos e sensores posicionados ao longo do percurso.

O objetivo principal foi garantir **boa precisão** e **não perder eventos de sensores**, mesmo quando a bolinha passa muito rapidamente.

---

## Objetivos do cronômetro

- Medir o tempo com base em `millis()` (sem “travar” o programa).
- Registrar marcações (voltas) tanto por **botão** quanto por **sensores**.
- Exibir resultados no LCD 16x2, incluindo:
  - tempo acumulado até cada marca
  - tempo da volta (delta entre marcas)
- Permitir controle com **um único botão**, usando:
  - 1 clique: marcar volta
  - 2 cliques: iniciar/pausar/retomar
  - 3 cliques: finalizar e exibir as marcas
  - clique longo: reset

---

## Desafios enfrentados

### 1) “Bounce” do botão (leituras falsas)
**Problema:** Botões mecânicos não mudam de estado de forma “limpa”. Ao pressionar, o contato pode oscilar rapidamente (HIGH/LOW) por alguns milissegundos, gerando múltiplos acionamentos indevidos.

**Impacto no projeto:**
- Um único clique podia ser interpretado como vários cliques.
- Isso quebrava a lógica de 1, 2 e 3 cliques (multi-clique).

---

### 2) Sensores com pulsos muito curtos (bolinha passa rápido)
**Problema:** Em experimentos, o sensor pode ficar em LOW por um intervalo muito pequeno. Mesmo que o tempo entre sensores seja ~0,1 s, o **pulso do sensor** (tempo em que ele fica acionado) pode ser de poucos milissegundos.

**Impacto no projeto:**
- Usar debounce “clássico” (esperar estabilidade por X ms) pode falhar, porque o sinal não fica “estável por tempo suficiente”.
- O evento pode ser perdido caso o `loop()` esteja ocupado no momento do pulso.

---

### 3) Atualização do LCD (I2C) travando o loop
**Problema:** Atualizar o LCD via I2C pode consumir alguns milissegundos e bloquear momentaneamente o programa.

**Impacto no projeto:**
- Durante a escrita no LCD, o `loop()` pode deixar de ler os sensores.
- Pulsos curtos podem não ser capturados.

---

### 4) Limitação do display 16x2
**Problema:** O LCD 16x2 tem pouco espaço para mostrar informações completas.

**Impacto no projeto:**
- Era necessário exibir “tempo acumulado” e “tempo da volta (delta)”, mas não cabia tudo em uma única tela de forma legível.

---

## Soluções implementadas

### 1) Debounce por software no botão (com eventos de borda)
**Solução:** Implementamos uma leitura debounced do botão, aceitando mudanças apenas quando o sinal permanece estável por alguns milissegundos.

Além disso, o código trabalha com eventos:
- `fell()` = quando o botão foi pressionado (HIGH → LOW)
- `rose()` = quando o botão foi solto (LOW → HIGH)

**Benefícios:**
- Elimina cliques fantasmas.
- Permite multi-clique (1/2/3) de forma confiável.

---

### 2) Sensores com “edge detection + rearm” (sem debounce por estabilidade)
**Solução:** Para sensores rápidos, evitamos o debounce clássico e usamos:

1. **Detecção de borda de descida (HIGH → LOW)** na leitura crua
2. Um tempo de “rearm” (ex.: 25 ms), que impede múltiplos disparos por ruído

**Benefícios:**
- Captura pulsos muito curtos (basta o loop ver o LOW uma vez).
- Evita marcações duplicadas por ruído ou flutuações rápidas.

---

### 3) Otimização do LCD: atualização em taxa fixa + cache
**Solução:** Reduzimos o custo de escrita no LCD com duas estratégias:

1. **Atualização em taxa fixa** (ex.: ~8 Hz), em vez de escrever o tempo a cada iteração do `loop()`.
2. **Cache por linha**: o LCD só é atualizado se o texto da linha realmente mudou.

**Benefícios:**
- Menos tempo travado no I2C.
- `loop()` mais leve e rápido.
- Menor chance de perder pulsos dos sensores.

---

### 4) Exibição em duas telas no modo final (ACUM/DELT)
**Solução:** Após finalizar (3 cliques), o cronômetro alterna automaticamente as telas para cada volta:

- **ACUM**: tempo acumulado até a volta
- **DELT**: tempo somente daquela volta (diferença para a anterior)

Exemplo:
- `V03/08 ACUM` → `00:12:345`
- `V03/08 DELT` → `00:04:210`

**Benefícios:**
- Cabe no LCD 16x2 sem poluição visual.
- Mantém a leitura clara e didática.

---

## Observações e próximos aprimoramentos (opcional)

- Se ainda houver perda de pulsos em condições extremas, existem caminhos adicionais:
  - usar interrupções (quando suportado pela placa/pinos)
  - “esticar” o pulso via hardware (RC + Schmitt trigger / monoestável)
- Outra melhoria possível é registrar a **origem da marca** (botão, S1, S2, S3) para exibir no popup final.

---

## Arquivo principal

O código está em:

`curso-arduino-parque-da-ciencia/aulas/aula-04-cronometro/cronometro.ino`