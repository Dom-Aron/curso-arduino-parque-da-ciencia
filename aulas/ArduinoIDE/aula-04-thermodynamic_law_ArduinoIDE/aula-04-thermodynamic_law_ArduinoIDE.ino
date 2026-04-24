/*
 * Curso de Formação de Professores – Atividades Experimentais de Física
 * Local: Parque da Ciência Newton Freire Maia (PR, Brasil)
 * Plataforma: Arduino
 * Ambiente alvo: Arduino IDE
 * Autor dos códigos originais: Aron da Rocha Battistella
 * Repositório: https://github.com/Dom-Aron/curso-arduino-parque-da-ciencia
 * Data da última revisão: 22/04/2026
 *
 * Descrição:
 *   ============================================================
 *   Projeto   : Aula 04 - Lei Zero da Termodinâmica
 *   Arquivo   : aula-04-zero_law_two_lm35_ArduinoIDE_isolated.ino
 *
 *   Este sketch lê dois sensores LM35 ligados aos pinos A0 e A1 para
 *   acompanhar a evolução térmica de dois recipientes até o equilíbrio.
 *
 *   Para reduzir a influência entre canais analógicos, a aquisição foi
 *   reforçada com:
 *     1) maior acomodação após a troca de canal;
 *     2) leituras de descarte adicionais;
 *     3) leitura bidirecional do par de sensores (A0->A1 e A1->A0),
 *        seguida da média dos resultados.
 *
 *   Antes do experimento, o código executa uma calibração inicial:
 *     1) ambos os sensores ficam no mesmo meio térmico;
 *     2) o sensor de A1 é alinhado ao sensor de A0 por um offset médio;
 *     3) o sketch informa quando a calibração terminou e reporta o erro
 *        residual da correção antes de iniciar as medidas experimentais.
 *
 *   O Serial Plotter recebe, para cada sensor:
 *     - a média robusta das amostras do instante atual;
 *     - o valor pós-filtro.
 *   ============================================================
 */

#include <math.h>

constexpr uint8_t LM35_1_PIN = A0;
constexpr uint8_t LM35_2_PIN = A1;

constexpr unsigned long SERIAL_BAUD_RATE = 115200UL;
constexpr unsigned long UPDATE_INTERVAL_MS = 450UL;

constexpr unsigned long CALIBRATION_DURATION_MS = 8000UL;
constexpr unsigned long CALIBRATION_SAMPLE_INTERVAL_MS = 250UL;
constexpr uint8_t CALIBRATION_VALIDATION_SAMPLES = 8;

constexpr bool USE_INTERNAL_REFERENCE = true;
constexpr float ADC_REFERENCE_VOLTAGE = USE_INTERNAL_REFERENCE ? 1.1F : 5.0F;
constexpr float ADC_RESOLUTION = 1023.0F;

constexpr uint8_t SAMPLE_COUNT = 25;
constexpr uint8_t DISCARD_READS_AFTER_CHANNEL_SWITCH = 6;
constexpr unsigned int CHANNEL_SETTLING_DELAY_US = 300;
constexpr unsigned int SAMPLE_DELAY_US = 140;
constexpr float OUTLIER_WINDOW_C = 0.55F;

struct SensorState {
    uint8_t pin;
    float filtered_temperature_c;
    bool is_filter_initialized;
};

SensorState sensor_1 = {LM35_1_PIN, 0.0F, false};
SensorState sensor_2 = {LM35_2_PIN, 0.0F, false};

float sensor_2_offset_c = 0.0F;
float calibration_error_c = 0.0F;
float calibration_residual_bias_c = 0.0F;

unsigned long last_update_ms = 0UL;

/**
 * @brief Ordena um vetor de amostras em ordem crescente.
 *
 * @param values Vetor contendo amostras do ADC.
 * @param size Quantidade de elementos do vetor.
 */
void sort_samples(uint16_t* values, const uint8_t size)
{
    for (uint8_t i = 0; i < size - 1; ++i) {
        for (uint8_t j = i + 1; j < size; ++j) {
            if (values[j] < values[i]) {
                const uint16_t temp = values[i];
                values[i] = values[j];
                values[j] = temp;
            }
        }
    }
}

/**
 * @brief Converte uma leitura do ADC em temperatura do LM35.
 *
 * @param adc_value Leitura do ADC.
 * @return Temperatura correspondente em graus Celsius.
 */
float adc_to_temperature_c(const float adc_value)
{
    const float voltage = adc_value * (ADC_REFERENCE_VOLTAGE / ADC_RESOLUTION);
    return voltage * 100.0F;
}

/**
 * @brief Converte uma janela em graus Celsius para contagens do ADC.
 *
 * @param temperature_window_c Janela em graus Celsius.
 * @return Janela equivalente em contagens do ADC.
 */
uint16_t temperature_window_c_to_adc_counts(const float temperature_window_c)
{
    const float counts_per_celsius = ADC_RESOLUTION / (ADC_REFERENCE_VOLTAGE * 100.0F);
    return static_cast<uint16_t>(roundf(temperature_window_c * counts_per_celsius));
}

/**
 * @brief Coleta amostras em um único canal após acomodação extra.
 *
 * O objetivo é reduzir a memória do canal anterior no capacitor de sample-
 * and-hold do ADC. Para isso, o canal é lido várias vezes antes de guardar
 * as amostras válidas.
 *
 * @param pin Pino analógico a ser lido.
 * @param samples Vetor onde as amostras serão armazenadas.
 */
void collect_samples_from_single_channel(const uint8_t pin, uint16_t* samples)
{
    analogRead(pin);
    delayMicroseconds(CHANNEL_SETTLING_DELAY_US);

    for (uint8_t i = 0; i < DISCARD_READS_AFTER_CHANNEL_SWITCH; ++i) {
        analogRead(pin);
        delayMicroseconds(CHANNEL_SETTLING_DELAY_US);
    }

    for (uint8_t i = 0; i < SAMPLE_COUNT; ++i) {
        samples[i] = analogRead(pin);
        delayMicroseconds(SAMPLE_DELAY_US);
    }
}

/**
 * @brief Calcula uma média robusta com base na mediana.
 *
 * O vetor é ordenado, a mediana é obtida e apenas as amostras próximas da
 * mediana entram na média. Isso reduz a influência de picos espúrios sem
 * dar muito peso a leituras contaminadas por ruído.
 *
 * @param pin Pino analógico onde o LM35 está conectado.
 * @return Temperatura robusta em graus Celsius.
 */
float read_robust_mean_temperature_c(const uint8_t pin)
{
    uint16_t samples[SAMPLE_COUNT];
    collect_samples_from_single_channel(pin, samples);
    sort_samples(samples, SAMPLE_COUNT);

    const uint16_t median_adc = samples[SAMPLE_COUNT / 2];
    const uint16_t allowed_deviation_adc = temperature_window_c_to_adc_counts(OUTLIER_WINDOW_C);

    uint32_t sum = 0UL;
    uint8_t valid_count = 0;

    for (uint8_t i = 0; i < SAMPLE_COUNT; ++i) {
        const uint16_t value = samples[i];
        const uint16_t difference = (value > median_adc) ? (value - median_adc) : (median_adc - value);

        if (difference <= allowed_deviation_adc) {
            sum += value;
            ++valid_count;
        }
    }

    if (valid_count == 0) {
        return adc_to_temperature_c(static_cast<float>(median_adc));
    }

    const float average_adc = static_cast<float>(sum) / static_cast<float>(valid_count);
    return adc_to_temperature_c(average_adc);
}

/**
 * @brief Lê o par de sensores em dois sentidos para reduzir viés de ordem.
 *
 * A leitura é feita como A0->A1 e depois A1->A0. Em seguida, a média entre
 * as duas estimativas de cada canal é usada como valor final do instante.
 *
 * @param sensor_1_mean_c Saída da média robusta do sensor em A0.
 * @param sensor_2_mean_c Saída da média robusta do sensor em A1.
 */
void read_bidirectional_pair_temperatures(float& sensor_1_mean_c, float& sensor_2_mean_c)
{
    const float sensor_1_forward_c = read_robust_mean_temperature_c(sensor_1.pin);
    const float sensor_2_forward_c = read_robust_mean_temperature_c(sensor_2.pin);

    const float sensor_2_reverse_c = read_robust_mean_temperature_c(sensor_2.pin);
    const float sensor_1_reverse_c = read_robust_mean_temperature_c(sensor_1.pin);

    sensor_1_mean_c = 0.5F * (sensor_1_forward_c + sensor_1_reverse_c);
    sensor_2_mean_c = 0.5F * (sensor_2_forward_c + sensor_2_reverse_c);
}

/**
 * @brief Aplica um filtro adaptativo para sinais térmicos lentos.
 *
 * A banda morta evita tremulação muito pequena no valor final. Fora dessa
 * banda, o filtro responde mais rápido quando a diferença cresce e fica mais
 * suave quando o sistema já está próximo do equilíbrio.
 *
 * @param sensor Estado interno do sensor.
 * @param mean_temperature_c Temperatura robusta do instante atual.
 * @return Temperatura filtrada em graus Celsius.
 */
float apply_thermal_filter(SensorState& sensor, const float mean_temperature_c)
{
    if (!sensor.is_filter_initialized) {
        sensor.filtered_temperature_c = mean_temperature_c;
        sensor.is_filter_initialized = true;
        return sensor.filtered_temperature_c;
    }

    const float error_c = mean_temperature_c - sensor.filtered_temperature_c;
    const float abs_error_c = fabsf(error_c);

    if (abs_error_c < 0.03F) {
        return sensor.filtered_temperature_c;
    }

    float alpha = 0.040F;

    if (abs_error_c >= 4.0F) {
        alpha = 0.24F;
    } else if (abs_error_c >= 2.0F) {
        alpha = 0.16F;
    } else if (abs_error_c >= 0.70F) {
        alpha = 0.095F;
    } else if (abs_error_c >= 0.20F) {
        alpha = 0.060F;
    }

    sensor.filtered_temperature_c += alpha * error_c;
    return sensor.filtered_temperature_c;
}

/**
 * @brief Reinicia o estado interno dos filtros.
 */
void reset_filters()
{
    sensor_1.filtered_temperature_c = 0.0F;
    sensor_1.is_filter_initialized = false;

    sensor_2.filtered_temperature_c = 0.0F;
    sensor_2.is_filter_initialized = false;
}

/**
 * @brief Faz leituras iniciais para estabilizar ADC e referência.
 */
void warm_up_inputs()
{
    for (uint8_t i = 0; i < 16; ++i) {
        analogRead(sensor_1.pin);
        delayMicroseconds(CHANNEL_SETTLING_DELAY_US);
        analogRead(sensor_2.pin);
        delayMicroseconds(CHANNEL_SETTLING_DELAY_US);
        delay(4);
    }
}

/**
 * @brief Aguarda até o próximo instante de amostragem da calibração.
 *
 * @param iteration_start_ms Instante inicial da iteração atual.
 */
void wait_calibration_interval(const unsigned long iteration_start_ms)
{
    while ((millis() - iteration_start_ms) < CALIBRATION_SAMPLE_INTERVAL_MS) {
        delay(1);
    }
}

/**
 * @brief Executa a calibração inicial do sensor ligado em A1.
 *
 * Durante alguns segundos, os dois LM35 devem permanecer no mesmo meio
 * térmico. O código calcula o offset médio necessário para alinhar A1 a A0
 * e, em seguida, faz uma pequena validação para estimar o erro residual.
 *
 * @param offset_a1_c Offset estimado para corrigir o sensor A1.
 * @param mean_abs_error_c Erro médio absoluto após a correção.
 * @param residual_bias_c Viés residual médio após a correção.
 */
void perform_initial_calibration(
    float& offset_a1_c,
    float& mean_abs_error_c,
    float& residual_bias_c)
{
    Serial.println();
    Serial.println("CALIBRACAO_INICIAL");
    Serial.println("Mantenha os dois sensores no mesmo meio termico.");
    Serial.println("A1 sera alinhado com A0 antes do experimento.");
    Serial.println();

    const unsigned long calibration_start_ms = millis();
    float delta_sum_c = 0.0F;
    uint16_t sample_count = 0;

    while ((millis() - calibration_start_ms) < CALIBRATION_DURATION_MS) {
        const unsigned long iteration_start_ms = millis();

        float sensor_1_mean_c = 0.0F;
        float sensor_2_mean_c = 0.0F;
        read_bidirectional_pair_temperatures(sensor_1_mean_c, sensor_2_mean_c);

        delta_sum_c += (sensor_1_mean_c - sensor_2_mean_c);
        ++sample_count;

        wait_calibration_interval(iteration_start_ms);
    }

    if (sample_count == 0) {
        offset_a1_c = 0.0F;
        mean_abs_error_c = 0.0F;
        residual_bias_c = 0.0F;
        return;
    }

    offset_a1_c = delta_sum_c / static_cast<float>(sample_count);

    float abs_error_sum_c = 0.0F;
    float residual_sum_c = 0.0F;

    for (uint8_t i = 0; i < CALIBRATION_VALIDATION_SAMPLES; ++i) {
        const unsigned long iteration_start_ms = millis();

        float sensor_1_mean_c = 0.0F;
        float sensor_2_mean_c = 0.0F;
        read_bidirectional_pair_temperatures(sensor_1_mean_c, sensor_2_mean_c);

        const float residual_c = sensor_1_mean_c - (sensor_2_mean_c + offset_a1_c);
        abs_error_sum_c += fabsf(residual_c);
        residual_sum_c += residual_c;

        wait_calibration_interval(iteration_start_ms);
    }

    mean_abs_error_c = abs_error_sum_c / static_cast<float>(CALIBRATION_VALIDATION_SAMPLES);
    residual_bias_c = residual_sum_c / static_cast<float>(CALIBRATION_VALIDATION_SAMPLES);

    Serial.println("CALIBRACAO_COMPLETA");
    Serial.print("OFFSET_A1_C:");
    Serial.println(offset_a1_c, 3);
    Serial.print("ERRO_MEDIO_ABS_C:");
    Serial.println(mean_abs_error_c, 3);
    Serial.print("VIES_RESIDUAL_C:");
    Serial.println(residual_bias_c, 3);
    Serial.println("INICIO_EXPERIMENTO");
    Serial.println();

    reset_filters();
}

/**
 * @brief Envia os dados experimentais ao plotter.
 *
 * @param mean_sensor_1_c Média robusta corrigida do sensor 1.
 * @param filtered_sensor_1_c Valor filtrado do sensor 1.
 * @param mean_sensor_2_c Média robusta corrigida do sensor 2.
 * @param filtered_sensor_2_c Valor filtrado do sensor 2.
 */
void print_plotter_line(
    const float mean_sensor_1_c,
    const float filtered_sensor_1_c,
    const float mean_sensor_2_c,
    const float filtered_sensor_2_c)
{
    Serial.print("S1_media_C:");
    Serial.print(mean_sensor_1_c, 2);
    Serial.print(',');

    Serial.print("S1_filtrada_C:");
    Serial.print(filtered_sensor_1_c, 2);
    Serial.print(',');

    Serial.print("S2_media_C:");
    Serial.print(mean_sensor_2_c, 2);
    Serial.print(',');

    Serial.print("S2_filtrada_C:");
    Serial.println(filtered_sensor_2_c, 2);
}

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);

    if (USE_INTERNAL_REFERENCE) {
        analogReference(INTERNAL);
        delay(80);
    }

    warm_up_inputs();

    perform_initial_calibration(sensor_2_offset_c, calibration_error_c, calibration_residual_bias_c);

    last_update_ms = millis();
}

void loop()
{
    const unsigned long current_ms = millis();

    if ((current_ms - last_update_ms) < UPDATE_INTERVAL_MS) {
        return;
    }

    last_update_ms = current_ms;

    float sensor_1_mean_c = 0.0F;
    float sensor_2_mean_c = 0.0F;
    read_bidirectional_pair_temperatures(sensor_1_mean_c, sensor_2_mean_c);

    sensor_2_mean_c += sensor_2_offset_c;

    const float sensor_1_filtered_c = apply_thermal_filter(sensor_1, sensor_1_mean_c);
    const float sensor_2_filtered_c = apply_thermal_filter(sensor_2, sensor_2_mean_c);

    print_plotter_line(
        sensor_1_mean_c,
        sensor_1_filtered_c,
        sensor_2_mean_c,
        sensor_2_filtered_c);
}
