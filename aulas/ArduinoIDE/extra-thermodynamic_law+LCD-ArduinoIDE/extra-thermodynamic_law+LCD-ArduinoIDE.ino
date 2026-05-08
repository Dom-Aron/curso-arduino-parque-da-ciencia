/*
 * Curso de Formação de Professores – Atividades Experimentais de Física
 * Local: Parque da Ciência Newton Freire Maia (PR, Brasil)
 * Plataforma: Arduino
 * Ambiente alvo: Arduino IDE
 * Autores do curso: Aron da Rocha Battistella; Marcos Rocha; Alan Henrique Abreu Dias
 * Colaboradores do curso: Letícia Trzaskos Abbeg; Gabriel Cordeiro Chileider
 * Autoria dos códigos: Aron da Rocha Battistella e Marcos Rocha
 * Colaboração nos códigos: Letícia Trzaskos Abbeg, Gabriel Cordeiro Chileider e Alan Henrique Abreu Dias
 * Repositório: https://github.com/parquedaciencia/curso-arduino-parque-da-ciencia
 * Caminho no repositório: aulas/ArduinoIDE/extra-thermodynamic_law+LCD-ArduinoIDE/extra-thermodynamic_law+LCD-ArduinoIDE.ino
 * Data da última revisão: 29/04/2026
 *
 * Descrição:
 *   ============================================================
 *   Projeto   : Extra - Thermodynamic Law + LCD
 *   Arquivo   : extra-thermodynamic_law+LCD-ArduinoIDE.ino
 *   Pasta     : extra-thermodynamic_law+LCD-ArduinoIDE
 *   Este sketch lê o sensor LM35 ligado ao pino A0, aplica uma média
 *   aparada para reduzir ruídos, utiliza um filtro adaptativo e
 *   apresenta a temperatura filtrada em um display LCD 16x2 com
 *   interface I2C.
 *
 *   Esta versão foi organizada para demonstração direta em bancada,
 *   com exibição local no display.
 *   ============================================================
 */
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

constexpr uint8_t LM35_PIN = A0;
constexpr float ADC_REFERENCE_VOLTAGE = 5.0F;
constexpr float ADC_RESOLUTION = 1023.0F;

constexpr unsigned long UPDATE_INTERVAL_MS = 1500UL;
constexpr uint8_t SAMPLE_COUNT = 150;
constexpr uint8_t DISCARD_COUNT_PER_SIDE = 5;

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

unsigned long last_update_ms = 0UL;
float filtered_temperature_c = 0.0F;
bool is_filter_initialized = false;

/**
 * Sort an array in ascending order.
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
 * Read the LM35 raw temperature in Celsius using a trimmed mean.
 */
float read_raw_temperature_c()
{
    uint16_t samples[SAMPLE_COUNT];

    for (uint8_t i = 0; i < SAMPLE_COUNT; ++i) {
        samples[i] = analogRead(LM35_PIN);
        delay(2);
    }

    sort_samples(samples, SAMPLE_COUNT);

    uint32_t sum = 0;
    uint8_t valid_count = 0;

    for (uint8_t i = DISCARD_COUNT_PER_SIDE; i < SAMPLE_COUNT - DISCARD_COUNT_PER_SIDE; ++i) {
        sum += samples[i];
        ++valid_count;
    }

    const float average_adc = static_cast<float>(sum) / static_cast<float>(valid_count);
    const float voltage = average_adc * (ADC_REFERENCE_VOLTAGE / ADC_RESOLUTION);
    const float temperature_c = voltage * 100.0F;

    return temperature_c;
}

/**
 * Apply an adaptive filter.
 *
 * Large jumps are followed quickly.
 * Small oscillations are smoothed more strongly.
 */
float apply_adaptive_filter(const float raw_temperature_c)
{
    if (!is_filter_initialized) {
        filtered_temperature_c = raw_temperature_c;
        is_filter_initialized = true;
        return filtered_temperature_c;
    }

    const float difference_c = fabs(raw_temperature_c - filtered_temperature_c);

    if (difference_c >= 15.0F) {
        filtered_temperature_c = raw_temperature_c;
    } else if (difference_c >= 5.0F) {
        filtered_temperature_c = (0.40F * raw_temperature_c) + (0.60F * filtered_temperature_c);
    } else {
        filtered_temperature_c = (0.15F * raw_temperature_c) + (0.85F * filtered_temperature_c);
    }

    return filtered_temperature_c;
}

/**
 * Update the LCD content.
 */
void update_lcd(const float temperature_c)
{
    lcd.setCursor(0, 0);
    lcd.print('T');
    lcd.print('e');
    lcd.print('m');
    lcd.print('p');
    lcd.print(':');
    lcd.print(' ');
    lcd.print("         ");

    lcd.setCursor(6, 0);
    lcd.print(temperature_c, 1);
    lcd.write(static_cast<uint8_t>(223));
    lcd.print('C');

    lcd.setCursor(0, 1);
    lcd.print("LM35 + LCD I2C  ");
}

void setup()
{
    analogReference(DEFAULT);

    for (uint8_t i = 0; i < 10; ++i) {
        analogRead(LM35_PIN);
        delay(5);
    }

    lcd.begin(16, 2);
    lcd.setBacklight(HIGH);

    lcd.setCursor(0, 0);
    lcd.print("Iniciando...");
    lcd.setCursor(0, 1);
    lcd.print("LM35 + LCD I2C");
    delay(1200);
    lcd.clear();
}

void loop()
{
    const unsigned long current_ms = millis();

    if (current_ms - last_update_ms >= UPDATE_INTERVAL_MS) {
        last_update_ms = current_ms;

        const float raw_temperature_c = read_raw_temperature_c();
        const float stable_temperature_c = apply_adaptive_filter(raw_temperature_c);

        update_lcd(stable_temperature_c);
    }
}
