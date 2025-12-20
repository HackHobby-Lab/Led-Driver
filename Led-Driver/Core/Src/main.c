/* LED Controller with NeoPixel Support via SPI - Complete Code */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Instant single click ON/OFF, hold to ramp, 4C lockout with SPI NeoPixel
 ******************************************************************************
 */
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* Private variables */
ADC_HandleTypeDef hadc1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;

/* USER CODE END Includes */

/* USER CODE BEGIN PD */
// NTC Configuration
/* NTC Configuration */
#define NTC_BETA      4200
#define NTC_R25       10000
#define NTC_SERIES_R  1478
#define ADC_MAX       4095
#define T0_KELVIN     298       // 25°C in Kelvin

// Battery voltage divider configuration (adjust these based on your circuit)
#define VREF            3300     // Reference voltage in mV (3.3V)
#define BATTERY_R1      10000    // Upper resistor in voltage divider (ohms)
#define BATTERY_R2      10000    // Lower resistor in voltage divider (ohms)
/* USER CODE END PD */

// Temperature protection variables
uint8_t temp_protection_active = 0;
#define TEMP_SHUTDOWN_C     60
#define TEMP_RECOVERY_C     55

void check_temperature_protection(void);

/* NTC Function Prototypes */
int16_t read_ntc_temperature(void);
uint16_t read_ntc_adc(void);
int16_t calculate_temperature(uint16_t adc_value);
uint32_t calculate_resistance(uint16_t adc_value);
void display_ntc_temperature(void);

/* Private function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
void Error_Handler(void);
void delay(int x);
void uart_print(const char *str);
void smooth_fade_to(uint32_t target_brightness);
void handle_click_sequence(uint8_t click_count);

/* NeoPixel function prototypes */
void neopixel_init(void);
void neopixel_set_color(uint16_t pixel, uint8_t r, uint8_t g, uint8_t b);
void neopixel_set_all(uint8_t r, uint8_t g, uint8_t b);
void neopixel_set_brightness(uint8_t brightness_percent);
void neopixel_update(void);
void neopixel_clear(void);
void neopixel_encode_byte(uint8_t byte, uint8_t *buffer);

/* NeoPixel Configuration */
#define NUM_PIXELS 4              // Number of NeoPixels in your strip
#define RESET_BYTES 50            // >50us reset time

/* SPI Buffer Configuration */
#define SPI_BITS_PER_LED (24 * 4)  // 96 bits = 12 bytes per LED
#define SPI_BUFFER_SIZE ((NUM_PIXELS * SPI_BITS_PER_LED / 8) + RESET_BYTES)

/* NeoPixel color buffer */
typedef struct {
	uint8_t g;  // Green
	uint8_t r;  // Red
	uint8_t b;  // Blue
} RGB_Color;

RGB_Color pixel_buffer[NUM_PIXELS];
uint8_t spi_buffer[SPI_BUFFER_SIZE];
uint8_t neopixel_brightness = 100;  // 0-100%

/* Lookup table for SPI encoding */
const uint8_t spi_encode[4] = { 0b10001000,  // 00 = two '0' bits
		0b10001110,  // 01 = '0' bit + '1' bit
		0b11101000,  // 10 = '1' bit + '0' bit
		0b11101110   // 11 = two '1' bits
		};

/* Current color scheme */
uint8_t current_red = 255;
uint8_t current_green = 255;
uint8_t current_blue = 255;

/* Global variables */
uint8_t is_on = 0;
uint8_t is_locked = 0;
uint32_t current_brightness = 0;
uint32_t target_brightness = 2999;
uint32_t max_brightness = 9999;

uint8_t button_pressed = 0;
uint32_t button_press_time = 0;
uint8_t ramp_direction = 1;
uint8_t at_extreme = 0;

uint8_t hold_started = 0;
uint8_t press_handled = 0;
uint8_t instant_action_done = 0;
uint32_t brightness_at_press = 0;

/* Click detection */
#define MAX_CLICKS 4
uint8_t click_count = 0;
uint32_t last_click_time = 0;

uint32_t last_brightness = 0;
uint32_t min_ramp_brightness = 0;
uint32_t momentary_brightness = 0;
uint32_t ARR = 0;

#define CLICK_TIME_MS 200
#define INSTANT_CLICK_MS 20
#define CLICK_TIMEOUT_MS 450
#define FADE_TIME_MS 150
#define FADE_STEPS 10

/* ============ NeoPixel SPI Implementation ============ */

void neopixel_encode_byte(uint8_t byte, uint8_t *buffer) {
	// Encode 8 bits into 32 SPI bits (4 bytes)
	buffer[0] = spi_encode[(byte >> 6) & 0x03];  // Bits 7-6
	buffer[1] = spi_encode[(byte >> 4) & 0x03];  // Bits 5-4
	buffer[2] = spi_encode[(byte >> 2) & 0x03];  // Bits 3-2
	buffer[3] = spi_encode[(byte >> 0) & 0x03];  // Bits 1-0
}

void neopixel_init(void) {
	memset(spi_buffer, 0, sizeof(spi_buffer));
	memset(pixel_buffer, 0, sizeof(pixel_buffer));
	neopixel_clear();
	HAL_Delay(1);
	uart_print("NeoPixel initialized (SPI mode)\r\n");
}

void neopixel_update(void) {
	uint16_t buffer_index = 0;

	// Encode all pixels into SPI buffer
	for (uint16_t i = 0; i < NUM_PIXELS; i++) {
		// Apply brightness scaling
		uint8_t g = (pixel_buffer[i].g * neopixel_brightness) / 100;
		uint8_t r = (pixel_buffer[i].r * neopixel_brightness) / 100;
		uint8_t b = (pixel_buffer[i].b * neopixel_brightness) / 100;

		// WS2812B order: GRB
		neopixel_encode_byte(g, &spi_buffer[buffer_index]);
		buffer_index += 4;
		neopixel_encode_byte(r, &spi_buffer[buffer_index]);
		buffer_index += 4;
		neopixel_encode_byte(b, &spi_buffer[buffer_index]);
		buffer_index += 4;
	}

	// Add reset bytes (all zeros for >50us low)
	memset(&spi_buffer[buffer_index], 0, RESET_BYTES);

	// Send via SPI
	HAL_SPI_Transmit(&hspi1, spi_buffer, SPI_BUFFER_SIZE, 100);
}

void neopixel_set_color(uint16_t pixel, uint8_t r, uint8_t g, uint8_t b) {
	if (pixel < NUM_PIXELS) {
		pixel_buffer[pixel].r = r;
		pixel_buffer[pixel].g = g;
		pixel_buffer[pixel].b = b;
	}
}

void neopixel_set_all(uint8_t r, uint8_t g, uint8_t b) {
	for (uint16_t i = 0; i < NUM_PIXELS; i++) {
		neopixel_set_color(i, r, g, b);
	}
}

void neopixel_set_brightness(uint8_t brightness_percent) {
	if (brightness_percent > 100)
		brightness_percent = 100;
	neopixel_brightness = brightness_percent;
}

void neopixel_clear(void) {
	neopixel_set_all(0, 0, 0);
	neopixel_update();
}

typedef enum {
	ADC_STATE_BATTERY = 0, ADC_STATE_NTC
} adc_state_t;

#define ADC_READ_INTERVAL_MS   500
#define ADC_READS_PER_STATE    10

adc_state_t adc_state = ADC_STATE_NTC;   // 🔴 START WITH NTC
uint8_t adc_read_count = 0;
uint32_t last_adc_tick = 0;

/* ============ Main Application ============ */

int main(void) {
	char msg[100];

	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();

	// Configure PWM for traditional LED
	__HAL_TIM_SET_AUTORELOAD(&htim3, 9999);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

	HAL_Delay(500);
	uart_print("\r\n=== LED Controller with SPI NeoPixel Support ===\r\n");
	uart_print("Click: ON/OFF | Hold: Ramp\r\n");
	uart_print("4 Clicks: Lockout Mode\r\n");
	uart_print("2 Clicks: Color Change\r\n\r\n");

	// Initialize NeoPixels
	neopixel_init();
	neopixel_set_all(current_red, current_green, current_blue);
	neopixel_set_brightness(100);
	neopixel_update();

	// Simple startup test
	uart_print("NeoPixel Test: RED...\r\n");
	neopixel_set_all(255, 0, 0);
	neopixel_set_brightness(20);
	neopixel_update();
	HAL_Delay(500);

	uart_print("NeoPixel Test: GREEN...\r\n");
	neopixel_set_all(0, 255, 0);
	neopixel_update();
	HAL_Delay(500);

	uart_print("NeoPixel Test: BLUE...\r\n");
	neopixel_set_all(0, 0, 255);
	neopixel_update();
	HAL_Delay(500);

	uart_print("NeoPixel Test: OFF\r\n");
	neopixel_clear();
	neopixel_set_all(current_red, current_green, current_blue);

	uart_print("\r\nReady for operation!\r\n\r\n");

	last_brightness = 2999;
	ARR = __HAL_TIM_GET_AUTORELOAD(&htim3);
	min_ramp_brightness = (ARR + 1) * 50 / 1000;
	momentary_brightness = min_ramp_brightness;

	uint32_t last_temp_read = 0;
	bool adc_NTC_stop = false;
	bool adc_btry = true;

	while (1) {
		check_temperature_protection();
//		display_all_sensors();

		uint32_t now = HAL_GetTick();
		GPIO_PinState bs = HAL_GPIO_ReadPin(Button_GPIO_Port, Button_Pin);

		/* Check for click timeout */
		if (click_count > 0 && (now - last_click_time) > CLICK_TIMEOUT_MS) {
			handle_click_sequence(click_count);
			click_count = 0;
		}

		/* --- BUTTON PRESSED (edge) --- */
		if (bs == GPIO_PIN_RESET && !button_pressed) {
			button_pressed = 1;
			button_press_time = now;
			hold_started = 0;
			press_handled = 0;
			instant_action_done = 0;  // Reset instant action flag
			brightness_at_press = current_brightness; // Store brightness at press time
			uart_print("Button PRESSED\r\n");
		}

		/* --- BUTTON RELEASED (edge) --- */
		if (bs == GPIO_PIN_SET && button_pressed) {
			uint32_t press_duration = now - button_press_time;
			button_pressed = 0;

			at_extreme = 0;
			hold_started = 0;

			//            sprintf(msg, "Button RELEASED (%lums)\r\n", press_duration);
			//            uart_print(msg);

			if (press_duration < CLICK_TIME_MS) {
				// Count as click for multi-click detection FIRST
				click_count++;
				last_click_time = now;

				// SHORT PRESS - if instant action wasn't done, do it now
				if (!instant_action_done && !is_locked) {
					// Perform the toggle now on release
					if (is_on) {
						last_brightness = current_brightness;

						// Skip fade if we're in a multi-click sequence
						if (click_count == 1) {
							smooth_fade_to(0);
						} else {
							current_brightness = 0;
							__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
						}
						is_on = 0;
					} else {
						// Skip fade if we're in a multi-click sequence
						if (click_count == 1) {
							smooth_fade_to(last_brightness);
						} else {
							current_brightness = last_brightness;
							__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
									last_brightness);
						}
						is_on = 1;
					}
				}
			} else {
				// LONG PRESS released
				if (is_locked) {
					// Turn off momentary light
					smooth_fade_to(0);
					current_brightness = 0;
					uart_print(">> Momentary OFF\r\n");
				} else {
					// Update is_on state based on current brightness
					if (current_brightness > 0) {
						is_on = 1;
						last_brightness = current_brightness;
					} else {
						is_on = 0;
					}
					sprintf(msg,
							">> Hold released: is_on=%d, brightness=%lu\r\n",
							is_on, current_brightness);
					uart_print(msg);
				}
				// Reset click counter after hold
				click_count = 0;
			}

			press_handled = 0;
			instant_action_done = 0;
		}

		/* --- BUTTON HELD (after crossing CLICK_TIME_MS) --- */
		if (button_pressed && !hold_started
				&& (now - button_press_time) > CLICK_TIME_MS) {
			hold_started = 1;
			click_count = 0; // Clear clicks on hold

			if (is_locked) {
				// Lockout: turn on momentary light at 5%
				uart_print(">> LOCKOUT: Momentary ON (5%)\r\n");
				smooth_fade_to(momentary_brightness);
				current_brightness = momentary_brightness;
			} else {
				// Normal ramping mode - NO TOGGLE, just start ramping from current state
				uart_print(">> HOLD mode activated - starting ramp\r\n");

				// Determine ramp direction based on brightness at press time
				if (brightness_at_press <= min_ramp_brightness) {
					ramp_direction = 1;  // force ramp UP
					uart_print(">> At MIN - forcing ramp UP\r\n");
				} else if (brightness_at_press >= max_brightness) {
					ramp_direction = 0;  // force ramp DOWN
					uart_print(">> At MAX - forcing ramp DOWN\r\n");
				} else {
					// Toggle direction for middle brightness values
					ramp_direction ^= 1;
					sprintf(msg, ">> Toggling ramp direction to %s\r\n",
							ramp_direction ? "UP" : "DOWN");
					uart_print(msg);
				}

				sprintf(msg, ">> HOLD started from %lu, direction: %s\r\n",
						current_brightness, ramp_direction ? "UP" : "DOWN");
				uart_print(msg);
			}
		}

		/* --- Perform ramping if hold is active (not in lockout) --- */
		if (hold_started && !at_extreme && !is_locked) {
			if (ramp_direction == 1) // ramp UP
					{
				uint32_t step = 50;  // Reduced step size for smoother ramping
				if (current_brightness + step >= max_brightness) {
					current_brightness = max_brightness;
					at_extreme = 1;
					uart_print(">> HOLD: MAX reached\r\n");
				} else {
					current_brightness += step;
				}
			} else // ramp DOWN
			{
				uint32_t step = 50;  // Reduced step size for smoother ramping
				if (current_brightness <= min_ramp_brightness + step) {
					current_brightness = min_ramp_brightness;
					at_extreme = 1;
					uart_print(">> HOLD: MIN reached (5%)\r\n");
				} else {
					current_brightness -= step;
				}
			}

			target_brightness = current_brightness;
			last_brightness = current_brightness;
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, current_brightness);

			// Slower ramp - 2 seconds from 5% to 100%
			HAL_Delay(10);
		}

		HAL_Delay(1); // small loop delay
	}
}





/**
 * @brief Generic ADC channel read function
 * @param channel: ADC channel to read (e.g., ADC_CHANNEL_1, ADC_CHANNEL_8)
 * @return Raw ADC value (0-4095)
 */
uint16_t ADC_Read_Channel(uint32_t channel) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	uint16_t value = 0;

	// Configure the channel
	sConfig.Channel = channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;  // 19.5 cycles

	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	// Start conversion
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

	value = HAL_ADC_GetValue(&hadc1);

	HAL_ADC_Stop(&hadc1);

	return value;
}

// ==================== NTC THERMISTOR FUNCTIONS ====================

/**
 * @brief Read ADC value from NTC thermistor
 * @return Raw ADC value (0-4095)
 */
uint16_t read_ntc_adc(void) {
	return ADC_Read_Channel(ADC_CHANNEL_8);
}

/**
 * @brief Read averaged NTC ADC value (32 samples)
 * @return Averaged ADC value
 */
uint16_t read_ntc_adc_avg(void) {
	uint32_t sum = 0;
	for (int i = 0; i < 32; i++) {
		sum += read_ntc_adc();
	}
	return (uint16_t) (sum >> 5);  // Divide by 32
}

/**
 * @brief Calculate NTC resistance from ADC value
 * @param adc: ADC reading (0-4095)
 * @return Resistance in ohms
 */
uint32_t calculate_resistance(uint16_t adc) {
	if (adc == 0)
		return 1000000; // Open circuit
	if (adc >= ADC_MAX)
		return 1;       // Short circuit

	// Rntc = Rs * ADC / (ADCmax - ADC)
	return (NTC_SERIES_R * (uint32_t) adc) / (ADC_MAX - adc);
}

/**
 * @brief Integer logarithm approximation (ln(x) * 1000)
 * @param x: Input value
 * @return Natural logarithm scaled by 1000
 */
static int32_t ilog_ln(uint32_t x) {
	int32_t result = 0;

	// Normalize x to range [0.5, 2)
	while (x > 2000) {
		x >>= 1;
		result += 693;   // ln(2) * 1000
	}
	while (x < 1000) {
		x <<= 1;
		result -= 693;
	}

	// Now x in [1000, 2000] → scale to Q10
	int32_t y = (int32_t) x - 1500; // center
	int32_t y2 = (y * y) / 1000;

	// ln(1+z) ≈ z - z²/2
	result += (y * 1000) / 1500;
	result -= (y2 * 500) / (1500 * 1500);

	return result;
}

/**
 * @brief Calculate temperature from ADC value using Beta equation
 * @param adc: ADC reading (0-4095)
 * @return Temperature in Celsius (integer)
 */
int16_t calculate_temperature(uint16_t adc) {
	uint32_t r = calculate_resistance(adc);

	// ln(R/R25) × 1000
	int32_t ln_rr25 = ilog_ln((r * 1000) / NTC_R25);

	// 1/T = 1/T0 + ln(R/R25)/B
	// Multiply by 1e6 to keep precision
	int32_t invT = (1000000 / T0_KELVIN) + (ln_rr25 * 1000) / NTC_BETA;

	if (invT <= 0)
		return 25;

	int32_t tempK = 1000000 / invT;
	return (int16_t) (tempK - 273);
}

/**
 * @brief Read temperature from NTC thermistor
 * @return Temperature in Celsius (integer)
 */
int16_t read_ntc_temperature(void) {
	return calculate_temperature(read_ntc_adc_avg());
}

/**
 * @brief Check if temperature is within safe operating range
 * @param max_temp_c: Maximum safe temperature in Celsius
 * @return 1 if safe, 0 if too hot
 */
uint8_t is_temperature_safe(int16_t max_temp_c) {
	int16_t current_temp = read_ntc_temperature();
	return (current_temp <= max_temp_c) ? 1 : 0;
}

// ==================== BATTERY VOLTAGE FUNCTIONS ====================

/**
 * @brief Read ADC value from battery voltage divider
 * @return Raw ADC value (0-4095)
 */
uint16_t read_btry_adc(void) {
	return ADC_Read_Channel(ADC_CHANNEL_1);
}

/**
 * @brief Read averaged battery ADC value (16 samples)
 * @return Averaged ADC value
 */
uint16_t read_btry_adc_avg(void) {
	uint32_t sum = 0;
	for (int i = 0; i < 16; i++) {
		sum += read_btry_adc();
	}
	return (uint16_t) (sum >> 4);  // Divide by 16
}

/**
 * @brief Convert battery ADC to actual voltage in millivolts
 * @param adc: ADC reading (0-4095)
 * @return Battery voltage in mV
 */
uint32_t calculate_battery_voltage(uint16_t adc) {
	// First calculate voltage at ADC pin
	uint32_t adc_voltage = (adc * VREF) / ADC_MAX;

	// Then calculate actual battery voltage considering voltage divider
	// Vbat = Vadc * (R1 + R2) / R2
	uint32_t battery_voltage = (adc_voltage * (BATTERY_R1 + BATTERY_R2))
			/ BATTERY_R2;

	return battery_voltage;
}

/**
 * @brief Read battery voltage
 * @return Battery voltage in millivolts
 */
uint32_t read_battery_voltage(void) {
	uint16_t adc = read_btry_adc_avg();
	return calculate_battery_voltage(adc);
}

/**
 * @brief Calculate battery percentage (for Li-ion: 4.2V=100%, 3.0V=0%)
 * @param voltage_mv: Battery voltage in millivolts
 * @return Battery percentage (0-100)
 */
uint8_t calculate_battery_percentage(uint32_t voltage_mv) {
#define BATTERY_MAX_MV  4200  // Fully charged (adjust for your battery type)
#define BATTERY_MIN_MV  3000  // Empty (adjust for your battery type)

	if (voltage_mv >= BATTERY_MAX_MV)
		return 100;
	if (voltage_mv <= BATTERY_MIN_MV)
		return 0;

	uint32_t range = BATTERY_MAX_MV - BATTERY_MIN_MV;
	uint32_t current = voltage_mv - BATTERY_MIN_MV;

	return (uint8_t) ((current * 100) / range);
}

// ==================== DISPLAY FUNCTIONS ====================

/**
 * @brief Display NTC temperature data via UART
 */
void display_ntc_temperature(void) {
	char msg[80];
	uint16_t adc = read_ntc_adc_avg();
	uint32_t r = calculate_resistance(adc);
	int16_t t = calculate_temperature(adc);

	sprintf(msg, "NTC -> ADC:%u | R:%lu Ω | Temp:%d°C\r\n", adc, r, t);
	HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
}

/**
 * @brief Display battery voltage data via UART
 */
void display_battery_voltage(void) {
	char msg[80];
	uint16_t adc = read_btry_adc_avg();
	uint32_t voltage = calculate_battery_voltage(adc);
	uint8_t percentage = calculate_battery_percentage(voltage);

	sprintf(msg, "Battery -> ADC:%u | Voltage:%lu.%03lu V | Level:%u%%\r\n",
			adc, voltage / 1000, voltage % 1000, percentage);
	HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
}

/**
 * @brief Display both NTC and Battery data together
 */
void display_all_sensors(void) {
	char msg[120];

	// Read NTC
	uint16_t ntc_adc = read_ntc_adc_avg();
	int16_t temp = calculate_temperature(ntc_adc);

	// Read Battery
	uint16_t btry_adc = read_btry_adc_avg();
	uint32_t voltage = calculate_battery_voltage(btry_adc);
	uint8_t percentage = calculate_battery_percentage(voltage);

	sprintf(msg, "Temp:%d C | Battery:%lu.%02luV (%u%%)\r\n", temp,
			voltage / 1000, (voltage % 1000) / 10, percentage);
	HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
}


// Temperature protection check function
void check_temperature_protection(void) {
    static uint32_t last_temp_check = 0;
    uint32_t now = HAL_GetTick();

    // Check temperature every 500ms
    if ((now - last_temp_check) < 500) {
        return;
    }
    last_temp_check = now;

    int16_t current_temp = read_ntc_temperature();
    char msg[80];

    if (!temp_protection_active && current_temp >= TEMP_SHUTDOWN_C) {
        // Temperature too high - activate protection
        temp_protection_active = 1;

        sprintf(msg, "!! TEMP PROTECTION: %d°C - LED OFF !!\r\n", current_temp);
        uart_print(msg);

        // Turn off main LED immediately
        HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);
        current_brightness = 0;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);

//        // Visual warning on NeoPixels - fast red blink
//        for (uint8_t i = 0; i < 3; i++) {
//            neopixel_set_all(255, 0, 0);
//            neopixel_set_brightness(100);
//            neopixel_update();
//            HAL_Delay(100);
//            neopixel_clear();
//            HAL_Delay(100);
//        }
    }
    else if (temp_protection_active && current_temp <= TEMP_RECOVERY_C) {
        // Temperature back to safe level - deactivate protection
        temp_protection_active = 0;

        sprintf(msg, ">> TEMP OK: %d°C - LED can operate\r\n", current_temp);
        uart_print(msg);
    }
    else if (temp_protection_active) {
        // Still in protection mode - keep LED off
        if (current_brightness > 0) {
            HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);
            current_brightness = 0;
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        }
    }
}


void show_battery_check_animation(void) {
    char msg[80];

    // Read battery voltage
    uint32_t voltage = read_battery_voltage();

    sprintf(msg, "Battery Check: %lu.%02luV\r\n", voltage / 1000, (voltage % 1000) / 10);
    uart_print(msg);

    // Clear all LEDs first
    neopixel_clear();
    HAL_Delay(200);

    // Determine animation based on voltage ranges
    if (voltage >= 3300) {
        // 3.3V-4.2V = Slow fade green
        uart_print("Battery Good: Slow fade green\r\n");
        for (uint8_t brightness = 0; brightness <= 100; brightness += 2) {
            neopixel_set_all(0, 255, 0);  // Green
            neopixel_set_brightness(brightness);
            neopixel_update();
            HAL_Delay(20);  // Slow fade (2 seconds total)
        }
        HAL_Delay(1000);  // Hold at full brightness
        for (uint8_t brightness = 100; brightness > 0; brightness -= 2) {
            neopixel_set_brightness(brightness);
            neopixel_update();
            HAL_Delay(20);  // Slow fade out
        }
    }
    else if (voltage >= 2800) {
        // 2.8V-3.3V = Slow fade yellow
        uart_print("Battery Medium: Slow fade yellow\r\n");
        for (uint8_t brightness = 0; brightness <= 100; brightness += 2) {
            neopixel_set_all(255, 150, 0);  // Yellow/Orange
            neopixel_set_brightness(brightness);
            neopixel_update();
            HAL_Delay(20);  // Slow fade
        }
        HAL_Delay(1000);  // Hold at full brightness
        for (uint8_t brightness = 100; brightness > 0; brightness -= 2) {
            neopixel_set_brightness(brightness);
            neopixel_update();
            HAL_Delay(20);  // Slow fade out
        }
    }
    else {
        // 2.7V-2.8V = Fast pulse red
        uart_print("Battery Low: Fast pulse red\r\n");
        for (uint8_t pulse = 0; pulse < 5; pulse++) {
            neopixel_set_all(255, 0, 0);  // Red
            neopixel_set_brightness(100);
            neopixel_update();
            HAL_Delay(100);  // Fast pulse
            neopixel_clear();
            HAL_Delay(100);
        }
    }

    neopixel_clear();
}


// Updated handle_click_sequence function
void handle_click_sequence(uint8_t clicks)
{
    char msg[100];
    sprintf(msg, ">> Processing %d click(s)\r\n", clicks);
    uart_print(msg);

    if (is_locked)
    {
        /* LOCKOUT MODE BEHAVIOR */
        if (clicks == 4)
        {
            // 4C: Exit lockout
            is_locked = 0;
            is_on = 0;
            current_brightness = 0;
            smooth_fade_to(0);
            uart_print(">> UNLOCKED - Back to OFF\r\n");
        }
        else if (clicks == 3)
        {
            // 3C: Battery check with new animation
            uart_print(">> LOCKOUT: Battery Check\r\n");
            show_battery_check_animation();

            // Restore previous state (lockout stays off)
            neopixel_set_all(current_red, current_green, current_blue);
            neopixel_set_brightness(100);
        }
        else
        {
            sprintf(msg, ">> LOCKOUT: %d click(s) - no action\r\n", clicks);
            uart_print(msg);
        }
    }
    else
    {
        /* NORMAL MODE BEHAVIOR */
        if (clicks == 1)
        {
            // Single click was already handled instantly on press
            sprintf(msg, ">> Single click already handled (is_on=%d)\r\n", is_on);
            uart_print(msg);
        }
        else if (clicks == 4)
        {
            uart_print(">> ENTERING LOCKOUT MODE\r\n");

            is_locked = 1;
            is_on = 0;

            // Fade to 0 when entering lockout
            smooth_fade_to(0);
            current_brightness = 0;

            uart_print(">> Lockout active - brightness at 0\r\n");
        }
        else if (clicks == 3)
        {
            // 3C from Off: Battery check with voltage-based animation
            uart_print(">> Battery Check (3C)\r\n");
            show_battery_check_animation();
        }
        else
        {
            sprintf(msg, ">> %d click(s) - no action assigned\r\n", clicks);
            uart_print(msg);
        }
    }
}

void smooth_fade_to(uint32_t target)
{
    char msg[80];
    uint32_t start_brightness = current_brightness;
    int32_t brightness_diff = (int32_t)target - (int32_t)start_brightness;
    uint32_t step_delay = FADE_TIME_MS / FADE_STEPS;

    sprintf(msg, "Fading from %lu to %lu\r\n", start_brightness, target);
    uart_print(msg);

    for (int i = 1; i <= FADE_STEPS; i++)
    {
        current_brightness = start_brightness + (brightness_diff * i / FADE_STEPS);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, current_brightness);
        HAL_Delay(step_delay);
    }

    current_brightness = target;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, current_brightness);

    sprintf(msg, "Fade complete at %lu (%.1f%%)\r\n",
            current_brightness, (current_brightness * 100.0) / 9999.0);
    uart_print(msg);
}

void delay(int x) {
	volatile int i, j;
	for (i = 0; i < x; i++) {
		j++;
	}
}

void uart_print(const char *str) {
	HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str), 1000);
}

/* ============ System Configuration Functions ============ */

void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	__HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);

	/* Configure Button pin : INPUT with PULLUP */
	GPIO_InitStruct.Pin = Button_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

	/* Configure LED pin : OUTPUT Push-Pull */
	GPIO_InitStruct.Pin = Led_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);

	/* Configure SPI1 pins for NeoPixel */
	/* SPI1 MOSI (PA7) - Connect to NeoPixel DIN */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* SPI1 SCK (PA5) - Not connected but needs configuration */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* Configure USART pins : Alternate Function */
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;  // TX and RX
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.LowPowerAutoPowerOff = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DiscontinuousConvMode = ENABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_19CYCLES_5;
	hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_12CYCLES_5;
	hadc1.Init.OversamplingMode = DISABLE;
	hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

static void MX_SPI1_Init(void) {
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;                    // Changed for NeoPixel
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;  // 48MHz/8 = 6MHz
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;    // Disabled for software NSS
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_USART1_UART_Init(void) {
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_TIM1_Init(void) {
	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
}

static void MX_TIM3_Init(void) {
	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 11999;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	HAL_TIM_MspPostInit(&htim3);
}

void Error_Handler(void) {
	__disable_irq();
	while (1) {
		// Stay here if error occurs
	}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add implementation to report the error */
}
#endif
