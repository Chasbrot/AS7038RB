#include "stdint.h"
#include "stdbool.h"
#include "AS7038RB_Constants.h"

#ifdef NRF52
#include "app_error.h"
#include "nrf_drv_gpiote.h"
#endif

/*
* Initializes the AS7038 ic
*/

uint8_t as7038_init();

// Register Access
uint8_t as7038_read_random(uint8_t reg, uint8_t *data);
bool as7038_read_seq(uint8_t start_reg, uint8_t *rx_data, uint8_t count);
bool as7038_write_random(uint8_t reg, uint8_t data);
bool as7038_write_page(uint8_t start_reg, uint8_t *data, uint8_t count);
void as7038_set_bitmask(uint8_t reg, uint8_t mask);
void as7038_clear_bitmask(uint8_t reg, uint8_t mask);

// LED Management
void as7038_led_blink();
void as7038_led_current(as7038_led_t led, as7038_led_current_t current);
void as7038_led_activate(as7038_led_t led);
void as7038_led_deactivate(as7038_led_t led);
void as7038_led_enabled(as7038_led_t led, bool en);
void as7038_led_setMode(as7038_led_t led, as7038_led_mode_t mode);

// Photodiode Management
void as7038_pd_config(as7038_pd_config_t pd, bool en);

// TIA Management
void as7038_tia_setGain(as7038_tia_gain_t gain);

// Prefilter Management
void as7038_prefilter_enableStage(as7038_prefilter_config_t stage);
void as7038_prefilter_disableStage(as7038_prefilter_config_t stage);
void as7038_prefilter_setGain(as7038_prefilter_gain_t gain);
void as7038_prefilter_setAAFreq(as7038_prefilter_aa_freq_t freq);

// ADC Management
void as7038_adc_setClock(as7038_adc_clock_t c);
void as7038_adc_setSettlingTime(as7038_adc_settling_time_t t);
void as7038_adc_setSamples(as7038_adc_samples_t samples);
void as7038_adc_enable();
void as7038_adc_reset();
void as7038_adc_setChannel(as7038_adc_channel_t channel);
void as7038_adc_removeChannel(as7038_adc_channel_t channel);

// Sequencer Management
void as7038_seq_windowITG(uint8_t start, uint8_t e);
void as7038_seq_windowLED(uint8_t start, uint8_t e);
void as7038_seq_samplingPoint(uint8_t start);
void as7038_seq_cycleDuration(uint8_t duration);
void as7038_seq_cycleCount(uint8_t cycles);
void as7038_seq_clockDivider(uint8_t d);
void as7038_seq_enable(bool en);
void as7038_seq_active(bool en);

// Interrupt Management
void as7038_irq_setVector(as7038_irq_vector_t vec, bool en);
as7038_irq_source_t as7038_irq_checkFiredVector();
#ifdef NRF52
void as7038_irq_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
#endif

// CHIP Management
uint8_t as7038_pox_start();
void as7038_chip_setI2CSpeed(as7038_i2c_speed_t s);
void as7038_chip_setInternalClock(as7038_chip_clock_t c);
void as7038_chip_setOscillator(bool en);
void as7038_chip_setLDO(bool en);
uint8_t as7038_chip_getID();
uint8_t as7038_pox_man_start();

// FIFO Management
uint8_t as7038_read_fifo(uint16_t *array);
