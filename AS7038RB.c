#include "AS7038RB.h"

#ifdef NRF52
    #include "EasyTWI.h"
    #include "nrf.h"
    #include "boards.h"
    #include "nrf_gpio.h"
    #include "nrf_delay.h"
    #include "nrf_log.h"
    #include "nrf_log_ctrl.h"
#else
    #include "EasyTWI_Arduino.h"
    #include "Arduino.h"
#endif

#ifdef NRF52
    #define POX_SAMPLES 64
#else
    #define POX_SAMPLES 2
#endif

uint16_t pox_samples[POX_SAMPLES];
uint16_t sample_buffer[AS7038_FIFO_SIZE];

uint8_t as7038_init()
{
    // Init structures
    memset(pox_samples, 0, POX_SAMPLES * sizeof(uint16_t));
    #ifdef NRF52
        nrf_gpio_cfg_output(AS7038_ENABLE_PIN);
        nrf_gpio_pin_set(AS7038_ENABLE_PIN);
        nrf_delay_ms(1); // Wait for power-up ~700us
    #else
        pinMode(AS7038_ENABLE_PIN, OUTPUT);
        digitalWrite(AS7038_ENABLE_PIN, HIGH);
        delay(1000);
    #endif
    // Check if ic is available
    uint8_t id;
    as7038_read_random(AS7038_ID, &id);
    if (id != AS7038_DEVICE_ID)
    {
        return 1;
    }
    // Start osc and ldo
    as7038_write_random(AS7038_CONTROL, 0x03);
    // Enable Sigref
    as7038_write_random(AS7038_LED_CFG, 0x80);
    // LED config
    as7038_led_current(AS7038_LED_1, AS7038_LED_CURRENT_35mA);

    return 0;
}

uint8_t as7038_read_random(uint8_t reg, uint8_t *data)
{
    easytwi_tx_single(AS7038_I2C_ADDRESS, reg, true);
    return easytwi_rx_single(AS7038_I2C_ADDRESS, data);
}

bool as7038_read_seq(uint8_t start_reg, uint8_t *rx_data, uint8_t count)
{
    easytwi_tx_single(AS7038_I2C_ADDRESS, start_reg, true);
    easytwi_rx(AS7038_I2C_ADDRESS, rx_data, count);
    return true;
}

bool as7038_write_random(uint8_t reg, uint8_t data)
{
    uint8_t tx_data[2] = {reg, data};
    easytwi_tx(AS7038_I2C_ADDRESS, tx_data, 2, false);
    return true;
}

bool as7038_write_page(uint8_t start_reg, uint8_t *data, uint8_t count)
{
    uint8_t tx_data[count + 1];
    memset(tx_data, 0, (count + 1) * sizeof(uint8_t));
    tx_data[0] = start_reg;
    for (int i = 1; i < count + 1; i++)
    {
        tx_data[i] = data[i - 1];
    }
    easytwi_tx(AS7038_I2C_ADDRESS, tx_data, count + 1, false);
    return true;
}

void as7038_set_bitmask(uint8_t reg, uint8_t mask)
{
    uint8_t buf;
    as7038_read_random(reg, &buf);
    buf |= mask;
    as7038_write_random(reg, buf);
}

void as7038_clear_bitmask(uint8_t reg, uint8_t mask)
{
    uint8_t buf;
    as7038_read_random(reg, &buf);
    
    buf &= ~(mask);
    
    as7038_write_random(reg, buf);
}

void as7038_led_blink()
{
    // Enable LED1 manual mode and always on when seq is on
    as7038_write_random(AS7038_LED12_MODE, 0x0A);
    // Enable seq and manual mode
    as7038_write_random(AS7038_MAN_SEQ_CFG, 0x81);
    // Enable LED1, blinky blink
    for (int i = 0; i < 3; i++)
    {
        #ifdef NRF52
            nrf_delay_ms(1000);
            as7038_led_activate(AS7038_LED_1);
            nrf_delay_ms(1000);
            as7038_led_deactivate(AS7038_LED_1);
        #else
            delay(1000);
            as7038_led_activate(0);
            delay(1000);
            as7038_led_deactivate(0);
        #endif
    }
    // Disable seq and manual mode
    as7038_write_random(AS7038_MAN_SEQ_CFG, 0x00);
    as7038_write_random(AS7038_LED12_MODE, 0x00);
}

void as7038_led_current(as7038_led_t led, as7038_led_current_t current)
{
    // Write first 2 Bytes
    as7038_write_random(AS7038_LED1_CURRH + (led * 2), (current >> 2));
    // Write last byte
    as7038_write_random(AS7038_LED1_CURRL + (led * 2), (current << 6));
}

void as7038_led_activate(as7038_led_t led)
{
    uint8_t cfg;
    as7038_read_random(AS7038_LED_CFG, &cfg);
    cfg |= 1UL << led;
    as7038_write_random(AS7038_LED_CFG, cfg);
}
void as7038_led_deactivate(as7038_led_t led)
{
    uint8_t cfg;
    as7038_read_random(AS7038_LED_CFG, &cfg);
    cfg &= ~(1UL << led);
    as7038_write_random(AS7038_LED_CFG, cfg);
}

void as7038_led_enabled(as7038_led_t led, bool en)
{
    if (en)
    {
        as7038_set_bitmask(AS7038_LED_CFG, ((uint8_t)led - 1));
    }
    else
    {
        as7038_clear_bitmask(AS7038_LED_CFG, ((uint8_t)led - 1));
    }
}
void as7038_led_setMode(as7038_led_t led, as7038_led_mode_t mode)
{
    if (mode == AS7038_LED_ALWAYS_OFF)
    {
        switch (led)
        {
        case AS7038_LED_1:
            as7038_clear_bitmask(AS7038_LED12_MODE, 0x07);
            break;
        case AS7038_LED_2:
            as7038_clear_bitmask(AS7038_LED12_MODE, 0x70);
            break;
        case AS7038_LED_3:
            as7038_clear_bitmask(AS7038_LED34_MODE, 0x07);
            break;
        case AS7038_LED_4:
            as7038_clear_bitmask(AS7038_LED34_MODE, 0x70);
            break;
        }
    }
    else
    {
        switch (led)
        {
        case AS7038_LED_1:
            as7038_set_bitmask(AS7038_LED12_MODE, mode);
            break;
        case AS7038_LED_2:
            as7038_set_bitmask(AS7038_LED12_MODE, (mode << 4));
            break;
        case AS7038_LED_3:
            as7038_set_bitmask(AS7038_LED34_MODE, mode);
            break;
        case AS7038_LED_4:
            as7038_set_bitmask(AS7038_LED34_MODE, (mode << 4));
            break;
        }
    }
}
void as7038_tia_setGain(as7038_tia_gain_t gain)
{
    as7038_write_random(AS7038_PD_AMPCFG, gain | 0x80); //ENABLE
    as7038_write_random(AS7038_PD_AMPRCCFG, gain >> 8);
}

void as7038_prefilter_enableStage(as7038_prefilter_config_t stage)
{
    as7038_set_bitmask(AS7038_OFE_CFGC, stage);
}
void as7038_prefilter_disableStage(as7038_prefilter_config_t stage)
{
    as7038_clear_bitmask(AS7038_OFE_CFGC, stage);
}

void as7038_prefilter_setGain(as7038_prefilter_gain_t gain)
{
    uint8_t buf;
    as7038_read_random(AS7038_OFE_CFGA, &buf);
    if (gain == AS7038_PREFILTER_GAIN_1)
    {
        buf &= ~(AS7038_PREFILTER_AA_60kHz);
    }
    else
    {
        buf |= gain;
    }
    as7038_write_random(AS7038_OFE_CFGA, buf);
}
void as7038_prefilter_setAAFreq(as7038_prefilter_aa_freq_t freq)
{
    uint8_t buf;
    as7038_read_random(AS7038_OFE_CFGA, &buf);
    if (freq == AS7038_PREFILTER_AA_10kHz)
    {
        buf &= ~(freq);
    }
    else
    {
        buf |= freq;
    }
    as7038_write_random(AS7038_OFE_CFGA, buf);
}

void as7038_pd_config(as7038_pd_config_t pd, bool en)
{
    if (en)
    {
        as7038_set_bitmask(AS7038_PD_CFG, pd);
    }
    else
    {
        as7038_clear_bitmask(AS7038_PD_CFG, pd);
    }
}

void as7038_adc_setClock(as7038_adc_clock_t c)
{
    as7038_write_random(AS7038_ADC_CFGB, c);
}

void as7038_adc_enable()
{
    uint8_t t;
    as7038_read_random(AS7038_ADC_CFGB, &t);
    t |= 0x01;
    as7038_write_random(AS7038_ADC_CFGB, t);
}
void as7038_adc_reset()
{
    uint8_t t;
    as7038_read_random(AS7038_ADC_CFGB, &t);
    t &= ~(0x01);
    as7038_write_random(AS7038_ADC_CFGB, t);
}

void as7038_adc_setChannel(as7038_adc_channel_t channel)
{
    uint8_t maskl, maskh;
    as7038_read_random(AS7038_ADC_CHANNEL_MASK_L, &maskl);
    as7038_read_random(AS7038_ADC_CHANNEL_MASK_H, &maskh);
    maskl |= channel;
    maskh |= channel >> 8;
    as7038_write_random(AS7038_ADC_CHANNEL_MASK_H, maskh);
    as7038_write_random(AS7038_ADC_CHANNEL_MASK_L, maskl);
}
void as7038_adc_removeChannel(as7038_adc_channel_t channel)
{
    uint8_t maskl, maskh;
    as7038_read_random(AS7038_ADC_CHANNEL_MASK_L, &maskl);
    as7038_read_random(AS7038_ADC_CHANNEL_MASK_H, &maskh);
    maskl &= ~(channel);
    maskh &= ~(channel >> 8);
    as7038_write_random(AS7038_ADC_CHANNEL_MASK_H, maskh);
    as7038_write_random(AS7038_ADC_CHANNEL_MASK_L, maskl);
}

void as7038_adc_setSettlingTime(as7038_adc_settling_time_t t)
{
    as7038_write_random(AS7038_ADC_CFGC, t);
}

void as7038_adc_setSamples(as7038_adc_samples_t samples)
{
    if (samples == AS7038_ADC_SAMPLES_1)
    {
        as7038_write_random(AS7038_ADC_CFGA, 0x00);
    }
    else
    {
        as7038_write_random(AS7038_ADC_CFGA, samples | 0x01);
    }
}
/*
* Start or Stop cannot be zero
*/
void as7038_seq_windowITG(uint8_t start, uint8_t e){
    as7038_write_random(AS7038_SEQ_ITG_STA, start);
    as7038_write_random(AS7038_SEQ_ITG_STO, e);
}
/*
* Start or Stop cannot be zero
*/
void as7038_seq_windowLED(uint8_t start, uint8_t e){
    as7038_write_random(AS7038_SEQ_LED_STA, start);
    as7038_write_random(AS7038_SEQ_LED_STO, e);
}
/*
* Start cannot be zero
*/
void as7038_seq_samplingPoint(uint8_t start){
    as7038_write_random(AS7038_SEQ_ADC, start);
}

void as7038_seq_active(bool en){
    if(en){
        as7038_set_bitmask(AS7038_SEQ_START, 0x01);
    }else{
        as7038_clear_bitmask(AS7038_SEQ_START, 0x01);
    }
}
void as7038_seq_enable(bool en){
    if(en){
        as7038_set_bitmask(AS7038_MAN_SEQ_CFG, 0x01);
    }else{
        as7038_clear_bitmask(AS7038_MAN_SEQ_CFG, 0x01);
    }
}

void as7038_seq_cycleDuration(uint8_t duration){
    as7038_write_random(AS7038_SEQ_PER, duration);
}

void as7038_seq_cycleCount(uint8_t cycles){
    as7038_write_random(AS7038_SEQ_CNT, cycles);
}

void as7038_seq_clockDivider(uint8_t d){
    as7038_write_random(AS7038_SEQ_DIV, d);
}

void as7038_irq_setVector(as7038_irq_vector_t vec, bool en){
    if(en){
        as7038_set_bitmask(AS7038_INTENAB, vec >> 8);
        as7038_set_bitmask(AS7038_INTENAB2, vec);
    }else{
        as7038_clear_bitmask(AS7038_INTENAB, vec >> 8);
        as7038_clear_bitmask(AS7038_INTENAB2, vec);
    }
}

as7038_irq_source_t as7038_irq_checkFiredVector(){
    uint16_t stat=0;
    uint8_t buf;

    as7038_read_random(AS7038_INTR, &buf);
    stat |= (buf << 8);
    as7038_read_random(AS7038_INTR2, &buf);
    stat |= buf;

    // Read an clear interrupt from AS7038
    as7038_irq_source_t vec;
    if(stat & AS7038_IRQ_SOURCE_FIFOOVERFLOW){
        vec = AS7038_IRQ_SOURCE_FIFOOVERFLOW;
    }else if(stat & AS7038_IRQ_SOURCE_FIFOTHRESHOLD){
        vec = AS7038_IRQ_SOURCE_FIFOTHRESHOLD;
    }else if(stat & AS7038_IRQ_SOURCE_CLIPDETECT){
        vec = AS7038_IRQ_SOURCE_CLIPDETECT;
    }else if(stat & AS7038_IRQ_SOURCE_VCCLED_LOW){
        vec = AS7038_IRQ_SOURCE_VCCLED_LOW;
    }else if(stat & AS7038_IRQ_SOURCE_ADCTHRESHOLD){
        vec = AS7038_IRQ_SOURCE_ADCTHRESHOLD;
    }else if(stat & AS7038_IRQ_SOURCE_LTF){
        vec = AS7038_IRQ_SOURCE_LTF;
    }else if(stat & AS7038_IRQ_SOURCE_SEQ){
        vec = AS7038_IRQ_SOURCE_SEQ;
    }else if(stat & AS7038_IRQ_SOURCE_ADC){
        vec = AS7038_IRQ_SOURCE_ADC;
    }else if(stat & AS7038_IRQ_SOURCE_ECGTHRESHOLD){
        vec = AS7038_IRQ_SOURCE_ECGTHRESHOLD;
    }else if(stat & AS7038_IRQ_SOURCE_LTFTHRESHOLD_LOW){
        vec = AS7038_IRQ_SOURCE_LTFTHRESHOLD_LOW;
    }else if(stat & AS7038_IRQ_SOURCE_LTFTHRESHOLD_HIGH){
        vec = AS7038_IRQ_SOURCE_LTFTHRESHOLD_HIGH;
    }

    // Clear int vector bit
    as7038_set_bitmask(AS7038_STATUS, vec >> 8);
    as7038_set_bitmask(AS7038_STATUS2, vec);
    return vec;
}
#ifdef NRF52
void as7038_irq_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action){
    //NRF_GPIOTE->EVENTS_IN[15] = 0;
    //nrf_drv_gpiote_clr_task_trigger(15);
    NRF_LOG_INFO("Interrupt!");
    NRF_LOG_FLUSH();

    // Classify Interrupt vector
    as7038_irq_source_t vector = as7038_irq_checkFiredVector();
    // Handle Interrupt
    if(vector == AS7038_IRQ_SOURCE_FIFOOVERFLOW){
        as7038_read_fifo(sample_buffer);
    }
}
#endif

uint8_t as7038_pox_start()
{
    // Enable PD1
    as7038_pd_config(AS7038_PD_1, true);
  
    // Enable LED
    as7038_led_setMode(AS7038_LED_1, AS7038_LED_SEQ_CONTROL);
    
    as7038_led_enabled(AS7038_LED_1, true);

    // Enable pd_amp
    as7038_tia_setGain(AS7038_TIA_GAIN_1VuA_14);

    // Configure Prefilter
    /*
    as7038_prefilter_enableStage(AS7038_PREFILTER_AA_BYPASS);
    as7038_prefilter_enableStage(AS7038_PREFILTER_GAIN_STAGE);
    as7038_prefilter_enableStage(AS7038_PREFILTER_HP_FILTER);
    as7038_prefilter_setGain(AS7038_PREFILTER_GAIN_4);
    */
    
    // Configure ADC
    as7038_adc_setSettlingTime(AS7038_ADC_SETTLING_TIME_4);
    as7038_adc_setChannel(AS7038_ADC_CHANNEL_TIA);
    as7038_adc_setClock(AS7038_ADC_CLOCK_125kHz);
    as7038_adc_enable();
    
    // Configure Sequencer
    as7038_seq_samplingPoint(64);
    as7038_seq_windowLED(1, 128);
    as7038_seq_clockDivider(255);
    as7038_seq_cycleDuration(255);
    as7038_seq_enable(true);

    // Setup Interrupt vector
    //as7038_irq_setVector(AS7038_IRQ_VECTOR_FIFOOVERFLOW,true);


    return 3;
}

uint8_t as7038_pox_man_start()
{
    // LED Driver Configuration
    as7038_led_enabled(AS7038_LED_1, true);
    as7038_write_random(AS7038_LED12_MODE, 0x08); // Enable Manual Output LED1

    // Sequencer Configuration
    // Set Seqencer, Integrator to manual, enable seqencer
    as7038_write_random(AS7038_MAN_SEQ_CFG, 0x89); // 10001001

    // Photodiode Configuration
    as7038_pd_config(AS7038_PD_1, true);

    // Configure TIA 5V/uA
    as7038_tia_setGain(AS7038_TIA_GAIN_15VuA_12);

    // Configure Prefilter
    /*
    as7038_prefilter_enableStage(AS7038_PREFILTER_AA_FILTER);
    as7038_prefilter_enableStage(AS7038_PREFILTER_GAIN_STAGE);
    as7038_prefilter_enableStage(AS7038_PREFILTER_HP_FILTER);
    */

    // Configure ADC
    as7038_adc_setSettlingTime(AS7038_ADC_SETTLING_TIME_16);
    as7038_adc_setChannel(AS7038_ADC_CHANNEL_TIA);
    as7038_adc_setClock(AS7038_ADC_CLOCK_125kHz);
    as7038_adc_enable();

    // Start Sequencer
    as7038_write_random(AS7038_SEQ_START, 0x01);
    return 0;
}


void as7038_chip_setI2CSpeed(as7038_i2c_speed_t s){
    if(s == AS7038_I2C_400kHz){
        as7038_set_bitmask(AS7038_CONTROL, AS7038_I2C_400kHz); // hs_en Bit
    }else{
        as7038_clear_bitmask(AS7038_CONTROL, AS7038_I2C_400kHz); // hs_en Bit
    }
}

void as7038_chip_setInternalClock(as7038_chip_clock_t c){
    if(c == AS7038_CHIP_CLOCK_1MHz){
        as7038_set_bitmask(AS7038_CONTROL, AS7038_CHIP_CLOCK_1MHz);
    }else{
        as7038_clear_bitmask(AS7038_CONTROL, AS7038_CHIP_CLOCK_1MHz);
    }
}

uint8_t as7038_chip_getID(){
    uint8_t data;
    as7038_read_random(AS7038_ID, &data);
    return data >> 3;
}

/*
* Requires uint16_t[64] array because max fifo size
* Returns number of read samples
*/
uint8_t as7038_read_fifo(uint16_t *array)
{
    // Sadly this is not very efficient, but I don't understand how sequential read is supposed to work with the FIFO buffer.
    uint8_t anz, fifol, fifoh;
    as7038_read_random(AS7038_FIFOLEVEL, &anz);
    for (int i = 0; i < anz && i < AS7038_FIFO_BLOCK; i++)
    {
        as7038_read_random(AS7038_FIFOL, &fifol);
        as7038_read_random(AS7038_FIFOH, &fifoh);
        uint16_t buff = (fifoh << 8);
        buff |= fifol;
        array[i] = buff;
    }
    return anz;
}
