#include "EasyTWI.h"

#ifdef NRF52
/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
#endif

ret_code_t  easytwi_init (void)
{
    const nrf_drv_twi_config_t twi_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    ret_code_t r = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    if(r == NRF_SUCCESS){
        nrf_drv_twi_enable(&m_twi);
    }
    return r;
};


ret_code_t easytwi_tx(uint8_t address, uint8_t * tx_data, uint8_t length, bool no_stop){
    return nrf_drv_twi_tx(&m_twi, address, tx_data, length, no_stop);
};

ret_code_t easytwi_tx_single(uint8_t address, uint8_t data, bool no_stop){
    return nrf_drv_twi_tx(&m_twi, address, &data, 1, no_stop);
}

ret_code_t easytwi_rx(uint8_t address, uint8_t *rx_data, uint8_t length){
    return nrf_drv_twi_rx(&m_twi, address, rx_data, length);
};
ret_code_t easytwi_rx_single(uint8_t address, uint8_t *d){
    return nrf_drv_twi_rx(&m_twi, address, d, 1);
};

