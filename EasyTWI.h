#include "stdint.h"
#include "stdbool.h"

#ifdef NRF52
    #include "app_error.h"
    #include "nrf_drv_twi.h"
    #include "nrfx_twi.h"
    #include "boards.h"
#endif

/* TWI instance ID. */
#define TWI_INSTANCE_ID 0

ret_code_t easytwi_init();
ret_code_t easytwi_tx(uint8_t address, uint8_t * tx_data, uint8_t length, bool no_stop);
ret_code_t easytwi_tx_single(uint8_t address, uint8_t data, bool no_stop);
ret_code_t easytwi_rx(uint8_t address, uint8_t *rx_data, uint8_t length);
ret_code_t easytwi_rx_single(uint8_t address, uint8_t *d);