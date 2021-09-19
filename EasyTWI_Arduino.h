#include <stdbool.h>
#include "Arduino.h"


#ifdef __cplusplus
extern "C" {
#endif
  uint8_t easytwi_init();
  uint8_t easytwi_tx(uint8_t address, uint8_t * tx_data, uint8_t len, bool no_stop);
  uint8_t easytwi_tx_single(uint8_t address, uint8_t data, bool no_stop);
  uint8_t easytwi_rx(uint8_t address, uint8_t *rx_data, uint8_t len);
  uint8_t easytwi_rx_single(uint8_t address, uint8_t *d); 
#ifdef __cplusplus
}
#endif
