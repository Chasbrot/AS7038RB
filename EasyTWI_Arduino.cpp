#include "EasyTWI_Arduino.h"
#include "Wire.h"

uint8_t easytwi_init(){
    Wire.begin();
    return 0;
};

uint8_t easytwi_tx(uint8_t address, uint8_t * tx_data, uint8_t len, bool no_stop){
    Wire.beginTransmission(address);
    Wire.write(tx_data, len);
    return Wire.endTransmission(!no_stop);
};

uint8_t easytwi_tx_single(uint8_t address, uint8_t data, bool no_stop){
    return easytwi_tx(address, data, 1, no_stop);
}

uint8_t easytwi_rx(uint8_t address, uint8_t *rx_data, uint8_t len){
    Wire.requestFrom(address, len);
    int i = 0;
    while(Wire.available()){
        rx_data[++i] = Wire.read();
    }
    return i==len;
};

uint8_t easytwi_rx_single(uint8_t address, uint8_t *d){
    return easytwi_rx(address, d, 1);
};
