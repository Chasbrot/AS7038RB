# AS7038RB
A partial Arduino/ NRF52 driver for the AMS Vital Sign Sensor AS7038RB.
# Current functions
- Initialization of the chip
- Support for NRF52
- Support for Arduino, but it doesn't work stable with a ATmega328p due to limited RAM
- Register and Data access
- Control over LED, OFE, Prefilter, TIA, Sequencer and ADC
- IRQ should be working, not tested
- But I don't know how all of this fits together and get useful data from this device

# Explainer
This driver was partially developed during a side/test project, but it was stopped due to the complexity of this sensor. Maybe someone can use this.
