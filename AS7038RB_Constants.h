// Register Map
#define AS7038_I2C_ADDRESS 0x30
#define AS7038_DEVICE_ID 0x54
#define AS7038_ENABLE_PIN 22
#define AS7038_FIFO_BLOCK 32
#define AS7038_FIFO_SIZE 64

#define AS7038_GPIO_SYNC 0x0F
#define AS7038_LED_CFG 0x10
#define AS7038_LED_WAIT_L_OW 0x11
#define AS7038_LED1_CURRL 0x12
#define AS7038_LED1_CURRH 0x13
#define AS7038_LED2_CURRL 0x14
#define AS7038_LED2_CURRH 0x15
#define AS7038_LED3_CURRL 0x16
#define AS7038_LED3_CURRH 0x17
#define AS7038_LED4_CURRL 0x18
#define AS7038_LED4_CURRH 0x19
#define AS7038_LED12_MODE 0x2C
#define AS7038_LED34_MODE 0x2D
#define AS7038_MAN_SEQ_CFG 0x2E
#define AS7038_PD_CFG 0x1A
#define AS7038_PDOFFX_LEDOFF 0x1B
#define AS7038_PDOFFX_LEDON 0x1C
#define AS7038_PD_AMPRCCFG 0x1D
#define AS7038_PD_AMPCFG 0x1E
#define AS7038_OFE1_PD_THCFG 0x1F
#define AS7038_SEQ_CNT 0x30
#define AS7038_SEQ_DIV 0x31
#define AS7038_SEQ_START 0x32
#define AS7038_SEQ_PER 0x33
#define AS7038_SEQ_LED_STA 0x34
#define AS7038_SEQ_LED_STO 0x35
#define AS7038_SEQ_SECLED_STA 0x36
#define AS7038_SEQ_SECLED_STO 0x37
#define AS7038_SEQ_ITG_STA 0x38
#define AS7038_SEQ_ITG_STO 0x39
#define AS7038_SEQ_SDP1_STA 0x3A
#define AS7038_SEQ_SDP1_STO 0x3B
#define AS7038_SEQ_SDP2_STA 0x3C
#define AS7038_SEQ_SDP2_STO 0x3D
#define AS7038_SEQ_SDM1_STA 0x3E
#define AS7038_SEQ_SDM1_STO 0x3F
#define AS7038_SEQ_SDM2_STA 0x40
#define AS7038_SEQ_SDM2_STO 0x41
#define AS7038_SEQ_ADC 0x42
#define AS7038_SEQ_ADC2TIA 0x43
#define AS7038_SEQ_ADC3TIA 0x44
#define AS7038_SD_SUBS 0x45
#define AS7038_SEQ_CFG 0x46
#define AS7038_SEQ_ERR 0x47
#define AS7038_SEQ_OVS_SEL 0x48
#define AS7038_SEQ_OVS_VAL 0x49
#define AS7038_SEQ_DIS_SEL 0x4A
#define AS7038_SEQ_DIS_VAL1 0x4B
#define AS7038_SEQ_DIS_VAL2 0x4C
#define AS7038_CYC_COUNTER 0x60
#define AS7038_SEQ_COUNTER 0x61
#define AS7038_SUBS_COUNTER 0x62
#define AS7038_OFE_CFGA 0x50
#define AS7038_OFE1_SD_THCFG 0x51
#define AS7038_OFE_CFGC 0x52
#define AS7038_OFE_CFGD 0x53
#define AS7038_OFE1_CFGA 0x54
#define AS7038_OFE1_CFGB 0x55
#define AS7038_OFE2_PD_THCFG 0x56
#define AS7038_OFE2_SD_THCFG 0x57
#define AS7038_LTFDATA0_L 0x20
#define AS7038_LTFDATA0_H 0x21
#define AS7038_LTFDATA1_L 0x22
#define AS7038_LTFDATA1_H 0x23
#define AS7038_ITIME 0x24
#define AS7038_LTF_CONFIG 0x25
#define AS7038_LTF_SEL 0x26
#define AS7038_LTF_GAIN 0x27
#define AS7038_LTF_CONTROL 0x28
#define AS7038_AZ_CONTROL 0x29
#define AS7038_OFFSET0 0x2A
#define AS7038_OFFSET1 0x2B
#define AS7038_LTF_THRESHOLD_LOW0 0x6C
#define AS7038_LTF_THRESHOLD_HIGH0 0x6D
#define AS7038_LTF_THRESHOLD_LOW1 0x6E
#define AS7038_LTF_THRESHOLD_HIGH1 0x6F
#define AS7038_EAF_CFG 0x70
#define AS7038_EAF_GST 0x80
#define AS7038_EAF_BIAS 0x81
#define AS7038_EAF_DAC 0x82
#define AS7038_EAF_DAC1_L 0x83
#define AS7038_EAF_DAC1_H 0x84
#define AS7038_EAF_DAC2_L 0x85
#define AS7038_EAF_DAC2_H 0x86
#define AS7038_EAF_DAC_CFG 0x87
#define AS7038_OFE_NOTCH 0x5A
#define AS7038_ECG_MODE 0x5B
#define AS7038_ECG_CFGA 0x5C
#define AS7038_ECG_CFGB 0x5D
#define AS7038_ECG_THRESHOLD_LOW 0x6A
#define AS7038_ECG_THRESHOLD_HIGH 0x6B
#define AS7038_ECG_CFGC 0x5E
#define AS7038_ECG_CFGD 0x5F
#define AS7038_ADC_TRHESHOLD 0x68
#define AS7038_ADC_THRESHOLD_CFG 0x69
#define AS7038_ADC_CFGA 0x88
#define AS7038_ADC_CFGB 0x89
#define AS7038_ADC_CFGC 0x8A
#define AS7038_ADC_CHANNEL_MASK_L 0x8B
#define AS7038_ADC_CHANNEL_MASK_H 0x8C
#define AS7038_ADC_DATA_L 0x8E
#define AS7038_ADC_DATA_H 0x8F
#define AS7038_FIFO_CFG 0x78
#define AS7038_FIFO_CNTRL 0x79
#define AS7038_FIFOL 0xFE
#define AS7038_FIFOH 0xFF
#define AS7038_CONTROL 0x00
#define AS7038_GPIO_A 0x08
#define AS7038_GPIO_E 0x09
#define AS7038_GPIO_O 0x0A
#define AS7038_GPIO_I 0x0B
#define AS7038_GPIO_P 0x0C
#define AS7038_GPIO_SR 0x0D
#define AS7038_SUBID 0x91
#define AS7038_ID 0x92
#define AS7038_STATUS 0xA0
#define AS7038_STATUS2 0xA1
#define AS7038_CLIPSTATUS 0xA2
#define AS7038_LEDSTATUS 0xA3
#define AS7038_FIFOSTATUS 0xA4
#define AS7038_LTFSTATUS 0xA5
#define AS7038_FIFOLEVEL 0xA6
#define AS7038_INTENAB 0xA8
#define AS7038_INTENAB2 0xA9
#define AS7038_INTR 0xAA
#define AS7038_INTR2 0xAB

typedef enum
{
    AS7038_LED_CURRENT_786uA = 0x000,
    AS7038_LED_CURRENT_883uA = 0x001,
    AS7038_LED_CURRENT_980uA = 0x002,
    AS7038_LED_CURRENT_35mA = 0x166,
    AS7038_LED_CURRENT_100mA = 0x3FF,
} as7038_led_current_t;

typedef enum
{
    AS7038_TIA_AMPRES_1 = 0x20, //00100000
    AS7038_TIA_AMPRES_2 = 0x40, //01000000
    AS7038_TIA_AMPRES_3 = 0x60, //01100000
    AS7038_TIA_AMPRES_4 = 0x80, //10000000
    AS7038_TIA_AMPRES_5 = 0xA0, //10100000
    AS7038_TIA_AMPRES_6 = 0xC0, //11000000
    AS7038_TIA_AMPRES_7 = 0xE0, //11100000
} as7038_tia_ampres_t;

typedef enum
{
    AS7038_TIA_AMPCAP_1 = 0x01,  //00000001
    AS7038_TIA_AMPCAP_2 = 0x02,  //00000010
    AS7038_TIA_AMPCAP_3 = 0x03,  //00000011
    AS7038_TIA_AMPCAP_5 = 0x05,  //00000101
    AS7038_TIA_AMPCAP_7 = 0x07,  //00000111
    AS7038_TIA_AMPCAP_10 = 0x0A, //00001010
    AS7038_TIA_AMPCAP_13 = 0x0D, //00001101
    /*
    AS7038_TIA_AMPCAP_20 = 0x00, //0000 Defuq?
    AS7038_TIA_AMPCAP_30 = 0x00, //0000
    AS7038_TIA_AMPCAP_31 = 0x00, //0000
    */
} as7038_tia_ampcap_t;

typedef enum
{
    AS7038_TIA_AMPCOMP_0 = 0x00, //00000000
    AS7038_TIA_AMPCOMP_1 = 0x01, //00000001
    AS7038_TIA_AMPCOMP_3 = 0x03, //00000011
} as7038_tia_ampcomp_t;

typedef enum
{
    AS7038_TIA_AMPVO_15 = 0x3C, //00111100
} as7038_tia_ampvo_t;

typedef enum
{
    // GAIN         V/uA  ACTIVE PD                                     AMPRCCFG | AMPCFG
    AS7038_TIA_GAIN_1VuA_14 = ((AS7038_TIA_AMPRES_1 | AS7038_TIA_AMPCAP_13) << 8) | (AS7038_TIA_AMPCOMP_1 | AS7038_TIA_AMPVO_15),
    AS7038_TIA_GAIN_2VuA_14 = ((AS7038_TIA_AMPRES_2 | AS7038_TIA_AMPCAP_7) << 8) | (AS7038_TIA_AMPCOMP_1 | AS7038_TIA_AMPVO_15),
    AS7038_TIA_GAIN_3VuA_14 = ((AS7038_TIA_AMPRES_3 | AS7038_TIA_AMPCAP_5) << 8) | (AS7038_TIA_AMPCOMP_1 | AS7038_TIA_AMPVO_15),
    AS7038_TIA_GAIN_5VuA_12 = ((AS7038_TIA_AMPRES_4 | AS7038_TIA_AMPCAP_2) << 8) | (AS7038_TIA_AMPCOMP_0 | AS7038_TIA_AMPVO_15),
    AS7038_TIA_GAIN_5VuA_34 = ((AS7038_TIA_AMPRES_4 | AS7038_TIA_AMPCAP_3) << 8) | (AS7038_TIA_AMPCOMP_0 | AS7038_TIA_AMPVO_15),
    AS7038_TIA_GAIN_7VuA_12 = ((AS7038_TIA_AMPRES_5 | AS7038_TIA_AMPCAP_2) << 8) | (AS7038_TIA_AMPCOMP_0 | AS7038_TIA_AMPVO_15),
    AS7038_TIA_GAIN_7VuA_34 = ((AS7038_TIA_AMPRES_5 | AS7038_TIA_AMPCAP_3) << 8) | (AS7038_TIA_AMPCOMP_0 | AS7038_TIA_AMPVO_15),
    AS7038_TIA_GAIN_10VuA_1 = ((AS7038_TIA_AMPRES_6 | AS7038_TIA_AMPCAP_1) << 8) | (AS7038_TIA_AMPCOMP_0 | AS7038_TIA_AMPVO_15),
    AS7038_TIA_GAIN_10VuA_24 = ((AS7038_TIA_AMPRES_6 | AS7038_TIA_AMPCAP_2) << 8) | (AS7038_TIA_AMPCOMP_0 | AS7038_TIA_AMPVO_15),
    AS7038_TIA_GAIN_15VuA_12= ((AS7038_TIA_AMPRES_7 | AS7038_TIA_AMPCAP_1) << 8) | (AS7038_TIA_AMPCOMP_0 | AS7038_TIA_AMPVO_15),
    AS7038_TIA_GAIN_15VuA_34 = ((AS7038_TIA_AMPRES_7 | AS7038_TIA_AMPCAP_2) << 8) | (AS7038_TIA_AMPCOMP_0 | AS7038_TIA_AMPVO_15)
} as7038_tia_gain_t;

typedef enum
{
    AS7038_ADC_CLOCK_1000kHz = 0x00, //00000000
    AS7038_ADC_CLOCK_500kHz = 0x08,  //00001000
    AS7038_ADC_CLOCK_333kHz = 0x10,  //00010000
    AS7038_ADC_CLOCK_250kHz = 0x18,  //00011000
    AS7038_ADC_CLOCK_200kHz = 0x20,  //00100000
    AS7038_ADC_CLOCK_167kHz = 0x28,  //00101000
    AS7038_ADC_CLOCK_143kHz = 0x30,  //00110000
    AS7038_ADC_CLOCK_125kHz = 0x38,  //00111000
} as7038_adc_clock_t;

typedef enum
{
    AS7038_ADC_SETTLING_TIME_0 = 0x00,  // XXXXX000
    AS7038_ADC_SETTLING_TIME_4 = 0x01,  // XXXXX001
    AS7038_ADC_SETTLING_TIME_8 = 0x02,  // XXXXX010
    AS7038_ADC_SETTLING_TIME_16 = 0x03, // XXXXX011
    AS7038_ADC_SETTLING_TIME_32 = 0x04, // XXXXX100
    AS7038_ADC_SETTLING_TIME_64 = 0x05, // XXXXX101
    AS7038_ADC_SETTLING_TIME_128 = 0x06,// XXXXX110
    AS7038_ADC_SETTLING_TIME_256 = 0x07 // XXXXX111
} as7038_adc_settling_time_t;

typedef enum
{
    AS7038_ADC_SAMPLES_1, // 0000
    AS7038_ADC_SAMPLES_2 = 0x00, // 0000
    AS7038_ADC_SAMPLES_4 = 0x02, // 0010
    AS7038_ADC_SAMPLES_8 = 0x04, // 0100
    AS7038_ADC_SAMPLES_16 = 0x06,// 0110
    AS7038_ADC_SAMPLES_32 = 0x08,// 1000
    AS7038_ADC_SAMPLES_48 = 0x0A,// 1010
    AS7038_ADC_SAMPLES_64 = 0x0C,// 1100
    AS7038_ADC_SAMPLES_96 = 0x0E // 1110
} as7038_adc_samples_t;

typedef enum
{
    // MASK_H MASK_L
    AS7038_ADC_CHANNEL_ECGO = 0x0100, 
    AS7038_ADC_CHANNEL_ECGI = 0x0200, 
    AS7038_ADC_CHANNEL_GPIO3 = 0x0400, 
    AS7038_ADC_CHANNEL_GPIO2 = 0x0800, 
    AS7038_ADC_CHANNEL_TIA = 0x0001, 
    AS7038_ADC_CHANNEL_OFE1 = 0x0002, 
    AS7038_ADC_CHANNEL_SD1 = 0x0004, 
    AS7038_ADC_CHANNEL_OFE2 = 0x0008, 
    AS7038_ADC_CHANNEL_SD2 = 0x0010, 
    AS7038_ADC_CHANNEL_TEMP = 0x0020, 
    AS7038_ADC_CHANNEL_AFE = 0x0040, 
    AS7038_ADC_CHANNEL_PREGAIN = 0x0080, 
} as7038_adc_channel_t;

typedef enum
{
    AS7038_PREFILTER_AA_BYPASS = 0x40, // 0x40
    AS7038_PREFILTER_HP_BYPASS = 0x20, // 0x20
    AS7038_PREFILTER_GAIN_BYPASS = 0x10, //0x10
    AS7038_PREFILTER_BYPASS = 0x08,    //0x08
    AS7038_PREFILTER_AA_FILTER = 0x04, //0x04
    AS7038_PREFILTER_HP_FILTER = 0x02, //0x02
    AS7038_PREFILTER_GAIN_STAGE = 0x01 //0x01
} as7038_prefilter_config_t;

typedef enum
{
    AS7038_PREFILTER_AA_10kHz,
    AS7038_PREFILTER_AA_20kHz = 0x08, //00001000
    AS7038_PREFILTER_AA_40kHz = 0x10, //00010000
    AS7038_PREFILTER_AA_60kHz = 0x18, //00011000
}
as7038_prefilter_aa_freq_t;

typedef enum
{
    AS7038_PREFILTER_GAIN_1,
    AS7038_PREFILTER_GAIN_2  = 0x01,
    AS7038_PREFILTER_GAIN_4  = 0x02,
    AS7038_PREFILTER_GAIN_8  = 0x03,
    AS7038_PREFILTER_GAIN_16 = 0x04,
    AS7038_PREFILTER_GAIN_32 = 0x05,
    AS7038_PREFILTER_GAIN_64 = 0x06,
}
as7038_prefilter_gain_t;

typedef enum
{
    AS7038_PD_1 = 0x04,
    AS7038_PD_2 = 0x08,
    AS7038_PD_3 = 0x10,
    AS7038_PD_4 = 0x20,
}as7038_pd_config_t;

typedef enum
{
    AS7038_LED_1=1,
    AS7038_LED_2=2,
    AS7038_LED_3=3,
    AS7038_LED_4=4,
}
as7038_led_t;

typedef enum 
{
    AS7038_LED_ALWAYS_OFF  = 0,
    AS7038_LED_ALWAYS_ON   = 1,
    AS7038_LED_SEQ_CONTROL = 2,
    AS7038_LED_ONLY_EVEN   = 3,
    AS7038_LED_ONLY_UNEVEN = 4,
    AS7038_LED_ONLY_FOURTH = 5,
    AS7038_LED_SEC_TIMING  = 6,
}as7038_led_mode_t;


typedef enum
{
    AS7038_IRQ_VECTOR_VCCLED_LOW = 0x8000,
    AS7038_IRQ_VECTOR_CLIPDETECT = 0x4000,
    AS7038_IRQ_VECTOR_FIFOOVERFLOW = 0x2000,
    AS7038_IRQ_VECTOR_FIFOTHRESHOLD = 0x1000,
    AS7038_IRQ_VECTOR_ADCTHRESHOLD = 0x0800,
    AS7038_IRQ_VECTOR_LTF = 0x0400,
    AS7038_IRQ_VECTOR_SEQ = 0x0200,
    AS7038_IRQ_VECTOR_ADC = 0x0100,
    AS7038_IRQ_VECTOR_LTF1 = 0x0004,
    AS7038_IRQ_VECTOR_LTF0 = 0x0002,
    AS7038_IRQ_VECTOR_ECGTHRESHOLD = 0x0001,
} as7038_irq_vector_t;

typedef enum
{
    AS7038_IRQ_SOURCE_VCCLED_LOW = 0x8000,
    AS7038_IRQ_SOURCE_CLIPDETECT = 0x4000,
    AS7038_IRQ_SOURCE_FIFOOVERFLOW = 0x2000,
    AS7038_IRQ_SOURCE_FIFOTHRESHOLD = 0x1000,
    AS7038_IRQ_SOURCE_ADCTHRESHOLD = 0x0800,
    AS7038_IRQ_SOURCE_LTF = 0x0400,
    AS7038_IRQ_SOURCE_SEQ = 0x0200,
    AS7038_IRQ_SOURCE_ADC = 0x0100,
    AS7038_IRQ_SOURCE_LTFTHRESHOLD_LOW = 0x0004,
    AS7038_IRQ_SOURCE_LTFTHRESHOLD_HIGH = 0x0002,
    AS7038_IRQ_SOURCE_ECGTHRESHOLD = 0x0001,
} as7038_irq_source_t;

typedef enum
{
    AS7038_CHIP_CLOCK_2MHz,
    AS7038_CHIP_CLOCK_1MHz = 0x04,
} as7038_chip_clock_t;

typedef enum
{
    AS7038_I2C_100kHz = 0x00,
    AS7038_I2C_400kHz = 0x10,
} as7038_i2c_speed_t;
