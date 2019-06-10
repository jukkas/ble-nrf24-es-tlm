#include <stdint.h>
#include "stm8.h"

/* 1-Wire (DS18B20 data) pin */
#define OW_PORT PB
#define OW_PIN  PIN4


static void delay_us(uint16_t i) {
    if (i < 9) { // FIXME: Really ugly
        nop();
        return;
    }
    TIM2_CNTRH = 0;
    TIM2_CNTRL = 0;
    TIM2_EGR = 0x01; // Update Generation
    while(1) {
        volatile uint16_t counter = (((TIM2_CNTRH) << 8) | TIM2_CNTRL);
        if (i-6 < counter)
            return;
    }
}

#define OW_INPUT_MODE()     PORT(OW_PORT,DDR) &= ~OW_PIN
#define OW_OUTPUT_MODE()    PORT(OW_PORT,DDR) |= OW_PIN
#define OW_LOW()            PORT(OW_PORT,ODR) &= ~OW_PIN
#define OW_HIGH()           PORT(OW_PORT,ODR) |= OW_PIN
#define OW_READ()           (PORT(OW_PORT,IDR) & OW_PIN)

static void ow_pull_low(unsigned int us) {
    OW_OUTPUT_MODE();
    OW_LOW();
    delay_us(us);
    OW_INPUT_MODE();
}

static void ow_write_byte(uint8_t out) {
    uint8_t i;
    for (i=0; i < 8; i++) {
        if ( out & ((uint8_t)1<<i) ) {
            // write 1
            ow_pull_low(1);
            delay_us(60);
        } else {
            // write 0
            ow_pull_low(60);
            delay_us(1);
        }
    }
}

static uint8_t ow_read_byte() {
    uint8_t val = 0;
    uint8_t i;
    for (i=0; i < 8; i++) {
        ow_pull_low(1);
        delay_us(5);
        if (OW_READ()) {
            val |= ((uint8_t)1<<i);
        }
        delay_us(55);
    }
    return val;
}

static unsigned int ow_init() {

    uint8_t input;

    ow_pull_low(480);
    delay_us(60);

    input = !OW_READ();
    delay_us(420);

    return input;
}

static unsigned int ow_convert_temperature() {
    int cycles = 1; // For debugging purposes

    ow_write_byte(0x44); // Convert Temperature

    while (1) {
        ow_pull_low(1);
        delay_us(5);
        if (OW_READ()) {
            return cycles;
        }
        delay_us(55);
        cycles++;
    }
}

// Returns temperature*100
static long decode_ds_temperature(uint8_t scratchpad1, uint8_t scratchpad0) {

    /*
    Temperature Register Format:
            Bit7 Bit6 Bit5 Bit4 Bit3 Bit2 Bit1 Bit0
    LS byte:   2^3  2^2  2^1  2^0  2^-1 2^-2 2^-3 2^-4    Scratchpad[0]
    MS byte:    S    S    S    S    S   2^6  2^5  2^4     Scratchpad[1]
    */
    int16_t t_raw = ((int16_t)scratchpad1 << 8) | (int16_t)scratchpad0;
    long t_100 = (long)t_raw * 625 / 100;

    return t_100;
}

// Reads temperature
// returns: temperature*100
long ds18b20_read() {
    uint8_t i;
    uint8_t scratchpad[9];

    if (ow_init()) {
        ow_write_byte(0xcc); // Skip ROM
        ow_convert_temperature();

        ow_init();
        ow_write_byte(0xcc); // Skip ROM
        ow_write_byte(0xbe); // Read Scratchpad
        for (i=0; i<9; i++) {
            scratchpad[i] = ow_read_byte();
        }

        return decode_ds_temperature(scratchpad[1], scratchpad[0]);

    } else {
        /* DS18B20 was not detected */
        return -9999;
    }
}


void ds18b20_init() {
    // Timer2 setup (for ds18b20 delay_us)
    TIM2_PSCR = 0x4; // Prescaler: to 1MHz
    TIM2_CR1 |= TIM_CR1_CEN; // Start timer
}
