/*

Some of the NRF24 BLE code copied/adapted from https://github.com/kasparsd/tinystone/blob/master/main.c ,
which in turn has been mostly copied from http://dmitry.gr/?r=05.Projects&proj=11.%20Bluetooth%20LE%20fakery ,
which says:
"All the code as well as the research that went into this and is published here is under this license:
you may use it in any way you please if and only if it is for non-commercial purposes,
you must provide a link to this page as well. Any commercial use must be discussed with me."(http://dmitry.gr/)

All other written by me is public domain. Use at your own risk.
*/
/*
  Connections:
  
  nRF24:
  CSN <-> A3
  CE <-> D2
  SCK <-> C5
  MOSI <-> C6
  MISO <-> C7

  DS18b20 data <->  B4
 */

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "stm8.h"

// Could not be bothered to create ds18b20.h...
extern void ds18b20_init();
extern long ds18b20_read();

/* nRF24 Definitions */
// Commands
#define NRF24_R_REGISTER    0x00
#define NRF24_W_REGISTER    (1 << 5)
#define NRF24_R_RX_PAYLOAD  0x61
#define NRF24_W_TX_PAYLOAD  0xA0
#define NRF24_FLUSH_TX      0xE1
#define NRF24_FLUSH_RX      0xE2
#define NRF24_REUSE_TX_PL   0xE3
#define NRF24_R_RX_PL_WID   0x60
#define NRF24_W_ACK_PAYLOAD 0xA8
#define NRF24_W_TX_PAYLOAD_NOACK 0x58
#define NRF24_NOP                0xFF

// Registers
#define NRF24_CONFIG      0x00
#define NRF24_EN_AA       0x01
#define NRF24_EN_RXADDR   0x02
#define NRF24_SETUP_AW    0x03
#define NRF24_SETUP_RETR  0x04
#define NRF24_RF_CH       0x05
#define NRF24_RF_SETUP    0x06
#define NRF24_STATUS      0x07
#define NRF24_OBSERVE_TX  0x08
#define NRF24_CD          0x09
#define NRF24_RX_ADDR_P0  0x0A
#define NRF24_RX_ADDR_P1  0x0B
#define NRF24_RX_ADDR_P2  0x0C
#define NRF24_RX_ADDR_P3  0x0D
#define NRF24_RX_ADDR_P4  0x0E
#define NRF24_RX_ADDR_P5  0x0F
#define NRF24_TX_ADDR     0x10
#define NRF24_RX_PW_P0    0x11
#define NRF24_RX_PW_P1    0x12
#define NRF24_RX_PW_P2    0x13
#define NRF24_RX_PW_P3    0x14
#define NRF24_RX_PW_P4    0x15
#define NRF24_RX_PW_P5    0x16
#define NRF24_FIFO_STATUS 0x17
#define NRF24_DYNPD       0x1C
#define NRF24_FEATURE     0x1D

/* Build in LED is in pin B5 (STM8S103 board) */
#define LED_PORT    PB
#define LED_PIN     PIN5


volatile static unsigned long seconds = 0;  // Time since boot
/* TIM1 Update/Overflow interrupt handling routine */
void TIM1_update(void) __interrupt(TIM1_OVR_UIF_IRQ) {
    ++seconds; // global second count

    // Blink internal LED. Port B output data register. Flip LED pin
    PORT(LED_PORT, ODR) ^= LED_PIN;

    // Clear Timer Status Register 1 Update Interrupt Flag (UIF) (bit 0)
    TIM1_SR1 &= ~TIM_SR1_UIF;
}


void SPI_init() {
    // CE
    PD_DDR |= PIN2;
    PD_CR1 |= PIN2;
    PD_ODR &= ~PIN2; // CE low

    /* Initialize CS pin */
    PA_DDR |= (1 << 3);
    PA_CR1 |= (1 << 3);
    PA_ODR |= (1 << 3);
    /* Initialize SPI master at 500kHz  */
    SPI_CR2 = SPI_CR2_SSM | SPI_CR2_SSI;
    SPI_CR1 = SPI_CR1_MSTR | SPI_CR1_SPE | SPI_CR1_BR(1);
}

void SPI_write(uint8_t data) {
    SPI_DR = data;
    while (!(SPI_SR & SPI_SR_TXE));
    data = SPI_DR;
}

uint8_t SPI_read() {
    SPI_write(0xFF);
    while (!(SPI_SR & SPI_SR_RxNE));
    return SPI_DR;
}

static void csn_low() {
    PA_ODR &= ~PIN3;
}

static void csn_high() {
    while ((SPI_SR & SPI_SR_BSY));
    PA_ODR |= PIN3;
}

void nrf24_write_command(uint8_t command) {
    csn_low();
    SPI_write(command);
    csn_high();
}

void nrf24_write_payload(const uint8_t *data, uint8_t length) {
    uint8_t i;
    csn_low();
    SPI_write(NRF24_W_TX_PAYLOAD);
    for (i=0; i<length; i++)
        SPI_write(data[i]);
    csn_high();
}

uint8_t nrf24_read_register(uint8_t reg) {
    uint8_t regval=0;
    csn_low();
    SPI_write(reg);
    regval = SPI_read();
    csn_high();
    return regval;
}

void nrf24_write_register_bytes(uint8_t reg, const uint8_t *values, uint8_t length) {
    uint8_t i;
    csn_low();
    SPI_write(NRF24_W_REGISTER | reg);
    for (i=0; i<length; i++) {
        SPI_write(values[i]);
    }
    csn_high();
}

void nrf24_read_register_bytes(uint8_t reg, uint8_t *values, uint8_t length) {
    uint8_t i;
    csn_low();
    SPI_write(reg);
    for (i=0; i<length; i++) {
        values[i] = SPI_read();
    }
    csn_high();
}

void nrf24_write_register(uint8_t cmd, uint8_t data) {
    nrf24_write_register_bytes(cmd, &data, 1);
}


void btLeCrc(const uint8_t* data, uint8_t len, uint8_t* dst)
{
    uint8_t v, t, d;

    while(len--){
        d = *data++;
        for(v = 0; v < 8; v++, d >>= 1){
            t = dst[0] >> 7;
            dst[0] <<= 1;

            if(dst[1] & 0x80)
                dst[0] |= 1;

            dst[1] <<= 1;

            if(dst[2] & 0x80)
                dst[1] |= 1;

            dst[2] <<= 1;

            if(t != (d & 1)){
                dst[2] ^= 0x5B;
                dst[1] ^= 0x06;
            }
        }
    }
}

uint8_t swapbits(uint8_t a)
{
    uint8_t v = 0;

    if(a & 0x80) v |= 0x01;
    if(a & 0x40) v |= 0x02;
    if(a & 0x20) v |= 0x04;
    if(a & 0x10) v |= 0x08;
    if(a & 0x08) v |= 0x10;
    if(a & 0x04) v |= 0x20;
    if(a & 0x02) v |= 0x40;
    if(a & 0x01) v |= 0x80;

    return v;
}

void btLeWhiten(uint8_t* data, uint8_t len, uint8_t whitenCoeff)
{
    uint8_t  m;

    while(len--){
        for(m = 1; m; m <<= 1){
            if(whitenCoeff & 0x80){
                whitenCoeff ^= 0x11;
                (*data) ^= m;
            }
            whitenCoeff <<= 1;
        }
        data++;
    }
}

static inline uint8_t btLeWhitenStart(uint8_t chan)
{
    //the value we actually use is what BT'd use left shifted one...makes our life easier
    return swapbits(chan) | 2;
}

void btLePacketEncode(uint8_t* packet, uint8_t len, uint8_t chan)
{
    //length is of packet, including crc. pre-populate crc in packet with initial crc value!
    uint8_t i, dataLen = len - 3;

    btLeCrc(packet, dataLen, packet + dataLen);

    for(i = 0; i < 3; i++, dataLen++)
        packet[dataLen] = swapbits(packet[dataLen]);

    btLeWhiten(packet, len, btLeWhitenStart(chan));

    for(i = 0; i < len; i++)
        packet[i] = swapbits(packet[i]);
}

// Construct BLE advertisement header
static int encode_BLE_header(uint8_t *buf) {
    // ADV_NONCONN_IND packet: 2 byte header + 6 byte address + 3 byte LE header
    uint8_t L = 0;

    buf[L++] = 0x42; // PDU type, ADV_NONCONN_IND
    buf[L++] = 0; // Payload length, we need to update this later with the correct value

    // BLE address / "MAC". This needs to be unique among your devices. Change this!!!!!
    buf[L++] = 10;
    buf[L++] = 11;
    buf[L++] = 12;
    buf[L++] = 13;
    buf[L++] = 14;
    buf[L++] = 15;

    // Advertisement Flags. CSS v5, Part A, § 1.3
    buf[L++] = 0x02; // Length
    buf[L++] = 0x01; // Flags data type value
    buf[L++] = 0x05; // Flags data

    return L;
}

// Construct Eddystone URL frame. In this example URL is https://www.wikipedia.org
// See https://github.com/google/eddystone/tree/master/eddystone-url
static uint8_t encode_ES_URL(uint8_t *buf) {
    uint8_t L = encode_BLE_header(buf);

    buf[L++] = 16; // Length of data (max 17)
    buf[L++] = 0x16; // Data type "Service Data"
    buf[L++] = 0xAA; // First part of the Eddystone Service UUID of 0xFEAA
    buf[L++] = 0xFE; // Second part of the Eddystone Service UUID of 0xFEAA
    buf[L++] = 0x10; // Frame Type: Eddystone-URL
    buf[L++] = 0xEF; // TX Power
    buf[L++] = 0x01; // URL Scheme, https://www.
    buf[L++] = 'w';
    buf[L++] = 'i';
    buf[L++] = 'k';
    buf[L++] = 'i';
    buf[L++] = 'p';
    buf[L++] = 'e';
    buf[L++] = 'd';
    buf[L++] = 'i';
    buf[L++] = 'a';
    buf[L++] = 0x01; // Domain extension, .org

    return L;
}

// Construct Eddystone Unencrypted TLM frame. Include our DS18B20 temperature, beacon count and seconds since boot
// Battery level not supported
// See https://github.com/google/eddystone/blob/master/eddystone-tlm/tlm-plain.md
static uint8_t encode_ES_TLM(uint8_t *buf, int16_t temperature_100, unsigned long count, uint32_t time) {
    uint8_t L = encode_BLE_header(buf);

    int16_t temperature = (long)256*temperature_100 / 100; // Encode as needed in Eddystone-tlm

    if (temperature_100 == -9999) { // No temperature
        temperature = 0x8000;
    }

    time = time*10;

    buf[L++] = 0x11; // Length of data
    buf[L++] = 0x16; // Data type "Service Data"
    buf[L++] = 0xAA; // First part of the Eddystone Service UUID of 0xFEAA
    buf[L++] = 0xFE; // Second part of the Eddystone Service UUID of 0xFEAA
    buf[L++] = 0x20; // Frame Type: Eddystone-TLM Unencrypted
    buf[L++] = 0x00; // TLM version
    buf[L++] = 0x00; // Battery voltage, 1 mV/bit // 0 means not available
    buf[L++] = 0x00;
    buf[L++] = (temperature >> 8) & 0xff; // Beacon temperature
    buf[L++] = temperature & 0xff;
    buf[L++] = (count >> 24) & 0xff; // Advertising PDU count
    buf[L++] = (count >> 16) & 0xff;
    buf[L++] = (count >> 8) & 0xff;
    buf[L++] = count & 0xff;
    buf[L++] = (time >> 24) & 0xff; // Time since power-on or reboot
    buf[L++] = (time >> 16) & 0xff;
    buf[L++] = (time >> 8) & 0xff;
    buf[L++] = time & 0xff;

    return L;
}


int main(void)
{

    /* Set clock to full speed (16 Mhz) */
    CLK_CKDIVR = 0;

    // Setup internal LED
    PORT(LED_PORT, DDR)  |= LED_PIN;
    PORT(LED_PORT, CR1)  |= LED_PIN;

    SPI_init();

    // Map nRF to BLE address space
    static const uint8_t chLe[] = {37,38,39}; // 2402 MHz, 2426 MHz, and 2480 MHz
    static const uint8_t chRf[] = {2,26,80}; // F0 = 2400 + RF_CH [MHz]

    uint8_t ch = 0, plen = 0;
    uint8_t buf[32];

    // nRF24L01+ must be in a standby or power down mode before writing to the configuration registers.

    // CONFIG: MASK_RX_DR=0, MASK_TX_DS=0, MASK_MAX_RT=1, EN_CRC=0, EN_CRC=0, PWR_UP=1, PRIM_RX=0
    // Reflect TX_DS as active low interrupt on the IRQ pin
    nrf24_write_register(NRF24_CONFIG, 0x12);

    // EN_AA: no auto-acknowledge
    nrf24_write_register(NRF24_EN_AA, 0x00);

    // EN_RXADDR: RX on pipe 0
    nrf24_write_register(NRF24_EN_RXADDR, 0x00);

    // SETUP_AW: use a 4 byte address (default is 5 bytes)
    nrf24_write_register(NRF24_SETUP_AW, 0x02);

    // SETUP_RETR: no auto-retransmit (ARC=0)
    nrf24_write_register(NRF24_SETUP_RETR, 0x00);

    // RF_SETUP: 1MBps at 0dBm (11.3mA)
    nrf24_write_register(NRF24_RF_SETUP, 0x06);

    // STATUS: Clear TX_DS and MAX_RT
    nrf24_write_register(NRF24_STATUS, 0x3E);

    // Set TX_ADDR address to 0x8E89BED6 or "Bed6" for BLE advertising packets
    // Note that the preamble is set automatically and using the same logic as BLE
    buf[0] = swapbits(0x8E);
    buf[1] = swapbits(0x89);
    buf[2] = swapbits(0xBE);
    buf[3] = swapbits(0xD6);
    nrf24_write_register_bytes(NRF24_TX_ADDR, buf, 4);



    /* TIM1 setup: Generate interrupt every 1000ms */
    TIM1_CR1 = 0;
    // Just for fun, let's count down (TIM1 can do that)
    TIM1_CR1 |= TIM_CR1_DIR;
    // Prescaler: divide clock with 16000 (0x3E7F + 1) (to 1ms)
    TIM1_PSCRH = 0x0E;
    TIM1_PSCRL = 0x7F;
    // Auto-reload registers. Count to 1000 (0x03E8)
    TIM1_ARRH = 0x13;
    TIM1_ARRL = 0xE8;
    // TIM1_IER (Interrupt Enable Register), Update interrupt (UIE) (bit 0)
    TIM1_IER |= TIM_IER_UIE;
    // TIM1_CR1 – Timer Control Register 1, Counter ENable bit (CEN) (bit 0)
    TIM1_CR1 |= TIM_CR1_CEN;

    ds18b20_init(); // Setups timer needed by ds18b20

    int16_t temperature_100 = -9999;
    unsigned long pdu_count = 0;
    while(1) {
        pdu_count++;

        if (ch == 0) {
            temperature_100 = ds18b20_read();
        }

        uint8_t L;
        if (seconds % 2) { // Every other second send URL or TLM
            L = encode_ES_TLM(buf, temperature_100, pdu_count, seconds);
        } else {
            L = encode_ES_URL(buf);
        }
        // CRC start value: 0x555555
        buf[L++] = 0x55;
        buf[L++] = 0x55;
        buf[L++] = 0x55;

        // Update the packet length
        buf[1] = L-2-3; // subtract PDU-type,length and crc

        // Add CRC and whitten the packet
        btLePacketEncode(buf, L, chLe[ch]);


        nrf24_write_register(NRF24_RF_CH, chRf[ch]); // set the channel, loop through chRf
        nrf24_write_register(NRF24_STATUS, 0x6E); // clear flags

        // Clear RX, TX FIFO before writing the payload
        nrf24_write_command(NRF24_FLUSH_TX);
        nrf24_write_command(NRF24_FLUSH_RX);

        // Write payload to the module
        nrf24_write_payload(buf, L);

        // Toggle CE to send the buffer
        uint32_t delay = 15; // experiment this?? In my case >8 works, 7 does not work!
        PD_ODR |= PIN2; // CE high
        while(delay--);
        PD_ODR &= ~PIN2; // CE low

        // Channel hopping
        if (++ch == sizeof(chRf))
        {
            ch = 0;
            wfi(); // After blasting all channels, wait till next second
        }
    }
}
