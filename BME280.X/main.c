#include <xc.h>

// PIC18F87J11 Configuration Bit Settings
// CONFIG1L
#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT disabled (control is placed on SWDTEN bit))
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Reset on stack overflow/underflow enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG1H
#pragma config CP0 = OFF        // Code Protection bit (Program memory is not code-protected)

// CONFIG2L
#pragma config FOSC = HSPLL     // Oscillator Selection bits (HS oscillator, PLL enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Two-Speed Start-up (Internal/External Oscillator Switchover) Control bit (Two-Speed Start-up disabled)

// CONFIG2H
#pragma config WDTPS = 32768    // Watchdog Timer Postscaler Select bits (1:32768)

// CONFIG3L
#pragma config EASHFT = ON      // External Address Bus Shift Enable bit (Address shifting enabled, address on external bus is offset to start at 000000h)
#pragma config MODE = MM        // External Memory Bus Configuration bits (Microcontroller mode - External bus disabled)
#pragma config BW = 16          // Data Bus Width Select bit (16-bit external bus mode)
#pragma config WAIT = OFF       // External Bus Wait Enable bit (Wait states on the external bus are disabled)

// CONFIG3H
#pragma config CCP2MX = DEFAULT // ECCP2 MUX bit (ECCP2/P2A is multiplexed with RC1)
#pragma config ECCPMX = DEFAULT // ECCPx MUX bit (ECCP1 outputs (P1B/P1C) are multiplexed with RE6 and RE5; ECCP3 outputs (P3B/P3C) are multiplexed with RE4 and RE3)
#pragma config PMPMX = DEFAULT  // PMP Pin Multiplex bit (PMP port pins connected to EMB (PORTD and PORTE))
#pragma config MSSPMSK = MSK7   // MSSP Address Masking Mode Select bit (7-Bit Address Masking mode enable)

//Project includes
#include "LCD.h"
#include "../src/bme280.h"
#include "../src/bme280_selftest.h"
#include "../src/pic18_i2c.h"
#include <stdio.h>
/*
Connections:
        Master RD5 <-> SDA
        Master RD6 <-> SCL
 */

char lcd[20];
struct bme280_dev dev;

void main(void) {
    OSCTUNEbits.PLLEN = 1;
    TRISDbits.TRISD0 = 0;
    LATDbits.LATD0 = 1;
    LCDInit();
    LCDClear();
    LCDWriteLine("Starting", 0);

    pic18_i2c_enable();
    dev.dev_id = BME280_I2C_ADDR_PRIM;
    dev.intf = BME280_I2C_INTF;
    dev.read = pic18_i2c_read;
    dev.write = pic18_i2c_write;
    dev.delay_ms = pic18_delay_ms;

    char rslt = bme280_crc_selftest(&dev);
    if (rslt != BME280_OK) {
        LCDWriteLine("Self test failed", 1);
        while (1);
    }

    rslt = bme280_init(&dev);
    if (rslt != BME280_OK) {
        LCDWriteLine("Init failed", 1);
        while (1);
    } else {
        LCDWriteLine("Initialized", 1);
        __delay_ms(500);
    }

    uint8_t settings_sel;
    /* Recommended mode of operation: Indoor navigation */
    dev.settings.osr_h = BME280_OVERSAMPLING_1X;
    dev.settings.osr_p = BME280_OVERSAMPLING_16X;
    dev.settings.osr_t = BME280_OVERSAMPLING_2X;
    dev.settings.filter = BME280_FILTER_COEFF_16;
    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;
    rslt = bme280_set_sensor_settings(settings_sel, &dev);
    rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);
    struct bme280_data comp_data;
    while (1) {
        __delay_ms(500);
        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
        sprintf(lcd, "P:%.2f Pa", comp_data.pressure / 100.0);
        LCDClearLine(0);
        LCDWriteLine(lcd, 0);
        sprintf(lcd, "T:%.2fC H:%.1f%%", comp_data.temperature / 100.0, comp_data.humidity / 1024.0);
        LCDClearLine(1);
        LCDWriteLine(lcd, 1);
        LATDbits.LATD0 ^= 1;
    }
}



