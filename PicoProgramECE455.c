#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"

// ============================================================
// SPI pin definitions
// ============================================================

#define ADS_SPI  spi0

#define PIN_MISO 16     // ADS1256 DOUT
#define PIN_CS   17
#define PIN_SCLK 18
#define PIN_MOSI 19     // ADS1256 DIN
#define PIN_DRDY 20


// ============================================================
// ADS1256 register addresses
// ============================================================

#define REG_STATUS  0x00
#define REG_MUX     0x01
#define REG_ADCON   0x02
#define REG_DRATE   0x03
#define REG_IO      0x04

#define REG_OFC0    0x05
#define REG_OFC1    0x06
#define REG_OFC2    0x07

#define REG_FSC0    0x08
#define REG_FSC1    0x09
#define REG_FSC2    0x0A


// ============================================================
// ADS1256 commands
// ============================================================

#define CMD_WAKEUP   0x00
#define CMD_RDATA    0x01
#define CMD_RDATAC   0x03
#define CMD_SDATAC   0x0F

#define CMD_RREG     0x10
#define CMD_WREG     0x50

#define CMD_SELFCAL  0xF0

#define CMD_SYNC     0xFC
#define CMD_STANDBY  0xFD
#define CMD_RESET    0xFE


// ============================================================
// Our ADS1256 configuration
// ============================================================

// STATUS:
// bit 3 ORDER = 0 -> MSB first
// bit 2 ACAL  = 0 -> auto calibration OFF
// bit 1 BUFEN = 1 -> analog buffer ON
#define ADS_STATUS_CONFIG  0x02

// MUX:
// Positive input = AIN1
// Negative input = AIN2
//
// upper nibble = 0001 = AIN1
// lower nibble = 0010 = AIN2
#define ADS_MUX_CONFIG     0x12

// ADCON:
// CLKOUT = OFF
// Sensor detect = OFF
// PGA = 64 -> 110
#define ADS_ADCON_CONFIG   0x06

// 15,000 samples/second
#define ADS_DRATE_CONFIG   0xE0

// Leave GPIO configuration at reset value
#define ADS_IO_CONFIG      0xE0


// ============================================================
// ADC scaling
// ============================================================

#define ADS_VREF   2.5
#define ADS_PGA    64.0

// Positive full-scale code = 2^23 - 1
#define ADS_MAX_CODE 8388607.0


// ============================================================
// Chip select
// ============================================================

static void ads_select(void)
{
    gpio_put(PIN_CS, 0);
}

static void ads_deselect(void)
{
    gpio_put(PIN_CS, 1);
}


// ============================================================
// Wait for DRDY LOW
//
// DRDY LOW = new ADC conversion is ready
// ============================================================

static bool ads_wait_drdy(uint32_t timeout_ms)
{
    uint64_t timeout =
        time_us_64() + ((uint64_t)timeout_ms * 1000);

    while (gpio_get(PIN_DRDY))
    {
        if (time_us_64() >= timeout)
        {
            return false;
        }

        tight_loop_contents();
    }

    return true;
}


// ============================================================
// Send simple one-byte ADS1256 command
// ============================================================

static void ads_send_command(uint8_t command)
{
    ads_select();

    spi_write_blocking(
        ADS_SPI,
        &command,
        1
    );

    ads_deselect();

    // Small command spacing margin
    sleep_us(2);
}


// ============================================================
// Read one ADS1256 register
// ============================================================

static uint8_t ads_read_register(uint8_t reg)
{
    uint8_t tx;
    uint8_t rx = 0;

    ads_select();

    // RREG command
    //
    // 0001 rrrr
    tx = CMD_RREG | (reg & 0x0F);

    spi_write_blocking(
        ADS_SPI,
        &tx,
        1
    );

    // Number of registers to read - 1
    //
    // 0 = read one register
    tx = 0x00;

    spi_write_blocking(
        ADS_SPI,
        &tx,
        1
    );

    // t6 >= 50 CLKIN periods.
    //
    // At 7.68 MHz:
    // 50 / 7.68 MHz = 6.51 us
    //
    // 10 us gives us some margin.
    sleep_us(10);

    // Send dummy byte to provide clocks
    tx = 0xFF;

    spi_write_read_blocking(
        ADS_SPI,
        &tx,
        &rx,
        1
    );

    ads_deselect();

    return rx;
}


// ============================================================
// Write one ADS1256 register
// ============================================================

static void ads_write_register(uint8_t reg, uint8_t value)
{
    uint8_t tx[3];

    // WREG command:
    // 0101 rrrr
    tx[0] = CMD_WREG | (reg & 0x0F);

    // Number of registers to write - 1
    //
    // 0 = write exactly one register
    tx[1] = 0x00;

    // New register value
    tx[2] = value;

    ads_select();

    spi_write_blocking(
        ADS_SPI,
        tx,
        3
    );

    ads_deselect();

    // Give the ADC time between commands
    sleep_us(2);
}


// ============================================================
// Configure ADS1256
// ============================================================

static void ads_configure(void)
{
    printf("\nConfiguring ADS1256...\n");

    // ACAL is OFF here.
    //
    // We don't want it recalibrating every time we change
    // PGA, data rate, or buffer state.
    ads_write_register(
        REG_STATUS,
        ADS_STATUS_CONFIG
    );

    // Measure:
    //
    // AIN1 - AIN2
    ads_write_register(
        REG_MUX,
        ADS_MUX_CONFIG
    );

    // PGA = 64
    ads_write_register(
        REG_ADCON,
        ADS_ADCON_CONFIG
    );

    // 15,000 SPS
    ads_write_register(
        REG_DRATE,
        ADS_DRATE_CONFIG
    );

    // GPIO config
    ads_write_register(
        REG_IO,
        ADS_IO_CONFIG
    );
}


// ============================================================
// Print configuration registers
// ============================================================

static void ads_print_registers(void)
{
    printf("\nADS1256 registers:\n");

    printf("STATUS = 0x%02X\n",
           ads_read_register(REG_STATUS));

    printf("MUX    = 0x%02X\n",
           ads_read_register(REG_MUX));

    printf("ADCON  = 0x%02X\n",
           ads_read_register(REG_ADCON));

    printf("DRATE  = 0x%02X\n",
           ads_read_register(REG_DRATE));

    printf("IO     = 0x%02X\n",
           ads_read_register(REG_IO));

    printf("OFC0   = 0x%02X\n",
           ads_read_register(REG_OFC0));

    printf("OFC1   = 0x%02X\n",
           ads_read_register(REG_OFC1));

    printf("OFC2   = 0x%02X\n",
           ads_read_register(REG_OFC2));

    printf("FSC0   = 0x%02X\n",
           ads_read_register(REG_FSC0));

    printf("FSC1   = 0x%02X\n",
           ads_read_register(REG_FSC1));

    printf("FSC2   = 0x%02X\n",
           ads_read_register(REG_FSC2));
}


// ============================================================
// Perform self calibration
// ============================================================

static bool ads_calibrate(void)
{
    printf("\nStarting SELFCAL...\n");

    // Make absolutely sure auto-calibration is disabled.
    //
    // STATUS =:
    // ORDER = 0
    // ACAL  = 0
    // BUFEN = 1
    ads_write_register(
        REG_STATUS,
        ADS_STATUS_CONFIG
    );

    // Self-calibration:
    //
    // performs both offset and gain calibration
    ads_send_command(CMD_SELFCAL);

    // Calibration causes DRDY to go HIGH.
    //
    // When calibration is finished AND valid data is
    // available again, DRDY goes LOW.
    //
    // Give it a moment to enter calibration before
    // checking DRDY.
    sleep_us(20);

    if (!ads_wait_drdy(100))
    {
        printf("ERROR: calibration timed out!\n");
        return false;
    }

    // ACAL was never enabled, but explicitly write STATUS
    // again to make our final state obvious.
    ads_write_register(
        REG_STATUS,
        ADS_STATUS_CONFIG
    );

    printf("Calibration complete.\n");

    return true;
}


// ============================================================
// Read one 24-bit ADC conversion
// ============================================================

static bool ads_read_data(
    int32_t *signed_code,
    uint32_t *raw_code
)
{
    // Wait until conversion data is ready
    if (!ads_wait_drdy(100))
    {
        return false;
    }

    uint8_t command = CMD_RDATA;
    uint8_t rx[3];

    ads_select();

    // Tell ADC we want the current conversion result
    spi_write_blocking(
        ADS_SPI,
        &command,
        1
    );

    // ADS1256 requires t6 delay before data clocks.
    sleep_us(10);

    // Clock out 24 bits:
    //
    // rx[0] = MSB
    // rx[1] = middle byte
    // rx[2] = LSB
    spi_read_blocking(
        ADS_SPI,
        0xFF,
        rx,
        3
    );

    ads_deselect();

    // Combine three bytes into one 24-bit value
    uint32_t raw =
        ((uint32_t)rx[0] << 16) |
        ((uint32_t)rx[1] << 8)  |
        ((uint32_t)rx[2]);

    // --------------------------------------------------------
    // Convert 24-bit two's-complement to signed 32-bit integer
    // --------------------------------------------------------

    int32_t code;

    if (raw & 0x800000)
    {
        // Negative number:
        //
        // Extend bit 23 through bits 24-31
        code = (int32_t)(raw | 0xFF000000);
    }
    else
    {
        code = (int32_t)raw;
    }

    *raw_code = raw;
    *signed_code = code;

    return true;
}


// ============================================================
// Convert ADC code to differential voltage
//
// Vin = AIN1 - AIN2
// ============================================================

static double ads_code_to_voltage(int32_t code)
{
    return ((double)code * (2.0 * ADS_VREF)) /
           (ADS_PGA * ADS_MAX_CODE);
}


// ============================================================
// Main
// ============================================================

int main()
{
    stdio_init_all();

    // Give USB serial time to enumerate
    sleep_ms(2000);

    printf("\n");
    printf("===============================\n");
    printf(" ADS1256 Magnetometer Test\n");
    printf("===============================\n");


    // --------------------------------------------------------
    // SPI setup
    // --------------------------------------------------------

    spi_init(
        ADS_SPI,
        1000 * 1000
    );      // 1 MHz SPI for initial testing

    gpio_set_function(
        PIN_SCLK,
        GPIO_FUNC_SPI
    );

    gpio_set_function(
        PIN_MOSI,
        GPIO_FUNC_SPI
    );

    gpio_set_function(
        PIN_MISO,
        GPIO_FUNC_SPI
    );


    // --------------------------------------------------------
    // Chip select
    // --------------------------------------------------------

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);

    // CS inactive HIGH
    gpio_put(PIN_CS, 1);


    // --------------------------------------------------------
    // DRDY input
    // --------------------------------------------------------

    gpio_init(PIN_DRDY);
    gpio_set_dir(PIN_DRDY, GPIO_IN);
    gpio_pull_up(PIN_DRDY);


    // --------------------------------------------------------
    // SPI mode
    //
    // ADS1256:
    //
    // DIN sampled on falling SCLK edge
    // DOUT changes on rising SCLK edge
    //
    // SPI Mode 1:
    //
    // CPOL = 0
    // CPHA = 1
    // --------------------------------------------------------

    spi_set_format(
        ADS_SPI,
        8,
        SPI_CPOL_0,
        SPI_CPHA_1,
        SPI_MSB_FIRST
    );


    // Allow ADC/reference to settle
    sleep_ms(100);


    // --------------------------------------------------------
    // Read initial STATUS
    // --------------------------------------------------------

    uint8_t initial_status =
        ads_read_register(REG_STATUS);

    printf(
        "Initial STATUS = 0x%02X\n",
        initial_status
    );

    printf(
        "Device ID nibble = 0x%X\n",
        initial_status >> 4
    );


    // --------------------------------------------------------
    // Configure ADC
    // --------------------------------------------------------

    ads_configure();


    // --------------------------------------------------------
    // Print settings before our explicit calibration
    // --------------------------------------------------------

    printf("\nBefore SELFCAL:");
    ads_print_registers();


    // --------------------------------------------------------
    // Calibrate
    // --------------------------------------------------------

    if (!ads_calibrate())
    {
        printf("Calibration failed.\n");

        while (true)
        {
            sleep_ms(1000);
        }
    }


    // --------------------------------------------------------
    // Print registers after calibration
    //
    // OFC and FSC contain the calibration coefficients.
    // --------------------------------------------------------

    printf("\nAfter SELFCAL:");
    ads_print_registers();


    // --------------------------------------------------------
    // Confirm ACAL is disabled
    // --------------------------------------------------------

    uint8_t status =
        ads_read_register(REG_STATUS);

    printf(
        "\nACAL bit = %d\n",
        (status >> 2) & 0x01
    );

    printf(
        "BUFEN bit = %d\n",
        (status >> 1) & 0x01
    );

    printf("\nBeginning ADC reads...\n\n");


    // --------------------------------------------------------
    // Read ADC continuously
    // --------------------------------------------------------

    while (true)
    {
        int32_t code;
        uint32_t raw;

        if (ads_read_data(&code, &raw))
        {
            double voltage =
                ads_code_to_voltage(code);

            printf(
                "RAW=0x%06lX   CODE=%ld   Vin=%+.6f V   %+.3f mV\n",
                (unsigned long)raw,
                (long)code,
                voltage,
                voltage * 1000.0
            );
        }
        else
        {
            printf("DRDY timeout!\n");
        }

        // IMPORTANT:
        //
        // This is only for slow human-readable testing.
        //
        // Remove this delay when you start doing real
        // 15 kSPS waveform acquisition.
        sleep_ms(100);
    }
}