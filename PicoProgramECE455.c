#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define ADS_SPI spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCLK  18
#define PIN_MOSI 19
#define PIN_DRDY 20

// ADS1256 register addresses
#define REG_STATUS  0x00

// ADS1256 commands
#define CMD_RREG    0x10

static void ads_select(void)
{
    gpio_put(PIN_CS, 0);
}

static void ads_deselect(void)
{
    gpio_put(PIN_CS, 1);
}

static uint8_t ads_read_register(uint8_t reg)
{
    uint8_t tx;
    uint8_t rx = 0;

    ads_select();

    // RREG command: 0001 rrrr
    tx = CMD_RREG | (reg & 0x0F);
    spi_write_blocking(ADS_SPI, &tx, 1);

    // Number of registers to read minus 1.
    // 0 = read exactly one register.
    tx = 0x00;
    spi_write_blocking(ADS_SPI, &tx, 1);

    // ADS1256 requires delay t6 before register data appears.
    // Datasheet specifies >= 50 CLKIN periods.
    sleep_us(10);

    // Send dummy byte to generate 8 SPI clocks
    tx = 0xFF;
    spi_write_read_blocking(ADS_SPI, &tx, &rx, 1);

    ads_deselect();

    return rx;
}


int main()
{
    // stdio_init_all();

    // // SPI initialisation. This example will use SPI at 1MHz.
    // spi_init(SPI_PORT, 1000*1000);
    // gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    // gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    // gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    // gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // // Chip select is active-low, so we'll initialise it to a driven-high state
    // gpio_set_dir(PIN_CS, GPIO_OUT);
    // gpio_put(PIN_CS, 1);
    // // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi

    // while (true) {
    //     printf("Hello, world!\n");
    //     sleep_ms(1000);
    // }
    
    stdio_init_all();

    // Give USB serial time to enumerate
    sleep_ms(2000);

    printf("\nADS1256 SPI test\n");

    // Start SPI slowly for initial testing
    spi_init(ADS_SPI, 1000 * 1000);   // 1 MHz

    gpio_set_function(PIN_SCLK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);

    // CS is controlled manually as ordinary GPIO
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    // DRDY is an input
    gpio_init(PIN_DRDY);
    gpio_set_dir(PIN_DRDY, GPIO_IN);
    gpio_pull_up(PIN_DRDY);

    // ADS1256 shifts DIN based on falling edges and changes DOUT
    // relative to rising edges, so use SPI mode 1:
    // CPOL = 0, CPHA = 1
    spi_set_format(
        ADS_SPI,
        8,
        SPI_CPOL_0,
        SPI_CPHA_1,
        SPI_MSB_FIRST
    );

     while (true)
    {
        uint8_t status = ads_read_register(REG_STATUS);

        printf("STATUS = 0x%02X   itDRDY = %d\n",
               status,
               gpio_get(PIN_DRDY));

        sleep_ms(1000);
    }
   
}
