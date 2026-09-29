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
// ADS1256 configuration
// ============================================================

// STATUS:
// ORDER = 0 -> MSB first
// ACAL  = 0 -> automatic calibration OFF
// BUFEN = 1 -> analog input buffer ON
#define ADS_STATUS_CONFIG  0x02


// MUX = 0x10:
//
// Positive input = AIN1
// Negative input = AIN0
//
// ADC measures:
//
//      AIN1 - AIN0
//
#define ADS_MUX_CONFIG     0x10


// ADCON:
//
// CLKOUT = OFF
// Sensor detect = OFF
// PGA = 64
#define ADS_ADCON_CONFIG   0x06


// DRATE:
//
// 0xE0 = nominal 15,000 samples/sec
// assuming the ADS1256 has the expected clock.
#define ADS_DRATE_CONFIG   0xE0


// GPIO register
#define ADS_IO_CONFIG      0xE0


// ============================================================
// ADC scaling
// ============================================================

// Your currently calibrated/measured reference value.
//
// Change this if you later determine a more accurate
// VREFP - VREFN value.
#define ADS_VREF           2.05

#define ADS_PGA            64.0

#define ADS_MAX_CODE       8388607.0


// ============================================================
// Sampling configuration
// ============================================================

// This is only the NOMINAL sample rate.
//
// We no longer use this number to calculate frequency.
//
// The actual sample timing is measured with the Pico clock.
#define ADS_NOMINAL_SAMPLE_RATE  15000.0


// Capture 3000 consecutive samples.
//
// At about 15 kSPS this is approximately 0.2 seconds.
#define NUM_SAMPLES 15000


// ============================================================
// Sample storage
// ============================================================

// ADC conversion codes
static int32_t sample_buffer[NUM_SAMPLES];

// Timestamp of each DRDY event in microseconds
static uint64_t time_buffer[NUM_SAMPLES];


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
// DRDY LOW = new conversion data is ready
// ============================================================

static bool ads_wait_drdy(uint32_t timeout_ms)
{
    uint64_t timeout =
        time_us_64() +
        ((uint64_t)timeout_ms * 1000);

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
// Send one ADS1256 command
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


    // RREG command:
    //
    // 0001 rrrr
    tx = CMD_RREG | (reg & 0x0F);

    spi_write_blocking(
        ADS_SPI,
        &tx,
        1
    );


    // Number of registers to read minus 1.
    //
    // 0 = one register
    tx = 0x00;

    spi_write_blocking(
        ADS_SPI,
        &tx,
        1
    );


    // ADS1256 t6 delay
    sleep_us(10);


    // Dummy byte to generate 8 SPI clocks
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

static void ads_write_register(
    uint8_t reg,
    uint8_t value
)
{
    uint8_t tx[3];


    // WREG command:
    //
    // 0101 rrrr
    tx[0] =
        CMD_WREG |
        (reg & 0x0F);


    // Number of registers to write minus 1.
    //
    // 0 = one register
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


    sleep_us(2);
}


// ============================================================
// Configure ADS1256
// ============================================================

static void ads_configure(void)
{
    printf("\nConfiguring ADS1256...\n");


    // ACAL OFF
    // Buffer ON
    ads_write_register(
        REG_STATUS,
        ADS_STATUS_CONFIG
    );


    // Measure:
    //
    // AIN1 - AIN0
    ads_write_register(
        REG_MUX,
        ADS_MUX_CONFIG
    );


    // PGA = 64
    ads_write_register(
        REG_ADCON,
        ADS_ADCON_CONFIG
    );


    // Nominal 15,000 SPS
    ads_write_register(
        REG_DRATE,
        ADS_DRATE_CONFIG
    );


    ads_write_register(
        REG_IO,
        ADS_IO_CONFIG
    );
}


// ============================================================
// Print ADS1256 registers
// ============================================================

static void ads_print_registers(void)
{
    printf("\nADS1256 registers:\n");

    printf(
        "STATUS = 0x%02X\n",
        ads_read_register(REG_STATUS)
    );

    printf(
        "MUX    = 0x%02X\n",
        ads_read_register(REG_MUX)
    );

    printf(
        "ADCON  = 0x%02X\n",
        ads_read_register(REG_ADCON)
    );

    printf(
        "DRATE  = 0x%02X\n",
        ads_read_register(REG_DRATE)
    );

    printf(
        "IO     = 0x%02X\n",
        ads_read_register(REG_IO)
    );

    printf(
        "OFC0   = 0x%02X\n",
        ads_read_register(REG_OFC0)
    );

    printf(
        "OFC1   = 0x%02X\n",
        ads_read_register(REG_OFC1)
    );

    printf(
        "OFC2   = 0x%02X\n",
        ads_read_register(REG_OFC2)
    );

    printf(
        "FSC0   = 0x%02X\n",
        ads_read_register(REG_FSC0)
    );

    printf(
        "FSC1   = 0x%02X\n",
        ads_read_register(REG_FSC1)
    );

    printf(
        "FSC2   = 0x%02X\n",
        ads_read_register(REG_FSC2)
    );
}


// ============================================================
// Perform ADS1256 self calibration
// ============================================================

static bool ads_calibrate(void)
{
    printf("\nStarting SELFCAL...\n");


    // Make sure auto-calibration remains OFF
    ads_write_register(
        REG_STATUS,
        ADS_STATUS_CONFIG
    );


    // Internal offset + gain calibration
    ads_send_command(
        CMD_SELFCAL
    );


    // Give calibration time to begin
    sleep_us(20);


    // DRDY becomes LOW again when calibration is
    // complete and valid conversion data is available.
    if (!ads_wait_drdy(100))
    {
        printf(
            "ERROR: calibration timed out!\n"
        );

        return false;
    }


    // Restore our STATUS configuration explicitly
    ads_write_register(
        REG_STATUS,
        ADS_STATUS_CONFIG
    );


    printf(
        "Calibration complete.\n"
    );


    return true;
}


// ============================================================
// Read conversion data
//
// IMPORTANT:
//
// This function assumes DRDY is ALREADY LOW.
//
// That lets the capture routine timestamp DRDY first,
// then immediately retrieve the corresponding ADC value.
// ============================================================

static void ads_read_data_ready(
    int32_t *signed_code,
    uint32_t *raw_code
)
{
    uint8_t command =
        CMD_RDATA;

    uint8_t rx[3];


    ads_select();


    // Request current conversion
    spi_write_blocking(
        ADS_SPI,
        &command,
        1
    );


    // ADS1256 t6 delay
    sleep_us(10);


    // Read 24-bit ADC result
    spi_read_blocking(
        ADS_SPI,
        0xFF,
        rx,
        3
    );


    ads_deselect();


    // Combine three bytes:
    //
    // MSB, middle, LSB
    uint32_t raw =
        ((uint32_t)rx[0] << 16) |
        ((uint32_t)rx[1] << 8)  |
        ((uint32_t)rx[2]);


    // Convert 24-bit two's complement
    // into signed 32-bit integer.
    int32_t code;


    if (raw & 0x800000)
    {
        code =
            (int32_t)(
                raw |
                0xFF000000
            );
    }
    else
    {
        code =
            (int32_t)raw;
    }


    *raw_code =
        raw;

    *signed_code =
        code;
}


// ============================================================
// Convert ADC code to differential voltage
//
// Voltage = AIN1 - AIN0
// ============================================================

static double ads_code_to_voltage(
    int32_t code
)
{
    return
        ((double)code *
         (2.0 * ADS_VREF)) /
        (ADS_PGA *
         ADS_MAX_CODE);
}


// ============================================================
// Capture consecutive samples
//
// We:
//
// 1. Wait for DRDY
// 2. Timestamp DRDY immediately
// 3. Read the ADC result
//
// There is NO printf() and NO long delay inside
// this loop.
// ============================================================

static bool ads_capture_samples(
    int32_t *samples,
    uint64_t *timestamps,
    uint32_t number_of_samples
)
{
    for (
        uint32_t i = 0;
        i < number_of_samples;
        i++
    )
    {
        // Wait for the next ADC conversion
        if (!ads_wait_drdy(100))
        {
            return false;
        }


        // Timestamp as soon as DRDY is detected LOW.
        //
        // This represents when this ADC conversion
        // became available.
        timestamps[i] =
            time_us_64();


        int32_t code;
        uint32_t raw;


        // Read the conversion associated with this DRDY
        ads_read_data_ready(
            &code,
            &raw
        );


        samples[i] =
            code;
    }


    return true;
}


// ============================================================
// Calculate peak-to-peak voltage
// ============================================================

static double calculate_vpp(
    const int32_t *samples,
    uint32_t number_of_samples
)
{
    int32_t minimum =
        samples[0];

    int32_t maximum =
        samples[0];


    for (
        uint32_t i = 1;
        i < number_of_samples;
        i++
    )
    {
        if (samples[i] < minimum)
        {
            minimum =
                samples[i];
        }


        if (samples[i] > maximum)
        {
            maximum =
                samples[i];
        }
    }


    double minimum_voltage =
        ads_code_to_voltage(
            minimum
        );


    double maximum_voltage =
        ads_code_to_voltage(
            maximum
        );


    return
        maximum_voltage -
        minimum_voltage;
}


// ============================================================
// Calculate actual average sample rate
//
// This tells us how quickly samples REALLY arrived.
//
// It does not assume 15,000 SPS.
// ============================================================

static double calculate_measured_sample_rate(
    const uint64_t *timestamps,
    uint32_t number_of_samples
)
{
    if (number_of_samples < 2)
    {
        return 0.0;
    }


    uint64_t elapsed_us =
        timestamps[number_of_samples - 1] -
        timestamps[0];


    if (elapsed_us == 0)
    {
        return 0.0;
    }


    // There are N - 1 intervals between N samples.
    return
        ((double)(number_of_samples - 1) *
         1000000.0) /
        (double)elapsed_us;
}


// ============================================================
// Calculate frequency using timestamped positive-going
// zero crossings.
//
// We do NOT assume a sample rate.
//
// First:
//     Remove DC offset using the average.
//
// Then:
//     Look for negative -> positive crossings.
//
// Then:
//     Interpolate both the SIGNAL VALUE and TIME
//     between the two surrounding samples.
//
// Finally:
//
//     frequency =
//        number of periods
//        -----------------
//        actual elapsed time
// ============================================================

static bool calculate_frequency(
    const int32_t *samples,
    const uint64_t *timestamps,
    uint32_t number_of_samples,
    double *frequency,
    uint32_t *crossing_count,
    double *dc_offset_voltage
)
{
    if (number_of_samples < 3)
    {
        return false;
    }


    // --------------------------------------------------------
    // Calculate average ADC value
    // --------------------------------------------------------

    int64_t sum = 0;


    for (
        uint32_t i = 0;
        i < number_of_samples;
        i++
    )
    {
        sum +=
            samples[i];
    }


    double average =
        (double)sum /
        (double)number_of_samples;


    // Report measured DC offset in volts
    *dc_offset_voltage =
        ads_code_to_voltage(
            (int32_t)average
        );


    // --------------------------------------------------------
    // Find positive-going zero crossings
    // --------------------------------------------------------

    bool found_first =
        false;


    double first_crossing_us =
        0.0;

    double last_crossing_us =
        0.0;


    uint32_t crossings =
        0;


    for (
        uint32_t i = 1;
        i < number_of_samples;
        i++
    )
    {
        // Remove DC component
        double previous =
            (double)samples[i - 1] -
            average;


        double current =
            (double)samples[i] -
            average;


        // Positive-going zero crossing
        if (
            (previous < 0.0) &&
            (current >= 0.0)
        )
        {
            double difference =
                current -
                previous;


            if (difference != 0.0)
            {
                // --------------------------------------------
                // Find where between the two voltage samples
                // the zero crossing occurred.
                //
                // fraction = 0 means exactly at sample i-1
                // fraction = 1 means exactly at sample i
                // --------------------------------------------

                double fraction =
                    (-previous) /
                    difference;


                // Actual time interval between samples
                double interval_us =
                    (double)(
                        timestamps[i] -
                        timestamps[i - 1]
                    );


                // Interpolated crossing time
                double crossing_time_us =
                    (double)timestamps[i - 1] +
                    fraction *
                    interval_us;


                if (!found_first)
                {
                    first_crossing_us =
                        crossing_time_us;

                    found_first =
                        true;
                }


                last_crossing_us =
                    crossing_time_us;


                crossings++;
            }
        }
    }


    *crossing_count =
        crossings;


    // Need at least two positive crossings
    // to measure a complete period.
    if (crossings < 2)
    {
        return false;
    }


    // Number of complete periods between
    // the first and last positive crossing.
    double periods =
        (double)(
            crossings - 1
        );


    // Actual elapsed time between crossings
    double elapsed_us =
        last_crossing_us -
        first_crossing_us;


    if (elapsed_us <= 0.0)
    {
        return false;
    }


    // Convert microseconds to seconds
    double elapsed_seconds =
        elapsed_us /
        1000000.0;


    // Frequency = cycles / time
    *frequency =
        periods /
        elapsed_seconds;


    return true;
}


// ============================================================
// MAIN
// ============================================================

int main()
{
    stdio_init_all();


    // Allow USB serial connection to enumerate
    sleep_ms(2000);


    printf("\n");
    printf("===============================\n");
    printf(" ADS1256 Magnetometer Test\n");
    printf("===============================\n");


    // ========================================================
    // SPI setup
    // ========================================================

    spi_init(
        ADS_SPI,
        1000 * 1000
    );


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


    // ========================================================
    // Chip select
    // ========================================================

    gpio_init(
        PIN_CS
    );


    gpio_set_dir(
        PIN_CS,
        GPIO_OUT
    );


    gpio_put(
        PIN_CS,
        1
    );


    // ========================================================
    // DRDY
    // ========================================================

    gpio_init(
        PIN_DRDY
    );


    gpio_set_dir(
        PIN_DRDY,
        GPIO_IN
    );


    gpio_pull_up(
        PIN_DRDY
    );


    // ========================================================
    // SPI mode
    //
    // Mode 1:
    //
    // CPOL = 0
    // CPHA = 1
    // ========================================================

    spi_set_format(
        ADS_SPI,
        8,
        SPI_CPOL_0,
        SPI_CPHA_1,
        SPI_MSB_FIRST
    );


    // Allow ADC/reference to settle
    sleep_ms(100);


    // ========================================================
    // Read initial STATUS
    // ========================================================

    uint8_t initial_status =
        ads_read_register(
            REG_STATUS
        );


    printf(
        "Initial STATUS = 0x%02X\n",
        initial_status
    );


    printf(
        "Device ID nibble = 0x%X\n",
        initial_status >> 4
    );


    // ========================================================
    // Configure ADC
    // ========================================================

    ads_configure();


    printf(
        "\nBefore SELFCAL:"
    );


    ads_print_registers();


    // ========================================================
    // Calibrate ADC
    // ========================================================

    if (!ads_calibrate())
    {
        printf(
            "Calibration failed.\n"
        );


        while (true)
        {
            sleep_ms(1000);
        }
    }


    printf(
        "\nAfter SELFCAL:"
    );


    ads_print_registers();


    // ========================================================
    // Verify STATUS configuration
    // ========================================================

    uint8_t status =
        ads_read_register(
            REG_STATUS
        );


    printf(
        "\nACAL bit = %d\n",
        (status >> 2) & 0x01
    );


    printf(
        "BUFEN bit = %d\n",
        (status >> 1) & 0x01
    );


    printf(
        "\nReady to measure frequency.\n"
    );


    printf(
        "Nominal ADC rate = %.0f SPS\n",
        ADS_NOMINAL_SAMPLE_RATE
    );


    printf(
        "Samples per capture = %d\n\n",
        NUM_SAMPLES
    );


    // ========================================================
    // Repeated waveform capture
    // ========================================================

    while (true)
    {
        printf(
            "Capturing waveform...\n"
        );


        // ----------------------------------------------------
        // Acquire samples and timestamps
        // ----------------------------------------------------

        if (!ads_capture_samples(
                sample_buffer,
                time_buffer,
                NUM_SAMPLES))
        {
            printf(
                "ERROR: DRDY timeout during capture!\n\n"
            );


            sleep_ms(1000);


            continue;
        }


        // ----------------------------------------------------
        // Calculate actual sample rate
        // ----------------------------------------------------

        double measured_sample_rate =
            calculate_measured_sample_rate(
                time_buffer,
                NUM_SAMPLES
            );


        // ----------------------------------------------------
        // Calculate peak-to-peak amplitude
        // ----------------------------------------------------

        double vpp =
            calculate_vpp(
                sample_buffer,
                NUM_SAMPLES
            );


        // ----------------------------------------------------
        // Calculate frequency
        // ----------------------------------------------------

        double frequency =
            0.0;


        uint32_t crossings =
            0;


        double dc_offset =
            0.0;


        bool frequency_valid =
            calculate_frequency(
                sample_buffer,
                time_buffer,
                NUM_SAMPLES,
                &frequency,
                &crossings,
                &dc_offset
            );


        // ----------------------------------------------------
        // Calculate total capture time
        // ----------------------------------------------------

        double capture_time_ms =
            (double)(
                time_buffer[NUM_SAMPLES - 1] -
                time_buffer[0]
            ) /
            1000.0;


        // ----------------------------------------------------
        // Print results AFTER acquisition
        // ----------------------------------------------------

        printf(
            "Captured %d samples\n",
            NUM_SAMPLES
        );


        printf(
            "Capture time = %.3f ms\n",
            capture_time_ms
        );


        printf(
            "Measured sample rate = %.2f SPS\n",
            measured_sample_rate
        );


        printf(
            "Measured Vpp = %.3f mV\n",
            vpp * 1000.0
        );


        printf(
            "Measured DC offset = %+.6f mV\n",
            dc_offset * 1000.0
        );


        printf(
            "Positive zero crossings = %lu\n",
            (unsigned long)crossings
        );


        if (frequency_valid)
        {
            printf(
                "Calculated frequency = %.3f Hz\n",
                frequency
            );
        }
        else
        {
            printf(
                "Could not calculate frequency.\n"
            );

            printf(
                "Not enough valid zero crossings.\n"
            );
        }


        printf("\n");


        // Wait before performing another capture.
        //
        // This occurs AFTER all samples have already
        // been collected, so it does not affect the
        // frequency calculation.
        sleep_ms(1000);
    }
}