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
// Frequency still uses the sample rate measured over the
// actual capture interval with the Pico clock.
#define ADS_NOMINAL_SAMPLE_RATE  15000.0


// Stop the capture when EITHER:
//
// 1. Four seconds have elapsed, OR
// 2. The measured AC signal amplitude has fallen to 10%
//    of the amplitude measured during the first window.
//
// The amplitude check uses 100 ms windows.  For a sinusoid,
// AC RMS falls by the same ratio as peak amplitude, but RMS
// is less sensitive to single-sample noise spikes.
#define MAX_CAPTURE_US           4000000ULL
#define AMPLITUDE_WINDOW_US      100000ULL
#define DECAY_STOP_FRACTION      0.10


// At 15 kSPS, 60,000 samples is approximately four seconds.
//
// IMPORTANT:
// We intentionally store only the ADC codes, not one 64-bit
// timestamp per sample.  60,000 int32_t samples use about
// 240 kB, which is already close to the RP2040 SRAM limit.
//
// Timing is measured with the first and last DRDY timestamps.
#define MAX_SAMPLES              60000


// ============================================================
// Sample storage
// ============================================================

// ADC conversion codes
static int32_t sample_buffer[MAX_SAMPLES];


typedef enum
{
    CAPTURE_STOP_TIME_LIMIT,
    CAPTURE_STOP_DECAY_LIMIT,
    CAPTURE_STOP_BUFFER_LIMIT
} capture_stop_reason_t;


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
// Capture stops when EITHER:
//
// 1. Four seconds have elapsed, OR
// 2. The AC RMS signal has fallen to 10% of the RMS measured
//    during the first 100 ms amplitude window.
//
// We compare AC POWER instead of taking square roots:
//
//      RMS <= 0.10 * initial RMS
//
// is equivalent to:
//
//      variance <= 0.01 * initial variance
//
// There is NO printf() and NO long delay inside this loop.
// ============================================================

static bool ads_capture_samples(
    int32_t *samples,
    uint32_t max_samples,
    uint32_t *samples_captured,
    uint64_t *first_sample_time_us,
    uint64_t *last_sample_time_us,
    capture_stop_reason_t *stop_reason
)
{
    *samples_captured =
        0;

    *first_sample_time_us =
        0;

    *last_sample_time_us =
        0;


    bool initial_amplitude_ready =
        false;

    double initial_ac_power =
        0.0;


    // Running statistics for the current 100 ms window.
    //
    // AC power = variance = E[x^2] - E[x]^2
    double window_sum =
        0.0;

    double window_sum_squares =
        0.0;

    uint32_t window_samples =
        0;


    uint64_t capture_start_us =
        0;

    uint64_t window_start_us =
        0;


    for (
        uint32_t i = 0;
        i < max_samples;
        i++
    )
    {
        // Wait for the next ADC conversion.
        if (!ads_wait_drdy(100))
        {
            return false;
        }


        // Timestamp immediately when DRDY is observed LOW.
        uint64_t sample_time_us =
            time_us_64();


        if (i == 0)
        {
            capture_start_us =
                sample_time_us;

            window_start_us =
                sample_time_us;

            *first_sample_time_us =
                sample_time_us;
        }


        int32_t code;
        uint32_t raw;


        // Read the conversion associated with this DRDY.
        ads_read_data_ready(
            &code,
            &raw
        );


        samples[i] =
            code;

        *samples_captured =
            i + 1;

        *last_sample_time_us =
            sample_time_us;


        // ----------------------------------------------------
        // Add this sample to the current amplitude window.
        // ----------------------------------------------------

        double value =
            (double)code;

        window_sum +=
            value;

        window_sum_squares +=
            value * value;

        window_samples++;


        // ----------------------------------------------------
        // Every 100 ms, estimate the AC signal amplitude.
        // ----------------------------------------------------

        if (
            (sample_time_us - window_start_us) >=
            AMPLITUDE_WINDOW_US
        )
        {
            double count =
                (double)window_samples;

            double mean =
                window_sum /
                count;

            double mean_square =
                window_sum_squares /
                count;

            double ac_power =
                mean_square -
                (mean * mean);


            // Numerical roundoff can produce a tiny
            // negative value very close to zero.
            if (ac_power < 0.0)
            {
                ac_power =
                    0.0;
            }


            if (!initial_amplitude_ready)
            {
                // The first complete 100 ms window defines
                // "100% original signal."
                initial_ac_power =
                    ac_power;

                initial_amplitude_ready =
                    true;
            }
            else if (initial_ac_power > 0.0)
            {
                // 10% RMS amplitude = 1% AC power.
                double stop_power =
                    initial_ac_power *
                    DECAY_STOP_FRACTION *
                    DECAY_STOP_FRACTION;


                if (ac_power <= stop_power)
                {
                    *stop_reason =
                        CAPTURE_STOP_DECAY_LIMIT;

                    return true;
                }
            }


            // Start a new 100 ms amplitude window.
            window_start_us =
                sample_time_us;

            window_sum =
                0.0;

            window_sum_squares =
                0.0;

            window_samples =
                0;
        }


        // ----------------------------------------------------
        // Hard four-second capture limit.
        // ----------------------------------------------------

        if (
            (sample_time_us - capture_start_us) >=
            MAX_CAPTURE_US
        )
        {
            *stop_reason =
                CAPTURE_STOP_TIME_LIMIT;

            return true;
        }
    }


    // At nominal 15 kSPS the 60,000-sample buffer reaches
    // essentially the same point as the four-second limit.
    *stop_reason =
        CAPTURE_STOP_BUFFER_LIMIT;

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
// The ADS1256 conversion timing is uniform, so we only need
// the timestamp of the first and last DRDY events.
//
// This avoids storing a 64-bit timestamp for every sample,
// which would not fit in Pico RAM for a four-second capture.
// ============================================================

static double calculate_measured_sample_rate(
    uint64_t first_sample_time_us,
    uint64_t last_sample_time_us,
    uint32_t number_of_samples
)
{
    if (number_of_samples < 2)
    {
        return 0.0;
    }


    uint64_t elapsed_us =
        last_sample_time_us -
        first_sample_time_us;


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
// Calculate frequency using positive-going zero crossings.
//
// We do NOT assume the nominal ADC sample rate.
//
// First:
//     Remove DC offset using the average.
//
// Then:
//     Look for negative -> positive crossings.
//
// Then:
//     Interpolate the crossing position between the two
//     surrounding samples.
//
// Finally:
//     Convert crossing separation from samples to time using
//     the measured sample rate for this capture.
// ============================================================

static bool calculate_frequency(
    const int32_t *samples,
    uint32_t number_of_samples,
    double measured_sample_rate,
    double *frequency,
    uint32_t *crossing_count,
    double *dc_offset_voltage
)
{
    if (
        (number_of_samples < 3) ||
        (measured_sample_rate <= 0.0)
    )
    {
        return false;
    }


    // --------------------------------------------------------
    // Calculate average ADC value
    // --------------------------------------------------------

    int64_t sum =
        0;


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


    // Report measured DC offset in volts.
    *dc_offset_voltage =
        ads_code_to_voltage(
            (int32_t)average
        );


    // --------------------------------------------------------
    // Find positive-going zero crossings
    // --------------------------------------------------------

    bool found_first =
        false;


    double first_crossing_sample =
        0.0;

    double last_crossing_sample =
        0.0;


    uint32_t crossings =
        0;


    for (
        uint32_t i = 1;
        i < number_of_samples;
        i++
    )
    {
        // Remove DC component.
        double previous =
            (double)samples[i - 1] -
            average;


        double current =
            (double)samples[i] -
            average;


        // Positive-going zero crossing.
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
                // Find where between samples i-1 and i
                // the crossing occurred.
                double fraction =
                    (-previous) /
                    difference;


                double crossing_sample =
                    (double)(i - 1) +
                    fraction;


                if (!found_first)
                {
                    first_crossing_sample =
                        crossing_sample;

                    found_first =
                        true;
                }


                last_crossing_sample =
                    crossing_sample;


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


    double elapsed_samples =
        last_crossing_sample -
        first_crossing_sample;


    if (elapsed_samples <= 0.0)
    {
        return false;
    }


    double elapsed_seconds =
        elapsed_samples /
        measured_sample_rate;


    if (elapsed_seconds <= 0.0)
    {
        return false;
    }


    // Frequency = cycles / time.
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
        "Maximum capture time = %.1f s\n",
        (double)MAX_CAPTURE_US /
        1000000.0
    );


    printf(
        "Decay stop threshold = %.0f%% of initial signal\n",
        DECAY_STOP_FRACTION *
        100.0
    );


    printf(
        "Maximum stored samples = %d\n\n",
        MAX_SAMPLES
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
        // Acquire samples
        //
        // Capture ends automatically at:
        //   - 4 seconds, OR
        //   - 10% of the initial AC signal amplitude.
        // ----------------------------------------------------

        uint32_t samples_captured =
            0;

        uint64_t first_sample_time_us =
            0;

        uint64_t last_sample_time_us =
            0;

        capture_stop_reason_t stop_reason =
            CAPTURE_STOP_BUFFER_LIMIT;


        if (!ads_capture_samples(
                sample_buffer,
                MAX_SAMPLES,
                &samples_captured,
                &first_sample_time_us,
                &last_sample_time_us,
                &stop_reason))
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
                first_sample_time_us,
                last_sample_time_us,
                samples_captured
            );


        // ----------------------------------------------------
        // Calculate peak-to-peak amplitude over the complete
        // captured record.
        // ----------------------------------------------------

        double vpp =
            calculate_vpp(
                sample_buffer,
                samples_captured
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
                samples_captured,
                measured_sample_rate,
                &frequency,
                &crossings,
                &dc_offset
            );


        // ----------------------------------------------------
        // Calculate total capture time
        // ----------------------------------------------------

        double capture_time_ms =
            (double)(
                last_sample_time_us -
                first_sample_time_us
            ) /
            1000.0;


        // ----------------------------------------------------
        // Print results AFTER acquisition
        // ----------------------------------------------------

        printf(
            "Captured %lu samples\n",
            (unsigned long)samples_captured
        );


        printf(
            "Capture time = %.3f ms\n",
            capture_time_ms
        );


        if (stop_reason == CAPTURE_STOP_DECAY_LIMIT)
        {
            printf(
                "Capture stopped: signal reached 10%% "
                "of initial amplitude.\n"
            );
        }
        else if (stop_reason == CAPTURE_STOP_TIME_LIMIT)
        {
            printf(
                "Capture stopped: 4.0 second limit reached.\n"
            );
        }
        else
        {
            printf(
                "Capture stopped: sample buffer full "
                "(approximately 4 seconds).\n"
            );
        }


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