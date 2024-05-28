// mipsel-linux-gcc -I ./linux-3.0.101/include/ BMP280_app.c -o BMP280_app -lm -pthread
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define SPI_DEVICE "/dev/spidev0.1"
#define SPI_MODE SPI_MODE_0
#define SPI_BITS_PER_WORD 8
#define SPI_SPEED 500000

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
enum
{
    BMP280_REGISTER_DIG_T1 = 0x88,
    BMP280_REGISTER_DIG_T2 = 0x8A,
    BMP280_REGISTER_DIG_T3 = 0x8C,
    BMP280_REGISTER_DIG_P1 = 0x8E,
    BMP280_REGISTER_DIG_P2 = 0x90,
    BMP280_REGISTER_DIG_P3 = 0x92,
    BMP280_REGISTER_DIG_P4 = 0x94,
    BMP280_REGISTER_DIG_P5 = 0x96,
    BMP280_REGISTER_DIG_P6 = 0x98,
    BMP280_REGISTER_DIG_P7 = 0x9A,
    BMP280_REGISTER_DIG_P8 = 0x9C,
    BMP280_REGISTER_DIG_P9 = 0x9E,
    BMP280_REGISTER_CHIPID = 0xD0,
    BMP280_REGISTER_VERSION = 0xD1,
    BMP280_REGISTER_SOFTRESET = 0xE0,
    BMP280_REGISTER_CONTROL = 0xF4,
    BMP280_REGISTER_CONFIG = 0xF5,
    BMP280_REGISTER_PRESSUREDATA = 0xF7,
    BMP280_REGISTER_TEMPDATA = 0xFA,
};

/*=========================================================================
    CALIBRATION DATA
    -----------------------------------------------------------------------*/
typedef struct
{
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} bmp280_calib_data;

static bmp280_calib_data bmp280_calib;
static int fd = -1;
static int32_t t_fine;

uint8_t read8(uint8_t reg);
uint16_t read16(uint8_t reg);
int16_t readS16(uint8_t reg);
uint16_t read16_LE(uint8_t reg);
int16_t readS16_LE(uint8_t reg);
void readCoefficients();
float readTemperature();
float readPressure();

int begin()
{
    fd = open(SPI_DEVICE, O_RDWR);
    if (fd < 0)
    {
        perror("Error opening SPI device");
        return 0;
    }

    uint8_t mode = SPI_MODE;
    uint8_t bits = SPI_BITS_PER_WORD;
    uint32_t speed = SPI_SPEED;

    // Using software SPI to manipulate
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) == -1 ||
        ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1 ||
        ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1)
    {
        perror("Error setting SPI parameters");
        return 0;
    }

    // Read chip id register in order to judge if connected
    if (read8(BMP280_REGISTER_CHIPID) != 0x58)
    {
        fprintf(stderr, "Error: BMP280 not found\n");
        return 0;
    }

    readCoefficients();
    uint8_t control = 0x3F;
    uint8_t tx[] = {BMP280_REGISTER_CONTROL & ~0x80, control};
    if (write(fd, tx, sizeof(tx)) != sizeof(tx))
    {
        perror("Error writing control register");
        return 0;
    }

    return 1;
}

// Reads an 8 bit value over SPI
uint8_t read8(uint8_t reg)
{
    uint8_t tx[] = {reg | 0x80};
    uint8_t rx[2] = {0, 0};
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 2,
        .delay_usecs = 0,
        .speed_hz = SPI_SPEED,
        .bits_per_word = SPI_BITS_PER_WORD,
    };
    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1)
    {
        perror("Error reading register");
    }
    return rx[1];
}

// Reads a 16 bit value over SPI
uint16_t read16(uint8_t reg)
{
    uint8_t tx[] = {reg | 0x80};
    uint8_t rx[3] = {0, 0, 0};
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 3,
        .delay_usecs = 0,
        .speed_hz = SPI_SPEED,
        .bits_per_word = SPI_BITS_PER_WORD,
    };
    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1)
    {
        perror("Error reading register");
    }
    return (rx[1] << 8) | rx[2];
}

// The same as above one, but in little endian (reverse byte order)
uint16_t read16_LE(uint8_t reg)
{
    uint16_t temp = read16(reg);
    return (temp >> 8) | (temp << 8);
}

// Reads a signed 16 bit value over SPI
int16_t readS16(uint8_t reg)
{
    return (int16_t)read16(reg);
}

int16_t readS16_LE(uint8_t reg)
{
    return (int16_t)read16_LE(reg);
}

// Reads the factory-set coefficients
void readCoefficients()
{
    bmp280_calib.dig_T1 = read16_LE(BMP280_REGISTER_DIG_T1);
    bmp280_calib.dig_T2 = readS16_LE(BMP280_REGISTER_DIG_T2);
    bmp280_calib.dig_T3 = readS16_LE(BMP280_REGISTER_DIG_T3);
    bmp280_calib.dig_P1 = read16_LE(BMP280_REGISTER_DIG_P1);
    bmp280_calib.dig_P2 = readS16_LE(BMP280_REGISTER_DIG_P2);
    bmp280_calib.dig_P3 = readS16_LE(BMP280_REGISTER_DIG_P3);
    bmp280_calib.dig_P4 = readS16_LE(BMP280_REGISTER_DIG_P4);
    bmp280_calib.dig_P5 = readS16_LE(BMP280_REGISTER_DIG_P5);
    bmp280_calib.dig_P6 = readS16_LE(BMP280_REGISTER_DIG_P6);
    bmp280_calib.dig_P7 = readS16_LE(BMP280_REGISTER_DIG_P7);
    bmp280_calib.dig_P8 = readS16_LE(BMP280_REGISTER_DIG_P8);
    bmp280_calib.dig_P9 = readS16_LE(BMP280_REGISTER_DIG_P9);
}

float readTemperature()
{
    int32_t var1, var2;
    int32_t adc_T = read16(BMP280_REGISTER_TEMPDATA);
    adc_T <<= 8;
    adc_T |= read8(BMP280_REGISTER_TEMPDATA + 2);
    adc_T >>= 4;

    var1 = ((((adc_T >> 3) - ((int32_t)bmp280_calib.dig_T1 << 1))) * ((int32_t)bmp280_calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)bmp280_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp280_calib.dig_T1))) >> 12) * ((int32_t)bmp280_calib.dig_T3)) >> 14;

    t_fine = var1 + var2;

    float T = (t_fine * 5 + 128) >> 8;
    return T / 100;
}

float readPressure()
{
    int64_t var1, var2, p;
    int32_t adc_P = read16(BMP280_REGISTER_PRESSUREDATA);
    adc_P <<= 8;
    adc_P |= read8(BMP280_REGISTER_PRESSUREDATA + 2);
    adc_P >>= 4;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp280_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280_calib.dig_P3) >> 8) + ((var1 * (int64_t)bmp280_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280_calib.dig_P1) >> 33;

    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp280_calib.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280_calib.dig_P7) << 4);
    return (float)p / 256;
}

float waterBoilingPoint(float pressure)
{
    // Magnusformular for calculation of the boiling point of water at a given
    // pressure
    return (234.175 * log(pressure / 6.1078)) /
           (17.08085 - log(pressure / 6.1078));
}

float getAltitude(float curhPa, float seaLevelhPa)
{
    return 44330 * (1.0 - pow(curhPa / seaLevelhPa, 0.1903));
}

int keep_run = 1;

void cleanup(int signum)
{
    if (fd != -1)
    {
        close(fd);
    }
    printf("Now exit!\n");
    keep_run = 0;
    exit(signum);
}

void* polling_thread(void* arg)
{
    while (keep_run)
    {
        float temp = readTemperature(), pres = readPressure();
        printf("Temperature = %.2f *C, Pressure = %.2f Pa\n", temp, pres);
        // 1013.25 hPa (standard atmospheric pressure) applies in most cases 
        // to 0 meters of barometric pressure
        printf("Water BoilPoint = %.2f *C, Altitude = %.2f m\n",
               waterBoilingPoint(pres / 100.0f), getAltitude(pres / 100.0f, 1013.25));
        usleep(1200 * 1000);
    }
    return NULL;
}

int main()
{
    if (!begin())
    {
        fprintf(stderr, "Could not find a valid BMP280 sensor, check wiring!\n");
        return 1;
    }

    // Register signal processing function for processing SIGINT (Ctrl+C)
    signal(SIGINT, cleanup);

    // Create background thread
    pthread_t thread_id;
    if (pthread_create(&thread_id, NULL, polling_thread, NULL) != 0)
    {
        fprintf(stderr, "Failed to create polling thread\n");
        return 1;
    }

    // The main thread waits for the background thread to finish
    pthread_join(thread_id, NULL);

    return 0;
}
