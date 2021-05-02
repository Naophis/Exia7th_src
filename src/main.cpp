#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include <driver/adc.h>

#define LED1 7
#define LED2 8 //何故か使えない SD1
#define LED3 5
#define LED4 18

#define EN_SSL_R 11 //CMD ok
#define R_IN1 16    //IO16 ng
#define R_IN2 17    //IO17 ng
#define SEN_RF 6    //CLK ng

#define SW1 16 //CLK ng

#define GPIO_SPI_MISO 25 //
#define GPIO_SPI_MOSI 35 // output is invalid.

#define GPIO_SPI_CLK 32
#define GPIO_SPI_CS 33

#define R_IN1 26
#define R_IN2 27

#define READ_FLAG 0x80

void init_uart()
{
    const int uart_num = UART_NUM_0;
    uart_config_t uart_config = {
        .baud_rate = 2000000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
        .rx_flow_ctrl_thresh = 122};
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
}

void init_gpio()
{
    gpio_config_t io_conf;
    // 割り込みをしない
    io_conf.intr_type = (gpio_int_type_t)GPIO_PIN_INTR_DISABLE;
    // 出力モード
    io_conf.mode = (gpio_mode_t)GPIO_MODE_OUTPUT;
    // 設定したいピンのビットマスク
    io_conf.pin_bit_mask = (BIT64(LED1))       //
                                               //    | (BIT64(LED2))      //
                           | (BIT64(LED3))     //
                           | (BIT64(LED4))     //
                           | (BIT64(R_IN1))    //
                           | (BIT64(R_IN2))    //
                           | (BIT64(EN_SSL_R)) //
                           | (BIT64(21))       //
                           | (BIT64(23));      //
                                               //    | ((1ULL << R_IN1));   //
                                               //    | ((1ULL << R_IN2));   //
                                               //    | ((1ULL << SEN_RF));  //
    // 内部プルダウンしない
    io_conf.pull_down_en = (gpio_pulldown_t)GPIO_PULLDOWN_DISABLE;
    // // 内部プルアップしない
    io_conf.pull_up_en = (gpio_pullup_t)GPIO_PULLUP_ENABLE;
    // // 設定をセットする
    gpio_config(&io_conf);
    // gpio_set_direction((gpio_num_t)GPIO_OUTPUT_IO_8, (gpio_mode_t)GPIO_MODE_INPUT);

    // gpio_set_direction((gpio_num_t)SW1, (gpio_mode_t)GPIO_MODE_INPUT);
}

uint8_t mpu9250_read1byte(spi_device_handle_t spi, const uint8_t address)
{
    // １バイト読み込み

    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t)); // Zero out the transaction
    t.length = 16;            // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
    t.flags = SPI_TRANS_USE_RXDATA;

    uint16_t tx_data = (address | READ_FLAG) << 8;
    tx_data = SPI_SWAP_DATA_TX(tx_data, 16);
    t.tx_buffer = &tx_data;

    ret = spi_device_polling_transmit(spi, &t); // Transmit!
    assert(ret == ESP_OK);                      // Should have had no issues.

    uint8_t data = SPI_SWAP_DATA_RX(*(uint16_t *)t.rx_data, 16) & 0x00FF; // FF + Data

    return data;
}
uint8_t mpu9250_write1byte(spi_device_handle_t spi, const uint8_t address, const uint8_t data)
{
    // １バイト読み込み

    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t)); // Zero out the transaction
    t.length = 16;            // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
    t.flags = SPI_TRANS_USE_RXDATA;

    uint16_t tx_data = (READ_FLAG) << 8 | (0x0f & data);
    tx_data = SPI_SWAP_DATA_TX(tx_data, 16);
    t.tx_buffer = &tx_data;

    ret = spi_device_polling_transmit(spi, &t); // Transmit!
    assert(ret == ESP_OK);                      // Should have had no issues.

    // uint16_t data = SPI_SWAP_DATA_RX(*(uint16_t *)t.rx_data, 16) & 0x00FF; // FF + Data

    return 0;
}
void mpu9250_init(spi_device_handle_t spi)
{
    // Who AM I
    uint8_t mpu_id = mpu9250_read1byte(spi, 0x75);

    printf("MPU ID: %02X\n", mpu_id);
    mpu9250_write1byte(spi, 0x6B, 0x80); //スリープ解除?
    vTaskDelay(100 / portTICK_RATE_MS);
    mpu9250_write1byte(spi, 0x68, 0x04); //ジャイロリセット
    vTaskDelay(100 / portTICK_RATE_MS);
    mpu9250_write1byte(spi, 0x6A, 0x10); //uercontrol i2c=disable
    vTaskDelay(100 / portTICK_RATE_MS);
    mpu9250_write1byte(spi, 0x1B, 0x18); //gyro config ジャイロのフルスケールを±2000°/s
    vTaskDelay(100 / portTICK_RATE_MS);
}

void init_spi(spi_device_handle_t &spi)
{
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_SPI_MOSI,
        .miso_io_num = GPIO_SPI_MISO,
        .sclk_io_num = GPIO_SPI_CLK,
        .quadwp_io_num = -1,  // unused
        .quadhd_io_num = -1,  // unused
        .max_transfer_sz = 4, // bytes
    };
    spi_device_interface_config_t devcfg = {
        .mode = 3,                     //SPI mode 3
        .clock_speed_hz = 1000 * 1000, //Clock out at 500 kHz
        .spics_io_num = GPIO_SPI_CS,   //CS pin
        .queue_size = 7,               //We want to be able to queue 7 transactions at a time
    };
    //Initialize the SPI bus
    ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}

void read_gyro(spi_device_handle_t &spi)
{
    uint8_t gyro_high = mpu9250_read1byte(spi, 0x47); //high
    uint8_t gyro_low = mpu9250_read1byte(spi, 0x48);  //low
    signed short gyro = (signed short)((((unsigned int)(gyro_high & 0xff)) << 8) | ((unsigned int)(gyro_low & 0xff)));
    printf("gyro=:%d \n", gyro);
}
#define GPIO_PWM0A_OUT 21
// #define GPIO_PWM0A_OUT 14
void init_pwm(mcpwm_config_t &pwm_config)
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    pwm_config.frequency = 1000; // PWM周波数= 10kHz,
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0; // アクティブハイ
}
#define GPIO_PWM0A_OUT2 21
void init_pwm2(mcpwm_config_t &pwm_config)
{
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, GPIO_PWM0A_OUT2);
    pwm_config.frequency = 1000; // PWM周波数= 10kHz,
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0; // アクティブハイ
}
#define GPIO_PWM0A_OUT3 23
void init_pwm3(mcpwm_config_t &pwm_config)
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT3);
    pwm_config.frequency = 10 * 10000; // PWM周波数= 10kHz,
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0; // アクティブハイ
}

extern "C" void app_main()
{
    init_uart();
    init_gpio();
    // spi_device_handle_t spi;
    // init_spi(spi);
    // mpu9250_init(spi);

    // mcpwm_config_t pwm_config;
    // init_pwm(pwm_config);
    // mcpwm_config_t pwm_config2;
    // init_pwm2(pwm_config2);
    // mcpwm_config_t pwm_config3;
    // init_pwm(pwm_config3);

    // esp_chip_info_t chip_info;
    // esp_chip_info(&chip_info);
    // adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_0db);
    // adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_0db);
    // adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_0db);
    // adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_0db);
    // adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_0db);
    // adc2_config_channel_atten(ADC2_CHANNEL_0, ADC_ATTEN_0db);
    int cnt = 0;
    // int read_raw;
    // adc2_config_channel_atten(ADC2_CHANNEL_7, ADC_ATTEN_0db);
    while (1)
    {
        cnt++;
        // int res = mpu9250_read1byte(spi, 117);

        // int read_raw0 = adc1_get_raw(ADC1_CHANNEL_0);
        // int read_raw1 = adc1_get_raw(ADC1_CHANNEL_1);
        // int read_raw2 = adc1_get_raw(ADC1_CHANNEL_2);
        // int read_raw3 = adc1_get_raw(ADC1_CHANNEL_3);
        // int read_raw5 = adc1_get_raw(ADC1_CHANNEL_3);
        // int read_raw4;
        // // adc2_get_raw(ADC2_CHANNEL_0, ADC_WIDTH_12Bit, &read_raw4);

        // esp_err_t r = adc2_get_raw(ADC2_CHANNEL_7, ADC_WIDTH_12Bit, &read_raw4);
        // if (r == ESP_OK)
        // {
        //     printf("%d\n", read_raw4);
        // }
        // else if (r == ESP_ERR_TIMEOUT)
        // {
        //     printf("ADC2 used by Wi-Fi.\n");
        // }
        printf("hello world %d\n", cnt);

        // printf("hello world %d %d %d %d %d %d %d %d\n", cnt, read_raw0, read_raw1, read_raw2, read_raw3, read_raw4, read_raw5, r);

        // pwm_config.cmpr_a = 25; // デューティサイクルの初期値（0%）
        // mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
        // pwm_config2.cmpr_a = 10; // デューティサイクルの初期値（0%）
        // mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config2);
        // pwm_config.cmpr_a = 25; // デューティサイクルの初期値（0%）
        // mcpwm_init(MCPWM_UNIT_2, MCPWM_TIMER_0, &pwm_config3);

        // gpio_set_level((gpio_num_t)LED1, cnt & 0x01);
        // gpio_set_level((gpio_num_t)LED2, cnt & 0x01);
        // gpio_set_level((gpio_num_t)LED3, cnt & 0x01);

        // gpio_set_level((gpio_num_t)R_IN1, 1);
        // gpio_set_level((gpio_num_t)R_IN2, 0);

        // gpio_set_level((gpio_num_t)21, 1);
        // gpio_set_level((gpio_num_t)23, 1);

        // gpio_set_level((gpio_num_t)R_IN1, 0);
        // gpio_set_level((gpio_num_t)R_IN2, 1);

        vTaskDelay(125 / portTICK_RATE_MS);
    }
}