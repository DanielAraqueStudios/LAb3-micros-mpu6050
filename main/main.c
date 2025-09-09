
#include <stdio.h>
// Clean, minimal MPU6050 backend for ESP32-S3 (ESP-IDF v6.x)
// - Reads accel/gyro/temp from MPU6050 over I2C
// - Triggered by GPTimer at SAMPLE_HZ
// - Streams CSV lines over UART: ax,ay,az,gx,gy,gz,temp\n

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "driver/gptimer.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define TAG "mpu"

#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_SDA_IO      7
#define I2C_MASTER_SCL_IO      6
#define I2C_MASTER_FREQ_HZ     400000

#define MPU6050_ADDR           0x68
#define MPU6050_PWR_MGMT_1     0x6B
#define MPU6050_ACCEL_XOUT_H   0x3B

#define SAMPLE_HZ              30

#define UART_PORT              UART_NUM_1
#define UART_TX_PIN            17
#define UART_RX_PIN            16
#define UART_BUF_SIZE          1024

typedef struct {
	int16_t accel_x, accel_y, accel_z;
	int16_t gyro_x, gyro_y, gyro_z;
	int16_t temp;
} mpu6050_data_t;

static QueueHandle_t s_timer_queue = NULL;
static gptimer_handle_t s_gptimer = NULL;

static esp_err_t i2c_master_init(void)
{
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = I2C_MASTER_SDA_IO,
		.scl_io_num = I2C_MASTER_SCL_IO,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_MASTER_FREQ_HZ,
	};
	esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
	if (err != ESP_OK) return err;
	return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t mpu6050_init(void)
{
	// Wake device (write 0 to PWR_MGMT_1)
	uint8_t data[2] = {MPU6050_PWR_MGMT_1, 0x00};
	return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, data, sizeof(data), 100 / portTICK_PERIOD_MS);
}

static esp_err_t mpu6050_read(mpu6050_data_t *out)
{
	uint8_t reg = MPU6050_ACCEL_XOUT_H;
	uint8_t buf[14];
	esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, &reg, 1, buf, sizeof(buf), 100 / portTICK_PERIOD_MS);
	if (ret != ESP_OK) return ret;
	out->accel_x = (int16_t)((buf[0] << 8) | buf[1]);
	out->accel_y = (int16_t)((buf[2] << 8) | buf[3]);
	out->accel_z = (int16_t)((buf[4] << 8) | buf[5]);
	out->temp    = (int16_t)((buf[6] << 8) | buf[7]);
	out->gyro_x  = (int16_t)((buf[8] << 8) | buf[9]);
	out->gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
	out->gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);
	return ESP_OK;
}

// GPTimer callback: send an event to the queue from ISR context
static bool IRAM_ATTR timer_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
	uint32_t evt = 1;
	BaseType_t hpw = pdFALSE;
	xQueueSendFromISR(s_timer_queue, &evt, &hpw);
	return hpw == pdTRUE; // return true if a higher priority task was woken
}

static void timer_setup(void)
{
	gptimer_config_t config = {
		.clk_src = GPTIMER_CLK_SRC_DEFAULT,
		.direction = GPTIMER_COUNT_UP,
		.resolution_hz = 1000000, // 1 MHz -> microsecond ticks
	};
	ESP_ERROR_CHECK(gptimer_new_timer(&config, &s_gptimer));

	gptimer_event_callbacks_t cbs = { .on_alarm = timer_alarm_cb };
	ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_gptimer, &cbs, NULL));
	ESP_ERROR_CHECK(gptimer_enable(s_gptimer));

	gptimer_alarm_config_t alarm_cfg = {
		.alarm_count = (1000000u / SAMPLE_HZ), // microseconds per tick
		.reload_count = 0,
		.flags = { .auto_reload_on_alarm = true },
	};
	ESP_ERROR_CHECK(gptimer_set_alarm_action(s_gptimer, &alarm_cfg));
	ESP_ERROR_CHECK(gptimer_start(s_gptimer));
}

static void uart_init(void)
{
	const uart_config_t uart_cfg = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	};
	ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
	ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_cfg));
	ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void mpu_task(void *arg)
{
	mpu6050_data_t sample;
	uint32_t evt;
	char line[128];
	while (1) {
		if (xQueueReceive(s_timer_queue, &evt, portMAX_DELAY) == pdTRUE) {
			if (mpu6050_read(&sample) == ESP_OK) {
				int n = snprintf(line, sizeof(line), "%d,%d,%d,%d,%d,%d,%d\n",
								 sample.accel_x, sample.accel_y, sample.accel_z,
								 sample.gyro_x, sample.gyro_y, sample.gyro_z,
								 sample.temp);
				uart_write_bytes(UART_PORT, line, n);
			} else {
				ESP_LOGW(TAG, "MPU6050 read failed");
			}
		}
	}
}

void app_main(void)
{
	ESP_LOGI(TAG, "Starting MPU6050 backend (SAMPLE_HZ=%d)" , SAMPLE_HZ);
	uart_init();
	ESP_ERROR_CHECK(i2c_master_init());
	ESP_ERROR_CHECK(mpu6050_init());

	s_timer_queue = xQueueCreate(10, sizeof(uint32_t));
	timer_setup();

	xTaskCreate(mpu_task, "mpu_task", 4096, NULL, 10, NULL);
}