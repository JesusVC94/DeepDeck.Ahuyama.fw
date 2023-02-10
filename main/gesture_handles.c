/*
 * gesture_test.c
 *
 *  Created on: Feb 4, 2023
 *      Author: mauro
 */

// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "gesture_handles.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"




i2c_bus_handle_t i2c_bus = NULL;
apds9960_handle_t apds9960 = NULL;

extern TaskHandle_t xOledTask;

extern SemaphoreHandle_t xSemaphore;

///////////////////////////////////
void IRAM_ATTR gesture_isr_handler(void *arg) {
	BaseType_t task_woken = pdFALSE;
	uint32_t gpio_num = (uint32_t) arg;
//	gpio_intr_disable(gpio_num);

	xSemaphoreGiveFromISR(xSemaphore, &task_woken);

	if (task_woken)
		portYIELD_FROM_ISR();

}
///////////////////////////////////

/**
 * @brief i2c master initialization
 */
void apds9960_init() {

	config_interrup_pin();

	int i2c_master_port = APDS9960_I2C_MASTER_NUM;
	i2c_config_t conf = { .mode = I2C_MODE_MASTER, .sda_io_num =
			APDS9960_I2C_MASTER_SDA_IO, .sda_pullup_en = GPIO_PULLUP_ENABLE,
			.scl_io_num = APDS9960_I2C_MASTER_SCL_IO, .scl_pullup_en =
					GPIO_PULLUP_ENABLE, .master.clk_speed =
					APDS9960_I2C_MASTER_FREQ_HZ, };
	i2c_bus = i2c_bus_create(i2c_master_port, &conf);
	apds9960 = apds9960_create(i2c_bus, APDS9960_I2C_ADDRESS);

	apds9960_gesture_init(apds9960);
}

void apds9960_deinit() {
	apds9960_delete(&apds9960);
	i2c_bus_delete(&i2c_bus);
}

void apds9960_test_gesture() {
	int cnt = 0;
	while (cnt < 10) {
		uint8_t gesture = apds9960_read_gesture(apds9960);
		if (gesture == APDS9960_DOWN) {
			printf("gesture APDS9960_DOWN*********************!\n");
		} else if (gesture == APDS9960_UP) {
			printf("gesture APDS9960_UP*********************!\n");
		} else if (gesture == APDS9960_LEFT) {
			printf("gesture APDS9960_LEFT*********************!\n");
			cnt++;
		} else if (gesture == APDS9960_RIGHT) {
			printf("gesture APDS9960_RIGHT*********************!\n");
			cnt++;
		}
		//vTaskDelay(100 / portTICK_RATE_MS);
	}
}

////Config Interrup PIN
void config_interrup_pin(void) {

	gpio_num_t interrupt_pin = (gpio_num_t) INTERRUPT_GPIO;
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_NEGEDGE; //GPIO_INTR_ANYEDGE;//
	io_conf.pin_bit_mask = (1ULL << INTERRUPT_GPIO);
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = GPIO_PULLUP_ENABLE; // (gpio_pullup_t) 0;
	io_conf.pull_down_en = (gpio_pulldown_t) 0;
	gpio_config(&io_conf);

//install gpio isr service
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
//hook isr handler for specific gpio pin
	gpio_isr_handler_add(interrupt_pin, gesture_isr_handler,
			(void*) interrupt_pin);

}

///////////////

void test() {

	apds9960_init();
//	apds9960_gesture_init(apds9960);
	vTaskDelay(pdMS_TO_TICKS(1000));
	apds9960_test_gesture();
	apds9960_deinit();
}

