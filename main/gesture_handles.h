/*
 * gesture_test.h
 *
 *  Created on: Feb 4, 2023
 *      Author: mauro
 */

#ifndef MAIN_GESTURE_HANDLES_H_
#define MAIN_GESTURE_HANDLES_H_

#include <stdio.h>
#include "esp_log.h"
#include "apds9960.h"

//#define APDS9960_VL_IO                       (gpio_num_t)19
#define APDS9960_I2C_MASTER_SCL_IO           (gpio_num_t)22          /*!< gpio number for I2C master clock */
#define APDS9960_I2C_MASTER_SDA_IO           (gpio_num_t)21          /*!< gpio number for I2C master data  */
#define APDS9960_I2C_MASTER_NUM              I2C_NUM_0   /*!< I2C port number for master dev */
#define APDS9960_I2C_MASTER_TX_BUF_DISABLE   0           /*!< I2C master do not need buffer */
#define APDS9960_I2C_MASTER_RX_BUF_DISABLE   0           /*!< I2C master do not need buffer */
#define APDS9960_I2C_MASTER_FREQ_HZ          400000      /*!< I2C master clock frequency */

#define INTERRUPT_GPIO 19
#define APDS9960_INT_PIN     19
#define GPIO_INPUT_PIN_SEL  (1ULL<<APDS9960_INT_PIN)
#define ESP_INTR_FLAG_DEFAULT 0

//extern apds9960_handle_t apds9960;
void IRAM_ATTR gesture_isr_handler(void *arg);


void apds9960_test_gesture();
void apds9960_deinit();
void apds9960_init();

void config_interrup_pin(void);
void test();

#endif /* MAIN_GESTURE_HANDLES_H_ */
