/*
 * KubOS HAL
 * Copyright (C) 2016 Kubos Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

/* Definition for I2Cx clock resources */
#define I2Cx                             I2C1
#define I2Cx_CLK_ENABLE()                __HAL_RCC_I2C1_CLK_ENABLE()
#define I2Cx_SDA_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()

#define I2Cx_FORCE_RESET()               __HAL_RCC_I2C1_FORCE_RESET()
#define I2Cx_RELEASE_RESET()             __HAL_RCC_I2C1_RELEASE_RESET()

/* Definition for I2Cx Pins */
#define I2Cx_SCL_PIN                    GPIO_PIN_6
#define I2Cx_SCL_GPIO_PORT              GPIOB
#define I2Cx_SCL_AF                     GPIO_AF4_I2C1
#define I2Cx_SDA_PIN                    GPIO_PIN_9
#define I2Cx_SDA_GPIO_PORT              GPIOB
#define I2Cx_SDA_AF                     GPIO_AF4_I2C1

/* Definition for I2Cx's NVIC */
#define I2Cx_EV_IRQn                    I2C1_EV_IRQn
#define I2Cx_EV_IRQHandler              I2C1_EV_IRQHandler
#define I2Cx_ER_IRQn                    I2C1_ER_IRQn
#define I2Cx_ER_IRQHandler              I2C1_ER_IRQHandler


/* I2C driver functions */
void HAL_IncTick(void);
uint32_t HAL_GetTick(void);

HAL_StatusTypeDef k_i2c_master_transmit_it(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, QueueHandle_t tx_queue);
HAL_StatusTypeDef k_i2c_master_receive_it(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, QueueHandle_t rx_queue);

HAL_StatusTypeDef K_I2C_MasterTransmit_TXE(I2C_HandleTypeDef *hi2c, QueueHandle_t tx_queue);
HAL_StatusTypeDef K_I2C_MasterTransmit_BTF(I2C_HandleTypeDef *hi2c, QueueHandle_t tx_queue);

HAL_StatusTypeDef K_I2C_MasterReceive_RXNE(I2C_HandleTypeDef *hi2c, QueueHandle_t rx_queue);
HAL_StatusTypeDef K_I2C_MasterReceive_BTF(I2C_HandleTypeDef *hi2c, QueueHandle_t rx_queue);
