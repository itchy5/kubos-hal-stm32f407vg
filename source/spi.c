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

#include "kubos-hal-stm32f4/spi.h"

static hal_spi_handle * hal_spi_get_handle(KSPINum spi);
static hal_spi_handle * hal_spi_device_init(KSPI * spi);
static KSPIStatus hal_spi_hw_init(hal_spi_handle * handle);
static void hal_spi_terminate(hal_spi_handle * handle);
static void hal_spi_gpio_init();

static hal_spi_handle hal_spi_dev[K_NUM_SPI];
static uint32_t spi_timeout = 1000;

/** Functions implemented from KubOS-HAL SPI Interface **/

void kprv_spi_dev_init(KSPINum spi_num)
{
    KSPI * spi = kprv_spi_get(spi_num);
    hal_spi_handle * handle = hal_spi_device_init(spi);
    hal_spi_hw_init(handle);
}

void kprv_spi_dev_terminate(KSPINum spi)
{
    hal_spi_handle * handle = hal_spi_get_handle(spi);
    hal_spi_terminate(handle);
}

KSPIStatus kprv_spi_write(KSPINum spi, uint8_t * buffer, uint32_t len)
{
    hal_spi_handle * handle = hal_spi_get_handle(spi);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&(handle->hal_handle), buffer, len, spi_timeout);
    return (KSPIStatus)status;
}

KSPIStatus kprv_spi_read(KSPINum spi, uint8_t * buffer, uint32_t len)
{
    hal_spi_handle * handle = hal_spi_get_handle(spi);
    HAL_StatusTypeDef status = HAL_SPI_Receive(&(handle->hal_handle), buffer, len, spi_timeout);
    return (KSPIStatus)status;
}

KSPIStatus kprv_spi_write_read(KSPINum spi, uint8_t * txBuffer, uint8_t * rxBuffer, uint32_t len)
{
    hal_spi_handle * handle = hal_spi_get_handle(spi);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&(handle->hal_handle), txBuffer, rxBuffer, len, spi_timeout);
    return (KSPIStatus)status;
}

/** Private functions **/

static hal_spi_handle * hal_spi_get_handle(KSPINum spi)
{
    return &hal_spi_dev[spi];
}

static hal_spi_handle * hal_spi_device_init(KSPI * spi)
{
    hal_spi_handle * handle = NULL;
    if (spi != NULL)
    {
        handle = hal_spi_get_handle(spi->bus_num);
        if (handle != NULL)
        {
            //KSPIConf conf = spi->config;
            handle->kspi = spi;
            switch(spi->bus_num)
            {
                case K_SPI1:
                {
                    handle->hal_handle.Instance = SPI1;
                    break;
                }
                default:
                {
                    handle = NULL;
                }
            }
        }
    }
    return handle;
}

static void hal_spi_terminate(hal_spi_handle * handle)
{
    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();

    /* de-init pins */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);
}

static KSPIStatus hal_spi_hw_init(hal_spi_handle * handle)
{
    SPI_HandleTypeDef * SPIHandle = &(handle->hal_handle);
    if (handle->kspi->bus_num == K_SPI1)
    {
        /* Enable SPI clock */
        __HAL_RCC_SPI1_CLK_ENABLE();

        /* Init pins */
        hal_spi_gpio_init();

        /* Set options */
        SPIHandle->Init.DataSize = SPI_DATASIZE_8BIT;
    }

    /* Fill SPI settings */
    SPIHandle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    SPIHandle->Init.FirstBit = SPI_FIRSTBIT_MSB;
    SPIHandle->Init.Mode = SPI_MODE_MASTER;

    SPIHandle->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    SPIHandle->Init.CRCPolynomial = 7;
    SPIHandle->Init.TIMode = SPI_TIMODE_DISABLE;
    SPIHandle->Init.NSS = SPI_NSS_SOFT;
    SPIHandle->Init.Direction = SPI_DIRECTION_2LINES;

    /* Disable first */
    __HAL_SPI_DISABLE(SPIHandle);

    if (HAL_SPI_Init(SPIHandle) != HAL_OK)
    {
        return SPI_ERROR;
    }

    /* Enable SPI */
    __HAL_SPI_ENABLE(SPIHandle);

    return SPI_OK;
}


static void hal_spi_gpio_init()
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    /* SPI GPIO Init */
    GPIO_InitStruct.Pin       = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}