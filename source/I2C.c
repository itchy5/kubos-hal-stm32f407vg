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
/* Master mode I2C */

#include "kubos-hal/I2C.h" /* hal header */
#include "I2C.h" /* stm header */
#include "FreeRTOS.h"
#include "task.h"
//#include "stm32f4xx_rcc.h"
//#include "stm32f4xx_gpio.h"
#include "stm32f4xx.h"


I2C_HandleTypeDef I2cHandle;

/* static functions */
//static HAL_StatusTypeDef K_I2C_MasterTransmit_TXE(I2C_HandleTypeDef *hi2c, QueueHandle_t tx_queue);
//static HAL_StatusTypeDef K_I2C_MasterTransmit_BTF(I2C_HandleTypeDef *hi2c, QueueHandle_t tx_queue);

//static HAL_StatusTypeDef K_I2C_MasterReceive_RXNE(I2C_HandleTypeDef *hi2c, QueueHandle_t rx_queue);
//static HAL_StatusTypeDef K_I2C_MasterReceive_BTF(I2C_HandleTypeDef *hi2c, QueueHandle_t rx_queue);


void kprv_i2c_dev_init(KI2CNum i2c)
{
    KI2C *k_i2c = kprv_i2c_get(i2c);

    HAL_Init();

    /* which i2c instance on chip */
    I2cHandle.Instance = I2Cx;

    /* set comm params for kubos hal */
    k_i2c->conf.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
    k_i2c->conf.ClockSpeed = 400000;
	k_i2c->conf.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	k_i2c->conf.DutyCycle = I2C_DUTYCYCLE_16_9;
	k_i2c->conf.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	k_i2c->conf.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	k_i2c->conf.OwnAddress1 = I2C_ADDRESS;
	k_i2c->conf.OwnAddress2 = 0xFE;

	/* set cubeF4 hal */
	I2cHandle.Init.AddressingMode  = k_i2c->conf.AddressingMode;
	I2cHandle.Init.ClockSpeed      = k_i2c->conf.ClockSpeed;
	I2cHandle.Init.DualAddressMode = k_i2c->conf.DualAddressMode;
	I2cHandle.Init.DutyCycle       = k_i2c->conf.DutyCycle;
	I2cHandle.Init.GeneralCallMode = k_i2c->conf.GeneralCallMode;
	I2cHandle.Init.NoStretchMode   = k_i2c->conf.NoStretchMode;
	I2cHandle.Init.OwnAddress1     = k_i2c->conf.OwnAddress1;
	I2cHandle.Init.OwnAddress2     = k_i2c->conf.OwnAddress2;

    HAL_I2C_Init(&I2cHandle);


}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{

	I2Cx_CLK_ENABLE();
	I2Cx_SDA_GPIO_CLK_ENABLE();
	I2Cx_SCL_GPIO_CLK_ENABLE();

	k_gpio_write(I2Cx_SCL_PIN, ENABLE);
	k_gpio_write(I2Cx_SDA_PIN, ENABLE);

    /* enable interrupts */
    __HAL_I2C_ENABLE_IT(&I2cHandle, I2C_IT_EVT | I2C_IT_ERR);

}


BaseType_t queue_push(QueueHandle_t *queue, uint8_t c,
                                    TickType_t timeout, void *task_woken)
{
    if (!task_woken) {
        return xQueueSendToBack(queue, &c, timeout);
    } else {
        return xQueueSendToBackFromISR(queue, &c, task_woken);
    }
}


int kprv_i2c_transmit_i2c(KI2CNum i2c, uint16_t DevAddress, uint8_t *ptr, int len)
{
	KI2C *k_i2c = kprv_i2c_get(i2c);

	int i = 0;
	/* put message in queue */
    for (; i < len; i++, ptr++) {
        BaseType_t result = queue_push(k_i2c->tx_queue, *ptr, portMAX_DELAY, NULL);
        if (result != pdTRUE) {
            return i;
        }
    }
    /* send queue to bus */
	if(k_i2c_master_transmit_it(&I2cHandle, DevAddress, k_i2c->tx_queue) == HAL_OK)
		return 0;
	else
		return -1;
}

int kprv_i2c_receive_i2c(KI2CNum i2c, uint16_t DevAddress)
{
	KI2C *k_i2c = kprv_i2c_get(i2c);

	if(k_i2c_master_receive_it(&I2cHandle, DevAddress, k_i2c->rx_queue) == HAL_OK)
			return 0;
		else
			return -1;
}

int kprv_i2c_get_state()
{
	if(HAL_I2C_GetState(&I2cHandle) == HAL_OK)
		return 0;
	else
		return -1;
}

int kprv_i2c_get_error()
{
	if(HAL_I2C_GetError(&I2cHandle) == HAL_OK)
		return 0;
	else
		return -1;
}



/**
  * @brief This function is called to increment  a global variable "uwTick"
  *        used as application time base.
  * @note In the default implementation, this variable is incremented each 1ms
  *       in Systick ISR.
 * @note This function is declared as __weak to be overwritten in case of other
  *      implementations in user file.
  * @retval None
  */
void HAL_IncTick(void)
{
	/* FRTOS tick */
	//xTaskIncrementTick();
}

/**
  * @brief Provides a tick value in millisecond.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @retval tick value
  */
uint32_t HAL_GetTick(void)
{
  return xTaskGetTickCount();
}




HAL_StatusTypeDef k_i2c_master_transmit_it(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, QueueHandle_t tx_queue)
{
	if(hi2c->State == HAL_I2C_STATE_READY)
	  {
	    if(tx_queue == 0 || uxQueueMessagesWaiting(tx_queue) == 0)
	    {
	      return  HAL_ERROR;
	    }

	    /*
	    if(__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET)
	    {
	    	return HAL_BUSY;
	    }
	    */
	    // Process Locked
	    __HAL_LOCK(hi2c);

	    hi2c->State = HAL_I2C_STATE_BUSY_TX;
	    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

	    // Send Slave Address
	    if(I2C_MasterRequestWrite(hi2c, DevAddress, I2C_TIMEOUT_FLAG) != HAL_OK)
	    {
	      if(hi2c->ErrorCode == HAL_I2C_ERROR_AF)
	      {
	        // Process Unlocked
	        __HAL_UNLOCK(hi2c);
	        return HAL_ERROR;
	      }
	      else
	      {
	        // Process Unlocked
	        __HAL_UNLOCK(hi2c);
	        return HAL_TIMEOUT;
	      }
	    }
	    // Clear ADDR flag
	    __HAL_I2C_CLEAR_ADDRFLAG(hi2c);

	    // Process Unlocked
	    __HAL_UNLOCK(hi2c);

	    // Enable EVT, BUF and ERR interrupt
	    __HAL_I2C_ENABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

	    return HAL_OK;
	  }
	  else
	  {
	    return HAL_BUSY;
	  }
}

HAL_StatusTypeDef k_i2c_master_receive_it(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, QueueHandle_t rx_queue)
{
  if(hi2c->State == HAL_I2C_STATE_READY)
  {
	  if(rx_queue == 0 || uxQueueMessagesWaiting(rx_queue) == 0)
    {
      return  HAL_ERROR;
    }

	  /*
    if(__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET)
    {
		return HAL_BUSY;
    }
    */
    // Process Locked
    __HAL_LOCK(hi2c);

    hi2c->State = HAL_I2C_STATE_BUSY_RX;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    // Send Slave Address
    if(I2C_MasterRequestRead(hi2c, DevAddress, I2C_TIMEOUT_FLAG) != HAL_OK)
    {
      if(hi2c->ErrorCode == HAL_I2C_ERROR_AF)
      {
        // Process Unlocked
        __HAL_UNLOCK(hi2c);
        return HAL_ERROR;
      }
      else
      {
        // Process Unlocked
        __HAL_UNLOCK(hi2c);
        return HAL_TIMEOUT;
      }
    }

    if(uxQueueMessagesWaiting(rx_queue) == 1)
    {
      // Disable Acknowledge
      hi2c->Instance->CR1 &= ~I2C_CR1_ACK;

      // Clear ADDR flag
      __HAL_I2C_CLEAR_ADDRFLAG(hi2c);

      // Generate Stop
      hi2c->Instance->CR1 |= I2C_CR1_STOP;
    }
    else if(uxQueueMessagesWaiting(rx_queue) == 2)
    {
      // Disable Acknowledge
      hi2c->Instance->CR1 &= ~I2C_CR1_ACK;

      // Enable Pos
      hi2c->Instance->CR1 |= I2C_CR1_POS;

      // Clear ADDR flag
      __HAL_I2C_CLEAR_ADDRFLAG(hi2c);
    }
    else
    {
      // Enable Acknowledge
      hi2c->Instance->CR1 |= I2C_CR1_ACK;

      // Clear ADDR flag
      __HAL_I2C_CLEAR_ADDRFLAG(hi2c);
    }
    // Process Unlocked
    __HAL_UNLOCK(hi2c);

    // Enable EVT, BUF and ERR interrupt
    __HAL_I2C_ENABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}


void K_HAL_I2C_EV_IRQHandler(I2C_HandleTypeDef *hi2c, KI2CNum i2c)
{
  KI2C *k_i2c = kprv_i2c_get(i2c);

  uint32_t tmp1 = 0, tmp2 = 0, tmp3 = 0, tmp4 = 0;
  // Master mode selected
  if(__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_MSL) == SET)
  {
    // I2C in mode Transmitter -----------------------------------------------
    if(__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TRA) == SET)
    {
      tmp1 = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TXE);
      tmp2 = __HAL_I2C_GET_IT_SOURCE(hi2c, I2C_IT_BUF);
      tmp3 = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BTF);
      tmp4 = __HAL_I2C_GET_IT_SOURCE(hi2c, I2C_IT_EVT);
      // TXE set and BTF reset -----------------------------------------------
      if((tmp1 == SET) && (tmp2 == SET) && (tmp3 == RESET))
      {
        K_I2C_MasterTransmit_TXE(hi2c, k_i2c->tx_queue);
      }
      // BTF set -------------------------------------------------------------
      else if((tmp3 == SET) && (tmp4 == SET))
      {
        K_I2C_MasterTransmit_BTF(hi2c, k_i2c->tx_queue);
      }
    }
    // I2C in mode Receiver --------------------------------------------------
    else
    {
      tmp1 = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_RXNE);
      tmp2 = __HAL_I2C_GET_IT_SOURCE(hi2c, I2C_IT_BUF);
      tmp3 = __HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BTF);
      tmp4 = __HAL_I2C_GET_IT_SOURCE(hi2c, I2C_IT_EVT);
      // RXNE set and BTF reset -----------------------------------------------
      if((tmp1 == SET) && (tmp2 == SET) && (tmp3 == RESET))
      {
        K_I2C_MasterReceive_RXNE(hi2c, k_i2c->rx_queue);
      }
      // BTF set -------------------------------------------------------------
      else if((tmp3 == SET) && (tmp4 == SET))
      {
        K_I2C_MasterReceive_BTF(hi2c, k_i2c->rx_queue);
      }
    }
  }
}

HAL_StatusTypeDef K_I2C_MasterTransmit_TXE(I2C_HandleTypeDef *hi2c, QueueHandle_t tx_queue)
{
	uint16_t* DRptr;
	BaseType_t xHigherPriorityTaskWoken;

	xHigherPriorityTaskWoken = pdFALSE;

  // Write data to DR
  xQueueReceiveFromISR( tx_queue, ( void * ) &DRptr, &xHigherPriorityTaskWoken);
  hi2c->Instance->DR = *DRptr;

  if(uxQueueMessagesWaiting(tx_queue) == 0)
  {
    // Disable BUF interrupt
    __HAL_I2C_DISABLE_IT(hi2c, I2C_IT_BUF);
  }

  return HAL_OK;
}

HAL_StatusTypeDef K_I2C_MasterTransmit_BTF(I2C_HandleTypeDef *hi2c, QueueHandle_t tx_queue)
{
	uint16_t* DRptr;
	BaseType_t xHigherPriorityTaskWoken;

	xHigherPriorityTaskWoken = pdFALSE;

  if(uxQueueMessagesWaiting(tx_queue) != 0)
  {
    // Write data to DR
	xQueueReceiveFromISR( tx_queue, ( void * ) &DRptr, &xHigherPriorityTaskWoken);
    hi2c->Instance->DR = *DRptr;
  }
  else
  {
    // Disable EVT, BUF and ERR interrupt
    __HAL_I2C_DISABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

    // Generate Stop
    hi2c->Instance->CR1 |= I2C_CR1_STOP;

    // Wait until BUSY flag is reset
    if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_FLAG) != HAL_OK)
    {
      return HAL_TIMEOUT;
    }

    if(hi2c->State == HAL_I2C_STATE_MEM_BUSY_TX)
    {
      hi2c->State = HAL_I2C_STATE_READY;
      HAL_I2C_MemTxCpltCallback(hi2c);
    }
    else
    {
      hi2c->State = HAL_I2C_STATE_READY;
      HAL_I2C_MasterTxCpltCallback(hi2c);
    }
  }

  return HAL_OK;
}

HAL_StatusTypeDef K_I2C_MasterReceive_RXNE(I2C_HandleTypeDef *hi2c, QueueHandle_t rx_queue)
{
  uint32_t tmp = 0;
  uint16_t* DRptr;
  BaseType_t xHigherPriorityTaskWoken;

  xHigherPriorityTaskWoken = pdFALSE;

  tmp = uxQueueMessagesWaiting(rx_queue);
  if(tmp > 3)
  {
    /* Read data from DR */
	  *DRptr = hi2c->Instance->DR;
	  xQueueSendToBackFromISR( rx_queue, &DRptr, &xHigherPriorityTaskWoken );
  }
  else if((tmp == 2) || (tmp == 3))
  {
    /* Disable BUF interrupt */
    __HAL_I2C_DISABLE_IT(hi2c, I2C_IT_BUF);
  }
  else
  {
    /* Disable EVT, BUF and ERR interrupt */
    __HAL_I2C_DISABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR);

    /* Read data from DR */
    *DRptr = hi2c->Instance->DR;
    xQueueSendToBackFromISR( rx_queue, &DRptr, &xHigherPriorityTaskWoken );

    /* Wait until BUSY flag is reset */
    if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_FLAG) != HAL_OK)
    {
      return HAL_TIMEOUT;
    }

    /* Disable Pos */
    hi2c->Instance->CR1 &= ~I2C_CR1_POS;

    if(hi2c->State == HAL_I2C_STATE_MEM_BUSY_RX)
    {
      hi2c->State = HAL_I2C_STATE_READY;

      HAL_I2C_MemRxCpltCallback(hi2c);
    }
    else
    {
      hi2c->State = HAL_I2C_STATE_READY;

      HAL_I2C_MasterRxCpltCallback(hi2c);
    }
  }

  return HAL_OK;
}

HAL_StatusTypeDef K_I2C_MasterReceive_BTF(I2C_HandleTypeDef *hi2c, QueueHandle_t rx_queue)
{
	uint16_t* DRptr;
	BaseType_t xHigherPriorityTaskWoken;

	xHigherPriorityTaskWoken = pdFALSE;

  if(uxQueueMessagesWaiting(rx_queue) == 3)
  {
    /* Disable Acknowledge */
    hi2c->Instance->CR1 &= ~I2C_CR1_ACK;

    /* Read data from DR */
    *DRptr = hi2c->Instance->DR;
    xQueueSendToBackFromISR( rx_queue, &DRptr, &xHigherPriorityTaskWoken );
  }
  else if(uxQueueMessagesWaiting(rx_queue) == 2)
  {
    /* Read data from DR */
	*DRptr = hi2c->Instance->DR;
	xQueueSendToBackFromISR( rx_queue, &DRptr, &xHigherPriorityTaskWoken );

    /* Read data from DR */
    *DRptr = hi2c->Instance->DR;
    xQueueSendToBackFromISR( rx_queue, &DRptr, &xHigherPriorityTaskWoken );

    /* Disable EVT and ERR interrupt */

    __HAL_I2C_DISABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_ERR);

    /* Disable Pos */
    hi2c->Instance->CR1 &= ~I2C_CR1_POS;

    if(hi2c->State == HAL_I2C_STATE_MEM_BUSY_RX)
    {
      hi2c->State = HAL_I2C_STATE_READY;

      HAL_I2C_MemRxCpltCallback(hi2c);
    }
    else
    {
      hi2c->State = HAL_I2C_STATE_READY;

      HAL_I2C_MasterRxCpltCallback(hi2c);
    }
  }
  else
  {
    /* Read data from DR */
    *DRptr = hi2c->Instance->DR;
    xQueueSendToBackFromISR( rx_queue, &DRptr, &xHigherPriorityTaskWoken );
  }

  return HAL_OK;
}
