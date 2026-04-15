/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PS_Pin|IO1_Pin|DDS_SW_S1_Pin|DDS_SW_S0_Pin
                          |IO_SW_S1_Pin|IO2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IR_S0_Pin|IR_S1_Pin|GPIO_PIN_5|RESET_Pin
                          |GPIO_PIN_7|FS_Pin|VR_S1_Pin|VR_S0_Pin
                          |EN__5V_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_DDS_GPIO_Port, CS_DDS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Q7_RELAY_Pin|FIL_SW_S1_Pin|FIL_SW_S0_Pin|GAIN_SW_S1_Pin
                          |GAIN_SW_S0_Pin|EN__3_3V_Pin|EN__3_3VB4_Pin|I2C_SCL_Pin
                          |I2C_SDA_Pin|ANGLE_90_SW_Pin|ANGLE_0_SW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IO_SW_S3_GPIO_Port, IO_SW_S3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VOUT_EN_GPIO_Port, VOUT_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SOGI_VOP_COMP_Pin */
  GPIO_InitStruct.Pin = SOGI_VOP_COMP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SOGI_VOP_COMP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY4_Pin */
  GPIO_InitStruct.Pin = KEY4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY1_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PS_Pin IO1_Pin DDS_SW_S1_Pin DDS_SW_S0_Pin
                           IO_SW_S1_Pin IO_SW_S3_Pin IO2_Pin */
  GPIO_InitStruct.Pin = PS_Pin|IO1_Pin|DDS_SW_S1_Pin|DDS_SW_S0_Pin
                          |IO_SW_S1_Pin|IO_SW_S3_Pin|IO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IR_S0_Pin IR_S1_Pin CS_DDS_Pin RESET_Pin
                           FS_Pin VR_S1_Pin VR_S0_Pin EN__5V_Pin */
  GPIO_InitStruct.Pin = IR_S0_Pin|IR_S1_Pin|CS_DDS_Pin|RESET_Pin
                          |FS_Pin|VR_S1_Pin|VR_S0_Pin|EN__5V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Q7_RELAY_Pin GAIN_SW_S1_Pin GAIN_SW_S0_Pin EN__3_3V_Pin
                           EN__3_3VB4_Pin */
  GPIO_InitStruct.Pin = Q7_RELAY_Pin|GAIN_SW_S1_Pin|GAIN_SW_S0_Pin|EN__3_3V_Pin
                          |EN__3_3VB4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : FIL_SW_S1_Pin FIL_SW_S0_Pin ANGLE_90_SW_Pin ANGLE_0_SW_Pin */
  GPIO_InitStruct.Pin = FIL_SW_S1_Pin|FIL_SW_S0_Pin|ANGLE_90_SW_Pin|ANGLE_0_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CHRG_Pin STDBY_Pin */
  GPIO_InitStruct.Pin = CHRG_Pin|STDBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VOUT_EN_Pin */
  GPIO_InitStruct.Pin = VOUT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(VOUT_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C_SCL_Pin I2C_SDA_Pin */
  GPIO_InitStruct.Pin = I2C_SCL_Pin|I2C_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
