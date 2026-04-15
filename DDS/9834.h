#ifndef __AD9834_H__
#define __AD9834_H__

#include <stdint.h>

#include "main.h"

/* Set this to the actual MCLK on the AD9834 module. */
#define AD9834_DEFAULT_MCLK_HZ 75000000UL

/* AD9834 control register bits */
#define AD9834_CTRL_B28       (1U << 13)
#define AD9834_CTRL_HLB       (1U << 12)
#define AD9834_CTRL_FSEL      (1U << 11)
#define AD9834_CTRL_PSEL      (1U << 10)
#define AD9834_CTRL_PIN_SW    (1U << 9)
#define AD9834_CTRL_RESET     (1U << 8)
#define AD9834_CTRL_SLEEP1    (1U << 7)
#define AD9834_CTRL_SLEEP12   (1U << 6)
#define AD9834_CTRL_OPBITEN   (1U << 5)
#define AD9834_CTRL_SIGN_PIB  (1U << 4)
#define AD9834_CTRL_DIV2      (1U << 3)
#define AD9834_CTRL_MODE      (1U << 1)

/* Waveform presets for the IOUT/SIGN BIT OUT outputs */
#define SINE_WAVE      (AD9834_CTRL_B28)
#define TRIANGLE_WAVE  (AD9834_CTRL_B28 | AD9834_CTRL_MODE)
#define SQUARE_WAVE    (AD9834_CTRL_B28 | AD9834_CTRL_OPBITEN | AD9834_CTRL_DIV2)

/* Compatibility aliases */
#define FREQ_0   0U
#define FREQ_1   1U
#define PHASE_0  0U
#define PHASE_1  1U

#define CS_0()    HAL_GPIO_WritePin(CS_DDS_GPIO_Port, CS_DDS_Pin, GPIO_PIN_RESET)
#define CS_1()    HAL_GPIO_WritePin(CS_DDS_GPIO_Port, CS_DDS_Pin, GPIO_PIN_SET)
#define RESET_0() HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET)
#define RESET_1() HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET)

#define AD9834_PS_SET() HAL_GPIO_WritePin(PS_GPIO_Port, PS_Pin, GPIO_PIN_SET)
#define AD9834_PS_CLR() HAL_GPIO_WritePin(PS_GPIO_Port, PS_Pin, GPIO_PIN_RESET)
#define AD9834_FS_SET() HAL_GPIO_WritePin(FS_GPIO_Port, FS_Pin, GPIO_PIN_SET)
#define AD9834_FS_CLR() HAL_GPIO_WritePin(FS_GPIO_Port, FS_Pin, GPIO_PIN_RESET)
#define AD9834_SCLK_SET() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)
#define AD9834_SCLK_CLR() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)
#define AD9834_SDATA_SET() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET)
#define AD9834_SDATA_CLR() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET)

void AD9834_Write_16Bits(uint16_t data);
void AD9834_Select_Wave(uint16_t waveform);
void AD9834_Init(void);
void AD9834_Set_Freq(uint8_t freq_number, uint32_t freq_hz);
void AD9834_Set_Phase(uint8_t phase_number, float phase_in_degrees);
void AD9834_Set_MclkHz(uint32_t mclk_hz);
uint32_t AD9834_Get_MclkHz(void);
void AD9834_Set_OutputEnabled(uint8_t enabled);
void AD9834_Set_Reset(uint8_t asserted);



#endif
