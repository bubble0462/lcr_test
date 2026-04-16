#include "9834.h"

static uint16_t ad9834_control_word = AD9834_CTRL_B28;
static uint32_t ad9834_mclk_hz = AD9834_DEFAULT_MCLK_HZ;

static void AD9834_Write_ControlRegister(void)
{
  AD9834_Write_16Bits(ad9834_control_word);
}

void AD9834_Write_16Bits(uint16_t data)
{
  uint16_t tx_word = data;
  uint8_t bit_index;

  AD9834_SCLK_SET();
  CS_0();

  for (bit_index = 0U; bit_index < 16U; bit_index++)
  {
    if ((tx_word & 0x8000U) != 0U)
    {
      AD9834_SDATA_SET();
    }
    else
    {
      AD9834_SDATA_CLR();
    }

    AD9834_SCLK_CLR();
    __NOP();
    AD9834_SCLK_SET();

    tx_word <<= 1;
  }

  CS_1();
  AD9834_SDATA_CLR();
}

void AD9834_Set_MclkHz(uint32_t mclk_hz)
{
  if (mclk_hz != 0U)
  {
    ad9834_mclk_hz = mclk_hz;
  }
}

uint32_t AD9834_Get_MclkHz(void)
{
  return ad9834_mclk_hz;
}

void AD9834_Set_Reset(uint8_t asserted)
{
  if (asserted != 0U)
  {
    ad9834_control_word |= AD9834_CTRL_RESET;
  }
  else
  {
    ad9834_control_word &= (uint16_t)~AD9834_CTRL_RESET;
  }

  AD9834_Write_ControlRegister();
}

void AD9834_Set_OutputEnabled(uint8_t enabled)
{
  if (enabled != 0U)
  {
    ad9834_control_word &= (uint16_t)~(AD9834_CTRL_SLEEP1 | AD9834_CTRL_SLEEP12);
  }
  else
  {
    ad9834_control_word |= (AD9834_CTRL_SLEEP1 | AD9834_CTRL_SLEEP12);
  }

  AD9834_Write_ControlRegister();
}

void AD9834_Init(void)
{
  AD9834_FS_CLR();
  AD9834_PS_CLR();
  RESET_0();

  ad9834_control_word = AD9834_CTRL_B28 | AD9834_CTRL_RESET;
  AD9834_Write_ControlRegister();

  AD9834_Set_Freq(FREQ_0, 0U);
  AD9834_Set_Freq(FREQ_1, 0U);
  AD9834_Set_Phase(PHASE_0, 0.0f);
  AD9834_Set_Phase(PHASE_1, 0.0f);

  ad9834_control_word = AD9834_CTRL_B28;
  AD9834_Write_ControlRegister();
}

void AD9834_Select_Wave(uint16_t waveform)
{
  const uint16_t waveform_mask = AD9834_CTRL_OPBITEN |
                                 AD9834_CTRL_SIGN_PIB |
                                 AD9834_CTRL_DIV2 |
                                 AD9834_CTRL_MODE;

  ad9834_control_word &= (uint16_t)~(AD9834_CTRL_RESET | waveform_mask);
  ad9834_control_word |= (uint16_t)(AD9834_CTRL_B28 | (waveform & waveform_mask));

  AD9834_Write_ControlRegister();
}

void AD9834_Set_Freq(uint8_t freq_number, uint32_t freq_hz)
{
  AD9834_Set_FreqMilliHz(freq_number, freq_hz * 1000U);
}

void AD9834_Set_FreqMilliHz(uint8_t freq_number, uint32_t freq_millihz)
{
  uint32_t freq_reg;
  uint16_t reg_addr;
  uint16_t freq_lsb;
  uint16_t freq_msb;

  if ((ad9834_control_word & AD9834_CTRL_B28) == 0U)
  {
    ad9834_control_word |= AD9834_CTRL_B28;
    AD9834_Write_ControlRegister();
  }

  if (ad9834_mclk_hz == 0U)
  {
    return;
  }

  freq_reg = (uint32_t)(((((uint64_t)freq_millihz << 28) + ((uint64_t)ad9834_mclk_hz * 500ULL)) /
                         ((uint64_t)ad9834_mclk_hz * 1000ULL)));
  if (freq_reg > 0x0FFFFFFFU)
  {
    freq_reg = 0x0FFFFFFFU;
  }

  reg_addr = (freq_number == FREQ_1) ? 0x8000U : 0x4000U;
  freq_lsb = (uint16_t)(reg_addr | (freq_reg & 0x3FFFU));
  freq_msb = (uint16_t)(reg_addr | ((freq_reg >> 14) & 0x3FFFU));

  AD9834_Write_16Bits(freq_lsb);
  AD9834_Write_16Bits(freq_msb);
}

void AD9834_Set_Phase(uint8_t phase_number, float phase_in_degrees)
{
  uint16_t phase_word;
  uint16_t phase_value;

  while (phase_in_degrees < 0.0f)
  {
    phase_in_degrees += 360.0f;
  }

  while (phase_in_degrees >= 360.0f)
  {
    phase_in_degrees -= 360.0f;
  }

  phase_value = (uint16_t)(((phase_in_degrees * 4096.0f) / 360.0f) + 0.5f) & 0x0FFFU;
  phase_word = (phase_number == PHASE_1) ? (uint16_t)(0xE000U | phase_value)
                                         : (uint16_t)(0xC000U | phase_value);

  AD9834_Write_16Bits(phase_word);
}
