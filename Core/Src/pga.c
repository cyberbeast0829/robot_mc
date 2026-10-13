#include "pga.h"

void MX_PGA_Init(void)
{
  PGA_InitType PGA_Initial;
  GPIO_InitType GPIO_InitStructure;

  RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_PGA | RCC_APB2_PERIPH_AFIO, ENABLE);
  RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_GPIOA, ENABLE);
  
  GPIO_InitStructure.Pin       = IU_OP1P_Pin | IU_OP1N_Pin;
  GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_ANALOG;
  GPIO_InitPeripheral(IU_OP1_GPIO_Port, &GPIO_InitStructure);

  GPIO_InitStructure.Pin       = IV_OP2P_Pin;
  GPIO_InitPeripheral(IV_OP2_GPIO_Port, &GPIO_InitStructure);
  GPIO_InitStructure.Pin       = IV_OP2N_Pin;
  GPIO_InitPeripheral(IV_OP2_GPIO_Port, &GPIO_InitStructure);

  GPIO_InitStructure.Pin       = IW_OP3P_Pin | IW_OP3N_Pin;
  GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_ANALOG;
  GPIO_InitPeripheral(IW_OP3_GPIO_Port, &GPIO_InitStructure);

  PGA_StructInit(&PGA_Initial);
  PGA_Initial.DiffModeEn = ENABLE;
  PGA_Initial.Gain1 = PGA_CTRL_CH1GAIN_32_DIFF_64;
  PGA_Initial.Gain2 = PGA_CTRL_CH1GAIN_32_DIFF_64;
  PGA_Initial.En1 = DISABLE;
  PGA_Initial.En2 = DISABLE;
  
  PGA_Init(PGA1, &PGA_Initial);
  PGA_SetVpSel(PGA1, PGA1_CTRL_VPSEL_PA1);
  PGA_SetVmSel(PGA1, PGA1_CTRL_VMSEL_PA2);
  PGA_Enable(PGA1, PGA_Channel1, ENABLE);
  PGA_Enable(PGA1, PGA_Channel2, ENABLE);

  PGA_Init(PGA2, &PGA_Initial);
  PGA_SetVpSel(PGA2, PGA2_CTRL_VPSEL_PA4);
  PGA_SetVmSel(PGA2, PGA2_CTRL_VMSEL_PA3);
  PGA_Enable(PGA2, PGA_Channel1, ENABLE);
  PGA_Enable(PGA2, PGA_Channel2, ENABLE);

  PGA_Init(PGA3, &PGA_Initial);
  PGA_SetVpSel(PGA3, PGA3_CTRL_VPSEL_PA6);
  PGA_SetVmSel(PGA3, PGA3_CTRL_VMSEL_PA5);
  PGA_Enable(PGA3, PGA_Channel1, ENABLE);
  PGA_Enable(PGA3, PGA_Channel2, ENABLE);
}


