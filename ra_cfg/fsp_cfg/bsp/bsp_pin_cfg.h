/* generated configuration header file - do not edit */
#ifndef BSP_PIN_CFG_H_
#define BSP_PIN_CFG_H_
#include "r_ioport.h"

/* Common macro for FSP header files. There is also a corresponding FSP_FOOTER macro at the end of this file. */
FSP_HEADER

#define ADC2_1 (BSP_IO_PORT_00_PIN_01)
#define ADC2_2 (BSP_IO_PORT_00_PIN_02)
#define ADC1_1 (BSP_IO_PORT_00_PIN_13)
#define ADC1_2 (BSP_IO_PORT_00_PIN_14)
#define PWMB_3_p100 (BSP_IO_PORT_01_PIN_00)
#define PWMA_3_p102 (BSP_IO_PORT_01_PIN_02)
#define can_tx (BSP_IO_PORT_01_PIN_03)
#define PWMA_2_p104 (BSP_IO_PORT_01_PIN_04)
#define PWMA_1_p108 (BSP_IO_PORT_01_PIN_08)
#define usbtx (BSP_IO_PORT_01_PIN_09)
#define usbrx (BSP_IO_PORT_01_PIN_10)
#define PWMB_1_p112 (BSP_IO_PORT_01_PIN_12)
#define as5600scl (BSP_IO_PORT_02_PIN_06)
#define as5600sda (BSP_IO_PORT_02_PIN_07)
#define PWMB_2_P301 (BSP_IO_PORT_03_PIN_01)
#define can_rx (BSP_IO_PORT_04_PIN_02)
#define oled_sda (BSP_IO_PORT_04_PIN_07)
#define oled_scl (BSP_IO_PORT_04_PIN_08)

extern const ioport_cfg_t g_bsp_pin_cfg; /* R7FA4M2AD3CFL.pincfg */

void BSP_PinConfigSecurityInit();

/* Common macro for FSP header files. There is also a corresponding FSP_HEADER macro at the top of this file. */
FSP_FOOTER
#endif /* BSP_PIN_CFG_H_ */
