/* generated HAL header file - do not edit */
#ifndef HAL_DATA_H_
#define HAL_DATA_H_
#include <stdint.h>
#include "bsp_api.h"
#include "common_data.h"
#include "r_dtc.h"
#include "r_transfer_api.h"
#include "r_sci_uart.h"
#include "r_uart_api.h"
#include "r_sci_i2c.h"
#include "r_i2c_master_api.h"
#include "r_can.h"
#include "r_can_api.h"
#include "r_adc.h"
#include "r_adc_api.h"
#include "r_iic_master.h"
#include "r_i2c_master_api.h"
#include "r_gpt.h"
#include "r_timer_api.h"
#include "r_gpt_three_phase.h"
#include "r_three_phase_api.h"
FSP_HEADER
/* Transfer on DTC Instance. */
extern const transfer_instance_t g_transfer_uart9_tx;

/** Access the DTC instance using these structures when calling API functions directly (::p_api is not used). */
extern dtc_instance_ctrl_t g_transfer_uart9_tx_ctrl;
extern const transfer_cfg_t g_transfer_uart9_tx_cfg;
/** UART on SCI Instance. */
extern const uart_instance_t g_uart9;

/** Access the UART instance using these structures when calling API functions directly (::p_api is not used). */
extern sci_uart_instance_ctrl_t g_uart9_ctrl;
extern const uart_cfg_t g_uart9_cfg;
extern const sci_uart_extended_cfg_t g_uart9_cfg_extend;

#ifndef renesas_uart_callback
void renesas_uart_callback(uart_callback_args_t *p_args);
#endif
extern const i2c_master_cfg_t g_i2c_as5600_cfg;
/* I2C on SCI Instance. */
extern const i2c_master_instance_t g_i2c_as5600;
#ifndef sci_i2c_master_callback
void sci_i2c_master_callback(i2c_master_callback_args_t *p_args);
#endif

extern const sci_i2c_extended_cfg_t g_i2c_as5600_cfg_extend;
extern sci_i2c_instance_ctrl_t g_i2c_as5600_ctrl;
/** CAN on CAN Instance. */
extern const can_instance_t g_can0;
/** Access the CAN instance using these structures when calling API functions directly (::p_api is not used). */
extern can_instance_ctrl_t g_can0_ctrl;
extern const can_cfg_t g_can0_cfg;
extern const can_extended_cfg_t g_can0_extended_cfg;

#ifndef can_callback
void can_callback(can_callback_args_t *p_args);
#endif
#define CAN_NO_OF_MAILBOXES_g_can0 (32)
/** ADC on ADC Instance. */
extern const adc_instance_t g_adc0;

/** Access the ADC instance using these structures when calling API functions directly (::p_api is not used). */
extern adc_instance_ctrl_t g_adc0_ctrl;
extern const adc_cfg_t g_adc0_cfg;
extern const adc_channel_cfg_t g_adc0_channel_cfg;

#ifndef NULL
void NULL(adc_callback_args_t *p_args);
#endif

#ifndef NULL
#define ADC_DMAC_CHANNELS_PER_BLOCK_NULL  0
#endif
/* I2C Master on IIC Instance. */
extern const i2c_master_instance_t g_i2c_master0;

/** Access the I2C Master instance using these structures when calling API functions directly (::p_api is not used). */
extern iic_master_instance_ctrl_t g_i2c_master0_ctrl;
extern const i2c_master_cfg_t g_i2c_master0_cfg;

#ifndef i2c_master_callback
void i2c_master_callback(i2c_master_callback_args_t *p_args);
#endif
/** Timer on GPT Instance. */
extern const timer_instance_t g_timer_micros;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_timer_micros_ctrl;
extern const timer_cfg_t g_timer_micros_cfg;

#ifndef NULL
void NULL(timer_callback_args_t *p_args);
#endif
/** Timer on GPT Instance. */
extern const timer_instance_t g_timer2;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_timer2_ctrl;
extern const timer_cfg_t g_timer2_cfg;

#ifndef NULL
void NULL(timer_callback_args_t *p_args);
#endif
/** Timer on GPT Instance. */
extern const timer_instance_t g_timer1;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_timer1_ctrl;
extern const timer_cfg_t g_timer1_cfg;

#ifndef NULL
void NULL(timer_callback_args_t *p_args);
#endif
/** Timer on GPT Instance. */
extern const timer_instance_t g_timer0;

/** Access the GPT instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_instance_ctrl_t g_timer0_ctrl;
extern const timer_cfg_t g_timer0_cfg;

#ifndef NULL
void NULL(timer_callback_args_t *p_args);
#endif
/** GPT Three-Phase Instance. */
extern const three_phase_instance_t g_three_phase_pwm0;

/** Access the GPT Three-Phase instance using these structures when calling API functions directly (::p_api is not used). */
extern gpt_three_phase_instance_ctrl_t g_three_phase_pwm0_ctrl;
extern const three_phase_cfg_t g_three_phase_pwm0_cfg;
void hal_entry(void);
void g_hal_init(void);
FSP_FOOTER
#endif /* HAL_DATA_H_ */
