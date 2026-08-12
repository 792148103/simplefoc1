#include "hal_data.h"

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#endif

namespace
{
constexpr uint32_t kPwmBPeriodCounts = 0xBB8U;     // 15 kHz triangle PWM.
constexpr uint32_t kPwmBDutyCounts = 0x5DCU;       // 50% duty at startup.

#define RESTORE_GPT_PWM_EXT(name)                                                    \
    const gpt_extended_pwm_cfg_t name##_pwm_extend =                                 \
    {                                                                                \
        .trough_ipl = (BSP_IRQ_DISABLED),                                            \
        .trough_irq = FSP_INVALID_VECTOR,                                            \
        .poeg_link = GPT_POEG_LINK_POEG0,                                            \
        .output_disable = (gpt_output_disable_t) (GPT_OUTPUT_DISABLE_NONE),          \
        .adc_trigger = (gpt_adc_trigger_t) (GPT_ADC_TRIGGER_NONE),                   \
        .dead_time_count_up = 0,                                                     \
        .dead_time_count_down = 0,                                                   \
        .adc_a_compare_match = 0,                                                    \
        .adc_b_compare_match = 0,                                                    \
        .interrupt_skip_source = GPT_INTERRUPT_SKIP_SOURCE_NONE,                     \
        .interrupt_skip_count = GPT_INTERRUPT_SKIP_COUNT_0,                          \
        .interrupt_skip_adc = GPT_INTERRUPT_SKIP_ADC_NONE,                           \
        .gtioca_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED,                      \
        .gtiocb_disable_setting = GPT_GTIOC_DISABLE_PROHIBITED,                      \
    }

#define RESTORE_GPT_EXT(name)                                                        \
    const gpt_extended_cfg_t name##_extend =                                         \
    {                                                                                \
        .gtioca = { .output_enabled = true, .stop_level = GPT_PIN_LEVEL_LOW },       \
        .gtiocb = { .output_enabled = true, .stop_level = GPT_PIN_LEVEL_LOW },       \
        .start_source = (gpt_source_t) (GPT_SOURCE_NONE),                            \
        .stop_source = (gpt_source_t) (GPT_SOURCE_NONE),                             \
        .clear_source = (gpt_source_t) (GPT_SOURCE_NONE),                            \
        .capture_a_source = (gpt_source_t) (GPT_SOURCE_NONE),                        \
        .capture_b_source = (gpt_source_t) (GPT_SOURCE_NONE),                        \
        .count_up_source = (gpt_source_t) (GPT_SOURCE_NONE),                         \
        .count_down_source = (gpt_source_t) (GPT_SOURCE_NONE),                       \
        .capture_filter_gtioca = GPT_CAPTURE_FILTER_NONE,                            \
        .capture_filter_gtiocb = GPT_CAPTURE_FILTER_NONE,                            \
        .capture_a_ipl = (BSP_IRQ_DISABLED),                                         \
        .capture_b_ipl = (BSP_IRQ_DISABLED),                                         \
        .compare_match_c_ipl = (BSP_IRQ_DISABLED),                                   \
        .compare_match_d_ipl = (BSP_IRQ_DISABLED),                                   \
        .compare_match_e_ipl = (BSP_IRQ_DISABLED),                                   \
        .compare_match_f_ipl = (BSP_IRQ_DISABLED),                                   \
        .capture_a_irq = FSP_INVALID_VECTOR,                                         \
        .capture_b_irq = FSP_INVALID_VECTOR,                                         \
        .compare_match_c_irq = FSP_INVALID_VECTOR,                                   \
        .compare_match_d_irq = FSP_INVALID_VECTOR,                                   \
        .compare_match_e_irq = FSP_INVALID_VECTOR,                                   \
        .compare_match_f_irq = FSP_INVALID_VECTOR,                                   \
        .compare_match_value = { 0U, 0U, 0U, 0U, 0U, 0U },                           \
        .compare_match_status = 0U,                                                  \
        .p_pwm_cfg = &name##_pwm_extend,                                             \
        .gtior_setting = { .gtior = 0U },                                             \
        .gtioca_polarity = GPT_GTIOC_POLARITY_NORMAL,                                \
        .gtiocb_polarity = GPT_GTIOC_POLARITY_NORMAL,                                \
    }

#define RESTORE_GPT_TIMER(name, channel_num)                                         \
    RESTORE_GPT_PWM_EXT(name);                                                       \
    RESTORE_GPT_EXT(name);                                                           \
    extern "C" gpt_instance_ctrl_t name##_ctrl;                                      \
    extern "C" const timer_cfg_t name##_cfg =                                       \
    {                                                                                \
        .mode = TIMER_MODE_TRIANGLE_WAVE_SYMMETRIC_PWM,                              \
        .period_counts = kPwmBPeriodCounts,                                          \
        .source_div = (timer_source_div_t) 0,                                        \
        .duty_cycle_counts = kPwmBDutyCounts,                                        \
        .channel = channel_num,                                                      \
        .cycle_end_ipl = (BSP_IRQ_DISABLED),                                         \
        .cycle_end_irq = FSP_INVALID_VECTOR,                                         \
        .p_callback = NULL,                                                          \
        .p_context = NULL,                                                           \
        .p_extend = &name##_extend,                                                  \
    };                                                                               \
    extern "C" const timer_instance_t name =                                         \
    {                                                                                \
        .p_ctrl = &name##_ctrl,                                                       \
        .p_cfg = &name##_cfg,                                                         \
        .p_api = &g_timer_on_gpt,                                                     \
    }
}

extern "C"
{
gpt_instance_ctrl_t g_timer3_ctrl;
gpt_instance_ctrl_t g_timer4_ctrl;
gpt_instance_ctrl_t g_timer5_ctrl;

RESTORE_GPT_TIMER(g_timer3, 3);
RESTORE_GPT_TIMER(g_timer4, 4);
RESTORE_GPT_TIMER(g_timer5, 5);

gpt_three_phase_instance_ctrl_t g_three_phase0_ctrl;

const three_phase_cfg_t g_three_phase0_cfg =
{
    .buffer_mode = (THREE_PHASE_BUFFER_MODE_SINGLE),
    .p_timer_instance = { &g_timer3, &g_timer4, &g_timer5 },
    .callback_ch = THREE_PHASE_CHANNEL_U,
    .channel_mask = (1 << 3) | (1 << 4) | (1 << 5),
    .p_context = NULL,
    .p_extend = NULL,
};

const three_phase_instance_t g_three_phase0 =
{
    .p_ctrl = &g_three_phase0_ctrl,
    .p_cfg = &g_three_phase0_cfg,
    .p_api = &g_gpt_three_phase_on_gpt_three_phase,
};
}

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
