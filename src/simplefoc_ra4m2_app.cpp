#include "renesas_simplefoc_port.h"

#include "SimpleFOC/SimpleFOC.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern "C" {
#include "bmp.h"
#include "oled.h"
}

#ifndef RA4M2_ENABLE_MOTOR_CONTROL
#define RA4M2_ENABLE_MOTOR_CONTROL 1
#endif

/*
 * J-Link tuning contract:
 * - Edit the requested fields in Live Watch/Expressions.
 * - Increment apply_sequence last. The control loop applies the whole set.
 * - Never edit SimpleFOC C++ object members directly from the debugger.
 */
struct Ra4m2JlinkTune
{
    volatile uint32_t magic;
    volatile uint32_t apply_sequence;
    volatile uint32_t requested_mode;        // 0: angle, 1: velocity
    volatile uint32_t requested_torque_mode; // 0: voltage, 1: FOC current
    volatile float target;
    volatile float current_limit;
    volatile float voltage_limit;
    volatile float angle_p;
    volatile float angle_velocity_limit;
    volatile float angle_velocity_p;
    volatile float angle_velocity_i;
    volatile float angle_velocity_tf;
    volatile float velocity_p;
    volatile float velocity_i;
    volatile float velocity_tf;
    volatile float current_q_p;
    volatile float current_q_i;
    volatile float current_d_p;
    volatile float current_d_i;
    volatile float current_tf;
    volatile uint32_t last_applied_sequence;
    volatile uint32_t apply_status;          // 0: waiting, 1: applied, 2: rejected
    volatile uint32_t active_mode;
    volatile uint32_t active_torque_mode;
    volatile float shaft_angle;
    volatile float shaft_velocity;
    volatile float phase_current_q;
    volatile float electrical_angle;
    volatile float voltage_q;
    volatile float voltage_d;
};

extern "C"
{
volatile Ra4m2JlinkTune g_ra4m2_jlink_tune =
{
    0x464F4334UL, 0U, 0U, 0U,
    0.0f, 0.5f, 12.0f,
    28.0f, 300.0f, 0.025f, 0.5f, 0.025f,
    0.05f, 1.0f, 0.02f,
    7.0f, 120.0f, 7.0f, 120.0f, 0.03f,
    0U, 0U, 0U, 0U,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};
}

namespace
{
#if !RA4M2_ENABLE_MOTOR_CONTROL
constexpr float kRadToDeg = 57.2957795f;
#endif
constexpr uint32_t kSerialPeriodMs = 20U;
constexpr uint32_t kOledPeriodMs = 200U;

MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);

#if !RA4M2_ENABLE_MOTOR_CONTROL
float as5600_angle_rad = 0.0f;
float as5600_mechanical_rad = 0.0f;
float as5600_velocity_rad_s = 0.0f;
int32_t as5600_turns = 0;
uint8_t as5600_i2c_error = 0U;
uint32_t sample_count = 0U;
#endif

#include "app/app_ui.inc"

#if RA4M2_ENABLE_MOTOR_CONTROL

void print_reset_status()
{
    const uint8_t reset_status0 = R_SYSTEM->RSTSR0;
    const uint16_t reset_status1 = R_SYSTEM->RSTSR1;

    Serial.print(F("RESET_STATUS,rstsr0="));
    Serial.print(static_cast<unsigned int>(reset_status0));
    Serial.print(F(",rstsr1="));
    Serial.println(static_cast<unsigned int>(reset_status1));

    if (0U != (reset_status0 & (R_SYSTEM_RSTSR0_LVD0RF_Msk |
                                R_SYSTEM_RSTSR0_LVD1RF_Msk |
                                R_SYSTEM_RSTSR0_LVD2RF_Msk)))
    {
        Serial.println(F("RESET_CAUSE:LOW_VOLTAGE"));
    }
    if (0U != (reset_status1 & R_SYSTEM_RSTSR1_WDTRF_Msk))
    {
        Serial.println(F("RESET_CAUSE:WATCHDOG"));
    }
    if (0U != (reset_status1 & R_SYSTEM_RSTSR1_IWDTRF_Msk))
    {
        Serial.println(F("RESET_CAUSE:INDEPENDENT_WATCHDOG"));
    }
    if (0U != (reset_status1 & R_SYSTEM_RSTSR1_SWRF_Msk))
    {
        Serial.println(F("RESET_CAUSE:SOFTWARE"));
    }
}

// 三相驱动器 EN：P302 高电平使能，低电平硬件失能。
constexpr bsp_io_port_pin_t kMotorEnablePin = BSP_IO_PORT_03_PIN_02;
constexpr bsp_io_port_pin_t kUserButtonPin = BSP_IO_PORT_00_PIN_00;
constexpr uint32_t kUserButtonIrqChannel = 6U;
constexpr uint32_t kAdcCalibrationSamples = 16U;
constexpr uint32_t kAdcMinValidCount = 16U;
constexpr uint32_t kAdcMaxValidCount = 4079U;
constexpr float kAdcReferenceVoltage = 3.3f;
constexpr float kAdcMaxCount = 4095.0f;
constexpr bool kUseCurrentTorqueStartup = false;
constexpr bool kAngleModeUsesCurrentLoop = false;
constexpr float kCurrentLoopEntryLimit = 0.05f;
constexpr float kDashboardMaxCurrentLimit = 1.0f;
constexpr float kCurrentPidDebugMaxTarget = 0.20f;
constexpr float kCurrentPidDebugVoltageLimit = 3.0f;
constexpr float kCurrentPidDebugMinLimit = 0.02f;
constexpr float kCurrentPidDebugMaxLimit = 0.30f;
constexpr float kCurrentPidDebugMinVoltageLimit = 0.50f;
constexpr uint32_t kCurrentPidDebugStepSettleMs = 100U;
constexpr uint32_t kCurrentPidDebugStepMinMs = 50U;
constexpr uint32_t kCurrentPidDebugStepMaxMs = 1000U;
constexpr float kVelocityModeNoLimit = 100000.0f;
constexpr float kSafeHoldVoltageLimit = 1.2f;
constexpr float kSafeHoldVelocityLimit = 20.0f;
constexpr float kSafeHoldVelocityI = 0.0f;
constexpr uint32_t kCurrentDiagPeriodMs = 100U;
// 原生 SimpleFOC 对齐前先连续验证 AS5600，避免偶发 I2C 错误进入长时间的对齐扫描。
constexpr uint32_t kAs5600VerifySamples = 32U;
constexpr uint32_t kAs5600VerifyIntervalMs = 5U;
constexpr uint8_t kAs5600RuntimeFaultThreshold = 3U;

// 完整校准已确认的机械安装参数。若改动电机、编码器位置、PWM 线序或电流采样接线，
// 必须先改为 false，完成一次完整校准后再更新下列值。
// 当前硬件的两相电流自动对齐存在持续加电风险，启动时仅使用已验证的对齐参数。
// 如需重新完整校准，请通过受控校准流程执行，禁止直接将此处改为 false 后上电运行。
// FOC 初始化方式：保存参数跳过对齐、受控校准、或 SimpleFOC 原生自动对齐。
enum class FocAlignmentMode : uint8_t
{
    Stored,
    Controlled,
    SimpleFOC,
};

// 仅修改这一行即可切换初始化方式。
// 原生自动对齐已确认会在本硬件上进入异常驻停，默认使用已验证的安装参数进入闭环。
// 保留 SimpleFOC/Controlled 两条校准路径，仅允许在人工看护下临时切换后使用。
constexpr FocAlignmentMode kFocAlignmentMode = FocAlignmentMode::Stored;
constexpr bool kUseStoredFocAlignment = (FocAlignmentMode::Stored == kFocAlignmentMode);
constexpr bool kUseSimpleFocAlignment = (FocAlignmentMode::SimpleFOC == kFocAlignmentMode);
// 当前禁止网页命令触发完整原生校准，防止误操作再次进入持续加电的对齐路径。
constexpr bool kAllowRuntimeFullFocCalibration = false;
// 自动校准结果通过一致性检查后才允许进入闭环。
constexpr bool kSensorAutoCalibrationReviewOnly = false;
// 自动校准当前只用于采集和诊断；闭环先使用已验证稳定的安装参数。
// 自动结果重复性满足要求后，可改为 true 直接应用候选零电角度。
constexpr bool kApplyAutoCalibrationResultToClosedLoop = false;
// P014/P013 是已确认的两相电流映射。禁止库自动交换通道或翻转增益，避免 driverAlign 持续加电。
constexpr bool kSkipCurrentSenseDriverAlignment = true;
// 原 ESP32 使用 5 V，但 RA4M2 驱动板实测校准电流约 0.8 A。
// 原生电压对齐不受 current_limit 限制，先以低能量 1 V 完成方向诊断。
constexpr float kSensorAutoAlignmentVoltage = 1.0f;
// 自动结果必须接近已验证的安装零电角度，避免错误结果直接驻停发热。
constexpr float kAutoAlignmentMaxZeroError = 0.10f;
// 受控校准不复用库内 1002 次连续 I2C 读取的阻塞扫描，降低校准阶段固定磁场持续时间。
constexpr float kControlledCalibrationVoltage = 3.5f;
constexpr uint32_t kControlledCalibrationSteps = 72U;
constexpr uint32_t kControlledCalibrationStepMs = 15U;
constexpr uint32_t kControlledCalibrationZeroHoldMs = 300U;
constexpr Direction kStoredSensorDirection = Direction::CCW;
constexpr float kStoredZeroElectricalAngle = 4.215380f;
constexpr float kStoredCurrentSenseGain = 1.0f / (0.01f * 50.0f);
constexpr int kStoredCurrentSensePinA = RA_ADC_U_CURRENT;
constexpr int kStoredCurrentSensePinB = RA_ADC_V_CURRENT;

const char *foc_alignment_mode_label(FocAlignmentMode mode)
{
    switch (mode)
    {
        case FocAlignmentMode::Stored: return "STORED_SENSOR";
        case FocAlignmentMode::Controlled: return "CONTROLLED_SENSOR";
        case FocAlignmentMode::SimpleFOC: return "SIMPLEFOC_SENSOR";
        default: return "UNKNOWN";
    }
}

BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(RA_PWM_A_U, RA_PWM_A_V, RA_PWM_A_W);
// Old order was P013/P014. The board routes P013 to V phase, so SimpleFOC A/B
// phase input must be U=P014 first, then V=P013.
// InlineCurrentSense current_sense2 = InlineCurrentSense(0.01f, 50.0f, RA_ADC1_1, RA_ADC1_2);
InlineCurrentSense current_sense2 = InlineCurrentSense(0.01f, 50.0f, RA_ADC_U_CURRENT, RA_ADC_V_CURRENT);
Commander command = Commander(Serial);

volatile bool user_button_disable_requested = false;
bool user_button_irq_opened = false;
bool motor_user_disabled = false;
bool diagnostic_quiet_mode = false;
// 默认开启。可用串口命令 NAV0/NAV1 关闭或开启，开关仅在当前上电周期有效。
bool as5600_verification_enabled = true;
// 节点级故障字：供串口诊断、J-Link 和后续 CAN 状态帧复用。故障锁存后必须复位才能恢复输出。
enum MotorFaultBit : uint32_t
{
    MotorFaultNone              = 0U,
    MotorFaultUserEmergencyStop = (1UL << 0),
    MotorFaultAs5600Comm        = (1UL << 1),
    MotorFaultCurrentSenseInit  = (1UL << 2),
    MotorFaultDriverInit        = (1UL << 3),
    MotorFaultFocInit           = (1UL << 4),
    MotorFaultCalibration       = (1UL << 5),
};
volatile uint32_t motor_fault_code = MotorFaultNone;
volatile bool motor_fault_latched = false;
bool motor_driver_ready = false;
uint8_t as5600_runtime_error_count = 0U;
bool current_diagnostic_mode = false;
bool current_pid_debug_active = false;
bool current_gain_reference_valid = false;
uint8_t current_gain_mode = 0U;
float current_gain_a_reference = 0.0f;
float current_gain_b_reference = 0.0f;
float current_gain_c_reference = 0.0f;
float current_pid_debug_target = 0.0f;
float current_pid_debug_d_target = 0.0f;
float current_pid_debug_iq_reference = 0.0f;
float current_pid_debug_id_reference = 0.0f;
float current_pid_debug_limit = kCurrentPidDebugMaxTarget;
float current_pid_debug_voltage_limit = kCurrentPidDebugVoltageLimit;
float current_pid_debug_step_amplitude = 0.0f;
uint32_t current_pid_debug_step_start_ms = 0U;
uint32_t current_pid_debug_step_duration_ms = 0U;
bool current_pid_debug_step_active = false;
bool current_pid_debug_manual_output = false;

struct CurrentPidDebugRestore
{
    MotionControlType motion_mode;
    TorqueControlType torque_mode;
    float current_limit;
    float voltage_limit;
    bool valid;
};
CurrentPidDebugRestore current_pid_debug_restore = {};
// 安全驻停会暂时降至 1.2 V；该变量始终保存离开驻停后应恢复的工作电压上限。
float operating_voltage_limit = 12.0f;

struct AdcCalibrationResult
{
    uint32_t adc1;
    uint32_t adc2;
    float voltage1;
    float voltage2;
};

int current_mode = 0; // 0 = angle mode, 1 = velocity mode

float angle_vel_limit = 300.0f;
float angle_P = 28.0f;
float angle_vel_P = 0.025f;
float angle_vel_I = 0.5f;
float angle_vel_Tf = 0.025f;
float angle_curr_q_P = 7.0f;
float angle_curr_q_I = 120.0f;
float angle_curr_d_P = 7.0f;
float angle_curr_d_I = 120.0f;
float angle_curr_Tf = 0.03f;

float vel_vel_limit = 50.0f;
float vel_vel_P = 0.05f;
float vel_vel_I = 1.0f;
float vel_vel_Tf = 0.02f;
float vel_curr_q_P = 7.0f;
float vel_curr_q_I = 120.0f;
float vel_curr_d_P = 7.0f;
float vel_curr_d_I = 120.0f;
float vel_curr_Tf = 0.03f;

void doMotor2(char *cmd)
{
    command.motor(&motor2, cmd);
}

bool starts_with(const char *text, const char *prefix)
{
    return 0 == strncmp(text, prefix, strlen(prefix));
}

float command_float_value(const char *text, size_t prefix_len)
{
    return static_cast<float>(atof(text + prefix_len));
}

const char *phase_order_label(uint8_t order)
{
    static const char *const labels[] = {"UVW", "UWV", "VUW", "VWU", "WUV", "WVU"};
    if (order < (sizeof(labels) / sizeof(labels[0])))
    {
        return labels[order];
    }

    return "UNKNOWN";
}

const char *current_gain_mode_label(uint8_t mode)
{
    static const char *const labels[] = {"ALIGN", "INV_AB", "INV_A", "INV_B", "INV_ABC"};
    if (mode < (sizeof(labels) / sizeof(labels[0])))
    {
        return labels[mode];
    }

    return "UNKNOWN";
}

float limit_symmetric(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

float motor_target_velocity()
{
    if (MotionControlType::angle == motor2.controller)
    {
        return limit_symmetric(motor2.P_angle.P * (motor2.target - motor2.shaft_angle), motor2.velocity_limit);
    }

    if (MotionControlType::velocity == motor2.controller)
    {
        return motor2.target;
    }

    return 0.0f;
}

float motor_target_current(float target_velocity)
{
    if (TorqueControlType::foc_current == motor2.torque_controller)
    {
        return limit_symmetric(motor2.current_sp, motor2.current_limit);
    }

    return limit_symmetric(motor2.PID_velocity.P * (target_velocity - motor2.shaft_velocity), motor2.current_limit);
}

bool valid_sensor_value(float value)
{
    return (value > -1000000.0f) && (value < 1000000.0f);
}

bool valid_adc_count(uint32_t value)
{
    return (value > kAdcMinValidCount) && (value < kAdcMaxValidCount);
}

float adc_count_to_voltage(uint32_t value)
{
    return (static_cast<float>(value) * kAdcReferenceVoltage) / kAdcMaxCount;
}

void capture_current_gain_reference()
{
    current_gain_a_reference = current_sense2.gain_a;
    current_gain_b_reference = current_sense2.gain_b;
    current_gain_c_reference = current_sense2.gain_c;
    current_gain_reference_valid = true;
}

bool apply_current_gain_mode(uint8_t mode)
{
    if (mode > 4U)
    {
        return false;
    }

    if (!current_gain_reference_valid)
    {
        capture_current_gain_reference();
    }

    const float sign_a = ((1U == mode) || (2U == mode) || (4U == mode)) ? -1.0f : 1.0f;
    const float sign_b = ((1U == mode) || (3U == mode) || (4U == mode)) ? -1.0f : 1.0f;
    const float sign_c = (4U == mode) ? -1.0f : 1.0f;

    current_gain_mode = mode;
    current_sense2.gain_a = current_gain_a_reference * sign_a;
    current_sense2.gain_b = current_gain_b_reference * sign_b;
    current_sense2.gain_c = current_gain_c_reference * sign_c;

    motor2.PID_velocity.reset();
    motor2.PID_current_q.reset();
    motor2.PID_current_d.reset();
    return true;
}

// 已迁移至 app/app_safety.inc，保留原实现以便与硬件安全时序对比。
#if 0
void set_motor_enable(bool enabled)
{
    const uint32_t pin_cfg = IOPORT_CFG_PORT_DIRECTION_OUTPUT |
                             (enabled ? IOPORT_CFG_PORT_OUTPUT_HIGH : IOPORT_CFG_PORT_OUTPUT_LOW);
    (void) R_IOPORT_PinCfg(&g_ioport_ctrl, kMotorEnablePin, pin_cfg);
    (void) R_IOPORT_PinWrite(&g_ioport_ctrl,
                             kMotorEnablePin,
                             enabled ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW);
}

void print_motor_fault_status()
{
    char fault_text[12];
    snprintf(fault_text, sizeof(fault_text), "%08lX", static_cast<unsigned long>(motor_fault_code));
    Serial.print(F("FAULT,code=0x"));
    Serial.print(fault_text);
    Serial.print(F(",latched="));
    Serial.println(motor_fault_latched ? 1 : 0);
}

void latch_motor_fault(uint32_t fault_bit)
{
    const uint32_t previous_code = motor_fault_code;
    motor_fault_code |= fault_bit;
    motor_fault_latched = true;

    // 此函数可从常规代码调用；紧急 P000 ISR 仍保留直接拉低 P302 的最快路径。
    // 驱动器初始化失败时尚未绑定到 motor2，因此不能调用其 PWM 收尾方法。
    set_motor_enable(false);
    if (motor_driver_ready)
    {
        motor2.disable();
    }

    if (previous_code != motor_fault_code)
    {
        print_motor_fault_status();
    }
}

void monitor_runtime_as5600_fault()
{
    if (0U == sensor2.currWireError)
    {
        as5600_runtime_error_count = 0U;
        return;
    }

    if (as5600_runtime_error_count < UINT8_MAX)
    {
        as5600_runtime_error_count++;
    }
    if (as5600_runtime_error_count >= kAs5600RuntimeFaultThreshold)
    {
        latch_motor_fault(MotorFaultAs5600Comm);
    }
}

void user_button_irq_callback(external_irq_callback_args_t *p_args)
{
    if ((!p_args) || (kUserButtonIrqChannel == p_args->channel))
    {
        // 紧急路径不能等待主循环：initFOC() 的自动对齐是阻塞过程，
        // 此处直接拉低 P302，立即关闭三相驱动器的使能输入。
        // 不在中断内调用 SimpleFOC、串口或 OLED；这些收尾工作仍由主循环完成。
        R_PORT3->PODR_b.PODR2 = 0U;
        user_button_disable_requested = true;
    }
}

void init_user_button_irq()
{
    if (user_button_irq_opened)
    {
        return;
    }

    (void) R_IOPORT_PinCfg(&g_ioport_ctrl,
                           kUserButtonPin,
                           IOPORT_CFG_PORT_DIRECTION_INPUT | IOPORT_CFG_IRQ_ENABLE);

    external_irq_cfg_t user_button_irq_cfg = g_external_irq6_cfg;
    user_button_irq_cfg.channel = static_cast<uint8_t>(kUserButtonIrqChannel);
    user_button_irq_cfg.trigger = EXTERNAL_IRQ_TRIG_FALLING;
    user_button_irq_cfg.filter_enable = false;
    user_button_irq_cfg.p_callback = user_button_irq_callback;
    user_button_irq_cfg.p_context = NULL;

    fsp_err_t irq_err = R_ICU_ExternalIrqOpen(&g_external_irq6_ctrl, &user_button_irq_cfg);
    if ((FSP_SUCCESS == irq_err) || (FSP_ERR_ALREADY_OPEN == irq_err))
    {
        (void) R_ICU_ExternalIrqEnable(&g_external_irq6_ctrl);
        user_button_irq_opened = true;
        Serial.println("USER_BUTTON_IRQ_READY:P000 falling");
    }
    else
    {
        Serial.print("USER_BUTTON_IRQ_FAIL:");
        Serial.println(static_cast<int>(irq_err));
    }
}

void handle_user_button_motor_disable()
{
    if (!user_button_disable_requested)
    {
        return;
    }

    user_button_disable_requested = false;
    if (motor_user_disabled)
    {
        return;
    }

    motor_user_disabled = true;
    motor2.disable();
    set_motor_enable(false);
    latch_motor_fault(MotorFaultUserEmergencyStop);
    Serial.println("USER_BUTTON_MOTOR_DISABLED");

    OLED_Clear();
    OLED_ShowLine(0, 0, "USER BUTTON");
    OLED_ShowLine(0, 16, "Motor disabled");
    OLED_ShowLine(0, 32, "MCU still reads");
    OLED_ShowLine(0, 48, "Reset to enable");
    OLED_Refresh();
}

[[maybe_unused]] void halt_startup_calibration(const char *title, const char *detail)
{
    set_motor_enable(false);

    Serial.print("CAL_FAIL: ");
    Serial.print(title);
    Serial.print(" ");
    Serial.println(detail);

    OLED_Clear();
    OLED_ShowLine(0, 0, "STARTUP CAL FAIL");
    OLED_ShowLine(0, 16, title);
    OLED_ShowLine(0, 32, detail);
    OLED_ShowLine(0, 48, "Motor disabled");
    OLED_Refresh();

    while (true)
    {
        delay(1000UL);
    }
}

#endif

#include "app/app_safety.inc"

// 已迁移至 app/app_sensor.inc，保留原实现以便核对启动诊断的输出协议。
#if 0
bool calibrate_as5600_startup()
{
    OLED_Clear();
    OLED_ShowLine(0, 0, "AS5600 CAL...");
    OLED_ShowLine(0, 16, "Checking I2C");
    OLED_Refresh();

    Serial.println("CAL_AS5600_START");

    for (uint8_t attempt = 0U; attempt < 5U; attempt++)
    {
        sensor2.update();
        delay(20UL);
        sensor2.update();
        delay(20UL);
        sensor2.update();

        const float angle = sensor2.getAngle();
        const float velocity = sensor2.getVelocity();
        const uint8_t wire_error = sensor2.currWireError;

        if ((0U == wire_error) && valid_sensor_value(angle) && valid_sensor_value(velocity))
        {
            Serial.print("AS5600_ANGLE_RAD=");
            SerialPrintFixed(angle, 1000U, 3U);
            Serial.println();
            Serial.print("AS5600_VEL_RAD_S=");
            SerialPrintFixed(velocity, 1000U, 3U);
            Serial.println();

            OLED_Clear();
            OLED_ShowLine(0, 0, "AS5600 OK");
            OLED_ShowFixedLine(16, "A", angle, 1000U, 3U, "rad");
            OLED_ShowFixedLine(32, "V", velocity, 1000U, 3U, "rad/s");
            OLED_ShowLine(0, 48, "I2C addr:0x36");
            OLED_Refresh();
            delay(800UL);
            return true;
        }

        Serial.print("AS5600_RETRY=");
        Serial.print(static_cast<int>(attempt + 1U));
        Serial.print(" ERR=");
        Serial.println(static_cast<int>(wire_error));
        delay(80UL);
    }

    return false;
}

// 在 FOC 对齐前验证连续读取质量。角度静止时 min/max 相同是正常现象，判据仅为 I2C 读取是否连续成功。
bool verify_as5600_communication()
{
    uint32_t good_samples = 0U;
    uint32_t error_samples = 0U;
    float min_angle = _2PI;
    float max_angle = 0.0f;

    Serial.print(F("AS5600_VERIFY_START,samples="));
    Serial.println(kAs5600VerifySamples);

    for (uint32_t sample = 0U; sample < kAs5600VerifySamples; sample++)
    {
        sensor2.update();
        const uint8_t wire_error = sensor2.currWireError;
        const float angle = sensor2.getMechanicalAngle();

        if ((0U == wire_error) && sensor2.isValid() && valid_sensor_value(angle) &&
            (angle >= 0.0f) && (angle <= _2PI))
        {
            good_samples++;
            min_angle = (angle < min_angle) ? angle : min_angle;
            max_angle = (angle > max_angle) ? angle : max_angle;
        }
        else
        {
            error_samples++;
        }

        delay(kAs5600VerifyIntervalMs);
    }

    const bool passed = (kAs5600VerifySamples == good_samples);
    Serial.print(F("AS5600_VERIFY_RESULT,ok="));
    Serial.print(passed ? 1 : 0);
    Serial.print(F(",good="));
    Serial.print(good_samples);
    Serial.print(F(",error="));
    Serial.print(error_samples);
    if (good_samples > 0U)
    {
        Serial.print(F(",min="));
        SerialPrintFixed(min_angle, 1000U, 3U);
        Serial.print(F(",max="));
        SerialPrintFixed(max_angle, 1000U, 3U);
    }
    Serial.println();

    OLED_Clear();
    OLED_ShowLine(0, 0, passed ? "AS5600 VERIFY OK" : "AS5600 VERIFY FAIL");
    OLED_ShowFixedLine(16, "GOOD", static_cast<float>(good_samples), 1U, 0U, "");
    OLED_ShowFixedLine(24, "ERR", static_cast<float>(error_samples), 1U, 0U, "");
    OLED_Refresh();

    return passed;
}

uint32_t read_adc_average(int pin)
{
    uint32_t sum = 0U;

    for (uint32_t sample = 0U; sample < kAdcCalibrationSamples; sample++)
    {
        sum += analogRead(pin);
        delayMicroseconds(200U);
    }

    return (sum + (kAdcCalibrationSamples / 2U)) / kAdcCalibrationSamples;
}

bool calibrate_adc_startup(AdcCalibrationResult *result)
{
    if (!result)
    {
        return false;
    }

    OLED_Clear();
    OLED_ShowLine(0, 0, "ADC CAL...");
    OLED_ShowLine(0, 16, "P014=U P013=V");
    OLED_Refresh();

    Serial.println("CAL_ADC_START");

    result->adc1 = read_adc_average(RA_ADC_U_CURRENT);
    result->adc2 = read_adc_average(RA_ADC_V_CURRENT);
    result->voltage1 = adc_count_to_voltage(result->adc1);
    result->voltage2 = adc_count_to_voltage(result->adc2);

    Serial.print("ADC_U_P014_COUNT=");
    Serial.println(static_cast<unsigned long>(result->adc1));
    Serial.print("ADC_V_P013_COUNT=");
    Serial.println(static_cast<unsigned long>(result->adc2));
    Serial.print("ADC_U_P014_V=");
    SerialPrintFixed(result->voltage1, 1000U, 3U);
    Serial.println();
    Serial.print("ADC_V_P013_V=");
    SerialPrintFixed(result->voltage2, 1000U, 3U);
    Serial.println();

    const bool adc_ok = valid_adc_count(result->adc1) && valid_adc_count(result->adc2);

    OLED_Clear();
    OLED_ShowLine(0, 0, adc_ok ? "ADC OK" : "ADC FAIL");
    OLED_ShowFixedLine(16, "AN1", static_cast<float>(result->adc1), 1U, 0U, "");
    OLED_ShowFixedLine(24, "AN2", static_cast<float>(result->adc2), 1U, 0U, "");
    OLED_ShowFixedLine(40, "V1", result->voltage1, 1000U, 3U, "V");
    OLED_ShowFixedLine(48, "V2", result->voltage2, 1000U, 3U, "V");
    OLED_Refresh();
    delay(800UL);

    return adc_ok;
}

void wait_until_as5600_calibrated()
{
    uint32_t retry_count = 0U;

    while (!calibrate_as5600_startup())
    {
        retry_count++;
        set_motor_enable(false);

        Serial.print("CAL_AS5600_FAIL_RETRY=");
        Serial.println(static_cast<unsigned long>(retry_count));

        OLED_Clear();
        OLED_ShowLine(0, 0, "AS5600 CAL FAIL");
        OLED_ShowLine(0, 16, "Retrying...");
        OLED_ShowLine(0, 32, "Check addr/wire");
        OLED_ShowFixedLine(48, "Retry", static_cast<float>(retry_count), 1U, 0U, "");
        OLED_Refresh();
        delay(1000UL);
    }
}

void wait_until_as5600_verified()
{
    uint32_t retry_count = 0U;

    while (!verify_as5600_communication())
    {
        retry_count++;
        set_motor_enable(false);
        Serial.print(F("AS5600_VERIFY_FAIL_RETRY="));
        Serial.println(retry_count);
        delay(500UL);
    }
}

void wait_until_adc_calibrated(AdcCalibrationResult *result)
{
    uint32_t retry_count = 0U;

    while (!calibrate_adc_startup(result))
    {
        retry_count++;
        set_motor_enable(false);

        Serial.print("CAL_ADC_FAIL_RETRY=");
        Serial.println(static_cast<unsigned long>(retry_count));

        OLED_Clear();
        OLED_ShowLine(0, 0, "ADC CAL FAIL");
        OLED_ShowLine(0, 16, "Retrying...");
        OLED_ShowLine(0, 32, "Check P013/P014");
        OLED_ShowFixedLine(48, "Retry", static_cast<float>(retry_count), 1U, 0U, "");
        OLED_Refresh();
        delay(1000UL);
    }
}

#endif

#include "app/app_sensor.inc"

void applyCurrentModePID()
{
    if (0 == current_mode)
    {
        motor2.velocity_limit = angle_vel_limit;
        motor2.P_angle.P = angle_P;
        motor2.PID_velocity.P = angle_vel_P;
        motor2.PID_velocity.I = angle_vel_I;
        motor2.LPF_velocity.Tf = angle_vel_Tf;
        motor2.PID_current_q.P = angle_curr_q_P;
        motor2.PID_current_q.I = angle_curr_q_I;
        motor2.PID_current_d.P = angle_curr_d_P;
        motor2.PID_current_d.I = angle_curr_d_I;
        motor2.LPF_current_q.Tf = angle_curr_Tf;
        motor2.LPF_current_d.Tf = angle_curr_Tf;
    }
    else
    {
        (void) vel_vel_limit;
        motor2.velocity_limit = kVelocityModeNoLimit;
        motor2.PID_velocity.P = vel_vel_P;
        motor2.PID_velocity.I = vel_vel_I;
        motor2.LPF_velocity.Tf = vel_vel_Tf;
        motor2.PID_current_q.P = vel_curr_q_P;
        motor2.PID_current_q.I = vel_curr_q_I;
        motor2.PID_current_d.P = vel_curr_d_P;
        motor2.PID_current_d.I = vel_curr_d_I;
        motor2.LPF_current_q.Tf = vel_curr_Tf;
        motor2.LPF_current_d.Tf = vel_curr_Tf;
    }

    motor2.updateVelocityLimit(motor2.velocity_limit);
    motor2.updateCurrentLimit(motor2.current_limit);
    motor2.updateVoltageLimit(motor2.voltage_limit);
}

bool valid_jlink_tune_value(float value, float minimum, float maximum)
{
    return isfinite(value) && (value >= minimum) && (value <= maximum);
}

void update_jlink_tune_status()
{
#if RA4M2_ENABLE_JLINK_TUNING
    static uint32_t last_status = 0U;
    const uint32_t now = millis();
    if ((now - last_status) < kSerialPeriodMs)
    {
        return;
    }
    last_status = now;

    g_ra4m2_jlink_tune.active_mode = static_cast<uint32_t>(current_mode);
    g_ra4m2_jlink_tune.active_torque_mode =
        (TorqueControlType::foc_current == motor2.torque_controller) ? 1U : 0U;
    g_ra4m2_jlink_tune.shaft_angle = motor2.shaft_angle;
    g_ra4m2_jlink_tune.shaft_velocity = motor2.shaft_velocity;
    g_ra4m2_jlink_tune.phase_current_q = motor2.current.q;
    g_ra4m2_jlink_tune.electrical_angle = motor2.electrical_angle;
    g_ra4m2_jlink_tune.voltage_q = motor2.voltage.q;
    g_ra4m2_jlink_tune.voltage_d = motor2.voltage.d;
#endif
}

void apply_jlink_tuning_request()
{
#if RA4M2_ENABLE_JLINK_TUNING
    static uint32_t handled_sequence = 0U;
    const uint32_t requested_sequence = g_ra4m2_jlink_tune.apply_sequence;
    if (requested_sequence == handled_sequence)
    {
        return;
    }

    if ((0x464F4334UL != g_ra4m2_jlink_tune.magic) ||
        (g_ra4m2_jlink_tune.requested_mode > 1U) ||
        (g_ra4m2_jlink_tune.requested_torque_mode > 1U) ||
        !valid_jlink_tune_value(g_ra4m2_jlink_tune.target, -100000.0f, 100000.0f) ||
        !valid_jlink_tune_value(g_ra4m2_jlink_tune.current_limit, 0.01f, kDashboardMaxCurrentLimit) ||
        !valid_jlink_tune_value(g_ra4m2_jlink_tune.voltage_limit, 0.1f, driver2.voltage_power_supply) ||
        !valid_jlink_tune_value(g_ra4m2_jlink_tune.angle_p, 0.0f, 200.0f) ||
        !valid_jlink_tune_value(g_ra4m2_jlink_tune.angle_velocity_limit, 0.1f, 1000.0f) ||
        !valid_jlink_tune_value(g_ra4m2_jlink_tune.angle_velocity_p, 0.0f, 20.0f) ||
        !valid_jlink_tune_value(g_ra4m2_jlink_tune.angle_velocity_i, 0.0f, 200.0f) ||
        !valid_jlink_tune_value(g_ra4m2_jlink_tune.angle_velocity_tf, 0.0f, 1.0f) ||
        !valid_jlink_tune_value(g_ra4m2_jlink_tune.velocity_p, 0.0f, 20.0f) ||
        !valid_jlink_tune_value(g_ra4m2_jlink_tune.velocity_i, 0.0f, 200.0f) ||
        !valid_jlink_tune_value(g_ra4m2_jlink_tune.velocity_tf, 0.0f, 1.0f) ||
        !valid_jlink_tune_value(g_ra4m2_jlink_tune.current_q_p, 0.0f, 100.0f) ||
        !valid_jlink_tune_value(g_ra4m2_jlink_tune.current_q_i, 0.0f, 2000.0f) ||
        !valid_jlink_tune_value(g_ra4m2_jlink_tune.current_d_p, 0.0f, 100.0f) ||
        !valid_jlink_tune_value(g_ra4m2_jlink_tune.current_d_i, 0.0f, 2000.0f) ||
        !valid_jlink_tune_value(g_ra4m2_jlink_tune.current_tf, 0.0f, 1.0f))
    {
        g_ra4m2_jlink_tune.apply_status = 2U;
        handled_sequence = requested_sequence;
        return;
    }

    angle_P = g_ra4m2_jlink_tune.angle_p;
    angle_vel_limit = g_ra4m2_jlink_tune.angle_velocity_limit;
    angle_vel_P = g_ra4m2_jlink_tune.angle_velocity_p;
    angle_vel_I = g_ra4m2_jlink_tune.angle_velocity_i;
    angle_vel_Tf = g_ra4m2_jlink_tune.angle_velocity_tf;
    vel_vel_P = g_ra4m2_jlink_tune.velocity_p;
    vel_vel_I = g_ra4m2_jlink_tune.velocity_i;
    vel_vel_Tf = g_ra4m2_jlink_tune.velocity_tf;
    angle_curr_q_P = vel_curr_q_P = g_ra4m2_jlink_tune.current_q_p;
    angle_curr_q_I = vel_curr_q_I = g_ra4m2_jlink_tune.current_q_i;
    angle_curr_d_P = vel_curr_d_P = g_ra4m2_jlink_tune.current_d_p;
    angle_curr_d_I = vel_curr_d_I = g_ra4m2_jlink_tune.current_d_i;
    angle_curr_Tf = vel_curr_Tf = g_ra4m2_jlink_tune.current_tf;
    current_mode = static_cast<int>(g_ra4m2_jlink_tune.requested_mode);
    motor2.controller = (0 == current_mode) ? MotionControlType::angle : MotionControlType::velocity;
    motor2.current_limit = g_ra4m2_jlink_tune.current_limit;
    operating_voltage_limit = g_ra4m2_jlink_tune.voltage_limit;
    motor2.voltage_limit = operating_voltage_limit;
    applyCurrentModePID();
    motor2.updateTorqueControlType((0U == g_ra4m2_jlink_tune.requested_torque_mode) ?
                                       TorqueControlType::voltage : TorqueControlType::foc_current);
    motor2.PID_velocity.reset();
    motor2.PID_current_q.reset();
    motor2.PID_current_d.reset();
    motor2.target = g_ra4m2_jlink_tune.target;
    g_ra4m2_jlink_tune.last_applied_sequence = requested_sequence;
    g_ra4m2_jlink_tune.apply_status = 1U;
    handled_sequence = requested_sequence;
#endif
}

void apply_safe_startup_hold()
{
    motor2.updateTorqueControlType(TorqueControlType::voltage);
    motor2.updateVoltageLimit(kSafeHoldVoltageLimit);
    motor2.updateVelocityLimit(kSafeHoldVelocityLimit);
    motor2.PID_velocity.I = kSafeHoldVelocityI;
    motor2.PID_velocity.reset();
    motor2.PID_current_q.reset();
    motor2.PID_current_d.reset();

    Serial.print(F("TORQUE_MODE:ANGLE_VOLTAGE,limit="));
    SerialPrintFixed(motor2.voltage_limit, 100U, 2U);
    Serial.print(F("V,vel_limit="));
    SerialPrintFixed(motor2.velocity_limit, 100U, 2U);
    Serial.println(F("rad/s"));
}

void apply_voltage_torque_control(const char *reason = "manual_entry")
{
    const bool angle_mode = (MotionControlType::angle == motor2.controller);

    // 切换力矩类型时先将目标置为当前状态，防止在位置/速度模式之间产生突变输出。
    if (angle_mode)
    {
        motor2.target = motor2.shaft_angle;
    }
    else if (MotionControlType::velocity == motor2.controller)
    {
        motor2.target = 0.0f;
    }

    motor2.updateTorqueControlType(TorqueControlType::voltage);
    // 位置电压保持原有的低电压驻停保护；速度电压使用正常母线工作上限。
    motor2.updateVoltageLimit(angle_mode ? kSafeHoldVoltageLimit : operating_voltage_limit);
    applyCurrentModePID();
    motor2.PID_velocity.reset();
    motor2.PID_current_q.reset();
    motor2.PID_current_d.reset();

    Serial.print(F("TORQUE_MODE:"));
    Serial.print(angle_mode ? F("ANGLE_VOLTAGE") : F("VELOCITY_VOLTAGE"));
    Serial.print(F(",limit="));
    SerialPrintFixed(motor2.voltage_limit, 100U, 2U);
    Serial.print(F("V,"));
    Serial.println(reason);
}

void apply_current_torque_control(const char *reason = "manual_entry")
{
    if (MotionControlType::angle == motor2.controller)
    {
        motor2.target = motor2.shaft_angle;
    }
    else if (MotionControlType::velocity == motor2.controller)
    {
        motor2.target = 0.0f;
    }

    // 不继承安全驻停阶段的 1.2 V 临时限制，否则高速时速度环会过早饱和。
    motor2.updateVoltageLimit(operating_voltage_limit);
    motor2.updateTorqueControlType(TorqueControlType::foc_current);
    motor2.updateCurrentLimit(motor2.current_limit);
    motor2.updateVoltageLimit(motor2.voltage_limit);
    applyCurrentModePID();
    motor2.PID_velocity.reset();
    motor2.PID_current_q.reset();
    motor2.PID_current_d.reset();

    Serial.print(F("TORQUE_MODE:FOC_CURRENT,current_limit="));
    SerialPrintFixed(motor2.current_limit, 1000U, 3U);
    Serial.print(F("A,voltage_limit="));
    SerialPrintFixed(motor2.voltage_limit, 100U, 2U);
    Serial.print(F("V,"));
    Serial.println(reason);
}

void apply_zero_current_diagnostic_control()
{
    if (motor2.current_limit > kCurrentLoopEntryLimit)
    {
        motor2.current_limit = kCurrentLoopEntryLimit;
    }

    motor2.updateMotionControlType(MotionControlType::torque);
    motor2.updateTorqueControlType(TorqueControlType::foc_current);
    motor2.updateCurrentLimit(motor2.current_limit);
    motor2.updateVoltageLimit(motor2.voltage_limit);
    motor2.target = 0.0f;
    motor2.current_sp = 0.0f;
    motor2.PID_velocity.reset();
    motor2.PID_current_q.reset();
    motor2.PID_current_d.reset();

    Serial.print(F("TORQUE_MODE:FOC_CURRENT_ZERO,current_limit="));
    SerialPrintFixed(motor2.current_limit, 1000U, 3U);
    Serial.println(F("A,diagnostic"));
}

bool enter_current_pid_debug()
{
    if (motor_user_disabled)
    {
        Serial.println(F("CURRENT_PID_DEBUG:FAIL,MOTOR_DISABLED"));
        return false;
    }

    if (!current_pid_debug_active)
    {
        current_pid_debug_restore.motion_mode = motor2.controller;
        current_pid_debug_restore.torque_mode = motor2.torque_controller;
        current_pid_debug_restore.current_limit = motor2.current_limit;
        current_pid_debug_restore.voltage_limit = motor2.voltage_limit;
        current_pid_debug_restore.valid = true;
    }

    current_pid_debug_active = true;
    current_pid_debug_target = 0.0f;
    current_pid_debug_d_target = 0.0f;
    current_pid_debug_iq_reference = 0.0f;
    current_pid_debug_id_reference = 0.0f;
    current_pid_debug_limit = kCurrentPidDebugMaxTarget;
    current_pid_debug_voltage_limit = kCurrentPidDebugVoltageLimit;
    current_pid_debug_step_active = false;
    current_pid_debug_manual_output = false;
    motor2.updateMotionControlType(MotionControlType::torque);
    motor2.updateTorqueControlType(TorqueControlType::foc_current);
    motor2.current_limit = current_pid_debug_limit;
    motor2.voltage_limit = current_pid_debug_voltage_limit;
    applyCurrentModePID();
    motor2.target = 0.0f;
    motor2.current_sp = 0.0f;
    motor2.feed_forward_current.q = 0.0f;
    motor2.feed_forward_current.d = 0.0f;
    motor2.PID_velocity.reset();
    motor2.PID_current_q.reset();
    motor2.PID_current_d.reset();

    Serial.print(F("CURRENT_PID_DEBUG:ON,pool="));
    Serial.println((0 == current_mode) ? F("ANGLE") : F("VELOCITY"));
    return true;
}

bool exit_current_pid_debug()
{
    if (!current_pid_debug_active)
    {
        return true;
    }

    current_pid_debug_active = false;
    current_pid_debug_target = 0.0f;
    current_pid_debug_d_target = 0.0f;
    current_pid_debug_iq_reference = 0.0f;
    current_pid_debug_id_reference = 0.0f;
    current_pid_debug_step_active = false;
    current_pid_debug_manual_output = false;
    motor2.target = 0.0f;
    motor2.current_sp = 0.0f;
    motor2.feed_forward_current.q = 0.0f;
    motor2.feed_forward_current.d = 0.0f;

    if (current_pid_debug_restore.valid)
    {
        motor2.updateMotionControlType(current_pid_debug_restore.motion_mode);
        motor2.current_limit = current_pid_debug_restore.current_limit;
        motor2.voltage_limit = current_pid_debug_restore.voltage_limit;
        applyCurrentModePID();

        if (TorqueControlType::foc_current == current_pid_debug_restore.torque_mode)
        {
            apply_current_torque_control("current_pid_debug_exit");
        }
        else
        {
            apply_voltage_torque_control("current_pid_debug_exit");
        }
        current_pid_debug_restore.valid = false;
    }

    motor2.PID_velocity.reset();
    motor2.PID_current_q.reset();
    motor2.PID_current_d.reset();
    Serial.println(F("CURRENT_PID_DEBUG:OFF"));
    return true;
}

bool set_current_pid_debug_target(float target)
{
    if (!current_pid_debug_active || !isfinite(target) ||
        (target < -kCurrentPidDebugMaxLimit) || (target > kCurrentPidDebugMaxLimit))
    {
        return false;
    }

    current_pid_debug_target = target;
    current_pid_debug_step_active = false;
    current_pid_debug_manual_output = true;
    return true;
}

bool set_current_pid_debug_d_target(float target)
{
    if (!current_pid_debug_active || !isfinite(target) ||
        (target < -kCurrentPidDebugMaxLimit) || (target > kCurrentPidDebugMaxLimit))
    {
        return false;
    }

    current_pid_debug_d_target = target;
    current_pid_debug_step_active = false;
    current_pid_debug_manual_output = true;
    return true;
}

bool set_current_pid_debug_limit(float limit)
{
    if (!current_pid_debug_active || !isfinite(limit) ||
        (limit < kCurrentPidDebugMinLimit) || (limit > kCurrentPidDebugMaxLimit))
    {
        return false;
    }

    current_pid_debug_limit = limit;
    motor2.current_limit = limit;
    motor2.updateCurrentLimit(limit);
    return true;
}

bool set_current_pid_debug_voltage_limit(float limit)
{
    if (!current_pid_debug_active || !isfinite(limit) ||
        (limit < kCurrentPidDebugMinVoltageLimit) || (limit > kCurrentPidDebugVoltageLimit))
    {
        return false;
    }

    current_pid_debug_voltage_limit = limit;
    motor2.voltage_limit = limit;
    motor2.updateVoltageLimit(limit);
    return true;
}

bool start_current_pid_debug_step(uint32_t duration_ms)
{
    if (!current_pid_debug_active || (duration_ms < kCurrentPidDebugStepMinMs) ||
        (duration_ms > kCurrentPidDebugStepMaxMs))
    {
        return false;
    }

    current_pid_debug_step_amplitude = current_pid_debug_target;
    current_pid_debug_step_start_ms = millis();
    current_pid_debug_step_duration_ms = duration_ms;
    current_pid_debug_step_active = true;
    current_pid_debug_manual_output = false;
    Serial.println(F("CURRENT_PID_STEP:START"));
    return true;
}

void apply_current_pid_debug_setpoint(float iq_target, float id_target)
{
    const float magnitude = sqrtf(iq_target * iq_target + id_target * id_target);
    if ((magnitude > current_pid_debug_limit) && (magnitude > 0.0f))
    {
        const float scale = current_pid_debug_limit / magnitude;
        iq_target *= scale;
        id_target *= scale;
    }

    motor2.target = iq_target;
    motor2.feed_forward_current.q = 0.0f;
    motor2.feed_forward_current.d = id_target;
    current_pid_debug_iq_reference = iq_target;
    current_pid_debug_id_reference = id_target;
}

void update_current_pid_debug_stimulus()
{
    if (!current_pid_debug_active)
    {
        return;
    }

    if (current_pid_debug_step_active)
    {
        const uint32_t elapsed_ms = millis() - current_pid_debug_step_start_ms;
        if (elapsed_ms < kCurrentPidDebugStepSettleMs)
        {
            apply_current_pid_debug_setpoint(0.0f, 0.0f);
        }
        else if (elapsed_ms < (kCurrentPidDebugStepSettleMs + current_pid_debug_step_duration_ms))
        {
            apply_current_pid_debug_setpoint(current_pid_debug_step_amplitude, current_pid_debug_d_target);
        }
        else
        {
            current_pid_debug_step_active = false;
            current_pid_debug_manual_output = false;
            apply_current_pid_debug_setpoint(0.0f, 0.0f);
            Serial.println(F("CURRENT_PID_STEP:DONE,IQ_ID_ZERO"));
        }
        return;
    }

    if (current_pid_debug_manual_output)
    {
        apply_current_pid_debug_setpoint(current_pid_debug_target, current_pid_debug_d_target);
    }
    else
    {
        apply_current_pid_debug_setpoint(0.0f, 0.0f);
    }
}

void print_current_pid_debug_config()
{
    Serial.print(F("CURPID_CFG,iq="));
    SerialPrintFixed(current_pid_debug_target, 1000U, 3U);
    Serial.print(F(",id="));
    SerialPrintFixed(current_pid_debug_d_target, 1000U, 3U);
    Serial.print(F(",limit="));
    SerialPrintFixed(current_pid_debug_limit, 1000U, 3U);
    Serial.print(F(",voltage="));
    SerialPrintFixed(current_pid_debug_voltage_limit, 1000U, 3U);
    Serial.print(F(",step="));
    Serial.print(current_pid_debug_step_active ? 1U : 0U);
    Serial.println();
}

const char *current_sense_pin_label(int pin)
{
    if (RA_ADC_U_CURRENT == pin)
    {
        return "U";
    }
    if (RA_ADC_V_CURRENT == pin)
    {
        return "V";
    }
    return "NC";
}

void print_foc_calibration_profile()
{
    Serial.print(F("FOC_CAL_RESULT,dir="));
    Serial.print((Direction::CCW == motor2.sensor_direction) ? F("CCW") : F("CW"));
    Serial.print(F(",zero="));
    SerialPrintFixed(motor2.zero_electric_angle, 1000000U, 6U);
    Serial.print(F(",pinA="));
    Serial.print(current_sense_pin_label(current_sense2.pinA));
    Serial.print(F(",pinB="));
    Serial.print(current_sense_pin_label(current_sense2.pinB));
    Serial.print(F(",gainA="));
    SerialPrintFixed(current_sense2.gain_a, 1000000U, 6U);
    Serial.print(F(",gainB="));
    SerialPrintFixed(current_sense2.gain_b, 1000000U, 6U);
    Serial.print(F(",offsetA="));
    SerialPrintFixed(current_sense2.offset_ia, 1000000U, 6U);
    Serial.print(F(",offsetB="));
    SerialPrintFixed(current_sense2.offset_ib, 1000000U, 6U);
    Serial.println();
}

void stop_calibration_voltage()
{
    // 无论校准成功、I2C 异常还是用户中断，均先撤销三相测试电压。
    motor2.setPhaseVoltage(0.0f, 0.0f, 0.0f);
}

bool update_calibration_sensor()
{
    sensor2.update();
    if (0U != sensor2.currWireError)
    {
        stop_calibration_voltage();
        Serial.print(F("FOC_CAL_FAIL,AS5600_ERR="));
        Serial.println(static_cast<unsigned int>(sensor2.currWireError));
        return false;
    }
    return true;
}

bool run_controlled_sensor_calibration()
{
    Serial.println(F("FOC_CAL_CUSTOM_START"));
    // 方向由已验证的 PWM 线序、编码器安装和极对数共同决定，不能由单次扫描覆盖。
    motor2.sensor_direction = kStoredSensorDirection;
    motor2.zero_electric_angle = NOT_SET;

    if (!update_calibration_sensor())
    {
        return false;
    }
    const float start_angle = sensor2.getAngle();

    // 正向扫描一圈电角度。每一步都允许按键中止或检查 AS5600 通信。
    for (uint32_t step = 0U; step <= kControlledCalibrationSteps; step++)
    {
        if (user_button_disable_requested)
        {
            stop_calibration_voltage();
            Serial.println(F("FOC_CAL_ABORTED,P000"));
            return false;
        }
        const float angle = _3PI_2 + _2PI * static_cast<float>(step) /
                                          static_cast<float>(kControlledCalibrationSteps);
        motor2.setPhaseVoltage(kControlledCalibrationVoltage, 0.0f, angle);
        delay(kControlledCalibrationStepMs);
        if (!update_calibration_sensor())
        {
            return false;
        }
    }
    const float mid_angle = sensor2.getAngle();

    /*
     * 原 SimpleFOC 还会执行反向扫描，并用“正向终点 - 反向终点”判断位移。
     * 当前功率级在反向扫描中转子可能不跟随，导致中点和终点近似相同而误报未移动。
     * 保留原思路供后续比较；当前使用“起点 - 正向终点”作为方向和极对数判据。
    for (int32_t step = static_cast<int32_t>(kControlledCalibrationSteps); step >= 0; step--)
    {
        if (user_button_disable_requested)
        {
            stop_calibration_voltage();
            Serial.println(F("FOC_CAL_ABORTED,P000"));
            return false;
        }
        const float angle = _3PI_2 + _2PI * static_cast<float>(step) /
                                          static_cast<float>(kControlledCalibrationSteps);
        motor2.setPhaseVoltage(kControlledCalibrationVoltage, 0.0f, angle);
        delay(kControlledCalibrationStepMs);
        if (!update_calibration_sensor())
        {
            return false;
        }
    }
    const float end_angle = sensor2.getAngle();
    */
    stop_calibration_voltage();

    const float moved = fabsf(mid_angle - start_angle);
    Serial.print(F("FOC_CAL_SCAN,start="));
    SerialPrintFixed(start_angle, 1000U, 3U);
    Serial.print(F(",forward="));
    SerialPrintFixed(mid_angle, 1000U, 3U);
    Serial.print(F(",moved="));
    SerialPrintFixed(moved, 1000U, 3U);
    Serial.println();

    if (moved < MIN_ANGLE_DETECT_MOVEMENT)
    {
        Serial.println(F("FOC_CAL_FAIL,NO_SENSOR_MOVEMENT"));
        return false;
    }

    const Direction scan_direction = (mid_angle < start_angle) ? Direction::CCW : Direction::CW;
    motor2.pp_check_result = !(fabsf(moved * 7.0f - _2PI) > 0.5f);
    Serial.print(F("FOC_CAL_SCAN_DIR,candidate="));
    Serial.print((Direction::CCW == scan_direction) ? F("CCW") : F("CW"));
    Serial.print(F(",fixed="));
    Serial.println((Direction::CCW == kStoredSensorDirection) ? F("CCW") : F("CW"));
    if (scan_direction != kStoredSensorDirection)
    {
        Serial.println(F("FOC_CAL_WARN,SCAN_DIRECTION_IGNORED"));
    }
    motor2.sensor_direction = kStoredSensorDirection;

    // 零电角度仅保持短时间，读取完成后先撤销三相电压，再输出结果。
    motor2.setPhaseVoltage(kControlledCalibrationVoltage, 0.0f, _3PI_2);
    delay(kControlledCalibrationZeroHoldMs);
    if (!update_calibration_sensor())
    {
        return false;
    }
    motor2.zero_electric_angle = 0.0f;
    motor2.zero_electric_angle = motor2.electricalAngle();
    stop_calibration_voltage();

    motor2.motor_status = FOCMotorStatus::motor_ready;
    Serial.println(F("FOC_CAL_CUSTOM_DONE"));
    return true;
}

bool run_simplefoc_sensor_calibration()
{
    // 完整保留 SimpleFOC 原生流程：方向扫描、零电角度对齐、CurrentSense::driverAlign()。
    // 当前两相电流采样已设置 skip_align=true，因此不会自动交换 P014/P013 或翻转增益。
    Serial.println(F("FOC_CAL_SIMPLEFOC_START"));
    motor2.sensor_direction = Direction::UNKNOWN;
    motor2.zero_electric_angle = NOT_SET;
    if (0 == motor2.initFOC())
    {
        Serial.println(F("FOC_CAL_SIMPLEFOC_FAIL"));
        return false;
    }

    print_foc_calibration_profile();

    // 电角度是环形量，不能直接使用普通减法比较 0 与 2PI 附近的两个值。
    float zero_error = fabsf(motor2.zero_electric_angle - kStoredZeroElectricalAngle);
    if (zero_error > _PI)
    {
        zero_error = _2PI - zero_error;
    }

    const bool direction_ok = (motor2.sensor_direction == kStoredSensorDirection);
    const bool pole_pairs_ok = motor2.pp_check_result;
    const bool zero_ok = isfinite(motor2.zero_electric_angle) && (zero_error <= kAutoAlignmentMaxZeroError);
    if (!direction_ok || !pole_pairs_ok || !zero_ok)
    {
        stop_calibration_voltage();
        Serial.print(F("FOC_CAL_SIMPLEFOC_REJECT,dir="));
        Serial.print(direction_ok ? F("OK") : F("BAD"));
        Serial.print(F(",pp="));
        Serial.print(pole_pairs_ok ? F("OK") : F("BAD"));
        Serial.print(F(",zero_error="));
        SerialPrintFixed(zero_error, 1000U, 3U);
        Serial.println(F("rad"));
        return false;
    }

    Serial.println(F("FOC_CAL_SIMPLEFOC_DONE"));
    return true;
}

void print_foc_calibration_applied()
{
    // 校准结果已经写入当前运行对象；复位后仍会回到源码中的默认常量。
    Serial.println(F("FOC_CAL_APPLIED,RAM_ONLY"));
}

bool run_full_foc_calibration()
{
    if (!kAllowRuntimeFullFocCalibration)
    {
        Serial.println(F("FOC_CAL_LOCKED,USE_STORED_ALIGNMENT"));
        return false;
    }

    if (motor_user_disabled)
    {
        Serial.println(F("FOC_CAL_FAIL,MOTOR_DISABLED"));
        return false;
    }

    if (as5600_verification_enabled && !verify_as5600_communication())
    {
        motor2.disable();
        set_motor_enable(false);
        Serial.println(F("FOC_CAL_FAIL,AS5600_VERIFY"));
        return false;
    }

    current_pid_debug_active = false;
    current_pid_debug_restore.valid = false;
    Serial.println(F("FOC_CAL_START"));
    motor2.target = motor2.shaft_angle;

    // 每次完整校准都从板级原始 ADC 定义开始，避免继承上次 driverAlign() 的交换结果。
    current_sense2.pinA = RA_ADC_U_CURRENT;
    current_sense2.pinB = RA_ADC_V_CURRENT;
    current_sense2.gain_a = kStoredCurrentSenseGain;
    current_sense2.gain_b = kStoredCurrentSenseGain;
    current_sense2.gain_c = kStoredCurrentSenseGain;
    // 网页完整校准也只校准传感器方向/零电角度，不执行两相电流通道自动对齐。
    current_sense2.skip_align = kSkipCurrentSenseDriverAlignment;
    if (0 == current_sense2.init())
    {
        latch_motor_fault(MotorFaultCurrentSenseInit);
        Serial.println(F("FOC_CAL_FAIL,CURRENT_SENSE_INIT"));
        return false;
    }

    set_motor_enable(true);
    motor2.enable();
    const bool use_simplefoc = kUseSimpleFocAlignment;
    if (!(use_simplefoc ? run_simplefoc_sensor_calibration() : run_controlled_sensor_calibration()))
    {
        latch_motor_fault(MotorFaultCalibration);
        return false;
    }

    capture_current_gain_reference();
    (void) apply_current_gain_mode(current_gain_mode);
    print_foc_calibration_profile();

    // 自动 driverAlign 在两相、非 PWM 同步的电流采样条件下可能判错相序或增益符号。
    // 校准结果仅供人工确认，绝不在本函数结束后直接带着新结果进入闭环。
    // P302 先拉低，即使后续主循环仍运行也不会向功率级持续施加错误的定子磁场。
    motor2.disable();
    set_motor_enable(false);
    Serial.println(F("FOC_CAL_REVIEW_REQUIRED,MOTOR_DISABLED"));
    // print_foc_calibration_applied();
    return true;
}

void process_dashboard_command(const char *input)
{
    bool success = false;

    if (starts_with(input, "NAV"))
    {
        const int request = atoi(input + 3);
        if ((0 == request) || (1 == request))
        {
            as5600_verification_enabled = (0 != request);
            Serial.print(F("AS5600_VERIFY_ENABLED:"));
            Serial.println(as5600_verification_enabled ? 1 : 0);
            success = true;
        }
        else if (2 == request)
        {
            // 手动验证时先停止 PWM 和 P302，验证完成后维持失能，避免暂停 FOC 循环时带电驻停。
            motor2.disable();
            set_motor_enable(false);
            success = verify_as5600_communication();
            Serial.println(F("AS5600_VERIFY_MANUAL_DONE,MOTOR_DISABLED"));
        }
    }
    else if (starts_with(input, "NCA"))
    {
        if (1 == atoi(input + 3))
        {
            success = run_full_foc_calibration();
        }
    }
    else if (starts_with(input, "NCE"))
    {
        print_foc_calibration_profile();
        print_motor_fault_status();
        success = true;
    }
    else if (starts_with(input, "NMD"))
    {
        (void) exit_current_pid_debug();
        const int mode = atoi(input + 3);
        if (0 == mode)
        {
            current_mode = 0;
            motor2.controller = MotionControlType::angle;
            motor2.target = motor2.shaft_angle;
            applyCurrentModePID();
            if (TorqueControlType::foc_current == motor2.torque_controller)
            {
                apply_current_torque_control("angle_mode");
            }
            else
            {
                apply_voltage_torque_control("angle_mode");
            }
            success = true;
        }
        else if (1 == mode)
        {
            current_mode = 1;
            motor2.controller = MotionControlType::velocity;
            motor2.target = 0.0f;
            applyCurrentModePID();
            if (TorqueControlType::foc_current == motor2.torque_controller)
            {
                apply_current_torque_control("velocity_mode");
            }
            else
            {
                apply_voltage_torque_control("velocity_mode");
            }
            success = true;
        }
    }
    else if (starts_with(input, "NTC"))
    {
        (void) exit_current_pid_debug();
        const int torque_mode = atoi(input + 3);
        if (0 == torque_mode)
        {
            apply_voltage_torque_control();
            success = true;
        }
        else if (1 == torque_mode)
        {
            apply_current_torque_control();
            success = true;
        }
        else if (2 == torque_mode)
        {
            apply_zero_current_diagnostic_control();
            success = true;
        }
    }
    else if (starts_with(input, "NDB"))
    {
        const char subcommand = input[3];
        if ('Q' == subcommand)
        {
            success = set_current_pid_debug_target(command_float_value(input, 4U));
        }
        else if ('D' == subcommand)
        {
            success = set_current_pid_debug_d_target(command_float_value(input, 4U));
        }
        else if ('L' == subcommand)
        {
            success = set_current_pid_debug_limit(command_float_value(input, 4U));
        }
        else if ('V' == subcommand)
        {
            success = set_current_pid_debug_voltage_limit(command_float_value(input, 4U));
        }
        else if ('S' == subcommand)
        {
            success = start_current_pid_debug_step(static_cast<uint32_t>(atoi(input + 4)));
        }

        if (success)
        {
            print_current_pid_debug_config();
        }
    }
    else if (starts_with(input, "NCP"))
    {
        const int enabled = atoi(input + 3);
        success = (0 != enabled) ? enter_current_pid_debug() : exit_current_pid_debug();
    }
    else if (starts_with(input, "NCR"))
    {
        success = set_current_pid_debug_target(command_float_value(input, 3U));
    }
    else if (starts_with(input, "NCL"))
    {
        const float value = command_float_value(input, 3U);
        if ((value > 0.0f) && (value <= kDashboardMaxCurrentLimit))
        {
            motor2.current_limit = value;
            motor2.updateCurrentLimit(motor2.current_limit);
            success = true;
        }
    }
    else if (starts_with(input, "NPS"))
    {
        const int order = atoi(input + 3);
        if ((order >= 0) && (order < 6) && renesas_simplefoc_set_phase_order(static_cast<uint8_t>(order)))
        {
            if (MotionControlType::angle == motor2.controller)
            {
                motor2.target = motor2.shaft_angle;
            }
            else if (MotionControlType::velocity == motor2.controller)
            {
                motor2.target = 0.0f;
            }

            apply_safe_startup_hold();
            motor2.PID_velocity.reset();
            motor2.PID_current_q.reset();
            motor2.PID_current_d.reset();

            Serial.print(F("PHASE_ORDER:"));
            Serial.print(order);
            Serial.print(F(","));
            Serial.println(phase_order_label(static_cast<uint8_t>(order)));
            success = true;
        }
    }
    else if (starts_with(input, "NDS"))
    {
        const int quiet = atoi(input + 3);
        diagnostic_quiet_mode = (0 != quiet);
        Serial.print(F("DIAG_QUIET:"));
        Serial.println(diagnostic_quiet_mode ? 1 : 0);
        success = true;
    }
    else if (starts_with(input, "NID"))
    {
        const int enabled = atoi(input + 3);
        current_diagnostic_mode = (0 != enabled);
        Serial.print(F("CURRENT_DIAG:"));
        Serial.println(current_diagnostic_mode ? 1 : 0);
        success = true;
    }
    else if (starts_with(input, "NCI"))
    {
        const int mode = atoi(input + 3);
        if ((mode >= 0) && apply_current_gain_mode(static_cast<uint8_t>(mode)))
        {
            Serial.print(F("CURRENT_GAIN_MODE:"));
            Serial.print(mode);
            Serial.print(F(","));
            Serial.println(current_gain_mode_label(static_cast<uint8_t>(mode)));
            success = true;
        }
    }
    else if (starts_with(input, "NTG"))
    {
        // 电流 PID 调试期间只接受 NCR 命令，避免位置圈数被误写入 Iq 目标。
        if (!current_pid_debug_active)
        {
            motor2.target = command_float_value(input, 3U);
            success = true;
        }
    }
    else if (starts_with(input, "NAP"))
    {
        angle_P = command_float_value(input, 3U);
        applyCurrentModePID();
        success = true;
    }
    else if (starts_with(input, "NVL"))
    {
        float value = command_float_value(input, 3U);
        if (0 == current_mode)
        {
            angle_vel_limit = value;
        }
        else
        {
            vel_vel_limit = value;
        }
        applyCurrentModePID();
        success = true;
    }
    else if (starts_with(input, "NVP"))
    {
        float value = command_float_value(input, 3U);
        if (0 == current_mode)
        {
            angle_vel_P = value;
        }
        else
        {
            vel_vel_P = value;
        }
        applyCurrentModePID();
        success = true;
    }
    else if (starts_with(input, "NVI"))
    {
        float value = command_float_value(input, 3U);
        if (0 == current_mode)
        {
            angle_vel_I = value;
        }
        else
        {
            vel_vel_I = value;
        }
        applyCurrentModePID();
        success = true;
    }
    else if (starts_with(input, "NVF"))
    {
        float value = command_float_value(input, 3U);
        if (0 == current_mode)
        {
            angle_vel_Tf = value;
        }
        else
        {
            vel_vel_Tf = value;
        }
        applyCurrentModePID();
        success = true;
    }
    else if (starts_with(input, "NCQP"))
    {
        float value = command_float_value(input, 4U);
        if (0 == current_mode)
        {
            angle_curr_q_P = value;
        }
        else
        {
            vel_curr_q_P = value;
        }
        applyCurrentModePID();
        success = true;
    }
    else if (starts_with(input, "NCQI"))
    {
        float value = command_float_value(input, 4U);
        if (0 == current_mode)
        {
            angle_curr_q_I = value;
        }
        else
        {
            vel_curr_q_I = value;
        }
        applyCurrentModePID();
        success = true;
    }
    else if (starts_with(input, "NCDP"))
    {
        float value = command_float_value(input, 4U);
        if (0 == current_mode)
        {
            angle_curr_d_P = value;
        }
        else
        {
            vel_curr_d_P = value;
        }
        applyCurrentModePID();
        success = true;
    }
    else if (starts_with(input, "NCDI"))
    {
        float value = command_float_value(input, 4U);
        if (0 == current_mode)
        {
            angle_curr_d_I = value;
        }
        else
        {
            vel_curr_d_I = value;
        }
        applyCurrentModePID();
        success = true;
    }
    else if (starts_with(input, "NCCF"))
    {
        float value = command_float_value(input, 4U);
        if (0 == current_mode)
        {
            angle_curr_Tf = value;
        }
        else
        {
            vel_curr_Tf = value;
        }
        applyCurrentModePID();
        success = true;
    }

    if (success)
    {
        Serial.print("MCU_ACK: ");
        Serial.println(input);
    }
}

void handleDashboardCommand()
{
    static char input[80];
    static size_t input_len = 0U;

    while (Serial.available())
    {
        const int read_value = Serial.read();
        if (read_value < 0)
        {
            return;
        }

        const char in_char = static_cast<char>(read_value);
        if (('\n' == in_char) || ('\r' == in_char))
        {
            if (input_len > 0U)
            {
                input[input_len] = '\0';
                process_dashboard_command(input);
                input_len = 0U;
            }
        }
        else if (input_len < (sizeof(input) - 1U))
        {
            input[input_len++] = in_char;
        }
        else
        {
            input_len = 0U;
        }
    }
}

// 已迁移至 app/app_telemetry.inc，保留原实现以便核对网页串口协议。
#if 0
void print_motor_dashboard_frame()
{
    char target_text[24];
    char actual_text[24];
    char frame[56];

    if (MotionControlType::angle == motor2.controller)
    {
        format_fixed(target_text, sizeof(target_text), motor2.target, 1000U, 3U);
        format_fixed(actual_text, sizeof(actual_text), motor2.shaft_angle, 1000U, 3U);
    }
    else if (MotionControlType::velocity == motor2.controller)
    {
        format_fixed(target_text, sizeof(target_text), motor2.target, 1000U, 3U);
        format_fixed(actual_text, sizeof(actual_text), motor2.shaft_velocity, 1000U, 3U);
    }
    else
    {
        format_fixed(target_text, sizeof(target_text), motor2.target, 1000U, 3U);
        format_fixed(actual_text, sizeof(actual_text), motor2.shaft_angle, 1000U, 3U);
    }

    // 紧凑遥测协议：target,actual。一次写入保证队列中一帧连续，网页可直接解析。
    snprintf(frame, sizeof(frame), "%s,%s", target_text, actual_text);
    Serial.println(frame);
}

void print_current_pid_debug_frame()
{
    char iq_reference_text[24];
    char id_reference_text[24];
    char current_q_text[24];
    char current_d_text[24];
    char voltage_q_text[24];
    char voltage_d_text[24];
    char phase_a_text[24];
    char phase_b_text[24];
    char phase_c_text[24];
    char electrical_angle_text[24];
    char current_limit_text[24];
    char voltage_limit_text[24];
    char frame[320];
    const PhaseCurrent_s phase_current = current_sense2.getPhaseCurrents();

    format_fixed(iq_reference_text, sizeof(iq_reference_text), current_pid_debug_iq_reference, 1000U, 3U);
    format_fixed(id_reference_text, sizeof(id_reference_text), current_pid_debug_id_reference, 1000U, 3U);
    format_fixed(current_q_text, sizeof(current_q_text), motor2.current.q, 1000U, 3U);
    format_fixed(current_d_text, sizeof(current_d_text), motor2.current.d, 1000U, 3U);
    format_fixed(voltage_q_text, sizeof(voltage_q_text), motor2.voltage.q, 1000U, 3U);
    format_fixed(voltage_d_text, sizeof(voltage_d_text), motor2.voltage.d, 1000U, 3U);
    format_fixed(phase_a_text, sizeof(phase_a_text), phase_current.a, 1000U, 3U);
    format_fixed(phase_b_text, sizeof(phase_b_text), phase_current.b, 1000U, 3U);
    format_fixed(phase_c_text, sizeof(phase_c_text), phase_current.c, 1000U, 3U);
    format_fixed(electrical_angle_text, sizeof(electrical_angle_text), motor2.electrical_angle, 1000U, 3U);
    format_fixed(current_limit_text, sizeof(current_limit_text), current_pid_debug_limit, 1000U, 3U);
    format_fixed(voltage_limit_text, sizeof(voltage_limit_text), current_pid_debug_voltage_limit, 1000U, 3U);
    snprintf(frame, sizeof(frame), "CURPID,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s",
             iq_reference_text, id_reference_text, current_q_text, current_d_text,
             voltage_q_text, voltage_d_text, phase_a_text, phase_b_text, phase_c_text,
             electrical_angle_text, current_limit_text, voltage_limit_text);
    Serial.println(frame);
}

void print_current_diagnostic_frame()
{
    PhaseCurrent_s phase_current = current_sense2.getPhaseCurrents();
    ABCurrent_s ab_current = current_sense2.getABCurrents(phase_current);
    DQCurrent_s dq_current = current_sense2.getDQCurrents(ab_current, motor2.electrical_angle);

    Serial.print(F("CUR,"));
    Serial.print(static_cast<int>(current_gain_mode));
    Serial.print(F(","));
    Serial.print(current_gain_mode_label(current_gain_mode));
    Serial.print(F(",Ia="));
    SerialPrintFixed(phase_current.a, 1000U, 3U);
    Serial.print(F(",Ib="));
    SerialPrintFixed(phase_current.b, 1000U, 3U);
    Serial.print(F(",Ic="));
    SerialPrintFixed(phase_current.c, 1000U, 3U);
    Serial.print(F(",Id="));
    SerialPrintFixed(dq_current.d, 1000U, 3U);
    Serial.print(F(",Iq="));
    SerialPrintFixed(dq_current.q, 1000U, 3U);
    Serial.print(F(",el="));
    SerialPrintFixed(motor2.electrical_angle, 1000U, 3U);
    Serial.print(F(",sp="));
    SerialPrintFixed(motor2.current_sp, 1000U, 3U);
    Serial.print(F(",Uq="));
    SerialPrintFixed(motor2.voltage.q, 1000U, 3U);
    Serial.print(F(",Ud="));
    SerialPrintFixed(motor2.voltage.d, 1000U, 3U);
    Serial.println();
}

void update_motor_oled_display()
{
    const float target_vel = motor_target_velocity();
    const float target_curr = motor_target_current(target_vel);
    const bool current_torque = (TorqueControlType::foc_current == motor2.torque_controller);

    OLED_ShowFixedLine(0, "A", motor2.shaft_angle, 100U, 2U, "rad");
    OLED_ShowFixedLine(8, "V", motor2.shaft_velocity, 100U, 2U, "rad/s");
    OLED_ShowFixedLine(16, "Iq", motor2.current.q, 1000U, 3U, "A");

    if (MotionControlType::angle == motor2.controller)
    {
        OLED_ShowFixedLine(24, "TA", motor2.target, 100U, 2U, "rad");
    }
    else
    {
        OLED_ShowLine(0, 24, "TA:--rad");
    }

    OLED_ShowFixedLine(32, "TV", target_vel, 100U, 2U, "rad/s");
    OLED_ShowFixedLine(40, "TI", target_curr, 1000U, 3U, "A");
    if (MotionControlType::angle == motor2.controller)
    {
        OLED_ShowLine(0, 48, current_torque ? "ANG FOC CURRENT" : "ANG VOLT SAFE");
    }
    else
    {
        OLED_ShowLine(0, 48, current_torque ? "VEL FOC CURRENT" : "VEL VOLT SAFE");
    }
    OLED_ShowLine(0, 56, motor_user_disabled ? "P000 MOTOR OFF" : "");
    OLED_Refresh();
}

#endif

#include "app/app_telemetry.inc"

void motor_app_setup()
{
#if RA4M2_ENABLE_UART_DASHBOARD
    Serial.begin(115200);
#endif
    print_reset_status();
    init_oled_display("FOC MOTOR MODE");
    Serial.print(F("FOC_BOOT_ALIGN:"));
    Serial.println(foc_alignment_mode_label(kFocAlignmentMode));
    set_motor_enable(false);

    // 必须在任何 PWM/对齐电压输出前启用 P000。
    init_user_button_irq();

    as5600_i2c_init();
    wait_until_as5600_calibrated();
    if (as5600_verification_enabled)
    {
        // 连续通信质量不通过时不允许给 PWM/驱动器上电，更不能进入原生对齐扫描。
        wait_until_as5600_verified();
    }

    // ADC 启动校准流程保留为注释，当前电流零偏由 InlineCurrentSense 初始化流程处理。
    /*
    AdcCalibrationResult adc_calibration = {};
    wait_until_adc_calibrated(&adc_calibration);
    */

    set_motor_enable(true);
    motor2.linkSensor(&sensor2);

    driver2.voltage_power_supply = 12.0f;
    if (0 == driver2.init())
    {
        latch_motor_fault(MotorFaultDriverInit);
        Serial.println(F("FOC_BOOT_DRIVER_INIT_FAIL,MOTOR_DISABLED"));
        return;
    }
    motor2.linkDriver(&driver2);
    motor_driver_ready = true;

    current_sense2.linkDriver(&driver2);
    current_sense2.pinA = RA_ADC_U_CURRENT;
    current_sense2.pinB = RA_ADC_V_CURRENT;
    current_sense2.gain_a = kStoredCurrentSenseGain;
    current_sense2.gain_b = kStoredCurrentSenseGain;
    current_sense2.gain_c = kStoredCurrentSenseGain;
    current_sense2.skip_align = kSkipCurrentSenseDriverAlignment;
    if (0 == current_sense2.init())
    {
        latch_motor_fault(MotorFaultCurrentSenseInit);
        Serial.println(F("FOC_BOOT_CURRENT_SENSE_INIT_FAIL,MOTOR_DISABLED"));
        return;
    }
    motor2.linkCurrentSense(&current_sense2);

    motor2.torque_controller = kUseCurrentTorqueStartup ? TorqueControlType::foc_current : TorqueControlType::voltage;
    motor2.controller = MotionControlType::angle;
    motor2.foc_modulation = RA4M2_USE_SVPWM ? FOCModulationType::SpaceVectorPWM : FOCModulationType::SinePWM;
    motor2.modulation_centered = 1;
    motor2.voltage_sensor_align = kSensorAutoAlignmentVoltage;
    motor2.current_limit = 0.7f;
    motor2.voltage_limit = 12.5f;

    applyCurrentModePID();

    motor2.useMonitoring(Serial);
    motor2.monitor_downsample = 0;
    motor2.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_CURR_Q;

    motor2.init();
    operating_voltage_limit = motor2.voltage_limit;
    if (kUseStoredFocAlignment)
    {
        // 设置已确认的编码器方向和电角度零点后，initFOC() 会跳过传感器对齐动作。
        motor2.sensor_direction = kStoredSensorDirection;
        motor2.zero_electric_angle = kStoredZeroElectricalAngle;
    }
    if (kUseStoredFocAlignment)
    {
        if (0 == motor2.initFOC())
        {
            latch_motor_fault(MotorFaultFocInit);
            Serial.println(F("FOC_BOOT_CAL_FAIL,MOTOR_DISABLED"));
            return;
        }
    }
    else if (!(kUseSimpleFocAlignment ? run_simplefoc_sensor_calibration() : run_controlled_sensor_calibration()))
    {
        latch_motor_fault(MotorFaultCalibration);
        Serial.println(F("FOC_BOOT_CAL_FAIL,MOTOR_DISABLED"));
        return;
    }

    if ((FocAlignmentMode::Controlled == kFocAlignmentMode) && !kApplyAutoCalibrationResultToClosedLoop)
    {
        // 先保留并输出本次候选结果，再恢复已验证安装参数进入闭环。
        print_foc_calibration_profile();
        motor2.sensor_direction = kStoredSensorDirection;
        motor2.zero_electric_angle = kStoredZeroElectricalAngle;
        if (0 == motor2.initFOC())
        {
            latch_motor_fault(MotorFaultFocInit);
            Serial.println(F("FOC_BOOT_STORED_ALIGN_FAIL,MOTOR_DISABLED"));
            return;
        }
        Serial.println(F("FOC_BOOT_CAL_CANDIDATE_ONLY,USING_STORED_ALIGNMENT"));
    }

    if (!kUseStoredFocAlignment && kSensorAutoCalibrationReviewOnly)
    {
        // 自动对齐通过时仍先停在审查状态，防止错误候选零点直接造成闭环堵转。
        print_foc_calibration_profile();
        latch_motor_fault(MotorFaultCalibration);
        Serial.println(F("FOC_BOOT_CAL_REVIEW_REQUIRED,MOTOR_DISABLED"));
        return;
    }

    if (!kUseStoredFocAlignment)
    {
        print_foc_calibration_profile();
        Serial.println(F("FOC_BOOT_CAL_APPLIED,ENTER_ANGLE_HOLD"));
    }

    capture_current_gain_reference();
    apply_current_gain_mode(current_gain_mode);
    current_mode = 0;
    motor2.controller = MotionControlType::angle;
    applyCurrentModePID();
    if (kUseCurrentTorqueStartup)
    {
        apply_current_torque_control("startup_angle");
    }
    else
    {
        apply_safe_startup_hold();
    }
    motor2.loopFOC();
    motor2.target = motor2.shaft_angle;

    Serial.print(F("CONTROL_MODE:ANGLE_HOLD,target="));
    SerialPrintFixed(motor2.target, 1000U, 3U);
    Serial.println(F("rad"));

    // P000 中断已在 PWM 上电前初始化；此处保留原调用位置作为流程提示。
    // init_user_button_irq();

    command.add('B', doMotor2, "motor 2");
    Serial.print(F("PHASE_ORDER:"));
    Serial.print(static_cast<int>(renesas_simplefoc_get_phase_order()));
    Serial.print(F(","));
    Serial.println(phase_order_label(renesas_simplefoc_get_phase_order()));
    Serial.print(F("CURRENT_GAIN_MODE:"));
    Serial.print(static_cast<int>(current_gain_mode));
    Serial.print(F(","));
    Serial.println(current_gain_mode_label(current_gain_mode));
    Serial.println(F("RA4M2 SimpleFOC motor mode ready."));
}

void motor_app_loop()
{
    // J-Link 结构体调参入口：仅在检测到新的申请序号时写入参数，平时不影响 FOC 高频循环。
    apply_jlink_tuning_request();

    // 故障锁存后保持功率级失能，但保留传感器读取和故障状态回传，便于 CAN/串口诊断。
    if (motor_fault_latched)
    {
        sensor2.update();
        monitor_runtime_as5600_fault();
        handleDashboardCommand();
        update_jlink_tune_status();
        return;
    }

    // FOC 内环：更新编码器角度、读取相电流、执行 Clarke/Park 变换及 Id/Iq PID，必须尽可能高频执行。
    // 此调用前后不要加入 OLED 刷新、阻塞延时或大量串口打印。
    motor2.loopFOC();
    monitor_runtime_as5600_fault();
    if (motor_fault_latched)
    {
        return;
    }

    // P000 中断已经直接拉低 P302；在主循环中完成关 PWM、SimpleFOC 和界面收尾。
    handle_user_button_motor_disable();

    // 独立电流环调试时，生成 Iq/Id 手动给定或自动回零阶跃给定；普通位置/速度模式不执行此分支。
    update_current_pid_debug_stimulus();

    // 运动外环：位置/速度环计算 Iq 目标；电流调试时则把 torque 模式目标送入电流内环。
    motor2.move();

    // 处理网页和串口助手下发的命令。放在运动控制之后，避免接收解析占用 FOC 内环时间。
    handleDashboardCommand();

    // 回传 J-Link 观察量，供 Live Watch 确认实际生效的模式、目标和 PID 状态。
    update_jlink_tune_status();

    // 周期遥测：受 20 ms 限速，避免大量串口输出拖慢 FOC。电流调试使用更完整的 CURPID 帧。
    static uint32_t last_print = 0U;
    const uint32_t now = millis();
    if (!diagnostic_quiet_mode && ((now - last_print) >= kSerialPeriodMs))
    {
        last_print = now;
        if (current_pid_debug_active)
        {
            print_current_pid_debug_frame();
        }
        else
        {
            print_motor_dashboard_frame();
        }
    }

    // 可选的相电流诊断帧，频率更低；用于检查 ADC 零偏、相电流符号及 D/Q 变换结果。
    static uint32_t last_current_diag = 0U;
    if (current_diagnostic_mode && ((now - last_current_diag) >= kCurrentDiagPeriodMs))
    {
        last_current_diag = now;
        print_current_diagnostic_frame();
    }

//    static uint32_t last_oled = millis();
//    if (!diagnostic_quiet_mode && ((now - last_oled) >= kOledPeriodMs))
//    {
//        last_oled = now;
//        update_motor_oled_display();
//    }
}

#else

void read_as5600()
{
    sensor2.update();
    as5600_angle_rad = sensor2.getAngle();
    as5600_mechanical_rad = sensor2.getMechanicalAngle();
    as5600_velocity_rad_s = sensor2.getVelocity();
    as5600_turns = sensor2.getFullRotations();
    as5600_i2c_error = sensor2.currWireError;
    sample_count++;
}

void print_angle_frame()
{
    if (0U != as5600_i2c_error)
    {
        Serial.print("ERR,AS5600,");
        Serial.println(static_cast<int>(as5600_i2c_error));
        return;
    }

    SerialPrintFixed(as5600_angle_rad, 1000U, 3U);
    Serial.print(",");
    SerialPrintFixed(as5600_mechanical_rad, 1000U, 3U);
    Serial.print(",");
    SerialPrintFixed(as5600_velocity_rad_s, 1000U, 3U);
    Serial.print(",");
    Serial.println(static_cast<long>(as5600_turns));
}

void update_oled_display()
{
    char line[32];
    char value[16];

    OLED_ShowLine(0, 0, "AS5600 MONITOR");

    if (0U != as5600_i2c_error)
    {
        snprintf(line, sizeof(line), "I2C ERR:%u", static_cast<unsigned int>(as5600_i2c_error));
        OLED_ShowLine(0, 16, line);
        OLED_ShowLine(0, 24, "Check AS5600 bus");
        OLED_ShowLine(0, 32, "SCL/SDA/VCC/GND");
        OLED_ShowLine(0, 40, "");
        OLED_ShowLine(0, 48, "");
        OLED_ShowLine(0, 56, "");
        OLED_Refresh();
        return;
    }

    format_fixed(value, sizeof(value), as5600_angle_rad, 1000U, 3U);
    snprintf(line, sizeof(line), "Angle:%s rad", value);
    OLED_ShowLine(0, 16, line);

    format_fixed(value, sizeof(value), as5600_mechanical_rad * kRadToDeg, 10U, 1U);
    snprintf(line, sizeof(line), "Mech:%s deg", value);
    OLED_ShowLine(0, 24, line);

    format_fixed(value, sizeof(value), as5600_velocity_rad_s, 100U, 2U);
    snprintf(line, sizeof(line), "Vel:%s rad/s", value);
    OLED_ShowLine(0, 32, line);

    snprintf(line, sizeof(line), "Turns:%ld", static_cast<long>(as5600_turns));
    OLED_ShowLine(0, 40, line);

    snprintf(line, sizeof(line), "Samples:%lu", static_cast<unsigned long>(sample_count));
    OLED_ShowLine(0, 48, line);

    OLED_ShowLine(0, 56, "Motor output OFF");
    OLED_Refresh();
}

void as5600_monitor_setup()
{
    Serial.begin(115200);
    init_oled_display("AS5600 MONITOR");
    as5600_i2c_init();

    read_as5600();
    update_oled_display();

    Serial.println(F("RA4M2 AS5600 angle monitor ready."));
}

void as5600_monitor_loop()
{
    const uint32_t now = millis();

    static uint32_t last_serial = 0U;
    if ((now - last_serial) >= kSerialPeriodMs)
    {
        last_serial = now;
        read_as5600();
        print_angle_frame();
    }

    static uint32_t last_oled = 0U;
    if ((now - last_oled) >= kOledPeriodMs)
    {
        last_oled = now;
        update_oled_display();
    }
}

#endif

} // namespace

extern "C" void simplefoc_app_main(void)
{
#if RA4M2_ENABLE_MOTOR_CONTROL
    // 上电阶段完成 I2C/ADC/PWM/驱动器、AS5600 和 SimpleFOC 初始化；仅执行一次。
    motor_app_setup();
    while (true)
    {
        // 裸机主循环：不使用 delay，持续运行 FOC 内环与外环。
        motor_app_loop();
    }
#else
    // 未启用电机控制时，仅运行 AS5600 监视模式，便于独立排查编码器通信。
    as5600_monitor_setup();
    while (true)
    {
        as5600_monitor_loop();
    }
#endif
}
