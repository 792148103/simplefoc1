/*
 * CAN0 单电机节点协议。
 * 接收 ID 为节点号，当前默认节点号 1 使用 0x001。
 * 发送状态 ID：0x180 + 节点号、0x1A0 + 节点号；命令应答 ID：0x1C0 + 节点号。
 */

constexpr uint8_t kCanNodeId = 1U;
constexpr uint32_t kCanCommandId = kCanNodeId;
constexpr uint32_t kCanStatusMotionId = 0x180U + kCanNodeId;
constexpr uint32_t kCanStatusElectricalId = 0x1A0U + kCanNodeId;
constexpr uint32_t kCanResponseId = 0x1C0U + kCanNodeId;
constexpr uint32_t kCanTxMailbox = 0U;
constexpr uint32_t kCanStatusPeriodMs = 20U;
constexpr float kCanDefaultProfileSpeed = 5.0f;
constexpr float kCanDefaultAcceleration = 10.0f;
constexpr float kCanDefaultDeceleration = 15.0f;
constexpr float kCanMaxProfileSpeed = 1000.0f;
constexpr float kCanMaxAcceleration = 10000.0f;
constexpr float kCanMaxTarget = 100000.0f;

enum CanCommand : uint8_t
{
    CanCommandSetMode          = 0x01U,
    CanCommandSetTarget        = 0x02U,
    CanCommandSetProfileSpeed  = 0x03U,
    CanCommandSetAcceleration  = 0x04U,
    CanCommandSetDeceleration  = 0x05U,
    CanCommandStop             = 0x06U,
    CanCommandStatusRequest    = 0x07U,
};

volatile bool can_rx_pending = false;
volatile bool can_tx_ready = true;
volatile bool can_bus_off = false;
volatile uint32_t can_error_code = 0U;
can_frame_t can_rx_frame = {};
bool can_opened = false;
bool can_response_pending = false;
bool can_status_electrical_next = false;
uint8_t can_response_command = 0U;
uint8_t can_response_result = 0U;
float can_requested_target = 0.0f;
float can_profile_speed_limit = kCanDefaultProfileSpeed;
float can_acceleration_limit = kCanDefaultAcceleration;
float can_deceleration_limit = kCanDefaultDeceleration;
float can_profile_velocity = 0.0f;
bool can_target_active = false;

extern "C" void can_callback(can_callback_args_t *p_args)
{
    if (NULL == p_args)
    {
        return;
    }

    switch (p_args->event)
    {
        case CAN_EVENT_TX_COMPLETE:
            can_tx_ready = true;
            break;

        case CAN_EVENT_RX_COMPLETE:
            // 回调仅复制帧并置位；所有电机对象访问都在主循环进行。
            can_rx_frame = p_args->frame;
            can_rx_pending = true;
            break;

        case CAN_EVENT_ERR_BUS_OFF:
            can_bus_off = true;
            can_error_code = p_args->error;
            can_tx_ready = true;
            break;

        case CAN_EVENT_ERR_WARNING:
        case CAN_EVENT_ERR_PASSIVE:
        case CAN_EVENT_ERR_BUS_LOCK:
        case CAN_EVENT_ERR_CHANNEL:
        case CAN_EVENT_ERR_GLOBAL:
        case CAN_EVENT_TX_ABORTED:
        case CAN_EVENT_MAILBOX_MESSAGE_LOST:
        case CAN_EVENT_FIFO_MESSAGE_LOST:
            can_error_code = p_args->error;
            break;

        default:
            break;
    }
}

float can_decode_float(const uint8_t *data)
{
    float value = 0.0f;
    memcpy(&value, data, sizeof(value));
    return value;
}

void can_encode_float(uint8_t *data, float value)
{
    memcpy(data, &value, sizeof(value));
}

void can_encode_u32(uint8_t *data, uint32_t value)
{
    memcpy(data, &value, sizeof(value));
}

float can_move_towards(float value, float target, float maximum_delta)
{
    if (value < target)
    {
        return (value + maximum_delta < target) ? (value + maximum_delta) : target;
    }
    if (value > target)
    {
        return (value - maximum_delta > target) ? (value - maximum_delta) : target;
    }
    return value;
}

bool can_valid_float(float value, float minimum, float maximum)
{
    return isfinite(value) && (value >= minimum) && (value <= maximum);
}

void can_queue_response(uint8_t command_code, bool success)
{
    can_response_command = command_code;
    can_response_result = success ? 1U : 0U;
    can_response_pending = true;
}

bool can_apply_mode(uint8_t requested_motion_mode, uint8_t requested_torque_mode)
{
    if ((requested_motion_mode > 1U) || (requested_torque_mode > 1U) || motor_fault_latched || motor_user_disabled)
    {
        return false;
    }

    (void) exit_current_pid_debug();
    current_mode = static_cast<int>(requested_motion_mode);
    motor2.controller = (0U == requested_motion_mode) ? MotionControlType::angle : MotionControlType::velocity;
    motor2.target = (0U == requested_motion_mode) ? motor2.shaft_angle : 0.0f;
    can_requested_target = motor2.target;
    can_profile_velocity = 0.0f;
    can_target_active = false;
    applyCurrentModePID();

    if (0U == requested_torque_mode)
    {
        apply_voltage_torque_control("can_mode");
    }
    else
    {
        apply_current_torque_control("can_mode");
    }
    return true;
}

bool can_process_command(const can_frame_t *frame)
{
    if ((!frame) || (CAN_ID_MODE_STANDARD != frame->id_mode) ||
        (CAN_FRAME_TYPE_DATA != frame->type) || (kCanCommandId != frame->id) ||
        (frame->data_length_code < 1U))
    {
        return false;
    }

    const uint8_t command_code = frame->data[0];
    bool success = false;

    switch (command_code)
    {
        case CanCommandSetMode:
            success = (frame->data_length_code >= 3U) && can_apply_mode(frame->data[1], frame->data[2]);
            break;

        case CanCommandSetTarget:
            if ((frame->data_length_code >= 5U) && !motor_fault_latched && !motor_user_disabled)
            {
                const float target = can_decode_float(&frame->data[1]);
                if (can_valid_float(target, -kCanMaxTarget, kCanMaxTarget))
                {
                    can_requested_target = target;
                    can_profile_velocity = 0.0f;
                    can_target_active = true;
                    success = true;
                }
            }
            break;

        case CanCommandSetProfileSpeed:
            if (frame->data_length_code >= 5U)
            {
                const float value = can_decode_float(&frame->data[1]);
                if (can_valid_float(value, 0.01f, kCanMaxProfileSpeed))
                {
                    can_profile_speed_limit = value;
                    success = true;
                }
            }
            break;

        case CanCommandSetAcceleration:
            if (frame->data_length_code >= 5U)
            {
                const float value = can_decode_float(&frame->data[1]);
                if (can_valid_float(value, 0.01f, kCanMaxAcceleration))
                {
                    can_acceleration_limit = value;
                    success = true;
                }
            }
            break;

        case CanCommandSetDeceleration:
            if (frame->data_length_code >= 5U)
            {
                const float value = can_decode_float(&frame->data[1]);
                if (can_valid_float(value, 0.01f, kCanMaxAcceleration))
                {
                    can_deceleration_limit = value;
                    success = true;
                }
            }
            break;

        case CanCommandStop:
            can_requested_target = (MotionControlType::angle == motor2.controller) ? motor2.shaft_angle : 0.0f;
            motor2.target = can_requested_target;
            can_profile_velocity = 0.0f;
            can_target_active = false;
            motor2.PID_velocity.reset();
            success = !motor_fault_latched;
            break;

        case CanCommandStatusRequest:
            success = true;
            break;

        default:
            break;
    }

    can_queue_response(command_code, success);
    return success;
}

void can_update_motion_profile()
{
    static uint32_t previous_ms = 0U;
    const uint32_t now = millis();
    if (0U == previous_ms)
    {
        previous_ms = now;
        return;
    }

    float dt = static_cast<float>(now - previous_ms) * 0.001f;
    previous_ms = now;
    if (!can_target_active || (dt <= 0.0f))
    {
        return;
    }
    if (dt > 0.050f)
    {
        dt = 0.050f;
    }

    if (MotionControlType::velocity == motor2.controller)
    {
        const float target = limit_symmetric(can_requested_target, can_profile_speed_limit);
        const float current = motor2.target;
        const bool accelerating = (fabsf(target) > fabsf(current)) && ((target * current) >= 0.0f);
        const float rate = accelerating ? can_acceleration_limit : can_deceleration_limit;
        motor2.target = can_move_towards(current, target, rate * dt);
        if (fabsf(motor2.target - target) < 0.0001f)
        {
            can_target_active = false;
        }
        return;
    }

    if (MotionControlType::angle == motor2.controller)
    {
        const float position_error = can_requested_target - motor2.target;
        const float distance = fabsf(position_error);
        if ((distance < 0.0005f) && (fabsf(can_profile_velocity) < 0.001f))
        {
            motor2.target = can_requested_target;
            can_profile_velocity = 0.0f;
            can_target_active = false;
            return;
        }

        const float direction = (position_error >= 0.0f) ? 1.0f : -1.0f;
        const float braking_speed = sqrtf(2.0f * can_deceleration_limit * distance);
        const float requested_speed = direction * fminf(can_profile_speed_limit, braking_speed);
        const bool accelerating = (fabsf(requested_speed) > fabsf(can_profile_velocity)) &&
                                  ((requested_speed * can_profile_velocity) >= 0.0f);
        const float rate = accelerating ? can_acceleration_limit : can_deceleration_limit;
        can_profile_velocity = can_move_towards(can_profile_velocity, requested_speed, rate * dt);

        const float next_target = motor2.target + (can_profile_velocity * dt);
        if (((direction > 0.0f) && (next_target >= can_requested_target)) ||
            ((direction < 0.0f) && (next_target <= can_requested_target)))
        {
            motor2.target = can_requested_target;
            can_profile_velocity = 0.0f;
            can_target_active = false;
        }
        else
        {
            motor2.target = next_target;
        }
    }
}

bool can_send_frame(can_frame_t *frame)
{
    if (!can_opened || !can_tx_ready || !frame)
    {
        return false;
    }

    const fsp_err_t err = R_CAN_Write(&g_can0_ctrl, kCanTxMailbox, frame);
    if (FSP_SUCCESS == err)
    {
        can_tx_ready = false;
        return true;
    }
    return false;
}

void can_send_response()
{
    can_frame_t frame = {};
    frame.id = kCanResponseId;
    frame.id_mode = CAN_ID_MODE_STANDARD;
    frame.type = CAN_FRAME_TYPE_DATA;
    frame.data_length_code = 8U;
    frame.data[0] = can_response_command;
    frame.data[1] = can_response_result;
    frame.data[2] = static_cast<uint8_t>(current_mode);
    frame.data[3] = (TorqueControlType::foc_current == motor2.torque_controller) ? 1U : 0U;
    can_encode_u32(&frame.data[4], motor_fault_code);

    if (can_send_frame(&frame))
    {
        can_response_pending = false;
    }
}

void can_send_status()
{
    can_frame_t frame = {};
    frame.id_mode = CAN_ID_MODE_STANDARD;
    frame.type = CAN_FRAME_TYPE_DATA;
    frame.data_length_code = 8U;

    if (can_status_electrical_next)
    {
        frame.id = kCanStatusElectricalId;
        can_encode_float(&frame.data[0], motor2.current.q);
        can_encode_u32(&frame.data[4], motor_fault_code);
    }
    else
    {
        frame.id = kCanStatusMotionId;
        can_encode_float(&frame.data[0], motor2.shaft_angle);
        can_encode_float(&frame.data[4], motor2.shaft_velocity);
    }

    if (can_send_frame(&frame))
    {
        can_status_electrical_next = !can_status_electrical_next;
    }
}

void can_app_init()
{
#if RA4M2_ENABLE_CAN_CONTROL
    const fsp_err_t err = R_CAN_Open(&g_can0_ctrl, &g_can0_cfg);
    if ((FSP_SUCCESS == err) || (FSP_ERR_ALREADY_OPEN == err))
    {
        can_opened = true;
        Serial.print(F("CAN_READY,node="));
        Serial.print(kCanNodeId);
        Serial.println(F(",bitrate=500000"));
    }
    else
    {
        Serial.print(F("CAN_OPEN_FAIL:"));
        Serial.println(static_cast<int>(err));
    }
#endif
}

void can_app_loop()
{
#if RA4M2_ENABLE_CAN_CONTROL
    if (!can_opened)
    {
        return;
    }

    if (can_rx_pending)
    {
        const can_frame_t received_frame = can_rx_frame;
        can_rx_pending = false;
        (void) can_process_command(&received_frame);
    }

    if (!motor_fault_latched)
    {
        can_update_motion_profile();
    }

    if (can_response_pending)
    {
        can_send_response();
    }

    static uint32_t last_status_ms = 0U;
    const uint32_t now = millis();
    if ((now - last_status_ms) >= kCanStatusPeriodMs)
    {
        if (can_tx_ready)
        {
            last_status_ms = now;
            can_send_status();
        }
    }
#endif
}
