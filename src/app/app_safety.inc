/* 安全使能、故障锁存与 P000 紧急失能。 */

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
        // 中断内只直接拉低 P302；SimpleFOC 和显示收尾仍由主循环完成。
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
