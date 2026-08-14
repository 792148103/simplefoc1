/* 网页遥测帧、电流诊断帧及 OLED 运行页面。 */

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
