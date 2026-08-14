/* AS5600 与 ADC 启动校验。PWM 与 FOC 初始化不在这里执行。 */

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
