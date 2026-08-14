/*
 * 显示与数值格式化。
 *
 * 本文件由 simplefoc_ra4m2_app.cpp 在同一匿名命名空间中包含，
 * 因此不改变现有全局状态、初始化顺序或链接方式。
 */

void OLED_ShowLine(u8 x, u8 y, const char *text)
{
    constexpr size_t kMaxChars = 21U;
    char line[kMaxChars + 1U];
    size_t index = 0U;

    for (; index < kMaxChars; index++)
    {
        if ('\0' == text[index])
        {
            break;
        }
        line[index] = text[index];
    }
    for (; index < kMaxChars; index++)
    {
        line[index] = ' ';
    }
    line[kMaxChars] = '\0';

    OLED_ShowString(x, y, reinterpret_cast<u8 *>(line), 8, 1);
}

void format_fixed(char *buffer, size_t size, float value, uint32_t scale, uint8_t decimals)
{
    const bool negative = value < 0.0f;
    const float abs_value = negative ? -value : value;
    const uint32_t scaled = static_cast<uint32_t>((abs_value * static_cast<float>(scale)) + 0.5f);
    const uint32_t integer = scaled / scale;
    const uint32_t fraction = scaled % scale;

    if (negative)
    {
        snprintf(buffer, size, "-%lu.%0*lu", static_cast<unsigned long>(integer), decimals, static_cast<unsigned long>(fraction));
    }
    else
    {
        snprintf(buffer, size, "%lu.%0*lu", static_cast<unsigned long>(integer), decimals, static_cast<unsigned long>(fraction));
    }
}

void SerialPrintFixed(float value, uint32_t scale, uint8_t decimals)
{
    char buffer[24];
    format_fixed(buffer, sizeof(buffer), value, scale, decimals);
    Serial.print(buffer);
}

void OLED_ShowFixedLine(u8 y, const char *label, float value, uint32_t scale, uint8_t decimals, const char *unit)
{
    char value_text[16];
    char line[32];

    format_fixed(value_text, sizeof(value_text), value, scale, decimals);
    snprintf(line, sizeof(line), "%s:%s%s", label, value_text, unit);
    OLED_ShowLine(0, y, line);
}

void as5600_i2c_init()
{
    Wire.begin();
    sensor2.init(&Wire);
}

void init_oled_display(const char *title)
{
    (void) title;

#if RA4M2_ENABLE_OLED_DISPLAY
    fsp_err_t g_err = R_IIC_MASTER_Open(&g_i2c_master0_ctrl, &g_i2c_master0_cfg);
    assert((FSP_SUCCESS == g_err) || (FSP_ERR_ALREADY_OPEN == g_err));

    OLED_Init();
    OLED_Clear();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
    OLED_ShowPicture(0, 0, 128, 40, NJUPT, 1);
    OLED_ShowPicture(0, 40, 117, 19, BMP3, 1);
    // OLED_ShowPicture(0, 19, 128, 40, NJUPT, 1);

    OLED_Refresh();
    delay(1000UL);
#endif
}
