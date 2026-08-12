#include "renesas_simplefoc_port.h"

#include "Arduino.h"
#include "SPI.h"
#include "Wire.h"
#include "SimpleFOC/current_sense/hardware_api.h"
#include "SimpleFOC/drivers/hardware_api.h"

#include <string.h>

// B group restore is kept in ra4m2_three_phase0_restore.inc for reference, but
// the current project uses the FSP-generated A group g_three_phase_pwm0.

extern "C"
{
volatile uint32_t g_ra4m2_uart_tx_dropped_bytes = 0U;
volatile uint32_t g_ra4m2_uart_tx_start_errors = 0U;
}

namespace
{
constexpr float kAdcReferenceVoltage = 3.3f;
constexpr float kAdcMaxCount = 4095.0f;
constexpr uint32_t kDefaultI2CTimeoutUs = 20000U;
constexpr uint32_t kUartRxBufferSize = 256U;
// 上位机遥测与调试日志共用此队列。增大容量只增加 RAM 占用，不改变 DTC 发送方式。
constexpr uint32_t kUartTxBufferSize = 2048U;

struct Renesas3PwmParams
{
    const three_phase_instance_t *instance;
    uint32_t period_counts;
    bool opened;
};

struct RenesasAdcParams
{
    int pins[3];
    float adc_voltage_conv;
    uint32_t scan_mask;
    float cached_voltage[3];
    uint8_t cached_reads_left;
    bool cache_valid;
};

Renesas3PwmParams g_pwm_a = {&g_three_phase_pwm0, 0U, false};
Renesas3PwmParams g_pwm_b = {nullptr, 0U, false};
RenesasAdcParams g_adc_inline = {{-1, -1, -1}, kAdcReferenceVoltage / kAdcMaxCount, 0U, {0.0f, 0.0f, 0.0f}, 0U, false};

bool g_micros_counter_open = false;
uint32_t g_micros_last_count = 0U;
uint32_t g_micros_clock_hz = 1U;
uint64_t g_micros_accumulated_counts = 0U;
bool g_adc_open = false;
bool g_uart_open = false;

volatile bool g_i2c_transfer_done = false;
volatile i2c_master_event_t g_i2c_last_event = I2C_MASTER_EVENT_ABORTED;
volatile bool g_uart_tx_done = false;
uint8_t g_uart_tx_buffer[kUartTxBufferSize] = {};
volatile uint32_t g_uart_tx_head = 0U;
volatile uint32_t g_uart_tx_tail = 0U;
volatile uint32_t g_uart_tx_active_length = 0U;
volatile bool g_uart_tx_busy = false;
volatile uint8_t g_uart_rx_buffer[kUartRxBufferSize] = {};
volatile uint32_t g_uart_rx_head = 0U;
volatile uint32_t g_uart_rx_tail = 0U;
volatile uint8_t g_pwm_phase_order = 0U;

constexpr uint8_t kPwmPhaseOrderCount = 6U;
constexpr uint8_t kPwmPhaseOrderMap[kPwmPhaseOrderCount][3] = {
    {0U, 1U, 2U}, // UVW: P108=U, P104=V, P102=W
    {0U, 2U, 1U}, // UWV: P108=U, P104=W, P102=V
    {1U, 0U, 2U}, // VUW: P108=V, P104=U, P102=W
    {1U, 2U, 0U}, // VWU: P108=V, P104=W, P102=U
    {2U, 0U, 1U}, // WUV: P108=W, P104=U, P102=V
    {2U, 1U, 0U}, // WVU: P108=W, P104=V, P102=U
};

bool is_pwm_a_group(int pin_a, int pin_b, int pin_c)
{
    return (pin_a == RA_PWM_A_U) && (pin_b == RA_PWM_A_V) && (pin_c == RA_PWM_A_W);
}

bool is_pwm_b_group(int pin_a, int pin_b, int pin_c)
{
    return (pin_a == RA_PWM_B_U) && (pin_b == RA_PWM_B_V) && (pin_c == RA_PWM_B_W);
}

Renesas3PwmParams *select_pwm_group(int pin_a, int pin_b, int pin_c)
{
    if (is_pwm_a_group(pin_a, pin_b, pin_c))
    {
        return &g_pwm_a;
    }

    if (is_pwm_b_group(pin_a, pin_b, pin_c))
    {
        return &g_pwm_b;
    }

    return nullptr;
}

bool ensure_micros_counter()
{
    if (g_micros_counter_open)
    {
        return true;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    if (0U == (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk))
    {
        return false;
    }

    g_micros_clock_hz = R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_ICLK);
    if (0U == g_micros_clock_hz)
    {
        g_micros_clock_hz = 1U;
    }
    g_micros_last_count = DWT->CYCCNT;
    g_micros_accumulated_counts = 0U;
    g_micros_counter_open = true;
    return true;
}

uint32_t duty_to_counts(float duty, uint32_t period)
{
    if (period < 3U)
    {
        return 1U;
    }

    if (duty <= 0.0f)
    {
        return 1U;
    }

    if (duty >= 1.0f)
    {
        return period - 1U;
    }

    uint32_t counts = static_cast<uint32_t>(duty * static_cast<float>(period));
    if (counts < 1U)
    {
        counts = 1U;
    }
    if (counts >= period)
    {
        counts = period - 1U;
    }
    return counts;
}

bool ensure_three_phase_open(Renesas3PwmParams *params)
{
    if (!params || !params->instance)
    {
        return false;
    }

    if (!params->opened)
    {
        fsp_err_t err = params->instance->p_api->open(params->instance->p_ctrl, params->instance->p_cfg);
        if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err))
        {
            return false;
        }

        timer_info_t info = {};
        err = params->instance->p_cfg->p_timer_instance[0]->p_api->infoGet(
            params->instance->p_cfg->p_timer_instance[0]->p_ctrl,
            &info);
        if (FSP_SUCCESS != err)
        {
            return false;
        }

        params->period_counts = info.period_counts;
        params->opened = true;
    }

    fsp_err_t err = params->instance->p_api->start(params->instance->p_ctrl);
    return (FSP_SUCCESS == err) || (FSP_ERR_ALREADY_OPEN == err);
}

uint32_t adc_mask_for_pin(int pin)
{
    if ((pin < 0) || (pin > 31))
    {
        return 0U;
    }

    return 1UL << static_cast<uint32_t>(pin);
}

bool ensure_adc_open(uint32_t scan_mask)
{
    if (!g_adc_open)
    {
        fsp_err_t err = R_ADC_Open(&g_adc0_ctrl, &g_adc0_cfg);
        if ((FSP_SUCCESS != err) && (FSP_ERR_ALREADY_OPEN != err))
        {
            return false;
        }
        g_adc_open = true;
    }

    adc_channel_cfg_t channel_cfg = g_adc0_channel_cfg;
    channel_cfg.scan_mask = scan_mask;
    channel_cfg.scan_mask_group_b = 0U;
    channel_cfg.add_mask = 0U;

    return FSP_SUCCESS == R_ADC_ScanCfg(&g_adc0_ctrl, &channel_cfg);
}

bool wait_for_adc_idle()
{
    for (uint32_t timeout = 0; timeout < 100000U; timeout++)
    {
        adc_status_t status = {};
        if (FSP_SUCCESS != R_ADC_StatusGet(&g_adc0_ctrl, &status))
        {
            return false;
        }
        if (ADC_STATE_IDLE == status.state)
        {
            return true;
        }
    }

    return false;
}

int adc_inline_index_for_pin(const RenesasAdcParams *params, int pin)
{
    if (!params)
    {
        return -1;
    }

    for (int index = 0; index < 3; index++)
    {
        if (params->pins[index] == pin)
        {
            return index;
        }
    }

    return -1;
}

uint8_t adc_inline_configured_count(const RenesasAdcParams *params)
{
    if (!params)
    {
        return 0U;
    }

    uint8_t count = 0U;
    for (int index = 0; index < 3; index++)
    {
        if (0U != (params->scan_mask & adc_mask_for_pin(params->pins[index])))
        {
            count++;
        }
    }

    return count;
}

bool adc_inline_refresh_cache(RenesasAdcParams *params)
{
    if (!params || (0U == params->scan_mask))
    {
        return false;
    }

    if (FSP_SUCCESS != R_ADC_ScanStart(&g_adc0_ctrl))
    {
        params->cache_valid = false;
        params->cached_reads_left = 0U;
        return false;
    }

    if (!wait_for_adc_idle())
    {
        params->cache_valid = false;
        params->cached_reads_left = 0U;
        return false;
    }

    for (int index = 0; index < 3; index++)
    {
        uint16_t data = 0U;
        if (0U == (params->scan_mask & adc_mask_for_pin(params->pins[index])))
        {
            params->cached_voltage[index] = 0.0f;
            continue;
        }

        if (FSP_SUCCESS != R_ADC_Read(&g_adc0_ctrl, static_cast<adc_channel_t>(params->pins[index]), &data))
        {
            params->cache_valid = false;
            params->cached_reads_left = 0U;
            return false;
        }

        params->cached_voltage[index] = static_cast<float>(data) * params->adc_voltage_conv;
    }

    params->cached_reads_left = adc_inline_configured_count(params);
    params->cache_valid = params->cached_reads_left > 0U;
    return params->cache_valid;
}

bool i2c_wait_complete(uint32_t timeout_us)
{
    const unsigned long start = micros();
    while (!g_i2c_transfer_done)
    {
        if ((micros() - start) > timeout_us)
        {
            return false;
        }
    }

    return (I2C_MASTER_EVENT_TX_COMPLETE == g_i2c_last_event) ||
           (I2C_MASTER_EVENT_RX_COMPLETE == g_i2c_last_event);
}

bool i2c_start_transfer(const i2c_master_instance_t *instance,
                        uint8_t address,
                        uint8_t *buffer,
                        uint32_t bytes,
                        bool read,
                        bool restart)
{
    if (!instance || !buffer || (0U == bytes))
    {
        return false;
    }

    fsp_err_t err = instance->p_api->slaveAddressSet(instance->p_ctrl, address, I2C_MASTER_ADDR_MODE_7BIT);
    if (FSP_SUCCESS != err)
    {
        return false;
    }

    g_i2c_transfer_done = false;
    g_i2c_last_event = I2C_MASTER_EVENT_ABORTED;

    err = read ? instance->p_api->read(instance->p_ctrl, buffer, bytes, restart)
               : instance->p_api->write(instance->p_ctrl, buffer, bytes, restart);
    if (FSP_SUCCESS != err)
    {
        g_i2c_transfer_done = true;
        return false;
    }

    return i2c_wait_complete(kDefaultI2CTimeoutUs);
}

bool is_logical_pwm_pin(int pin)
{
    return (pin >= RA_PWM_A_U) && (pin <= RA_PWM_B_W);
}

void uart_rx_push(uint8_t data)
{
    const uint32_t next_head = (g_uart_rx_head + 1U) % kUartRxBufferSize;
    if (next_head != g_uart_rx_tail)
    {
        g_uart_rx_buffer[g_uart_rx_head] = data;
        g_uart_rx_head = next_head;
    }
}

uint32_t uart_tx_used_bytes(uint32_t head, uint32_t tail)
{
    return (head >= tail) ? (head - tail) : (kUartTxBufferSize - tail + head);
}

/* 此函数调用时必须已关闭中断，避免主循环和 UART 回调同时启动发送。 */
void uart_tx_start_next_critical()
{
    if (!g_uart_open || g_uart_tx_busy)
    {
        return;
    }

    const uint32_t tail = g_uart_tx_tail;
    const uint32_t head = g_uart_tx_head;
    if (tail == head)
    {
        return;
    }

    const uint32_t contiguous_length = (head > tail) ? (head - tail) : (kUartTxBufferSize - tail);
    g_uart_tx_busy = true;
    g_uart_tx_active_length = contiguous_length;

    const fsp_err_t uart_err = R_SCI_UART_Write(&g_uart9_ctrl, &g_uart_tx_buffer[tail], contiguous_length);
    if (FSP_SUCCESS != uart_err)
    {
        g_uart_tx_busy = false;
        g_uart_tx_active_length = 0U;
        g_ra4m2_uart_tx_start_errors++;
    }
}

void uart_tx_kick()
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uart_tx_start_next_critical();
    if (0U == primask)
    {
        __enable_irq();
    }
}

bool uart_tx_enqueue(const uint8_t *data, size_t length)
{
    if (!data || (0U == length))
    {
        return false;
    }

    if (length >= kUartTxBufferSize)
    {
        g_ra4m2_uart_tx_dropped_bytes += static_cast<uint32_t>(length);
        return false;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const uint32_t head = g_uart_tx_head;
    const uint32_t used = uart_tx_used_bytes(head, g_uart_tx_tail);
    const uint32_t free_bytes = (kUartTxBufferSize - 1U) - used;
    if (length > free_bytes)
    {
        g_ra4m2_uart_tx_dropped_bytes += static_cast<uint32_t>(length);
        if (0U == primask)
        {
            __enable_irq();
        }
        return false;
    }

    const uint32_t first_length = static_cast<uint32_t>(length) < (kUartTxBufferSize - head) ?
                                      static_cast<uint32_t>(length) : (kUartTxBufferSize - head);
    memcpy(&g_uart_tx_buffer[head], data, first_length);
    const uint32_t second_length = static_cast<uint32_t>(length) - first_length;
    if (second_length > 0U)
    {
        memcpy(&g_uart_tx_buffer[0], data + first_length, second_length);
    }

    __DMB();
    g_uart_tx_head = (head + static_cast<uint32_t>(length)) % kUartTxBufferSize;
    if (0U == primask)
    {
        __enable_irq();
    }

    uart_tx_kick();
    return true;
}
} // namespace

extern "C" bool renesas_simplefoc_set_phase_order(uint8_t order)
{
    if (order >= kPwmPhaseOrderCount)
    {
        return false;
    }

    g_pwm_phase_order = order;
    return true;
}

extern "C" uint8_t renesas_simplefoc_get_phase_order(void)
{
    return g_pwm_phase_order;
}

extern "C" void sci_i2c_master_callback(i2c_master_callback_args_t *p_args)
{
    if (p_args)
    {
        if ((I2C_MASTER_EVENT_TX_COMPLETE == p_args->event) ||
            (I2C_MASTER_EVENT_RX_COMPLETE == p_args->event) ||
            (I2C_MASTER_EVENT_ABORTED == p_args->event))
        {
            g_i2c_last_event = p_args->event;
            g_i2c_transfer_done = true;
        }
    }
}

extern "C" {
fsp_err_t err = FSP_SUCCESS;
volatile i2c_master_event_t i2c_event = I2C_MASTER_EVENT_ABORTED;
volatile uint32_t timeout_ms = 1000000U;
}

extern "C" void i2c_master_callback(i2c_master_callback_args_t *p_args)
{
    i2c_event = I2C_MASTER_EVENT_ABORTED;
    if (p_args)
    {
        i2c_event = p_args->event;
    }
}

extern "C" void renesas_uart_callback(uart_callback_args_t *p_args);

extern "C" void renesas_uart_callback(uart_callback_args_t *p_args)
{
    if (!p_args)
    {
        return;
    }

    if (UART_EVENT_TX_COMPLETE == p_args->event)
    {
        /*
         * DTC normal-mode TX finishes the memory-to-TDR transfer at
         * TX_DATA_EMPTY.  Waiting only for TX_COMPLETE can leave the ring
         * queue stalled after its first DTC block.  The source buffer is safe
         * to release at DATA_EMPTY because DTC has already consumed it.
         */
        if (g_uart_tx_busy)
        {
            g_uart_tx_tail = (g_uart_tx_tail + g_uart_tx_active_length) % kUartTxBufferSize;
            g_uart_tx_active_length = 0U;
            g_uart_tx_busy = false;
            uart_tx_start_next_critical();
        }
    }
    else if (UART_EVENT_TX_DATA_EMPTY == p_args->event)
    {
        // TX_COMPLETE 对应最后一个停止位离开串口线，不能再次推进环形队列。
        g_uart_tx_done = true;
    }
    else if (UART_EVENT_RX_CHAR == p_args->event)
    {
        uart_rx_push(static_cast<uint8_t>(p_args->data));
    }
}

extern "C" void can_callback(can_callback_args_t *p_args)
{
    (void) p_args;
}

extern "C" void _exit(int status);
extern "C" int _kill(int pid, int sig);
extern "C" int _getpid(void);
extern "C" int _close(int fd);
extern "C" int _lseek(int fd, int ptr, int dir);
extern "C" int _read(int fd, char *pBuffer, int size);

extern "C" void _exit(int status)
{
    (void) status;
    while (true)
    {
    }
}

extern "C" int _kill(int pid, int sig)
{
    (void) pid;
    (void) sig;
    return -1;
}

extern "C" int _getpid(void)
{
    return 1;
}

extern "C" int _close(int fd)
{
    (void) fd;
    return -1;
}

extern "C" int _lseek(int fd, int ptr, int dir)
{
    (void) fd;
    (void) ptr;
    (void) dir;
    return 0;
}

extern "C" int _read(int fd, char *pBuffer, int size)
{
    (void) fd;
    (void) pBuffer;
    (void) size;
    return -1;
}

extern "C" void renesas_serial_begin(uint32_t baud) __attribute__((weak));
extern "C" int renesas_serial_available(void) __attribute__((weak));
extern "C" int renesas_serial_read(void) __attribute__((weak));
extern "C" size_t renesas_serial_write(const uint8_t *data, size_t len) __attribute__((weak));
extern "C" int __io_putchar(int ch);
extern "C" int _write(int fd, char *pBuffer, int size);

/* 保留旧阻塞等待实现，便于与 DTC 环形发送方案对比。 */
[[maybe_unused]] static bool uart_wait_tx_complete(uint32_t timeout_count)
{
    while (!g_uart_tx_done && (timeout_count > 0U))
    {
        timeout_count--;
        R_BSP_SoftwareDelay(10U, BSP_DELAY_UNITS_MICROSECONDS);
    }

    return g_uart_tx_done;
}

extern "C" void renesas_serial_begin(uint32_t baud)
{
    (void) baud;

    if (g_uart_open)
    {
        return;
    }

    fsp_err_t uart_err = R_SCI_UART_Open(&g_uart9_ctrl, &g_uart9_cfg);
    if ((FSP_SUCCESS == uart_err) || (FSP_ERR_ALREADY_OPEN == uart_err))
    {
        g_uart_open = true;
    }
}

extern "C" int renesas_serial_available(void)
{
#if !RA4M2_ENABLE_UART_DASHBOARD
    return 0;
#else
    const uint32_t head = g_uart_rx_head;
    const uint32_t tail = g_uart_rx_tail;

    if (head >= tail)
    {
        return static_cast<int>(head - tail);
    }

    return static_cast<int>((kUartRxBufferSize - tail) + head);
#endif
}

extern "C" int renesas_serial_read(void)
{
#if !RA4M2_ENABLE_UART_DASHBOARD
    return -1;
#else
    if (g_uart_rx_head == g_uart_rx_tail)
    {
        return -1;
    }

    const uint8_t data = g_uart_rx_buffer[g_uart_rx_tail];
    g_uart_rx_tail = (g_uart_rx_tail + 1U) % kUartRxBufferSize;
    return static_cast<int>(data);
#endif
}

extern "C" uint32_t renesas_serial_tx_dropped_bytes(void)
{
    return g_ra4m2_uart_tx_dropped_bytes;
}

extern "C" uint32_t renesas_serial_tx_start_errors(void)
{
    return g_ra4m2_uart_tx_start_errors;
}

extern "C" size_t renesas_serial_write(const uint8_t *data, size_t len)
{
#if !RA4M2_ENABLE_UART_DASHBOARD
    (void) data;
    return len;
#else
    if (!data || (0U == len))
    {
        return 0U;
    }

    if (!g_uart_open)
    {
        renesas_serial_begin(115200U);
    }

    if (!g_uart_open)
    {
        return 0U;
    }

    return uart_tx_enqueue(data, len) ? len : 0U;
#endif
}

extern "C" int __io_putchar(int ch)
{
    uint8_t tx_byte = static_cast<uint8_t>(ch);
    return (1U == renesas_serial_write(&tx_byte, 1U)) ? ch : -1;
}

extern "C" int _write(int fd, char *pBuffer, int size)
{
    (void) fd;

    if (!pBuffer || (size <= 0))
    {
        return 0;
    }

    size_t written = renesas_serial_write(reinterpret_cast<const uint8_t *>(pBuffer), static_cast<size_t>(size));
    return static_cast<int>(written);
}

HardwareSerial Serial;
TwoWire Wire(&g_i2c_as5600);
SPIClass SPI;

void HardwareSerial::begin(unsigned long baud)
{
    renesas_serial_begin(static_cast<uint32_t>(baud));
}

int HardwareSerial::available()
{
    return renesas_serial_available();
}

int HardwareSerial::read()
{
    return renesas_serial_read();
}

size_t HardwareSerial::write(uint8_t c)
{
    return renesas_serial_write(&c, 1U);
}

size_t HardwareSerial::write(const uint8_t *buffer, size_t size)
{
    return renesas_serial_write(buffer, size);
}

TwoWire::TwoWire(int bus)
    : instance_((0 == bus) ? &g_i2c_master0 : &g_i2c_as5600),
      address_(0),
      tx_length_(0),
      rx_length_(0),
      rx_index_(0),
      begun_(false)
{
}

TwoWire::TwoWire(const i2c_master_instance_t *instance)
    : instance_(instance),
      address_(0),
      tx_length_(0),
      rx_length_(0),
      rx_index_(0),
      begun_(false)
{
}

void TwoWire::begin()
{
    if (begun_ || !instance_)
    {
        return;
    }

    fsp_err_t i2c_err = instance_->p_api->open(instance_->p_ctrl, instance_->p_cfg);
    if ((FSP_SUCCESS == i2c_err) || (FSP_ERR_ALREADY_OPEN == i2c_err))
    {
        begun_ = true;
    }
}

void TwoWire::begin(int sda, int scl, uint32_t frequency)
{
    (void) sda;
    (void) scl;
    (void) frequency;
    begin();
}

void TwoWire::setClock(uint32_t frequency)
{
    (void) frequency;
}

void TwoWire::beginTransmission(uint8_t address)
{
    begin();
    address_ = address;
    tx_length_ = 0U;
}

size_t TwoWire::write(uint8_t data)
{
    if (tx_length_ >= sizeof(tx_buffer_))
    {
        return 0U;
    }

    tx_buffer_[tx_length_++] = data;
    return 1U;
}

size_t TwoWire::write(const uint8_t *data, size_t quantity)
{
    size_t written = 0U;
    while (written < quantity)
    {
        written += write(data[written]);
        if (written < quantity && tx_length_ >= sizeof(tx_buffer_))
        {
            break;
        }
    }
    return written;
}

uint8_t TwoWire::endTransmission(bool sendStop)
{
    if (!begun_ || !instance_)
    {
        return 4U;
    }

    if (0U == tx_length_)
    {
        return 0U;
    }

    const bool ok = i2c_start_transfer(instance_, address_, tx_buffer_, tx_length_, false, !sendStop);
    tx_length_ = 0U;
    return ok ? 0U : 4U;
}

uint8_t TwoWire::requestFrom(uint8_t address, uint8_t quantity, bool sendStop)
{
    begin();
    if (!begun_ || !instance_)
    {
        return 0U;
    }

    if (quantity > sizeof(rx_buffer_))
    {
        quantity = sizeof(rx_buffer_);
    }

    rx_length_ = 0U;
    rx_index_ = 0U;
    if (0U == quantity)
    {
        return 0U;
    }

    const bool ok = i2c_start_transfer(instance_, address, rx_buffer_, quantity, true, !sendStop);
    if (!ok)
    {
        return 0U;
    }

    rx_length_ = quantity;
    return quantity;
}

int TwoWire::available()
{
    return static_cast<int>(rx_length_ - rx_index_);
}

int TwoWire::read()
{
    if (rx_index_ >= rx_length_)
    {
        return -1;
    }

    return rx_buffer_[rx_index_++];
}

void pinMode(int pin, int mode)
{
    if (is_logical_pwm_pin(pin))
    {
        return;
    }

    uint32_t cfg = 0U;
    switch (mode)
    {
        case OUTPUT:
            cfg = IOPORT_CFG_PORT_DIRECTION_OUTPUT;
            break;
        case INPUT_PULLUP:
            cfg = IOPORT_CFG_PORT_DIRECTION_INPUT | IOPORT_CFG_PULLUP_ENABLE;
            break;
        case INPUT:
        case ANALOG:
        default:
            cfg = IOPORT_CFG_PORT_DIRECTION_INPUT;
            break;
    }

    (void) R_IOPORT_PinCfg(&g_ioport_ctrl, static_cast<bsp_io_port_pin_t>(pin), cfg);
}

void digitalWrite(int pin, int value)
{
    if (is_logical_pwm_pin(pin))
    {
        return;
    }

    (void) R_IOPORT_PinWrite(&g_ioport_ctrl,
                             static_cast<bsp_io_port_pin_t>(pin),
                             value ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW);
}

int digitalRead(int pin)
{
    if (is_logical_pwm_pin(pin))
    {
        return LOW;
    }

    bsp_io_level_t level = BSP_IO_LEVEL_LOW;
    (void) R_IOPORT_PinRead(&g_ioport_ctrl, static_cast<bsp_io_port_pin_t>(pin), &level);
    return (BSP_IO_LEVEL_HIGH == level) ? HIGH : LOW;
}

uint32_t analogRead(int pin)
{
    const uint32_t mask = adc_mask_for_pin(pin);
    if (0U == mask)
    {
        return 0U;
    }

    if (!ensure_adc_open(mask))
    {
        return 0U;
    }

    if (FSP_SUCCESS != R_ADC_ScanStart(&g_adc0_ctrl))
    {
        return 0U;
    }

    if (!wait_for_adc_idle())
    {
        return 0U;
    }

    uint16_t data = 0U;
    if (FSP_SUCCESS != R_ADC_Read(&g_adc0_ctrl, static_cast<adc_channel_t>(pin), &data))
    {
        return 0U;
    }

    return data;
}

void analogWrite(int pin, int value)
{
    (void) pin;
    (void) value;
}

int digitalPinToInterrupt(int pin)
{
    return pin;
}

void attachInterrupt(int interruptNum, void (*userFunc)(void), int mode)
{
    (void) interruptNum;
    (void) userFunc;
    (void) mode;
}

void noInterrupts(void)
{
    __disable_irq();
}

void interrupts(void)
{
    __enable_irq();
}

unsigned long micros(void)
{
    if (!ensure_micros_counter())
    {
        return 0UL;
    }

    const uint32_t now = DWT->CYCCNT;
    const uint32_t delta = now - g_micros_last_count;

    g_micros_accumulated_counts += delta;
    g_micros_last_count = now;
    return static_cast<unsigned long>((g_micros_accumulated_counts * 1000000ULL) / g_micros_clock_hz);
}

unsigned long millis(void)
{
    return micros() / 1000UL;
}

void delay(unsigned long ms)
{
    const unsigned long start = millis();
    while ((millis() - start) < ms)
    {
    }
}

void delayMicroseconds(unsigned int us)
{
    const unsigned long start = micros();
    while ((micros() - start) < us)
    {
    }
}

unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout)
{
    const int target = state ? HIGH : LOW;
    const int idle = target ? LOW : HIGH;
    const unsigned long start = micros();

    while (digitalRead(pin) == target)
    {
        if ((micros() - start) >= timeout)
        {
            return 0UL;
        }
    }

    while (digitalRead(pin) == idle)
    {
        if ((micros() - start) >= timeout)
        {
            return 0UL;
        }
    }

    const unsigned long pulse_start = micros();
    while (digitalRead(pin) == target)
    {
        if ((micros() - start) >= timeout)
        {
            return 0UL;
        }
    }

    return micros() - pulse_start;
}

void *_configure1PWM(long pwm_frequency, const int pinA)
{
    (void) pwm_frequency;
    (void) pinA;
    return SIMPLEFOC_DRIVER_INIT_FAILED;
}

void *_configure2PWM(long pwm_frequency, const int pinA, const int pinB)
{
    (void) pwm_frequency;
    (void) pinA;
    (void) pinB;
    return SIMPLEFOC_DRIVER_INIT_FAILED;
}

void *_configure3PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC)
{
    (void) pwm_frequency;
    Renesas3PwmParams *params = select_pwm_group(pinA, pinB, pinC);
    if (!ensure_three_phase_open(params))
    {
        return SIMPLEFOC_DRIVER_INIT_FAILED;
    }

    return params;
}

void *_configure4PWM(long pwm_frequency, const int pin1A, const int pin1B, const int pin2A, const int pin2B)
{
    (void) pwm_frequency;
    (void) pin1A;
    (void) pin1B;
    (void) pin2A;
    (void) pin2B;
    return SIMPLEFOC_DRIVER_INIT_FAILED;
}

void *_configure6PWM(long pwm_frequency,
                     float dead_zone,
                     const int pinA_h,
                     const int pinA_l,
                     const int pinB_h,
                     const int pinB_l,
                     const int pinC_h,
                     const int pinC_l)
{
    (void) pwm_frequency;
    (void) dead_zone;
    (void) pinA_h;
    (void) pinA_l;
    (void) pinB_h;
    (void) pinB_l;
    (void) pinC_h;
    (void) pinC_l;
    return SIMPLEFOC_DRIVER_INIT_FAILED;
}

void _writeDutyCycle1PWM(float dc_a, void *params)
{
    (void) dc_a;
    (void) params;
}

void _writeDutyCycle2PWM(float dc_a, float dc_b, void *params)
{
    (void) dc_a;
    (void) dc_b;
    (void) params;
}

void _writeDutyCycle3PWM(float dc_a, float dc_b, float dc_c, void *params)
{
    Renesas3PwmParams *pwm = static_cast<Renesas3PwmParams *>(params);
    if (!pwm || !pwm->opened)
    {
        return;
    }

    const float source[3] = {dc_a, dc_b, dc_c};
    const uint8_t order = (g_pwm_phase_order < kPwmPhaseOrderCount) ? g_pwm_phase_order : 0U;
    const uint8_t *map = kPwmPhaseOrderMap[order];

    three_phase_duty_cycle_t duty = {};
    duty.duty[THREE_PHASE_CHANNEL_U] = duty_to_counts(source[map[0]], pwm->period_counts);
    duty.duty[THREE_PHASE_CHANNEL_V] = duty_to_counts(source[map[1]], pwm->period_counts);
    duty.duty[THREE_PHASE_CHANNEL_W] = duty_to_counts(source[map[2]], pwm->period_counts);
    duty.duty_buffer[THREE_PHASE_CHANNEL_U] = duty.duty[THREE_PHASE_CHANNEL_U];
    duty.duty_buffer[THREE_PHASE_CHANNEL_V] = duty.duty[THREE_PHASE_CHANNEL_V];
    duty.duty_buffer[THREE_PHASE_CHANNEL_W] = duty.duty[THREE_PHASE_CHANNEL_W];

    (void) pwm->instance->p_api->dutyCycleSet(pwm->instance->p_ctrl, &duty);
}

void _writeDutyCycle4PWM(float dc_1a, float dc_1b, float dc_2a, float dc_2b, void *params)
{
    (void) dc_1a;
    (void) dc_1b;
    (void) dc_2a;
    (void) dc_2b;
    (void) params;
}

void _writeDutyCycle6PWM(float dc_a, float dc_b, float dc_c, PhaseState *phase_state, void *params)
{
    (void) dc_a;
    (void) dc_b;
    (void) dc_c;
    (void) phase_state;
    (void) params;
}

void *_configureADCInline(const void *driver_params, const int pinA, const int pinB, const int pinC)
{
    (void) driver_params;

    uint32_t scan_mask = 0U;
    scan_mask |= adc_mask_for_pin(pinA);
    scan_mask |= adc_mask_for_pin(pinB);
    scan_mask |= adc_mask_for_pin(pinC);

    if ((0U == scan_mask) || !ensure_adc_open(scan_mask))
    {
        return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
    }

    g_adc_inline.pins[0] = pinA;
    g_adc_inline.pins[1] = pinB;
    g_adc_inline.pins[2] = pinC;
    g_adc_inline.scan_mask = scan_mask;
    g_adc_inline.adc_voltage_conv = kAdcReferenceVoltage / kAdcMaxCount;
    g_adc_inline.cached_voltage[0] = 0.0f;
    g_adc_inline.cached_voltage[1] = 0.0f;
    g_adc_inline.cached_voltage[2] = 0.0f;
    g_adc_inline.cached_reads_left = 0U;
    g_adc_inline.cache_valid = false;
    return &g_adc_inline;
}

float _readADCVoltageInline(const int pin, const void *cs_params)
{
    RenesasAdcParams *params = const_cast<RenesasAdcParams *>(static_cast<const RenesasAdcParams *>(cs_params));
    if (!params || (0U == (params->scan_mask & adc_mask_for_pin(pin))))
    {
        return 0.0f;
    }

    if (!params->cache_valid || (0U == params->cached_reads_left))
    {
        if (!adc_inline_refresh_cache(params))
        {
            return 0.0f;
        }
    }

    const int index = adc_inline_index_for_pin(params, pin);
    if (index < 0)
    {
        return 0.0f;
    }

    const float voltage = params->cached_voltage[index];
    if (params->cached_reads_left > 0U)
    {
        params->cached_reads_left--;
    }

    return voltage;
}

void *_configureADCLowSide(const void *driver_params, const int pinA, const int pinB, const int pinC)
{
    (void) driver_params;
    (void) pinA;
    (void) pinB;
    (void) pinC;
    return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
}

void _startADC3PinConversionLowSide()
{
}

float _readADCVoltageLowSide(const int pinA, const void *cs_params)
{
    (void) pinA;
    (void) cs_params;
    return 0.0f;
}

void *_driverSyncLowSide(void *driver_params, void *cs_params)
{
    (void) driver_params;
    (void) cs_params;
    return SIMPLEFOC_CURRENT_SENSE_INIT_FAILED;
}
