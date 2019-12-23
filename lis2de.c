#include "lib/lis2de-driver/include/lis2de.h"
#include "lib/i2cmaster/include/i2cmaster.h"
#include "CException.h"

/* I2C device slave adddress of LIS2DE: */
static const uint8_t LIS2DE_ADDR = 0x50U;

typedef struct reg
{
    const uint8_t adr;
    const uint8_t size;
} reg_t;

static const reg_t STATUS_AUX_REG   = {0x07, 1};
static const reg_t OUT_TEMP_REG     = {0x0C, 2};
static const reg_t INT_COUNTER_REG  = {0x0E, 1};
static const reg_t WHO_AM_I_REG     = {0x0F, 1};
static const reg_t TEMP_CFG_REG     = {0x1F, 1};

static const reg_t CTRL_REG1        = {0x20, 1};
static const reg_t CTRL_REG2        = {0x21, 1};
static const reg_t CTRL_REG3        = {0x22, 1};
static const reg_t CTRL_REG4        = {0x23, 1};
static const reg_t CTRL_REG5        = {0x24, 1};
static const reg_t CTRL_REG6        = {0x25, 1};

static const reg_t REFERENCE_REG    = {0x26, 1};
static const reg_t STATUS_REG2      = {0x27, 1};

static const reg_t OUT_REG_X        = {0x29, 1};
static const reg_t OUT_REG_Y        = {0x2B, 1};
static const reg_t OUT_REG_Z        = {0x2D, 1};

static const reg_t FIFO_CTRL_REG    = {0x2E, 1};
static const reg_t FIFO_SRC_REG     = {0x2F, 1};

static const reg_t IG1_CFG_REG      = {0x30, 1};
static const reg_t IG1_SOURCE_REG   = {0x31, 1};
static const reg_t IG1_THS_REG      = {0x32, 1};
static const reg_t IG1_DURATION_REG = {0x33, 1};

static const reg_t IG2_CFG_REG      = {0x34, 1};
static const reg_t IG2_SOURCE_REG   = {0x35, 1};
static const reg_t IG2_THS_REG      = {0x36, 1};
static const reg_t IG2_DURATION_REG = {0x37, 1};

static const reg_t CLICK_CFG_REG    = {0x38, 1};
static const reg_t CLICK_SRC_REG    = {0x39, 1};
static const reg_t CLICK_THS_REG    = {0x3A, 1};

static const reg_t TIME_LIMIT_REG   = {0x3B, 1};
static const reg_t TIME_LATENCY_REG = {0x3C, 1};
static const reg_t TIME_WINDOW_REG  = {0x3D, 1};

static const reg_t ACT_THS_REG      = {0x3E, 1};
static const reg_t ACT_DUR_REG      = {0x3F, 1};

typedef struct bitmask
{
    const uint8_t mask;
    const uint8_t shift;
} bitmask_t;

static const bitmask_t BITMASK_0       = {0b00000001, 0};
static const bitmask_t BITMASK_1       = {0b00000010, 1};
static const bitmask_t BITMASK_2       = {0b00000100, 2};
static const bitmask_t BITMASK_3       = {0b00001000, 3};
static const bitmask_t BITMASK_4       = {0b00010000, 4};
static const bitmask_t BITMASK_5       = {0b00100000, 5};
static const bitmask_t BITMASK_6       = {0b01000000, 6};
static const bitmask_t BITMASK_7       = {0b10000000, 7};

static const bitmask_t BITMASK_765     = {0b11100000, 5};
static const bitmask_t BITMASK_7654    = {0b11110000, 4};
static const bitmask_t BITMASK_6543210 = {0b01111111, 0};
static const bitmask_t BITMASK_76      = {0b11000000, 6};
static const bitmask_t BITMASK_54      = {0b00110000, 4};
static const bitmask_t BITMASK_43210   = {0b00011111, 0};
static const bitmask_t BITMASK_21      = {0b00000110, 1};
static const bitmask_t BITMASK_FULL    = {0b11111111, 0};

// Operating modes
static const uint8_t OP_MODE_NORMAL    = 0;
static const uint8_t OP_MODE_LOW_POWER = 1;

// High pass filter modes
static const uint8_t HPF_MODE_NORMAL     = 0b00;
static const uint8_t HPF_MODE_REFERENCE  = 0b01;
static const uint8_t HPF_MODE_AUTO_RESET = 0b11;

static void
lis2de_read_bytes(uint8_t bytes_to_read,
                  const uint8_t reg,
                  uint8_t *res)
{
    if (bytes_to_read > 0)
    {
        i2c_start_wait(LIS2DE_ADDR + I2C_WRITE);

        // In order to read multiple bytes, MSB of reg must be 1
        uint8_t reg_multi_bytes_read = (reg | (1 << 7));
        if (i2c_write(reg_multi_bytes_read))
        {
           Throw(E_LIS2DE_I2C_WRITE);
        }

        if (i2c_rep_start(LIS2DE_ADDR + I2C_READ))
        {
            Throw(E_LIS2DE_I2C_REP_START);
        }
        --bytes_to_read;
        for (uint8_t pos = 0; pos < bytes_to_read; pos++)
        {
            res[pos] = i2c_readAck();
        }
        res[bytes_to_read] = i2c_readNak();
        i2c_stop();
    }
}

static uint8_t
lis2de_read_byte(const uint8_t reg)
{
    i2c_start_wait(LIS2DE_ADDR + I2C_WRITE);
    if (i2c_write(reg))
    {
        Throw(E_LIS2DE_I2C_WRITE);
    }
    if (i2c_rep_start(LIS2DE_ADDR + I2C_READ))
    {
        Throw(E_LIS2DE_I2C_REP_START);
    }
    uint8_t res = i2c_readNak();
    i2c_stop();
    return res;
}

static void
lis2de_write_byte(const uint8_t reg,
                const uint8_t val)
{
    i2c_start_wait(LIS2DE_ADDR + I2C_WRITE);
    if (i2c_write(reg) || i2c_write(val))
    {
        Throw(E_LIS2DE_I2C_WRITE);
    }
    i2c_stop();
}

void
lis2de_init(void)
{
    i2c_init();
}

static uint8_t
lis2de_query(const reg_t reg,
             const bitmask_t bm)
{
    uint8_t data = lis2de_read_byte(reg.adr);
    data = ((data & bm.mask) >> bm.shift);
    return data;
}

static void
lis2de_set(const reg_t reg,
           const bitmask_t bm,
           uint8_t val)
{
    uint8_t data = lis2de_read_byte(reg.adr);
    data = data & ((uint8_t) ~bm.mask);
    val = (val << bm.shift) + data;

    lis2de_write_byte(reg.adr, val);
}

uint8_t
lis2de_query_temperature_sensor_enabled(void)
{
    return lis2de_query(TEMP_CFG_REG, BITMASK_7);
}

uint8_t
lis2de_query_temperature_data_overrun(void)
{
    return lis2de_query(STATUS_AUX_REG, BITMASK_6);
}

uint8_t
lis2de_query_temperature_new_data_available(void)
{
    return lis2de_query(STATUS_AUX_REG, BITMASK_2);
}

static void
lis2de_ensure_block_data_update_is_enabled(void)
{
    if (!lis2de_query_block_data_update_enabled())
    {
        Throw(E_BDU_NOT_ENABLED);
    }
}

/* Both high and low byte must be read, but the actual temperature
 * data is the high byte as two's complement */
int8_t
lis2de_query_temperature(void)
{
    uint8_t data[2] = {0};

    lis2de_ensure_block_data_update_is_enabled();
    lis2de_read_bytes(OUT_TEMP_REG.size, OUT_TEMP_REG.adr, data);

    return ((int8_t) data[0]);
}

uint8_t
lis2de_query_int_counter(void)
{
    return lis2de_query(INT_COUNTER_REG, BITMASK_FULL);
}

uint8_t
lis2de_query_device_id(void)
{
    return lis2de_query(WHO_AM_I_REG, BITMASK_FULL);
}

// CTRL_REG1 (0x20):

uint8_t
lis2de_query_data_rate_selection(void)
{
    return lis2de_query(CTRL_REG1, BITMASK_765);
}

uint8_t
lis2de_query_low_power_mode_enabled(void)
{
    return lis2de_query(CTRL_REG1, BITMASK_3);
}

uint8_t
lis2de_query_z_axis_enabled(void)
{
    return lis2de_query(CTRL_REG1, BITMASK_2);
}

uint8_t
lis2de_query_y_axis_enabled(void)
{
    return lis2de_query(CTRL_REG1, BITMASK_1);
}

uint8_t
lis2de_query_x_axis_enabled(void)
{
    return lis2de_query(CTRL_REG1, BITMASK_0);
}

// CTRL_REG2 (0x21):

uint8_t
lis2de_query_high_pass_filter_mode_selection(void)
{
    return lis2de_query(CTRL_REG2, BITMASK_76);
}

uint8_t
lis2de_query_high_pass_filter_in_normal_mode(void)
{
    uint8_t mode = lis2de_query_high_pass_filter_mode_selection();
    uint8_t res = 0;

    if (mode == HPF_MODE_NORMAL)
    {
        res = 1;
    }
    return res;
}

uint8_t
lis2de_query_high_pass_filter_in_reference_mode(void)
{
    uint8_t mode = lis2de_query_high_pass_filter_mode_selection();
    uint8_t res = 0;
    if (mode == HPF_MODE_REFERENCE)
    {
        res = 1;
    }
    return res;
}

uint8_t
lis2de_query_high_pass_filter_in_auto_reset_mode(void)
{
    uint8_t mode = lis2de_query_high_pass_filter_mode_selection();
    uint8_t res = 0;
    if (mode == HPF_MODE_AUTO_RESET)
    {
        res = 1;
    }
    return res;
}

uint8_t
lis2de_query_high_pass_filter_cutoff_frequency_selection(void)
{
    return lis2de_query(CTRL_REG2, BITMASK_54);
}

uint8_t
lis2de_query_internal_filter_enabled(void)
{
    return lis2de_query(CTRL_REG2, BITMASK_3);
}

uint8_t
lis2de_query_high_pass_filter_for_click_function_enabled(void)
{
    return lis2de_query(CTRL_REG2, BITMASK_2);
}

uint8_t
lis2de_query_high_pass_filter_for_ig2_enabled(void)
{
    return lis2de_query(CTRL_REG2, BITMASK_1);
}

uint8_t
lis2de_query_high_pass_filter_for_ig1_enabled(void)
{
    return lis2de_query(CTRL_REG2, BITMASK_0);
}

// CTRL_REG3 (0x22):

uint8_t
lis2de_query_click_interrupt_on_int1_enabled(void)
{
    return lis2de_query(CTRL_REG3, BITMASK_7);
}

uint8_t
lis2de_query_ig1_on_int1_enabled(void)
{
    return lis2de_query(CTRL_REG3, BITMASK_6);
}

uint8_t
lis2de_query_ig2_on_int1_enabled(void)
{
    return lis2de_query(CTRL_REG3, BITMASK_5);
}

uint8_t
lis2de_query_drdy1_interrupt_on_int1_enabled(void)
{
    return lis2de_query(CTRL_REG3, BITMASK_4);
}

uint8_t
lis2de_query_drdy2_interrupt_on_int1_enabled(void)
{
    return lis2de_query(CTRL_REG3, BITMASK_3);
}

uint8_t
lis2de_query_fifo_watermark_interrupt_on_int1_enabled(void)
{
    return lis2de_query(CTRL_REG3, BITMASK_2);
}

uint8_t
lis2de_query_fifo_overrun_interrupt_on_int1_enabled(void)
{
    return lis2de_query(CTRL_REG3, BITMASK_1);
}

// CTRL_REG4 (0x23):

uint8_t
lis2de_query_block_data_update_enabled(void)
{
    return lis2de_query(CTRL_REG4, BITMASK_7);
}

uint8_t
lis2de_query_FULL_scale_selection(void)
{
    return lis2de_query(CTRL_REG4, BITMASK_54);
}

uint8_t
lis2de_query_FULL_scale_selection_is_set_to_2g(void)
{
    return (lis2de_query(CTRL_REG4, BITMASK_54) == 0b00);
}

uint8_t
lis2de_query_FULL_scale_selection_is_set_to_4g(void)
{
    return (lis2de_query(CTRL_REG4, BITMASK_54) == 0b01);
}

uint8_t
lis2de_query_FULL_scale_selection_is_set_to_8g(void)
{
    return (lis2de_query(CTRL_REG4, BITMASK_54) == 0b10);
}

uint8_t
lis2de_query_FULL_scale_selection_is_set_to_16g(void)
{
    return (lis2de_query(CTRL_REG4, BITMASK_54) == 0b11);
}

uint8_t
lis2de_query_self_test_enabled(void)
{
    return (lis2de_query(CTRL_REG4, BITMASK_21) == 0);
}

uint8_t
lis2de_query_spi_mode_selection(void)
{
    return lis2de_query(CTRL_REG4, BITMASK_0);
}

// CTRL_REG5 (0x24):

uint8_t
lis2de_query_reboot_memory_content(void)
{
    return lis2de_query(CTRL_REG5, BITMASK_7);
}

uint8_t
lis2de_query_fifo_enabled(void)
{
    return lis2de_query(CTRL_REG5, BITMASK_6);
}

uint8_t
lis2de_query_latch_interruot_request_on_ig1_source_reg(void)
{
    return lis2de_query(CTRL_REG5, BITMASK_3);
}

uint8_t
lis2de_query_int1_4d_detection_enabled(void)
{
    return lis2de_query(CTRL_REG5, BITMASK_2);
}

uint8_t
lis2de_query_latch_interruot_request_on_ig2_source_reg(void)
{
    return lis2de_query(CTRL_REG5, BITMASK_1);
}

uint8_t
lis2de_query_int2_4d_detection_enabled(void)
{
    return lis2de_query(CTRL_REG5, BITMASK_0);
}

// CTRL_REG6 (0x25):

uint8_t
lis2de_query_click_interrupt_on_int2_pin_enabled(void)
{
    return lis2de_query(CTRL_REG6, BITMASK_7);
}

uint8_t
lis2de_query_ig1_on_int2_pin_enabled(void)
{
    return lis2de_query(CTRL_REG6, BITMASK_6);
}

uint8_t
lis2de_query_ig2_on_int2_pin_enabled(void)
{
    return lis2de_query(CTRL_REG6, BITMASK_5);
}

uint8_t
lis2de_query_boot_on_int2_pin_enabled(void)
{
    return lis2de_query(CTRL_REG6, BITMASK_4);
}

uint8_t
lis2de_query_sleep_to_wake_function_interrupt_on_int2_pin_enabled(void)
{
    return lis2de_query(CTRL_REG6, BITMASK_3);
}

uint8_t
lis2de_query_interrupt_active_value(void)
{
    return lis2de_query(CTRL_REG6, BITMASK_1);
}


// REFERENCE (0x26)
uint8_t
lis2de_query_reference(void)
{
    return lis2de_query(REFERENCE_REG, BITMASK_FULL);
}


// STATUS_REG22 (0x27)
uint8_t
lis2de_query_data_overrun_on_xyz_axes(void)
{
    return lis2de_query(STATUS_REG2, BITMASK_7);
}

uint8_t
lis2de_query_data_overrun_on_z_axis(void)
{
    return lis2de_query(STATUS_REG2, BITMASK_6);
}

uint8_t
lis2de_query_data_overrun_on_y_axis(void)
{
    return lis2de_query(STATUS_REG2, BITMASK_5);
}

uint8_t
lis2de_query_data_overrun_on_x_axis(void)
{
    return lis2de_query(STATUS_REG2, BITMASK_4);
}

uint8_t
lis2de_query_new_data_available_on_xyz_axes(void)
{
    return lis2de_query(STATUS_REG2, BITMASK_3);
}

uint8_t
lis2de_query_new_data_available_on_z_axis(void)
{
    return lis2de_query(STATUS_REG2, BITMASK_2);
}

uint8_t
lis2de_query_new_data_available_on_y_axis(void)
{
    return lis2de_query(STATUS_REG2, BITMASK_1);
}

uint8_t
lis2de_query_new_data_available_on_x_axis(void)
{
    return lis2de_query(STATUS_REG2, BITMASK_0);
}

lis2de_data_t
lis2de_query_accel_data(void)
{
    lis2de_data_t data = {0};

    data.x = lis2de_read_byte(OUT_REG_X.adr);
    data.y = lis2de_read_byte(OUT_REG_Y.adr);
    data.z = lis2de_read_byte(OUT_REG_Z.adr);

    return data;
}

// FIFO_CTRL_REG (0x2E):

uint8_t
lis2de_query_fifo_mode_selection(void)
{
    return lis2de_query(FIFO_CTRL_REG, BITMASK_76);
}

uint8_t lis2de_query_in_bypass_mode(void)
{
    return (lis2de_query(FIFO_CTRL_REG, BITMASK_76) == 0b00);
}

uint8_t
lis2de_query_in_fifo_mode(void)
{
    return (lis2de_query(FIFO_CTRL_REG, BITMASK_76) == 0b01);
}

uint8_t
lis2de_query_in_stream_mode(void)
{
    return (lis2de_query(FIFO_CTRL_REG, BITMASK_76) == 0b10);
}

uint8_t
lis2de_query_in_trigger_mode(void)
{
    return (lis2de_query(FIFO_CTRL_REG, BITMASK_76) == 0b11);
}

uint8_t
lis2de_query_trigger_selection(void)
{
    return lis2de_query(FIFO_CTRL_REG, BITMASK_5);
}

uint8_t
lis2de_query_fth(void)
{
    return lis2de_query(FIFO_CTRL_REG, BITMASK_43210);
}

// FIFO_SRC_REG (0x2F):

uint8_t
lis2de_query_fifo_watermark_level_exceeded(void)
{
    return lis2de_query(FIFO_SRC_REG, BITMASK_7);
}

uint8_t
lis2de_query_fifo_overrun(void)
{
    return lis2de_query(FIFO_SRC_REG, BITMASK_6);
}

uint8_t
lis2de_query_fifo_empty(void)
{
    return lis2de_query(FIFO_SRC_REG, BITMASK_5);
}

uint8_t
lis2de_query_fifo_current_number_of_unread_samples(void)
{
    return lis2de_query(FIFO_SRC_REG, BITMASK_43210);
}

// IG1_CFG (0x30)

uint8_t
lis2de_query_ig1_or_combination_of_interrupt_events_enabled(void)
{
    return (lis2de_query(IG1_CFG_REG, BITMASK_76) == 0b00);
}

uint8_t
lis2de_query_ig1_and_combination_of_interrupt_events_enabled(void)
{
    return (lis2de_query(IG1_CFG_REG, BITMASK_76) == 0b10);
}

uint8_t
lis2de_query_ig1_6_direction_movement_recognition_enabled(void)
{
    return (lis2de_query(IG1_CFG_REG, BITMASK_76) == 0b01);
}

uint8_t
lis2de_query_ig1_6_direction_position_recognition_enabled(void)
{
    return (lis2de_query(IG1_CFG_REG, BITMASK_76) == 0b11);
}

uint8_t
lis2de_query_ig1_ig_on_z_high_event_or_dir_recognition_enabled(void)
{
    return lis2de_query(IG1_CFG_REG, BITMASK_5);
}

uint8_t
lis2de_query_ig1_ig_on_z_low_event_or_dir_recognition_enabled(void)
{
    return lis2de_query(IG1_CFG_REG, BITMASK_4);
}

uint8_t
lis2de_query_ig1_ig_on_y_high_event_or_dir_recognition_enabled(void)
{
    return lis2de_query(IG1_CFG_REG, BITMASK_3);
}

uint8_t
lis2de_query_ig1_ig_on_y_low_event_or_dir_recognition_enabled(void)
{
    return lis2de_query(IG1_CFG_REG, BITMASK_2);
}

uint8_t
lis2de_query_ig1_ig_on_x_high_event_or_dir_recognition_enabled(void)
{
    return lis2de_query(IG1_CFG_REG, BITMASK_1);
}

uint8_t
lis2de_query_ig1_ig_on_x_low_event_or_dir_recognition_enabled(void)
{
    return lis2de_query(IG1_CFG_REG, BITMASK_0);
}

// IG1_SOURCE (0x31):

uint8_t
lis2de_query_ig1_interrupt_has_been_generated(void)
{
    return lis2de_query(IG1_SOURCE_REG, BITMASK_6);
}

uint8_t
lis2de_query_ig1_z_high_event_has_occured(void)
{
    return lis2de_query(IG1_SOURCE_REG, BITMASK_5);
}

uint8_t
lis2de_query_ig1_z_low_event_has_occured(void)
{
    return lis2de_query(IG1_SOURCE_REG, BITMASK_4);
}

uint8_t
lis2de_query_ig1_y_high_event_has_occured(void)
{
    return lis2de_query(IG1_SOURCE_REG, BITMASK_3);
}

uint8_t
lis2de_query_ig1_y_low_event_has_occured(void)
{
    return lis2de_query(IG1_SOURCE_REG, BITMASK_2);
}

uint8_t
lis2de_query_ig1_x_high_event_has_occured(void)
{
    return lis2de_query(IG1_SOURCE_REG, BITMASK_1);
}

uint8_t
lis2de_query_ig1_x_low_event_has_occured(void)
{
    return lis2de_query(IG1_SOURCE_REG, BITMASK_0);
}

// IG1_THS (0x32):

uint8_t
lis2de_query_ig1_threshold(void)
{
    return lis2de_query(IG1_THS_REG, BITMASK_FULL);
}

// IG1_DURATION (0x33):

uint8_t lis2de_query_ig1_duration(void)
{
    return lis2de_query(IG1_DURATION_REG, BITMASK_FULL);
}

// IG2_CFG (0x34)

uint8_t
lis2de_query_ig2_or_combination_of_interrupt_events_enabled(void)
{
    return (lis2de_query(IG2_CFG_REG, BITMASK_76) == 0b00);
}

uint8_t
lis2de_query_ig2_and_combination_of_interrupt_events_enabled(void)
{
    return (lis2de_query(IG2_CFG_REG, BITMASK_76) == 0b10);
}

uint8_t
lis2de_query_ig2_6_direction_movement_recognition_enabled(void)
{
    return (lis2de_query(IG2_CFG_REG, BITMASK_76) == 0b01);
}

uint8_t
lis2de_query_ig2_6_direction_position_recognition_enabled(void)
{
    return (lis2de_query(IG2_CFG_REG, BITMASK_76) == 0b11);
}

uint8_t
lis2de_query_ig2_ig_on_z_high_event_or_dir_recognition_enabled(void)
{
    return lis2de_query(IG2_CFG_REG, BITMASK_5);
}

uint8_t
lis2de_query_ig2_ig_on_z_low_event_or_dir_recognition_enabled(void)
{
    return lis2de_query(IG2_CFG_REG, BITMASK_4);
}

uint8_t
lis2de_query_ig2_ig_on_y_high_event_or_dir_recognition_enabled(void)
{
    return lis2de_query(IG2_CFG_REG, BITMASK_3);
}

uint8_t
lis2de_query_ig2_ig_on_y_low_event_or_dir_recognition_enabled(void)
{
    return lis2de_query(IG2_CFG_REG, BITMASK_2);
}

uint8_t
lis2de_query_ig2_ig_on_x_high_event_or_dir_recognition_enabled(void)
{
    return lis2de_query(IG2_CFG_REG, BITMASK_1);
}

uint8_t
lis2de_query_ig2_ig_on_x_low_event_or_dir_recognition_enabled(void)
{
    return lis2de_query(IG2_CFG_REG, BITMASK_0);
}

// IG2_SOURCE (0x35):

uint8_t
lis2de_query_ig2_interrupt_has_been_generated(void)
{
    return lis2de_query(IG2_SOURCE_REG, BITMASK_6);
}

uint8_t
lis2de_query_ig2_z_high_event_has_occured(void)
{
    return lis2de_query(IG2_SOURCE_REG, BITMASK_5);
}

uint8_t
lis2de_query_ig2_z_low_event_has_occured(void)
{
    return lis2de_query(IG2_SOURCE_REG, BITMASK_4);
}

uint8_t
lis2de_query_ig2_y_high_event_has_occured(void)
{
    return lis2de_query(IG2_SOURCE_REG, BITMASK_3);
}

uint8_t
lis2de_query_ig2_y_low_event_has_occured(void)
{
    return lis2de_query(IG2_SOURCE_REG, BITMASK_2);
}

uint8_t
lis2de_query_ig2_x_high_event_has_occured(void)
{
    return lis2de_query(IG2_SOURCE_REG, BITMASK_1);
}

uint8_t
lis2de_query_ig2_x_low_event_has_occured(void)
{
    return lis2de_query(IG2_SOURCE_REG, BITMASK_0);
}

// IG2_THS (0x36)

uint8_t
lis2de_query_ig2_threshold(void)
{
    return lis2de_query(IG2_THS_REG, BITMASK_FULL);
}

// IG2_DURATION (0x37)

uint8_t lis2de_query_ig2_duration(void)
{
    return lis2de_query(IG2_DURATION_REG, BITMASK_FULL);
}

// CLICK_CFG (0x38)

uint8_t
lis2de_query_interrupt_double_click_on_z_axis_enabled(void)
{
    return lis2de_query(CLICK_CFG_REG, BITMASK_5);
}

uint8_t
lis2de_query_interrupt_single_click_on_z_axis_enabled(void)
{
    return lis2de_query(CLICK_CFG_REG, BITMASK_4);
}

uint8_t
lis2de_query_interrupt_double_click_on_y_axis_enabled(void)
{
    return lis2de_query(CLICK_CFG_REG, BITMASK_3);
}

uint8_t
lis2de_query_interrupt_single_click_on_y_axis_enabled(void)
{
    return lis2de_query(CLICK_CFG_REG, BITMASK_2);
}

uint8_t
lis2de_query_interrupt_double_click_on_x_axis_enabled(void)
{
    return lis2de_query(CLICK_CFG_REG, BITMASK_1);
}

uint8_t
lis2de_query_interrupt_single_click_on_x_axis_enabled(void)
{
    return lis2de_query(CLICK_CFG_REG, BITMASK_0);
}

// CLICK_SRC (0x39)

uint8_t
lis2de_query_interrupts_have_been_generated(void)
{
    return lis2de_query(CLICK_SRC_REG, BITMASK_6);
}

uint8_t
lis2de_query_double_click_enabled(void)
{
    return lis2de_query(CLICK_SRC_REG, BITMASK_5);
}

uint8_t
lis2de_query_single_click_enabled(void)
{
    return lis2de_query(CLICK_SRC_REG, BITMASK_4);
}

uint8_t
lis2de_query_click_sign(void)
{
    return lis2de_query(CLICK_SRC_REG, BITMASK_3);
}

uint8_t
lis2de_query_z_click_high_event_has_occured(void)
{
    return lis2de_query(CLICK_SRC_REG, BITMASK_2);
}

uint8_t
lis2de_query_y_click_high_event_has_occured(void)
{
    return lis2de_query(CLICK_SRC_REG, BITMASK_1);
}

uint8_t
lis2de_query_x_click_high_event_has_occured(void)
{
    return lis2de_query(CLICK_SRC_REG, BITMASK_0);
}

// CLICK_THS (0x3A)

uint8_t
lis2de_query_latch_interrupt_request_on_click_src_reg_enabled(void)
{
    return lis2de_query(CLICK_THS_REG, BITMASK_7);
}

uint8_t
lis2de_query_click_threshold(void)
{
    return lis2de_query(CLICK_THS_REG, BITMASK_6543210);
}

// TIME_LIMIT (0x3B)

uint8_t
lis2de_query_time_limit(void)
{
    return lis2de_query(TIME_LIMIT_REG, BITMASK_FULL);
}

// TIME_LATENCY (0x3C)

uint8_t
lis2de_query_time_latency(void)
{
    return lis2de_query(TIME_LATENCY_REG, BITMASK_FULL);
}

// TIME_WINDOW (0x3D)

uint8_t
lis2de_query_time_window(void)
{
    return lis2de_query(TIME_WINDOW_REG, BITMASK_FULL);
}

// Act_THS (0x3E)

uint8_t
lis2de_query_act_threshold(void)
{
    return lis2de_query(ACT_THS_REG, BITMASK_FULL);
}

// Act_DUR (0x3F)

uint8_t
lis2de_query_act_duration(void)
{
    return lis2de_query(ACT_DUR_REG, BITMASK_FULL);
}

static uint8_t
lis2de_query_operating_mode(void)
{
    return lis2de_query(CTRL_REG1, BITMASK_3);
}

uint8_t
lis2de_query_current_operating_mode_is_normal_mode(void)
{
    return (lis2de_query_operating_mode() == OP_MODE_NORMAL);
}

uint8_t
lis2de_query_current_operating_mode_is_low_power_mode(void)
{
    return (lis2de_query_operating_mode() == OP_MODE_LOW_POWER);
}

// Set-functions for all writable registers:


void
lis2de_set_operating_mode_to_normal_mode(void)
{
    lis2de_set(CTRL_REG1, BITMASK_3, 0b0);
}

void
lis2de_set_operating_mode_to_low_power_mode(void)
{
    lis2de_set(CTRL_REG1, BITMASK_3, 0b1);
}

void
lis2de_set_operating_mode_to_high_resolution_mode(void)
{
    lis2de_set(CTRL_REG1, BITMASK_3, 0b0);
}


void
lis2de_enable_temperature_sensor(void)
{
    lis2de_set(TEMP_CFG_REG, BITMASK_76, 0b11);
}

void
lis2de_disable_temperature_sensor(void)
{
    lis2de_set(TEMP_CFG_REG, BITMASK_76, 0b00);
}

// CTRL_REG1 (0x20)

void lis2de_set_power_down_mode(void)
{
    lis2de_set(CTRL_REG1, BITMASK_7654, 0b0000);
}

void
lis2de_set_data_rate_to_1hz(void)
{
    lis2de_set(CTRL_REG1, BITMASK_7654, 0b0001);
}

void
lis2de_set_data_rate_to_10hz(void)
{
    lis2de_set(CTRL_REG1, BITMASK_7654, 0b0010);
}

void
lis2de_set_data_rate_to_25hz(void)
{
    lis2de_set(CTRL_REG1, BITMASK_7654, 0b0011);
}

void
lis2de_set_data_rate_to_50hz(void)
{
    lis2de_set(CTRL_REG1, BITMASK_7654, 0b0100);
}

void
lis2de_set_data_rate_to_100hz(void)
{
    lis2de_set(CTRL_REG1, BITMASK_7654, 0b0101);
}

void
lis2de_set_data_rate_to_200hz(void)
{
    lis2de_set(CTRL_REG1, BITMASK_7654, 0b0110);
}

void
lis2de_set_data_rate_to_400hz(void)
{
    lis2de_set(CTRL_REG1, BITMASK_7654, 0b0111);
}

void
lis2de_set_low_power_mode(void)
{
    lis2de_set(CTRL_REG1, BITMASK_7654, 0b1000);
}

void
lis2de_set_data_rate_to_max(void)
{
    lis2de_set(CTRL_REG1, BITMASK_7654, 0b1001);
}

void
lis2de_enable_z_axis(void)
{
    lis2de_set(CTRL_REG1, BITMASK_2, 0b1);
}

void
lis2de_disable_z_axis(void)
{
    lis2de_set(CTRL_REG1, BITMASK_2, 0b0);
}

void
lis2de_enable_y_axis(void)
{
    lis2de_set(CTRL_REG1, BITMASK_1, 0b1);
}

void
lis2de_disable_y_axis(void)
{
    lis2de_set(CTRL_REG1, BITMASK_1, 0b0);
}

void
lis2de_enable_x_axis(void)
{
    lis2de_set(CTRL_REG1, BITMASK_0, 0b1);
}

void
lis2de_disable_x_axis(void)
{
    lis2de_set(CTRL_REG1, BITMASK_0, 0b0);
}

// CTRL_REG2 (0x21):

void
lis2de_set_high_pass_filter_to_normal_mode(void)
{
    lis2de_set(CTRL_REG2, BITMASK_76, 0b00);
}

void
lis2de_set_high_pass_filter_to_reference_mode(void)
{
    lis2de_set(CTRL_REG2, BITMASK_76, 0b01);
}

void
lis2de_set_high_pass_filter_to_autoreset_mode(void)
{
    lis2de_set(CTRL_REG2, BITMASK_76, 0b11);
}

void
lis2de_set_high_pass_filter_cut_off_freq_to_8()
{
    lis2de_set(CTRL_REG2, BITMASK_54, 0b00);
}

void
lis2de_set_high_pass_filter_cut_off_freq_to_16()
{
    lis2de_set(CTRL_REG2, BITMASK_54, 0b01);
}

void
lis2de_set_high_pass_filter_cut_off_freq_to_32()
{
    lis2de_set(CTRL_REG2, BITMASK_54, 0b10);
}

void
lis2de_set_high_pass_filter_cut_off_freq_to_64()
{
    lis2de_set(CTRL_REG2, BITMASK_54, 0b11);
}

void
lis2de_enable_internal_filter_bypass(void)
{
    lis2de_set(CTRL_REG2, BITMASK_3, 0b0);
}

void
lis2de_disable_internal_filter_bypass(void)
{
    lis2de_set(CTRL_REG2, BITMASK_3, 0b1);
}

void
lis2de_enable_high_pass_filter_for_click_function(void)
{
    lis2de_set(CTRL_REG2, BITMASK_2, 0b1);
}

void
lis2de_disable_high_pass_filter_for_click_function(void)
{
    lis2de_set(CTRL_REG2, BITMASK_2, 0b0);
}

void
lis2de_enable_high_pass_filter_for_aoi_function_on_int2(void)
{
    lis2de_set(CTRL_REG2, BITMASK_1, 0b1);
}

void
lis2de_disable_high_pass_filter_for_aoi_function_on_int2(void)
{
    lis2de_set(CTRL_REG2, BITMASK_1, 0b0);
}

void
lis2de_enable_high_pass_filter_for_aoi_function_on_int1(void)
{
    lis2de_set(CTRL_REG2, BITMASK_0, 0b1);
}

void
lis2de_disable_high_pass_filter_for_aoi_function_on_int1(void)
{
    lis2de_set(CTRL_REG2, BITMASK_0, 0b0);
}


// CTRL_REG3 (0x22)

void
lis2de_enable_click_interrupt_on_int1(void)
{
    lis2de_set(CTRL_REG3, BITMASK_7, 0b1);
}

void
lis2de_disable_click_interrupt_on_int1(void)
{
    lis2de_set(CTRL_REG3, BITMASK_7, 0b0);
}

void
lis2de_enable_aoi_interrupt_on_int1(void)
{
    lis2de_set(CTRL_REG3, BITMASK_6, 0b1);
}

void
lis2de_disable_aoi_interrupt_on_int1(void)
{
    lis2de_set(CTRL_REG3, BITMASK_6, 0b0);
}

void
lis2de_enable_aoi_interrupt_on_int2(void)
{
    lis2de_set(CTRL_REG3, BITMASK_5, 0b1);
}

void
lis2de_disable_aoi_interrupt_on_int2(void)
{
    lis2de_set(CTRL_REG3, BITMASK_5, 0b0);
}

void
lis2de_enable_drdy1_interrupt_on_int1(void)
{
    lis2de_set(CTRL_REG3, BITMASK_4, 0b1);
}

void
lis2de_disable_drdy1_interrupt_on_int1(void)
{
    lis2de_set(CTRL_REG3, BITMASK_4, 0b0);
}

void
lis2de_enable_drdy2_interrupt_on_int1(void)
{
    lis2de_set(CTRL_REG3, BITMASK_3, 0b1);
}

void
lis2de_disable_drdy2_interrupt_on_int1(void)
{
    lis2de_set(CTRL_REG3, BITMASK_3, 0b0);
}

void
lis2de_enable_fifo_watermark_interrupt_on_int1(void)
{
    lis2de_set(CTRL_REG3, BITMASK_2, 0b1);
}

void
lis2de_disable_fifo_watermark_interrupt_on_int1(void)
{
    lis2de_set(CTRL_REG3, BITMASK_2, 0b0);
}

void
lis2de_enable_fifo_overrun_interrupt_on_int1(void)
{
    lis2de_set(CTRL_REG3, BITMASK_1, 0b1);
}

void
lis2de_disable_fifo_overrun_interrupt_on_int1(void)
{
    lis2de_set(CTRL_REG3, BITMASK_1, 0b0);
}

// CTRL_REG4 (0x23)

void
lis2de_enable_continuos_block_data_update(void)
{
    lis2de_set(CTRL_REG4, BITMASK_7, 0b0);
}

void
lis2de_disable_continuos_block_data_update(void)
{
    lis2de_set(CTRL_REG4, BITMASK_7, 0b1);
}

void
lis2de_set_full_scale_to_2g(void)
{
    lis2de_set(CTRL_REG4, BITMASK_54, 0b00);
}

void
lis2de_set_full_scale_to_4g(void)
{
    lis2de_set(CTRL_REG4, BITMASK_54, 0b01);
}

void
lis2de_set_full_scale_to_8g(void)
{
    lis2de_set(CTRL_REG4, BITMASK_54, 0b10);
}

void
lis2de_set_full_scale_to_16g(void)
{
    lis2de_set(CTRL_REG4, BITMASK_54, 0b11);
}

void
lis2de_enable_self_test_mode(void)
{
    lis2de_set(CTRL_REG4, BITMASK_21, 0b01);
}

void
lis2de_disable_self_test_mode(void)
{
    lis2de_set(CTRL_REG4, BITMASK_21, 0b00);
}

void
lis2de_set_spi_4_wire_interface_mode(void)
{
    lis2de_set(CTRL_REG4, BITMASK_0, 0b0);
}

void
lis2de_set_spi_3_wire_interface_mode(void)
{
    lis2de_set(CTRL_REG4, BITMASK_0, 0b1);
}


// CTRL_REG5 (0x24)
void
lis2de_reboot_memory_content(void)
{
    lis2de_set(CTRL_REG5, BITMASK_7, 0b1);
}

void
lis2de_enable_fifo(void)
{
    lis2de_set(CTRL_REG5, BITMASK_6, 0b1);
}

void
lis2de_disable_fifo(void)
{
    lis2de_set(CTRL_REG5, BITMASK_6, 0b0);
}

void
lis2de_enable_latch_interrupt_request_on_ig1_src_reg(void)
{
    lis2de_set(CTRL_REG5, BITMASK_3, 0b1);
}

void
lis2de_disable_latch_interrupt_request_on_ig1_src_reg(void)
{
    lis2de_set(CTRL_REG5, BITMASK_3, 0b0);
}

void
lis2de_enable_latch_interrupt_request_on_int2_src_reg(void)
{
    lis2de_set(CTRL_REG5, BITMASK_1, 0b1);
}

void
lis2de_disable_latch_interrupt_request_on_int2_src_reg(void)
{
    lis2de_set(CTRL_REG5, BITMASK_1, 0b0);
}

// CTRL_REG6 (0x25):

void
lis2de_enable_click_interrupt_on_ig2_pin(void)
{
    lis2de_set(CTRL_REG6, BITMASK_7, 0b1);
}

void
lis2de_disable_click_interrupt_on_ig2_pin(void)
{
    lis2de_set(CTRL_REG6, BITMASK_7, 0b0);
}

void
lis2de_enable_interrupt_1_function_on_ig2_pin(void)
{
    lis2de_set(CTRL_REG6, BITMASK_6, 0b1);
}

void
lis2de_disable_interrupt_1_function_on_ig2_pin(void)
{
    lis2de_set(CTRL_REG6, BITMASK_6, 0b0);
}

void
lis2de_enable_interrupt_2_function_on_ig2_pin(void)
{
    lis2de_set(CTRL_REG6, BITMASK_5, 0b1);
}

void
lis2de_disable_interrupt_2_function_on_ig2_pin(void)
{
    lis2de_set(CTRL_REG6, BITMASK_5, 0b0);
}

void
lis2de_enable_boot_on_ig2_pin(void)
{
    lis2de_set(CTRL_REG6, BITMASK_4, 0b1);
}

void
lis2de_disable_boot_on_ig2_pin(void)
{
    lis2de_set(CTRL_REG6, BITMASK_4, 0b0);
}

void
lis2de_enable_activity_interrupt_on_ig2_pin(void)
{
    lis2de_set(CTRL_REG6, BITMASK_3, 0b1);
}

void
lis2de_disable_activity_interrupt_on_ig2_pin(void)
{
    lis2de_set(CTRL_REG6, BITMASK_3, 0b0);
}

void
lis2de_set_interrupt_active_high(void)
{
    lis2de_set(CTRL_REG6, BITMASK_1, 0b1);
}

void
lis2de_set_interrupt_active_low(void)
{
    lis2de_set(CTRL_REG6, BITMASK_1, 0b1);
}

// REFERENCE (0x26):

void
lis2de_set_reference(uint8_t value)
{
    lis2de_set(REFERENCE_REG, BITMASK_FULL, value);
}


// FIFO_CTRL_REG (0x2E):
void
lis2de_set_fifo_mode_to_bypass_mode(void)
{
    lis2de_set(FIFO_CTRL_REG, BITMASK_76, 0b00);
}

void
lis2de_set_fifo_mode_to_fifo_mode(void)
{
    lis2de_set(FIFO_CTRL_REG, BITMASK_76, 0b01);
}

void
lis2de_set_fifo_mode_to_stream_mode(void)
{
    lis2de_set(FIFO_CTRL_REG, BITMASK_76, 0b10);
}

void
lis2de_set_fifo_mode_to_trigger_mode(void)
{
    lis2de_set(FIFO_CTRL_REG, BITMASK_76, 0b11);
}

void
lis2de_set_trigger_event_allows_to_trigger_signal_on_int1(void)
{
    lis2de_set(FIFO_CTRL_REG, BITMASK_5, 0b0);
}

void
lis2de_set_trigger_event_allows_to_trigger_signal_on_int2(void)
{
    lis2de_set(FIFO_CTRL_REG, BITMASK_5, 0b1);
}

void
lis2de_set_fth(uint8_t value)
{
    lis2de_set(FIFO_CTRL_REG, BITMASK_43210, value);
}


// IG1_CFG (0x30):
void
lis2de_set_ig1_or_combination_of_interrupt_events(void)
{
    lis2de_set(IG1_CFG_REG, BITMASK_76, 0b00);
}

void
lis2de_set_ig1_and_combination_of_interrupt_events(void)
{
    lis2de_set(IG1_CFG_REG, BITMASK_76, 0b10);
}

void lis2de_set_ig1_6_direction_movement_recognition(void)
{
    lis2de_set(IG1_CFG_REG, BITMASK_76, 0b01);
}

void
lis2de_set_ig1_6_direction_position_recognition(void)
{
    lis2de_set(IG1_CFG_REG, BITMASK_76, 0b11);
}

void
lis2de_enable_ig1_interrupt_generation_on_z_high_event(void)
{
    lis2de_set(IG1_CFG_REG, BITMASK_5, 0b1);
}

void
lis2de_disable_ig1_interrupt_generation_on_z_high_event(void)
{
    lis2de_set(IG1_CFG_REG, BITMASK_5, 0b0);
}

void
lis2de_enable_ig1_interrupt_generation_on_z_low_event(void)
{
    lis2de_set(IG1_CFG_REG, BITMASK_4, 0b1);
}

void
lis2de_disable_ig1_interrupt_generation_on_z_low_event(void)
{
    lis2de_set(IG1_CFG_REG, BITMASK_4, 0b0);
}

void
lis2de_enable_ig1_interrupt_generation_on_y_high_event(void)
{
    lis2de_set(IG1_CFG_REG, BITMASK_3, 0b1);
}

void
lis2de_disable_ig1_interrupt_generation_on_y_high_event(void)
{
    lis2de_set(IG1_CFG_REG, BITMASK_3, 0b0);
}

void
lis2de_enable_ig1_interrupt_generation_on_y_low_event(void)
{
    lis2de_set(IG1_CFG_REG, BITMASK_2, 0b1);
}

void
lis2de_disable_ig1_interrupt_generation_on_y_low_event(void)
{
    lis2de_set(IG1_CFG_REG, BITMASK_2, 0b0);
}

void
lis2de_enable_ig1_interrupt_generation_on_x_high_event(void)
{
    lis2de_set(IG1_CFG_REG, BITMASK_1, 0b1);
}

void
lis2de_disable_ig1_interrupt_generation_on_x_high_event(void)
{
    lis2de_set(IG1_CFG_REG, BITMASK_1, 0b0);
}

void
lis2de_enable_ig1_interrupt_generation_on_x_low_event(void)
{
    lis2de_set(IG1_CFG_REG, BITMASK_0, 0b1);
}

void
lis2de_disable_ig1_interrupt_generation_on_x_low_event(void)
{
    lis2de_set(IG1_CFG_REG, BITMASK_0, 0b0);
}

// IG1_THS (0x32):

void
lis2de_set_ig1_threshold(uint8_t ths)
{
    lis2de_set(IG1_THS_REG, BITMASK_FULL, ths);
}


// IG1_DURATION (0x33):

void
lis2de_set_ig1_duration(uint8_t dur)
{
    lis2de_set(IG1_DURATION_REG, BITMASK_FULL, dur);
}

// IG2_CFG (0x34):

void
lis2de_set_ig2_or_combination_of_interrupt_events(void)
{
    lis2de_set(IG2_CFG_REG, BITMASK_76, 0b00);
}

void
lis2de_set_ig2_and_combination_of_interrupt_events(void)
{
    lis2de_set(IG2_CFG_REG, BITMASK_76, 0b10);
}

void lis2de_set_ig2_6_direction_movement_recognition(void)
{
    lis2de_set(IG2_CFG_REG, BITMASK_76, 0b01);
}

void
lis2de_set_ig2_6_direction_position_recognition(void)
{
    lis2de_set(IG2_CFG_REG, BITMASK_76, 0b11);
}

void
lis2de_enable_ig2_interrupt_generation_on_z_high_event(void)
{
    lis2de_set(IG2_CFG_REG, BITMASK_5, 0b1);
}

void
lis2de_disable_ig2_interrupt_generation_on_z_high_event(void)
{
    lis2de_set(IG2_CFG_REG, BITMASK_5, 0b0);
}

void
lis2de_enable_ig2_interrupt_generation_on_z_low_event(void)
{
    lis2de_set(IG2_CFG_REG, BITMASK_4, 0b1);
}

void
lis2de_disable_ig2_interrupt_generation_on_z_low_event(void)
{
    lis2de_set(IG2_CFG_REG, BITMASK_4, 0b0);
}

void
lis2de_enable_ig2_interrupt_generation_on_y_high_event(void)
{
    lis2de_set(IG2_CFG_REG, BITMASK_3, 0b1);
}

void
lis2de_disable_ig2_interrupt_generation_on_y_high_event(void)
{
    lis2de_set(IG2_CFG_REG, BITMASK_3, 0b0);
}

void
lis2de_enable_ig2_interrupt_generation_on_y_low_event(void)
{
    lis2de_set(IG2_CFG_REG, BITMASK_2, 0b1);
}

void
lis2de_disable_ig2_interrupt_generation_on_y_low_event(void)
{
    lis2de_set(IG2_CFG_REG, BITMASK_2, 0b0);
}

void
lis2de_enable_ig2_interrupt_generation_on_x_high_event(void)
{
    lis2de_set(IG2_CFG_REG, BITMASK_1, 0b1);
}

void
lis2de_disable_ig2_interrupt_generation_on_x_high_event(void)
{
    lis2de_set(IG2_CFG_REG, BITMASK_1, 0b0);
}

void
lis2de_enable_ig2_interrupt_generation_on_x_low_event(void)
{
    lis2de_set(IG2_CFG_REG, BITMASK_0, 0b1);
}

void
lis2de_disable_ig2_interrupt_generation_on_x_low_event(void)
{
    lis2de_set(IG2_CFG_REG, BITMASK_0, 0b0);
}

// IG2_THS (0x36):

void
lis2de_set_ig2_threshold(uint8_t ths)
{
    lis2de_set(IG2_THS_REG, BITMASK_FULL, ths);
}


// IG2_DURATION (0x37):

void
lis2de_set_ig2_duration(uint8_t dur)
{
    lis2de_set(IG2_DURATION_REG, BITMASK_FULL, dur);
}

// CLICK_CFG (0x38):

void
lis2de_enable_interrupt_double_click_on_z_axis(void)
{
    lis2de_set(CLICK_CFG_REG, BITMASK_5, 0b1);
}

void
lis2de_disable_interrupt_double_click_on_z_axis(void)
{
    lis2de_set(CLICK_CFG_REG, BITMASK_5, 0b0);
}

void
lis2de_enable_interrupt_single_click_on_z_axis(void)
{
    lis2de_set(CLICK_CFG_REG, BITMASK_4, 0b1);
}

void
lis2de_disable_interrupt_single_click_on_z_axis(void)
{
    lis2de_set(CLICK_CFG_REG, BITMASK_4, 0b0);
}

void
lis2de_enable_interrupt_double_click_on_y_axis(void)
{
    lis2de_set(CLICK_CFG_REG, BITMASK_3, 0b1);
}

void
lis2de_disable_interrupt_double_click_on_y_axis(void)
{
    lis2de_set(CLICK_CFG_REG, BITMASK_3, 0b0);
}

void
lis2de_enable_interrupt_single_click_on_y_axis(void)
{
    lis2de_set(CLICK_CFG_REG, BITMASK_2, 0b1);
}

void
lis2de_disable_interrupt_single_click_on_y_axis(void)
{
    lis2de_set(CLICK_CFG_REG, BITMASK_2, 0b0);
}

void
lis2de_enable_interrupt_double_click_on_x_axis(void)
{
    lis2de_set(CLICK_CFG_REG, BITMASK_1, 0b1);
}

void
lis2de_disable_interrupt_double_click_on_x_axis(void)
{
    lis2de_set(CLICK_CFG_REG, BITMASK_1, 0b0);
}

void
lis2de_enable_interrupt_single_click_on_x_axis(void)
{
    lis2de_set(CLICK_CFG_REG, BITMASK_0, 0b1);
}

void
lis2de_disable_interrupt_single_click_on_x_axis(void)
{
    lis2de_set(CLICK_CFG_REG, BITMASK_0, 0b1);
}

// CLICK_THS (0x3A):

void
lis2de_set_click_threshold(uint8_t threshold)
{
    lis2de_set(CLICK_THS_REG, BITMASK_FULL, threshold);
}

// TIME_LIMIT (0x3B):

void
lis2de_set_time_limit(uint8_t limit)
{
    lis2de_set(TIME_LIMIT_REG, BITMASK_FULL, limit);
}

// TIME_LATENCY (0x3C):

void
lis2de_set_time_latency(uint8_t latency)
{
    lis2de_set(TIME_LATENCY_REG, BITMASK_FULL, latency);
}

// TIME_WINDOW (0x3D):

void
lis2de_set_time_window(uint8_t window)
{
    lis2de_set(TIME_WINDOW_REG, BITMASK_FULL, window);
}

// Act_THS (0x3E):

void
lis2de_set_act_threshold(uint8_t threshold)
{
    lis2de_set(ACT_THS_REG, BITMASK_FULL, threshold);
}

// Act_DUR (0x3F):

void
lis2de_set_act_duration(uint8_t duration)
{
    lis2de_set(ACT_DUR_REG, BITMASK_FULL, duration);
}
