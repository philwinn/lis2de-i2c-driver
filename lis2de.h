#include <stdint.h>

// LIS2DE exception constants:
static const uint8_t E_NOT_IN_HIGH_RES_MODE = 1;
static const uint8_t E_LIS2DE_I2C_WRITE     = 2;
static const uint8_t E_LIS2DE_I2C_REP_START = 3;
static const uint8_t E_BDU_NOT_ENABLED      = 4;

typedef struct lis2de_data
{
    int8_t x;
    int8_t y;
    int8_t z;
} lis2de_data_t;



/* The function lis2de_init() must be called first in
 * order to init I2C commuication */
void lis2de_init(void);

/* Query single accel data set for all three axes when in
 * bypass mode using the function lis2de_query_accel_data() */
lis2de_data_t lis2de_query_accel_data(void);



// STATUS_AUX (0x07)
uint8_t lis2de_query_temperature_data_overrun(void);
uint8_t lis2de_query_temperature_new_data_available(void);

// OUT_TEMP (0x0C, 0x0D)
int8_t lis2de_query_temperature(void);

// INT_COUNTER (0x0E)
uint8_t lis2de_query_int_counter(void);

// WHO_AM_I (0x0F)
uint8_t lis2de_query_device_id(void);

// TEMP_CFG_REG (0x1F)
uint8_t lis2de_query_temperature_sensor_enabled(void);

// CTRL_REG1 (0x20)
uint8_t lis2de_query_data_rate_selection(void);
uint8_t lis2de_query_low_power_mode_enabled(void);
uint8_t lis2de_query_z_axis_enabled(void);
uint8_t lis2de_query_y_axis_enabled(void);
uint8_t lis2de_query_x_axis_enabled(void);

// CTRL_REG2 (0x21)
uint8_t lis2de_query_high_pass_filter_mode_selection(void);
uint8_t lis2de_query_high_pass_filter_in_normal_mode(void);
uint8_t lis2de_query_high_pass_filter_in_reference_mode(void);
uint8_t lis2de_query_high_pass_filter_in_auto_reset_mode(void);

uint8_t lis2de_query_high_pass_filter_cutoff_frequency_selection(void);

uint8_t lis2de_query_internal_filter_bypassed(void);

uint8_t lis2de_query_high_pass_filter_for_click_function_enabled(void);
uint8_t lis2de_query_high_pass_filter_for_ig1_enabled(void);
uint8_t lis2de_query_high_pass_filter_for_ig2_enabled(void);

// CTRL_REG3 (0x22)
uint8_t lis2de_query_click_interrupt_on_int1_enabled(void);
uint8_t lis2de_query_ig1_on_int1_enabled(void);
uint8_t lis2de_query_ig2_on_int1_enabled(void);
uint8_t lis2de_query_drdy1_interrupt_on_int1_enabled(void);
uint8_t lis2de_query_drdy2_interrupt_on_int1_enabled(void);
uint8_t lis2de_query_fifo_watermark_interrupt_on_int1_enabled(void);
uint8_t lis2de_query_fifo_overrun_interrupt_on_int1_enabled(void);

// CTRL_REG4 (0x23)
uint8_t lis2de_query_block_data_update_enabled(void);
uint8_t lis2de_query_full_scale_selection(void);
uint8_t lis2de_query_self_test_enabled(void);
uint8_t lis2de_query_spi_mode_selection(void);

// CTRL_REG5 (0x24)
uint8_t lis2de_query_reboot_memory_content(void);
uint8_t lis2de_query_fifo_enabled(void);
uint8_t lis2de_query_latch_interruot_request_on_ig1_source_reg(void);
uint8_t lis2de_query_int1_4d_detection_enabled(void);
uint8_t lis2de_query_latch_interruot_request_on_ig2_source_reg(void);
uint8_t lis2de_query_int2_4d_detection_enabled(void);

// CTRL_REG6 (0x25)
uint8_t lis2de_query_click_interrupt_on_int2_pin_enabled(void);
uint8_t lis2de_query_ig1_on_int2_pin_enabled(void);
uint8_t lis2de_query_ig2_on_int2_pin_enabled(void);
uint8_t lis2de_query_boot_on_int2_pin_enabled(void);
uint8_t lis2de_query_sleep_to_wake_function_interrupt_on_int2_pin_enabled(void);
uint8_t lis2de_query_interrupt_active_value(void);

// REFERENCE/DATACAPTURE (0x26)
uint8_t lis2de_query_reference(void);

// STATUS_REG2 (0x27)
uint8_t lis2de_query_data_overrun_on_xyz_axes(void);
uint8_t lis2de_query_data_overrun_on_z_axis(void);
uint8_t lis2de_query_data_overrun_on_y_axis(void);
uint8_t lis2de_query_data_overrun_on_x_axis(void);
uint8_t lis2de_query_new_data_available_on_xyz_axes(void);
uint8_t lis2de_query_new_data_available_on_z_axis(void);
uint8_t lis2de_query_new_data_available_on_y_axis(void);
uint8_t lis2de_query_new_data_available_on_x_axis(void);

// FIFO_CTRL_REG (0x2E)
uint8_t lis2de_query_fifo_mode_selection(void);
uint8_t lis2de_query_in_bypass_mode(void);
uint8_t lis2de_query_in_fifo_mode(void);
uint8_t lis2de_query_in_stream_mode(void);
uint8_t lis2de_query_in_trigger_mode(void);
uint8_t lis2de_query_trigger_selection(void);
uint8_t lis2de_query_fth(void);

// FIFO_SRC_REG (0x2F)
uint8_t lis2de_query_fifo_watermark_level_exceeded(void);
uint8_t lis2de_query_fifo_overrun(void);
uint8_t lis2de_query_fifo_empty(void);
uint8_t lis2de_query_fifo_current_number_of_unread_samples(void);

// IG1_CFG (0x30)
uint8_t lis2de_query_ig1_or_combination_of_interrupt_events_enabled(void);
uint8_t lis2de_query_ig1_and_combination_of_interrupt_events_enabled(void);
uint8_t lis2de_query_ig1_6_direction_movement_recognition_enabled(void);
uint8_t lis2de_query_ig1_6_direction_position_recognition_enabled(void);
uint8_t lis2de_query_ig1_ig_on_z_high_event_or_dir_recognition_enabled(void);
uint8_t lis2de_query_ig1_ig_on_z_low_event_or_dir_recognition_enabled(void);
uint8_t lis2de_query_ig1_ig_on_y_high_event_or_dir_recognition_enabled(void);
uint8_t lis2de_query_ig1_ig_on_y_low_event_or_dir_recognition_enabled(void);
uint8_t lis2de_query_ig1_ig_on_x_high_event_or_dir_recognition_enabled(void);
uint8_t lis2de_query_ig1_ig_on_x_low_event_or_dir_recognition_enabled(void);

// IG1_SOURCE (0x31)
uint8_t lis2de_query_ig1_interrupt_has_been_generated(void);
uint8_t lis2de_query_ig1_z_high_event_has_occured(void);
uint8_t lis2de_query_ig1_z_low_event_has_occured(void);
uint8_t lis2de_query_ig1_y_high_event_has_occured(void);
uint8_t lis2de_query_ig1_y_low_event_has_occured(void);
uint8_t lis2de_query_ig1_x_high_event_has_occured(void);
uint8_t lis2de_query_ig1_x_low_event_has_occured(void);

// IG1_THS (0x32)
uint8_t lis2de_query_ig1_threshold(void);

// IG1_DURATION (0x33)
uint8_t lis2de_query_ig1_duration(void);

// IG2_CFG (0x34)
uint8_t lis2de_query_ig2_or_combination_of_interrupt_events_enabled(void);
uint8_t lis2de_query_ig2_and_combination_of_interrupt_events_enabled(void);
uint8_t lis2de_query_ig2_6_direction_movement_recognition_enabled(void);
uint8_t lis2de_query_ig2_6_direction_position_recognition_enabled(void);
uint8_t lis2de_query_ig2_ig_on_z_high_event_or_dir_recognition_enabled(void);
uint8_t lis2de_query_ig2_ig_on_z_low_event_or_dir_recognition_enabled(void);
uint8_t lis2de_query_ig2_ig_on_y_high_event_or_dir_recognition_enabled(void);
uint8_t lis2de_query_ig2_ig_on_y_low_event_or_dir_recognition_enabled(void);
uint8_t lis2de_query_ig2_ig_on_x_high_event_or_dir_recognition_enabled(void);
uint8_t lis2de_query_ig2_ig_on_x_low_event_or_dir_recognition_enabled(void);

// IG2_SOURCE (0x35)
uint8_t lis2de_query_ig2_interrupt_has_been_generated(void);
uint8_t lis2de_query_ig2_z_high_event_has_occured(void);
uint8_t lis2de_query_ig2_z_low_event_has_occured(void);
uint8_t lis2de_query_ig2_y_high_event_has_occured(void);
uint8_t lis2de_query_ig2_y_low_event_has_occured(void);
uint8_t lis2de_query_ig2_x_high_event_has_occured(void);
uint8_t lis2de_query_ig2_x_low_event_has_occured(void);

// IG2_THS (0x36)
uint8_t lis2de_query_ig2_threshold(void);

// IG2_DURATION (0x37)
uint8_t lis2de_query_ig2_duration(void);

// CLICK_CFG (0x38)
uint8_t lis2de_query_interrupt_double_click_on_z_axis_enabled(void);
uint8_t lis2de_query_interrupt_single_click_on_z_axis_enabled(void);
uint8_t lis2de_query_interrupt_double_click_on_y_axis_enabled(void);
uint8_t lis2de_query_interrupt_single_click_on_y_axis_enabled(void);
uint8_t lis2de_query_interrupt_double_click_on_x_axis_enabled(void);
uint8_t lis2de_query_interrupt_single_click_on_x_axis_enabled(void);

// CLICK_SRC (0x39)
uint8_t lis2de_query_interrupts_have_been_generated(void);
uint8_t lis2de_query_double_click_enabled(void);
uint8_t lis2de_query_single_click_enabled(void);
uint8_t lis2de_query_click_sign(void);
uint8_t lis2de_query_z_click_high_event_has_occured(void);
uint8_t lis2de_query_y_click_high_event_has_occured(void);
uint8_t lis2de_query_x_click_high_event_has_occured(void);

// CLICK_THS (0x3A)
uint8_t lis2de_query_latch_interrupt_request_on_click_src_reg_enabled(void);
uint8_t lis2de_query_click_threshold(void);

// TIME_LIMIT (0x3B)
uint8_t lis2de_query_time_limit(void);

// TIME_LATENCY (0x3C)
uint8_t lis2de_query_time_latency(void);

// TIME_WINDOW (0x3D)
uint8_t lis2de_query_time_window(void);

// Act_THS (0x3E)
uint8_t lis2de_query_act_threshold(void);

// Act_DUR (0x3F)
uint8_t lis2de_query_act_duration(void);


// Set functions for all writable registers:

void lis2de_set_operating_mode_to_normal_mode(void);
void lis2de_set_operating_mode_to_low_power_mode(void);

// TEMP_CFG_REG (0x1F)
void lis2de_enable_temperature_sensor(void);
void lis2de_disable_temperature_sensor(void);

// CTRL_REG1 (0x20)
void lis2de_set_power_down_mode(void);
// Data rates in Normal / Low-power mode:
void lis2de_set_data_rate_to_1hz(void);
void lis2de_set_data_rate_to_10hz(void);
void lis2de_set_data_rate_to_25hz(void);
void lis2de_set_data_rate_to_50hz(void);
void lis2de_set_data_rate_to_100hz(void);
void lis2de_set_data_rate_to_200hz(void);
void lis2de_set_data_rate_to_400hz(void);
// Data rate in Low-power mode, 1.6 kHz:
void lis2de_set_low_power_mode(void);
// HR/normal (1.344 kHz); Low power mode (5.376 kHz):
void lis2de_set_data_rate_to_max(void);

void lis2de_enable_z_axis(void);
void lis2de_enable_y_axis(void);
void lis2de_enable_x_axis(void);
void lis2de_disable_z_axis(void);
void lis2de_disable_y_axis(void);
void lis2de_disable_x_axis(void);

// CTRL_REG2 (0x21)
void lis2de_set_high_pass_filter_to_normal_mode(void);
void lis2de_set_high_pass_filter_to_reference_mode(void);
void lis2de_set_high_pass_filter_to_autoreset_mode(void);

void lis2de_set_high_pass_filter_cut_off_freq_to_8(void);
void lis2de_set_high_pass_filter_cut_off_freq_to_16(void);
void lis2de_set_high_pass_filter_cut_off_freq_to_32(void);
void lis2de_set_high_pass_filter_cut_off_freq_to_64(void);
void lis2de_enable_internal_filter_bypass(void);
void lis2de_disable_internal_filter_bypass(void);

void lis2de_enable_high_pass_filter_for_click_function(void);
void lis2de_disable_high_pass_filter_for_click_function(void);

void lis2de_enable_high_pass_filter_for_aoi_function_on_int1(void);
void lis2de_disable_high_pass_filter_for_aoi_function_on_int1(void);

void lis2de_enable_high_pass_filter_for_aoi_function_on_int2(void);
void lis2de_disable_high_pass_filter_for_aoi_function_on_int2(void);

// CTRL_REG3 (0x22)
void lis2de_enable_click_interrupt_on_int1(void);
void lis2de_disable_click_interrupt_on_int1(void);

void lis2de_enable_aoi_interrupt_on_int1(void);
void lis2de_disable_aoi_interrupt_on_int1(void);

void lis2de_enable_aoi_interrupt_on_int2(void);
void lis2de_disable_aoi_interrupt_on_int2(void);

void lis2de_enable_drdy1_interrupt_on_int1(void);
void lis2de_disable_drdy1_interrupt_on_int1(void);

void lis2de_enable_drdy2_interrupt_on_int1(void);
void lis2de_disable_drdy2_interrupt_on_int1(void);

void lis2de_enable_fifo_watermark_interrupt_on_int1(void);
void lis2de_disable_fifo_watermark_interrupt_on_int1(void);

void lis2de_enable_fifo_overrun_interrupt_on_int1(void);
void lis2de_disable_fifo_overrun_interrupt_on_int1(void);

// CTRL_REG4 (0x23)
void lis2de_enable_continuos_block_data_update(void);
void lis2de_disable_continuos_block_data_update(void);

void lis2de_set_full_scale_to_2g(void);
void lis2de_set_full_scale_to_4g(void);
void lis2de_set_full_scale_to_8g(void);
void lis2de_set_full_scale_to_16g(void);

void lis2de_enable_self_test_mode(void);
void lis2de_disable_self_test_mode(void);

void lis2de_set_spi_4_wire_interface_mode(void);
void lis2de_set_spi_3_wire_interface_mode(void);


// CTRL_REG5 (0x24)
void lis2de_reboot_memory_content(void);

void lis2de_enable_fifo(void);
void lis2de_disable_fifo(void);

void lis2de_enable_latch_interrupt_request_on_ig1_src_reg(void);
void lis2de_disable_latch_interrupt_request_on_ig1_src_reg(void);

void lis2de_enable_latch_interrupt_request_on_ig2_src_reg(void);
void lis2de_disable_latch_interrupt_request_on_ig2_src_reg(void);


// CTRL_REG6 (0x25)
void lis2de_enable_click_interrupt_on_ig2_pin(void);
void lis2de_disable_click_interrupt_on_ig2_pin(void);

void lis2de_enable_interrupt_1_function_on_ig2_pin(void);
void lis2de_disable_interrupt_1_function_on_ig2_pin(void);

void lis2de_enable_interrupt_2_function_on_ig2_pin(void);
void lis2de_disable_interrupt_2_function_on_ig2_pin(void);

void lis2de_enable_boot_on_ig2_pin(void);
void lis2de_disable_boot_on_ig2_pin(void);

void lis2de_enable_activity_interrupt_on_ig2_pin(void);
void lis2de_disable_activity_interrupt_on_ig2_pin(void);

void lis2de_set_interrupt_active_high(void);
void lis2de_set_interrupt_active_low(void);

// REFERENCE/DATACAPTURE (0x26)
void lis2de_set_reference(uint8_t value);

// FIFO_CTRL_REG (0x2E)
void lis2de_set_fifo_mode_to_bypass_mode(void);
void lis2de_set_fifo_mode_to_fifo_mode(void);
void lis2de_set_fifo_mode_to_stream_mode(void);
void lis2de_set_fifo_mode_to_trigger_mode(void);

void lis2de_set_trigger_event_allows_to_trigger_signal_on_int1(void);
void lis2de_set_trigger_event_allows_to_trigger_signal_on_int2(void);

void lis2de_set_fth(uint8_t value);

// IG1_CFG (0x30)
void lis2de_set_ig1_or_combination_of_interrupt_events(void);
void lis2de_set_ig1_and_combination_of_interrupt_events(void);
void lis2de_set_ig1_6_direction_movement_recognition(void);
void lis2de_set_ig1_6_direction_position_recognition(void);

void lis2de_enable_ig1_interrupt_generation_on_z_high_event(void);
void lis2de_disable_ig1_interrupt_generation_on_z_high_event(void);
void lis2de_enable_ig1_interrupt_generation_on_z_low_event(void);
void lis2de_disable_ig1_interrupt_generation_on_z_low_event(void);

void lis2de_enable_ig1_interrupt_generation_on_y_high_event(void);
void lis2de_disable_ig1_interrupt_generation_on_y_high_event(void);
void lis2de_enable_ig1_interrupt_generation_on_y_low_event(void);
void lis2de_disable_ig1_interrupt_generation_on_y_low_event(void);

void lis2de_enable_ig1_interrupt_generation_on_x_high_event(void);
void lis2de_disable_ig1_interrupt_generation_on_x_high_event(void);
void lis2de_enable_ig1_interrupt_generation_on_x_low_event(void);
void lis2de_disable_ig1_interrupt_generation_on_x_low_event(void);

// IG1_THS (0x32)
void lis2de_set_ig1_threshold(uint8_t ths);

// IG1_DURATION (0x33)
void lis2de_set_ig1_duration(uint8_t dur);


// IG2_CFG (0x34)
void lis2de_set_ig2_or_combination_of_interrupt_events(void);
void lis2de_set_ig2_and_combination_of_interrupt_events(void);
void lis2de_set_ig2_6_direction_movement_recognition(void);
void lis2de_set_ig2_6_direction_position_recognition(void);

void lis2de_enable_ig2_interrupt_generation_on_z_high_event(void);
void lis2de_disable_ig2_interrupt_generation_on_z_high_event(void);
void lis2de_enable_ig2_interrupt_generation_on_z_low_event(void);
void lis2de_disable_ig2_interrupt_generation_on_z_low_event(void);

void lis2de_enable_ig2_interrupt_generation_on_y_high_event(void);
void lis2de_disable_ig2_interrupt_generation_on_y_high_event(void);
void lis2de_enable_ig2_interrupt_generation_on_y_low_event(void);
void lis2de_disable_ig2_interrupt_generation_on_y_low_event(void);

void lis2de_enable_ig2_interrupt_generation_on_x_high_event(void);
void lis2de_disable_ig2_interrupt_generation_on_x_high_event(void);
void lis2de_enable_ig2_interrupt_generation_on_x_low_event(void);
void lis2de_disable_ig2_interrupt_generation_on_x_low_event(void);

// INT2_THS (0x36)
void lis2de_set_ig2_threshold(uint8_t ths);

// INT2_DURATION (0x37)
void lis2de_set_ig2_duration(uint8_t dur);

// CLICK_CFG (0x38)
void lis2de_enable_interrupt_double_click_on_z_axis(void);
void lis2de_disable_interrupt_double_click_on_z_axis(void);
void lis2de_enable_interrupt_single_click_on_z_axis(void);
void lis2de_disable_interrupt_single_click_on_z_axis(void);

void lis2de_enable_interrupt_double_click_on_y_axis(void);
void lis2de_disable_interrupt_double_click_on_y_axis(void);
void lis2de_enable_interrupt_single_click_on_y_axis(void);
void lis2de_disable_interrupt_single_click_on_y_axis(void);

void lis2de_enable_interrupt_double_click_on_x_axis(void);
void lis2de_disable_interrupt_double_click_on_x_axis(void);
void lis2de_enable_interrupt_single_click_on_x_axis(void);
void lis2de_disable_interrupt_single_click_on_x_axis(void);

// CLICK_THS (0x3A)
void lis2de_set_click_threshold(uint8_t threshold);

// TIME_LIMIT (0x3B)
void lis2de_set_time_limit(uint8_t limit);

// TIME_LATENCY (0x3C)
void lis2de_set_time_limit(uint8_t limit);

// TIME_WINDOW (0x3D)
void lis2de_set_time_window(uint8_t window);

// Act_THS (0x3E)
void lis2de_set_act_threshold(uint8_t threshold);

// Act_DUR (0x3F)
void lis2de_set_act_duration(uint8_t duration);
