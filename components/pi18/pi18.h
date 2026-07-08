#pragma once
#include <array>
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/select/select.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text/text.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace esphome
{
    namespace pi18
    {

        enum SelectKind : uint8_t
        {
            SELECT_INPUT_VOLTAGE_RANGE = 0,
            SELECT_OUTPUT_SOURCE_PRIORITY,
            SELECT_CHARGER_SOURCE_PRIORITY,
            SELECT_BATTERY_TYPE,
            SELECT_SOLAR_POWER_PRIORITY,
            SELECT_OUTPUT_MODEL,
            SELECT_AC_OUTPUT_FREQUENCY,
            SELECT_MACHINE_TYPE,
            SELECT_KIND_COUNT,
        };

        enum NumberKind : uint8_t
        {
            NUMBER_BATTERY_CUTOFF_VOLTAGE = 0,
            NUMBER_MAX_AC_CHARGING_CURRENT,
            NUMBER_MAX_CHARGING_CURRENT,
            NUMBER_BATTERY_MAX_CHARGE_VOLTAGE,
            NUMBER_BATTERY_FLOAT_VOLTAGE,
            NUMBER_BATTERY_RECHARGE_VOLTAGE,
            NUMBER_BATTERY_REDISCHARGE_VOLTAGE,
            NUMBER_KIND_COUNT,
        };

        enum SwitchKind : uint8_t
        {
            SWITCH_LOAD_POWER = 0,
            SWITCH_SILENCE_BUZZER,
            SWITCH_OVERLOAD_BYPASS,
            SWITCH_LCD_ESCAPE,
            SWITCH_OVERLOAD_RESTART,
            SWITCH_OVER_TEMP_RESTART,
            SWITCH_BACKLIGHT,
            SWITCH_ALARM_PRIMARY_SOURCE_INTERRUPT,
            SWITCH_FAULT_CODE_RECORD,
            SWITCH_KIND_COUNT,
        };

        class PI18Component;

        class PI18SettingSelect : public select::Select, public Parented<PI18Component>
        {
        public:
            explicit PI18SettingSelect(uint8_t kind) : kind_(static_cast<SelectKind>(kind)) {}

        protected:
            void control(size_t index) override;
            SelectKind kind_;
        };

        class PI18SettingNumber : public number::Number, public Parented<PI18Component>
        {
        public:
            explicit PI18SettingNumber(uint8_t kind) : kind_(static_cast<NumberKind>(kind)) {}

        protected:
            void control(float value) override;
            NumberKind kind_;
        };

        class PI18SettingSwitch : public switch_::Switch, public Parented<PI18Component>
        {
        public:
            explicit PI18SettingSwitch(uint8_t kind) : kind_(static_cast<SwitchKind>(kind)) {}

        protected:
            void write_state(bool state) override;
            SwitchKind kind_;
        };

        class PI18CommandText : public text::Text, public Component
        {
        public:
            void set_parent(PI18Component *parent) { this->parent_ = parent; }
            void dump_config() override;

        protected:
            void control(const std::string &value) override;
            PI18Component *parent_{nullptr};
        };

        class PI18Component : public PollingComponent, public uart::UARTDevice
        {
        public:
            void set_grid_voltage_sensor(sensor::Sensor *s) { this->grid_voltage_ = s; }
            void set_grid_frequency_sensor(sensor::Sensor *s) { this->grid_frequency_ = s; }
            void set_ac_output_voltage_sensor(sensor::Sensor *s) { this->ac_output_voltage_ = s; }
            void set_ac_output_frequency_sensor(sensor::Sensor *s) { this->ac_output_frequency_ = s; }
            void set_output_apparent_power_sensor(sensor::Sensor *s) { this->output_apparent_power_ = s; }
            void set_output_active_power_sensor(sensor::Sensor *s) { this->output_active_power_ = s; }
            void set_load_percent_sensor(sensor::Sensor *s) { this->load_percent_ = s; }
            void set_battery_voltage_sensor(sensor::Sensor *s) { this->battery_voltage_ = s; }
            void set_battery_charge_current_sensor(sensor::Sensor *s) { this->battery_charge_current_ = s; }
            void set_battery_discharge_current_sensor(sensor::Sensor *s) { this->battery_discharge_current_ = s; }
            void set_battery_capacity_sensor(sensor::Sensor *s) { this->battery_capacity_ = s; }
            void set_heatsink_temperature_sensor(sensor::Sensor *s) { this->heatsink_temperature_ = s; }
            void set_pv1_power_sensor(sensor::Sensor *s) { this->pv1_power_ = s; }
            void set_pv1_voltage_sensor(sensor::Sensor *s) { this->pv1_voltage_ = s; }
            void set_mode_text_sensor(text_sensor::TextSensor *ts) { this->mode_text_ = ts; }
            void set_manual_response_text_sensor(text_sensor::TextSensor *ts) { this->manual_response_text_ = ts; }
            void set_select(uint8_t kind, select::Select *s)
            {
                if (kind < SELECT_KIND_COUNT)
                    this->selects_[kind] = s;
            }
            void set_number(uint8_t kind, number::Number *n)
            {
                if (kind < NUMBER_KIND_COUNT)
                    this->numbers_[kind] = n;
            }
            void set_switch(uint8_t kind, switch_::Switch *sw)
            {
                if (kind < SWITCH_KIND_COUNT)
                    this->switches_[kind] = sw;
            }
            bool has_battery_max_charge_voltage() const { return this->has_battery_max_charge_voltage_; }
            float battery_max_charge_voltage() const { return this->battery_max_charge_voltage_; }
            void set_battery_max_charge_voltage(float value)
            {
                this->battery_max_charge_voltage_ = value;
                this->has_battery_max_charge_voltage_ = true;
            }
            bool has_battery_float_voltage() const { return this->has_battery_float_voltage_; }
            float battery_float_voltage() const { return this->battery_float_voltage_; }
            void set_battery_float_voltage(float value)
            {
                this->battery_float_voltage_ = value;
                this->has_battery_float_voltage_ = true;
            }
            bool has_battery_recharge_voltage() const { return this->has_battery_recharge_voltage_; }
            float battery_recharge_voltage() const { return this->battery_recharge_voltage_; }
            void set_battery_recharge_voltage(float value)
            {
                this->battery_recharge_voltage_ = value;
                this->has_battery_recharge_voltage_ = true;
            }
            bool has_battery_redischarge_voltage() const { return this->has_battery_redischarge_voltage_; }
            float battery_redischarge_voltage() const { return this->battery_redischarge_voltage_; }
            void set_battery_redischarge_voltage(float value)
            {
                this->battery_redischarge_voltage_ = value;
                this->has_battery_redischarge_voltage_ = true;
            }
            bool has_manual_response_text_sensor() const { return this->manual_response_text_ != nullptr; }
            void publish_manual_response(const std::string &state);
            bool send_protocol_command(char type, std::string_view cmd, std::string *response = nullptr,
                                       uint32_t timeout_ms = 0);
            bool send_manual_command(const std::string &cmd, std::string *response = nullptr, uint32_t timeout_ms = 0)
            {
                return this->send_protocol_command('P', cmd, response, timeout_ms);
            }

            void setup() override;
            void update() override;
            void dump_config() override;
            float get_setup_priority() const override { return setup_priority::DATA; }

        private:
            bool sync_configuration_();
            bool parse_piri_(const std::string &frame);
            bool parse_flag_(const std::string &frame);

            // sensors
            sensor::Sensor *grid_voltage_{nullptr};
            sensor::Sensor *grid_frequency_{nullptr};
            sensor::Sensor *ac_output_voltage_{nullptr};
            sensor::Sensor *ac_output_frequency_{nullptr};
            sensor::Sensor *output_apparent_power_{nullptr};
            sensor::Sensor *output_active_power_{nullptr};
            sensor::Sensor *load_percent_{nullptr};
            sensor::Sensor *battery_voltage_{nullptr};
            sensor::Sensor *battery_charge_current_{nullptr};
            sensor::Sensor *battery_discharge_current_{nullptr};
            sensor::Sensor *battery_capacity_{nullptr};
            sensor::Sensor *heatsink_temperature_{nullptr};
            sensor::Sensor *pv1_power_{nullptr};
            sensor::Sensor *pv1_voltage_{nullptr};
            float battery_max_charge_voltage_{0.0f};
            bool has_battery_max_charge_voltage_{false};
            float battery_float_voltage_{0.0f};
            bool has_battery_float_voltage_{false};
            float battery_recharge_voltage_{0.0f};
            bool has_battery_recharge_voltage_{false};
            float battery_redischarge_voltage_{0.0f};
            bool has_battery_redischarge_voltage_{false};
            bool initial_config_synced_{false};
            text_sensor::TextSensor *mode_text_{nullptr};
            text_sensor::TextSensor *manual_response_text_{nullptr};
            std::array<select::Select *, SELECT_KIND_COUNT> selects_{};
            std::array<number::Number *, NUMBER_KIND_COUNT> numbers_{};
            std::array<switch_::Switch *, SWITCH_KIND_COUNT> switches_{};

            // helpers
            std::string build_command_(char type, std::string_view cmd);
            bool query_(const char *cmd, std::string &frame, uint32_t timeout_ms);
            bool read_frame_(std::string &out, uint32_t timeout_ms);
            void drain_rx_buffer_();
            bool parse_mod_(const std::string &frame);
            bool parse_gs_(const std::string &payload);
            void publish_mode_(uint8_t code);
            void log_frame_(const char *label, std::string frame) const;

            // pi18 CRC (16-bit). See protocol docs & open-source implementations.
            static uint16_t crc16_pi18_(const uint8_t *data, size_t len);
        };

    } // namespace pi18
} // namespace esphome
