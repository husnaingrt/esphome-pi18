#include "pi18.h"

#include <cmath>
#include <algorithm>
#include <array>
#include <cctype>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string_view>

#include "esphome/core/helpers.h"
#include "esphome/core/application.h"

namespace esphome
{
    namespace pi18
    {
        namespace
        {
            static const char *const TAG = "pi18";
            static constexpr uint32_t MOD_QUERY_TIMEOUT_MS = 500;
            static constexpr uint32_t GS_QUERY_TIMEOUT_MS = 1200;
            static constexpr uint32_t COMMAND_GAP_MS = 100;
            static constexpr uint32_t PIRI_POLL_RETRY_MS = 25;
            static constexpr uint32_t PIRI_RESPONSE_TIMEOUT_MS = 300;
            static constexpr size_t GS_MIN_FIELDS = 19;
            static constexpr size_t GS_MAX_FIELDS = 32;
            static constexpr size_t FRAME_HEADER_SIZE = 8;
            static constexpr size_t NUMERIC_BUFFER_SIZE = 24;

            size_t split_csv(std::string_view payload, std::array<std::string_view, GS_MAX_FIELDS> &fields)
            {
                size_t count = 0;
                size_t start = 0;

                while (start <= payload.size() && count < fields.size())
                {
                    size_t pos = payload.find(',', start);
                    if (pos == std::string_view::npos)
                        pos = payload.size();

                    fields[count++] = payload.substr(start, pos - start);

                    if (pos == payload.size())
                        break;
                    start = pos + 1;
                }

                return count;
            }

            float parse_float(std::string_view value)
            {
                if (value.empty())
                    return NAN;

                char buffer[NUMERIC_BUFFER_SIZE];
                size_t len = std::min(value.size(), sizeof(buffer) - 1);
                std::memcpy(buffer, value.data(), len);
                buffer[len] = '\0';
                return static_cast<float>(atof(buffer));
            }

            bool parse_uint8(std::string_view value, uint8_t *out)
            {
                if (value.empty() || out == nullptr)
                    return false;

                char buffer[NUMERIC_BUFFER_SIZE];
                size_t len = std::min(value.size(), sizeof(buffer) - 1);
                std::memcpy(buffer, value.data(), len);
                buffer[len] = '\0';

                char *end = nullptr;
                unsigned long parsed = std::strtoul(buffer, &end, 10);
                if (end == buffer || *end != '\0' || parsed > 255)
                    return false;

                *out = static_cast<uint8_t>(parsed);
                return true;
            }

            std::string_view strip_frame_suffix(std::string_view frame)
            {
                while (!frame.empty() && (frame.back() == '\r' || frame.back() == '\n'))
                    frame.remove_suffix(1);
                return frame;
            }

            std::string normalize_manual_command(std::string value)
            {
                auto is_space = [](unsigned char ch) { return std::isspace(ch) != 0; };

                value.erase(value.begin(), std::find_if_not(value.begin(), value.end(), is_space));
                value.erase(std::find_if_not(value.rbegin(), value.rend(), is_space).base(), value.end());

                if (value.size() >= 2)
                {
                    const char first = value.front();
                    const char last = value.back();
                    if ((first == '"' && last == '"') || (first == '\'' && last == '\''))
                    {
                        value = value.substr(1, value.size() - 2);
                        value.erase(value.begin(), std::find_if_not(value.begin(), value.end(), is_space));
                        value.erase(std::find_if_not(value.rbegin(), value.rend(), is_space).base(), value.end());
                    }
                }

                return value;
            }

            constexpr std::array<const char *, 2> INPUT_VOLTAGE_RANGE_OPTIONS = {"Appliance", "UPS"};
            constexpr std::array<const char *, 2> OUTPUT_SOURCE_PRIORITY_OPTIONS = {
                "Solar-Utility-Battery",
                "Solar-Battery-Utility",
            };
            constexpr std::array<const char *, 3> CHARGER_SOURCE_PRIORITY_OPTIONS = {
                "Solar first",
                "Solar and Utility",
                "Only solar",
            };
            constexpr std::array<const char *, 3> BATTERY_TYPE_OPTIONS = {
                "AGM",
                "Flooded",
                "User",
            };
            constexpr std::array<const char *, 2> SOLAR_POWER_PRIORITY_OPTIONS = {
                "Battery-Load-Utility",
                "Load-Battery-Utility",
            };
            constexpr std::array<const char *, 5> OUTPUT_MODEL_OPTIONS = {
                "Single module",
                "Parallel output",
                "Phase 1 of three phase output",
                "Phase 2 of three phase output",
                "Phase 3 of three phase output",
            };
            constexpr std::array<const char *, 2> AC_OUTPUT_FREQUENCY_OPTIONS = {"50 Hz", "60 Hz"};
            constexpr std::array<const char *, 2> MACHINE_TYPE_OPTIONS = {"Off-Grid Tie", "Grid-Tie"};

            template<size_t N> void publish_select_state(select::Select *select, const std::array<const char *, N> &options,
                                                         uint8_t index)
            {
                if (select != nullptr && index < options.size())
                    select->publish_state(options[index]);
            }

            inline void publish_number_state(number::Number *number, float value)
            {
                if (number != nullptr)
                    number->publish_state(value);
            }

            inline void publish_switch_state(switch_::Switch *sw, bool state)
            {
                if (sw != nullptr)
                    sw->publish_state(state);
            }

            inline bool accepted_response(const std::string &response)
            {
                return response.size() >= 2 && response[1] == '1';
            }
        }  // namespace

        void PI18Component::setup()
        {
            if (this->parent_ == nullptr)
            {
                ESP_LOGW(TAG, "UART parent not configured, delaying PI18 setup");
                return;
            }
            ESP_LOGI(TAG, "PI18 driver init (UART %" PRIu32 " baud)", this->parent_->get_baud_rate());
        }

        void PI18Component::dump_config()
        {
            ESP_LOGCONFIG(TAG, "PI18 Inverter (PI18 protocol)");
            LOG_UPDATE_INTERVAL(this);
        }

        void PI18Component::update()
        {
            if (this->parent_ == nullptr)
            {
                ESP_LOGW(TAG, "UART parent not configured, skipping PI18 update");
                return;
            }

            if (!this->polling_enabled_)
                return;

            if (!this->initial_config_synced_)
            {
                if (this->sync_configuration())
                {
                    ESP_LOGI(TAG, "PI18 initial configuration synced");
                }
                return;
            }

            this->start_piri_polling_();
        }

        std::string PI18Component::build_command_(char type, std::string_view cmd)
        {
            // Frame format: "^Pnnn<cmd><crc><cr>" where nnn counts the CRC bytes and the CR.
            const uint16_t len = static_cast<uint16_t>(cmd.size() + 3);

            std::array<char, FRAME_HEADER_SIZE> header{};
            const int written = std::snprintf(header.data(), header.size(), "^%c%03u", type, static_cast<unsigned>(len));
            if (written < 0 || written >= static_cast<int>(header.size()))
            {
                ESP_LOGW(TAG, "Failed to build PI18 command header for %.*s", static_cast<int>(cmd.size()),
                         cmd.data());
                return {};
            }

            std::string out(header.data(), static_cast<size_t>(written));
            out.reserve(out.size() + cmd.size() + 3);
            out.append(cmd.data(), cmd.size());

            const uint16_t crc = crc16_pi18_(reinterpret_cast<const uint8_t *>(out.data()), out.size());
            out.push_back(static_cast<char>((crc >> 8) & 0xFF));
            out.push_back(static_cast<char>(crc & 0xFF));
            out.push_back('\r');
            return out;
        }

        size_t PI18Component::drain_rx_buffer_()
        {
            uint8_t discarded;
            uint8_t watchdog = 128;
            uint8_t yielded = 0;
            size_t count = 0;
            while (this->available() > 0 && watchdog-- > 0)
            {
                if (!this->read_byte(&discarded))
                    break;
                count++;
                if (++yielded >= 16)
                {
                    yielded = 0;
                    App.feed_wdt();
                    delay(0);
                }
            }

            return count;
        }

        bool PI18Component::read_frame_(std::string &out, uint32_t timeout_ms)
        {
            out.clear();
            out.reserve(192);

            const uint32_t start = millis();
            bool started = false;
            uint16_t yielded = 0;

            while (millis() - start < timeout_ms)
            {
                if (this->available() == 0)
                {
                    delay(2);
                    continue;
                }

                uint8_t ch;
                if (!this->read_byte(&ch))
                    continue;

                if (!started)
                {
                    if (ch != '^')
                    {
                        ESP_LOGD(TAG, "Skipping stray UART byte 0x%02X before frame start", ch);
                        if (++yielded >= 16)
                        {
                            yielded = 0;
                            App.feed_wdt();
                            delay(0);
                        }
                        continue;
                    }
                    started = true;
                }

                out.push_back(static_cast<char>(ch));
                if (++yielded >= 16)
                {
                    yielded = 0;
                    App.feed_wdt();
                    delay(0);
                }
                if (ch == '\r')
                    return true;
            }

            return false;
        }

        bool PI18Component::sync_configuration()
        {
            this->cancel_piri_poll_();
            const bool ok = this->sync_configuration_();
            if (ok)
            {
                this->initial_config_synced_ = true;
                if (this->polling_enabled_)
                    this->start_piri_polling_();
            }
            return ok;
        }

        void PI18Component::set_polling_enabled(bool enabled)
        {
            this->polling_enabled_ = enabled;
            if (!enabled)
            {
                this->cancel_piri_poll_();
                this->cancel_interval("piri_poll");
                this->piri_poll_started_ = false;
                return;
            }

            if (this->initial_config_synced_)
                this->start_piri_polling_();
        }

        void PI18Component::start_piri_polling_()
        {
            if (this->piri_poll_started_ || !this->polling_enabled_)
                return;

            this->piri_poll_started_ = true;
            this->set_interval("piri_poll", this->get_update_interval(), [this]() { this->poll_piri_(); });
            this->cancel_interval(InternalSchedulerID::POLLING_UPDATE);
        }

        void PI18Component::cancel_piri_poll_()
        {
            this->cancel_timeout("piri_read");
            this->piri_poll_pending_ = false;
            this->piri_poll_started_at_ = 0;
            this->piri_poll_frame_len_ = 0;
        }

        bool PI18Component::parse_mod_(const std::string &frame)
        {
            std::string_view view = strip_frame_suffix(frame);
            const size_t dpos = view.find("^D");
            if (dpos == std::string_view::npos || view.size() <= dpos + 5)
            {
                ESP_LOGW(TAG, "Unexpected MOD response: %s", frame.c_str());
                return false;
            }

            size_t pos = dpos + 5;
            size_t end = pos;
            while (end < view.size() && std::isdigit(static_cast<unsigned char>(view[end])))
                ++end;

            if (end <= pos)
            {
                ESP_LOGW(TAG, "Failed to parse MOD response: %s", frame.c_str());
                return false;
            }

            uint8_t mode = 0;
            if (!parse_uint8(view.substr(pos, end - pos), &mode))
            {
                ESP_LOGW(TAG, "Failed to decode MOD response: %s", frame.c_str());
                return false;
            }

            this->publish_mode_(mode);
            return true;
        }

        bool PI18Component::parse_gs_(const std::string &payload)
        {
            std::string_view view = strip_frame_suffix(payload);
            const size_t dpos = view.find("^D");
            if (dpos == std::string_view::npos || view.size() <= dpos + 5)
            {
                ESP_LOGW(TAG, "Unexpected GS response: %s", payload.c_str());
                return false;
            }

            std::string_view csv = view.substr(dpos + 5);
            const size_t csv_cut = csv.find_first_not_of("0123456789,");
            if (csv_cut != std::string_view::npos)
                csv = csv.substr(0, csv_cut);

            std::array<std::string_view, GS_MAX_FIELDS> fields{};
            const size_t field_count = split_csv(csv, fields);
            if (field_count < GS_MIN_FIELDS)
            {
                ESP_LOGW(TAG, "Unexpected GS field count: %u (need at least %u)", static_cast<unsigned>(field_count),
                         static_cast<unsigned>(GS_MIN_FIELDS));
                return false;
            }

            ESP_LOGD(TAG, "GS[0..5]=%.*s,%.*s,%.*s,%.*s,%.*s,%.*s", static_cast<int>(fields[0].size()),
                     fields[0].data(), static_cast<int>(fields[1].size()), fields[1].data(),
                     static_cast<int>(fields[2].size()), fields[2].data(), static_cast<int>(fields[3].size()),
                     fields[3].data(), static_cast<int>(fields[4].size()), fields[4].data(),
                     static_cast<int>(fields[5].size()), fields[5].data());

            if (this->grid_voltage_)
                this->grid_voltage_->publish_state(parse_float(fields[0]) / 10.0f);
            if (this->grid_frequency_)
                this->grid_frequency_->publish_state(parse_float(fields[1]) / 10.0f);
            if (this->ac_output_voltage_)
                this->ac_output_voltage_->publish_state(parse_float(fields[2]) / 10.0f);
            if (this->ac_output_frequency_)
                this->ac_output_frequency_->publish_state(parse_float(fields[3]) / 10.0f);
            if (this->output_apparent_power_)
                this->output_apparent_power_->publish_state(parse_float(fields[4]));
            if (this->output_active_power_)
                this->output_active_power_->publish_state(parse_float(fields[5]));
            if (this->load_percent_)
                this->load_percent_->publish_state(parse_float(fields[6]));
            if (this->battery_voltage_)
                this->battery_voltage_->publish_state(parse_float(fields[7]) / 10.0f);
            if (this->battery_discharge_current_)
                this->battery_discharge_current_->publish_state(parse_float(fields[10]));
            if (this->battery_charge_current_)
                this->battery_charge_current_->publish_state(parse_float(fields[11]));
            if (this->battery_capacity_)
                this->battery_capacity_->publish_state(parse_float(fields[12]));
            if (this->heatsink_temperature_)
                this->heatsink_temperature_->publish_state(parse_float(fields[13]));
            if (this->pv1_power_)
                this->pv1_power_->publish_state(parse_float(fields[16]));
            if (this->pv1_voltage_)
                this->pv1_voltage_->publish_state(parse_float(fields[18]) / 10.0f);
            if (this->switches_[SWITCH_LOAD_POWER] != nullptr)
            {
                uint8_t load_connected = 0;
                if (parse_uint8(fields[23], &load_connected))
                    this->switches_[SWITCH_LOAD_POWER]->publish_state(load_connected != 0);
            }

            return true;
        }

        void PI18Component::log_frame_(const char *label, std::string_view frame) const
        {
            while (!frame.empty() && (frame.back() == '\r' || frame.back() == '\n'))
                frame.remove_suffix(1);
            ESP_LOGD(TAG, "%s: %.*s", label, static_cast<int>(frame.size()), frame.data());
        }

        bool PI18Component::poll_piri_()
        {
            if (!this->polling_enabled_ || this->parent_ == nullptr)
                return false;

            const uint32_t now = millis();

            if (!this->piri_poll_pending_)
            {
                if (!this->uart_mutex_.try_lock())
                    return false;

                this->cancel_timeout("piri_read");
                this->piri_poll_frame_len_ = 0;

                const bool sent = this->send_protocol_command_locked_('P', "PIRI", nullptr, 0);
                this->uart_mutex_.unlock();
                if (!sent)
                    return false;

                this->piri_poll_pending_ = true;
                this->piri_poll_started_at_ = now;
                this->set_timeout("piri_read", PIRI_POLL_RETRY_MS, [this]() { this->poll_piri_(); });
                return true;
            }

            if (now - this->piri_poll_started_at_ > PIRI_RESPONSE_TIMEOUT_MS)
            {
                ESP_LOGW(TAG, "PIRI poll timed out");
                this->cancel_piri_poll_();
                return false;
            }

            if (!this->uart_mutex_.try_lock())
            {
                this->set_timeout("piri_read", PIRI_POLL_RETRY_MS, [this]() { this->poll_piri_(); });
                return false;
            }

            bool complete = false;
            bool overflow = false;
            while (this->available() > 0)
            {
                uint8_t ch = 0;
                if (!this->read_byte(&ch))
                    break;

                if (this->piri_poll_frame_len_ == 0 && ch != '^')
                    continue;

                if (this->piri_poll_frame_len_ + 1 >= this->piri_poll_frame_.size())
                {
                    overflow = true;
                    break;
                }

                this->piri_poll_frame_[this->piri_poll_frame_len_++] = static_cast<char>(ch);
                if (ch == '\r')
                {
                    this->piri_poll_frame_[this->piri_poll_frame_len_] = '\0';
                    complete = true;
                    break;
                }
            }

            this->uart_mutex_.unlock();

            if (overflow)
            {
                ESP_LOGW(TAG, "PIRI frame too large");
                this->cancel_piri_poll_();
                return false;
            }

            if (!complete)
            {
                this->set_timeout("piri_read", PIRI_POLL_RETRY_MS, [this]() { this->poll_piri_(); });
                return false;
            }

            std::string_view frame(this->piri_poll_frame_.data(), this->piri_poll_frame_len_);
            this->log_frame_("PIRI", frame);
            if (!this->parse_piri_(frame))
            {
                ESP_LOGW(TAG, "Failed to parse PIRI response");
                this->cancel_piri_poll_();
                return false;
            }

            this->cancel_piri_poll_();
            return true;
        }

        bool PI18Component::sync_configuration_()
        {
            bool ok = true;
            bool piri_ok = false;
            bool mod_ok = false;
            bool gs_ok = false;
            bool flag_ok = false;
            std::string piri_frame;
            std::string mod_frame;
            std::string gs_frame;
            std::string flag_frame;

            if (!this->uart_mutex_.try_lock())
            {
                ESP_LOGW(TAG, "UART busy, cannot sync PI18 configuration");
                return false;
            }

            piri_ok = this->query_locked_("PIRI", piri_frame, GS_QUERY_TIMEOUT_MS);
            delay(COMMAND_GAP_MS);
            mod_ok = this->query_locked_("MOD", mod_frame, MOD_QUERY_TIMEOUT_MS);
            delay(COMMAND_GAP_MS);
            gs_ok = this->query_locked_("GS", gs_frame, GS_QUERY_TIMEOUT_MS);
            delay(COMMAND_GAP_MS);
            flag_ok = this->query_locked_("FLAG", flag_frame, MOD_QUERY_TIMEOUT_MS);
            this->uart_mutex_.unlock();

            if (!piri_ok)
            {
                ESP_LOGW(TAG, "Failed to read PIRI response");
                ok = false;
            }
            else if (!this->parse_piri_(piri_frame))
            {
                ESP_LOGW(TAG, "Failed to parse PIRI response");
                ok = false;
            }

            if (!mod_ok)
            {
                ESP_LOGW(TAG, "Failed to read MOD response");
                ok = false;
            }
            else if (!this->parse_mod_(mod_frame))
            {
                ESP_LOGW(TAG, "Failed to parse MOD response");
                ok = false;
            }

            if (!gs_ok)
            {
                ESP_LOGW(TAG, "Failed to read GS response");
                ok = false;
            }
            else if (!this->parse_gs_(gs_frame))
            {
                ESP_LOGW(TAG, "Failed to parse GS response");
                ok = false;
            }

            if (!flag_ok)
                ESP_LOGW(TAG, "Failed to read FLAG response");
            else if (!this->parse_flag_(flag_frame))
                ESP_LOGW(TAG, "Failed to parse FLAG response");

            return ok;
        }

        bool PI18Component::parse_piri_(std::string_view frame)
        {
            std::string_view view = strip_frame_suffix(frame);
            const size_t dpos = view.find("^D");
            if (dpos == std::string_view::npos || view.size() <= dpos + 5)
            {
                ESP_LOGW(TAG, "Unexpected PIRI response: %.*s", static_cast<int>(frame.size()), frame.data());
                return false;
            }

            std::string_view csv = view.substr(dpos + 5);
            const size_t csv_cut = csv.find_first_not_of("0123456789,");
            if (csv_cut != std::string_view::npos)
                csv = csv.substr(0, csv_cut);

            std::array<std::string_view, 32> fields{};
            const size_t field_count = split_csv(csv, fields);
            if (field_count < 25)
            {
                ESP_LOGW(TAG, "Unexpected PIRI field count: %u (need at least 25)", static_cast<unsigned>(field_count));
                return false;
            }

            if (uint8_t frequency = 0; parse_uint8(fields[3], &frequency))
            {
                if (frequency == 50)
                    publish_select_state(this->selects_[SELECT_AC_OUTPUT_FREQUENCY], AC_OUTPUT_FREQUENCY_OPTIONS, 0);
                else if (frequency == 60)
                    publish_select_state(this->selects_[SELECT_AC_OUTPUT_FREQUENCY], AC_OUTPUT_FREQUENCY_OPTIONS, 1);
            }

            if (uint8_t battery_type = 0; parse_uint8(fields[13], &battery_type))
                publish_select_state(this->selects_[SELECT_BATTERY_TYPE], BATTERY_TYPE_OPTIONS, battery_type);

            if (uint8_t input_voltage_range = 0; parse_uint8(fields[16], &input_voltage_range))
                publish_select_state(this->selects_[SELECT_INPUT_VOLTAGE_RANGE], INPUT_VOLTAGE_RANGE_OPTIONS,
                                     input_voltage_range);

            if (uint8_t output_source_priority = 0; parse_uint8(fields[17], &output_source_priority))
                publish_select_state(this->selects_[SELECT_OUTPUT_SOURCE_PRIORITY], OUTPUT_SOURCE_PRIORITY_OPTIONS,
                                     output_source_priority);

            if (uint8_t charger_source_priority = 0; parse_uint8(fields[18], &charger_source_priority))
                publish_select_state(this->selects_[SELECT_CHARGER_SOURCE_PRIORITY], CHARGER_SOURCE_PRIORITY_OPTIONS,
                                     charger_source_priority);

            if (uint8_t machine_type = 0; parse_uint8(fields[20], &machine_type))
                publish_select_state(this->selects_[SELECT_MACHINE_TYPE], MACHINE_TYPE_OPTIONS, machine_type);

            if (uint8_t output_model = 0; parse_uint8(fields[22], &output_model))
                publish_select_state(this->selects_[SELECT_OUTPUT_MODEL], OUTPUT_MODEL_OPTIONS, output_model);

            if (uint8_t solar_power_priority = 0; parse_uint8(fields[23], &solar_power_priority))
                publish_select_state(this->selects_[SELECT_SOLAR_POWER_PRIORITY], SOLAR_POWER_PRIORITY_OPTIONS,
                                     solar_power_priority);

            if (float battery_recharge_voltage = parse_float(fields[8]); !std::isnan(battery_recharge_voltage))
            {
                this->battery_recharge_voltage_ = battery_recharge_voltage / 10.0f;
                this->has_battery_recharge_voltage_ = true;
                publish_number_state(this->numbers_[NUMBER_BATTERY_RECHARGE_VOLTAGE],
                                     this->battery_recharge_voltage_);
            }

            if (float battery_redischarge_voltage = parse_float(fields[9]); !std::isnan(battery_redischarge_voltage))
            {
                this->battery_redischarge_voltage_ = battery_redischarge_voltage / 10.0f;
                this->has_battery_redischarge_voltage_ = true;
                publish_number_state(this->numbers_[NUMBER_BATTERY_REDISCHARGE_VOLTAGE],
                                     this->battery_redischarge_voltage_);
            }

            if (float battery_cut_off_voltage = parse_float(fields[10]); !std::isnan(battery_cut_off_voltage))
                publish_number_state(this->numbers_[NUMBER_BATTERY_CUTOFF_VOLTAGE], battery_cut_off_voltage / 10.0f);

            if (float battery_max_charge_voltage = parse_float(fields[11]); !std::isnan(battery_max_charge_voltage))
            {
                this->battery_max_charge_voltage_ = battery_max_charge_voltage / 10.0f;
                this->has_battery_max_charge_voltage_ = true;
                publish_number_state(this->numbers_[NUMBER_BATTERY_MAX_CHARGE_VOLTAGE],
                                     this->battery_max_charge_voltage_);
            }

            if (float battery_float_voltage = parse_float(fields[12]); !std::isnan(battery_float_voltage))
            {
                this->battery_float_voltage_ = battery_float_voltage / 10.0f;
                this->has_battery_float_voltage_ = true;
                publish_number_state(this->numbers_[NUMBER_BATTERY_FLOAT_VOLTAGE], this->battery_float_voltage_);
            }

            publish_number_state(this->numbers_[NUMBER_MAX_AC_CHARGING_CURRENT], parse_float(fields[14]));
            publish_number_state(this->numbers_[NUMBER_MAX_CHARGING_CURRENT], parse_float(fields[15]));

            return true;
        }

        bool PI18Component::parse_flag_(std::string_view frame)
        {
            std::string_view view = strip_frame_suffix(frame);
            const size_t dpos = view.find("^D");
            if (dpos == std::string_view::npos || view.size() <= dpos + 5)
            {
                ESP_LOGW(TAG, "Unexpected FLAG response: %.*s", static_cast<int>(frame.size()), frame.data());
                return false;
            }

            std::string_view csv = view.substr(dpos + 5);
            const size_t csv_cut = csv.find_first_not_of("01,");
            if (csv_cut != std::string_view::npos)
                csv = csv.substr(0, csv_cut);

            std::array<std::string_view, GS_MAX_FIELDS> fields{};
            const size_t field_count = split_csv(csv, fields);
            if (field_count < 8)
            {
                ESP_LOGW(TAG, "Unexpected FLAG field count: %u (need at least 8)", static_cast<unsigned>(field_count));
                return false;
            }

            uint8_t flag = 0;
            if (parse_uint8(fields[0], &flag))
                publish_switch_state(this->switches_[SWITCH_SILENCE_BUZZER], flag != 0);
            if (parse_uint8(fields[1], &flag))
                publish_switch_state(this->switches_[SWITCH_OVERLOAD_BYPASS], flag != 0);
            if (parse_uint8(fields[2], &flag))
                publish_switch_state(this->switches_[SWITCH_LCD_ESCAPE], flag != 0);
            if (parse_uint8(fields[3], &flag))
                publish_switch_state(this->switches_[SWITCH_OVERLOAD_RESTART], flag != 0);
            if (parse_uint8(fields[4], &flag))
                publish_switch_state(this->switches_[SWITCH_OVER_TEMP_RESTART], flag != 0);
            if (parse_uint8(fields[5], &flag))
                publish_switch_state(this->switches_[SWITCH_BACKLIGHT], flag != 0);
            if (parse_uint8(fields[6], &flag))
                publish_switch_state(this->switches_[SWITCH_ALARM_PRIMARY_SOURCE_INTERRUPT], flag != 0);
            if (parse_uint8(fields[7], &flag))
                publish_switch_state(this->switches_[SWITCH_FAULT_CODE_RECORD], flag != 0);

            return true;
        }

        void PI18Component::publish_manual_response(const std::string &state)
        {
            if (this->manual_response_text_ != nullptr)
                this->manual_response_text_->publish_state(state);
        }

        bool PI18Component::send_protocol_command(char type, std::string_view cmd, std::string *response,
                                                  uint32_t timeout_ms)
        {
            if (!this->uart_mutex_.try_lock())
            {
                ESP_LOGW(TAG, "UART busy, skipping PI18 command");
                return false;
            }

            this->cancel_piri_poll_();
            const bool ok = this->send_protocol_command_locked_(type, cmd, response, timeout_ms);
            this->uart_mutex_.unlock();
            return ok;
        }

        bool PI18Component::query_locked_(const char *cmd, std::string &frame, uint32_t timeout_ms)
        {
            if (!this->send_protocol_command_locked_('P', cmd, &frame, timeout_ms))
                return false;

            this->log_frame_(cmd, frame);
            return true;
        }

        bool PI18Component::send_protocol_command_locked_(char type, std::string_view cmd, std::string *response,
                                                          uint32_t timeout_ms)
        {
            this->drain_rx_buffer_();

            std::string request = this->build_command_(type, cmd);
            if (request.empty())
                return false;

            ESP_LOGD(TAG, "%c TX %.*s", type, static_cast<int>(cmd.size()), cmd.data());
            this->write_array(reinterpret_cast<const uint8_t *>(request.data()), request.size());

            if (response != nullptr)
            {
                response->clear();
                if (timeout_ms <= 0 || !this->read_frame_(*response, timeout_ms))
                    return false;
            }

            return true;
        }

        void PI18SettingSelect::control(size_t index)
        {
            if (this->parent_ == nullptr)
            {
                ESP_LOGW(TAG, "PI18 select has no parent");
                return;
            }

            char payload[24];
            const char *label = nullptr;
            const char *selected = nullptr;

            switch (this->kind_)
            {
            case SELECT_INPUT_VOLTAGE_RANGE:
                if (index >= INPUT_VOLTAGE_RANGE_OPTIONS.size())
                    return;
                std::snprintf(payload, sizeof(payload), "PGR%u", static_cast<unsigned>(index));
                label = "input voltage range";
                selected = INPUT_VOLTAGE_RANGE_OPTIONS[index];
                break;
            case SELECT_OUTPUT_SOURCE_PRIORITY:
                if (index >= OUTPUT_SOURCE_PRIORITY_OPTIONS.size())
                    return;
                std::snprintf(payload, sizeof(payload), "POP%u", static_cast<unsigned>(index));
                label = "output source priority";
                selected = OUTPUT_SOURCE_PRIORITY_OPTIONS[index];
                break;
            case SELECT_CHARGER_SOURCE_PRIORITY:
                if (index >= CHARGER_SOURCE_PRIORITY_OPTIONS.size())
                    return;
                std::snprintf(payload, sizeof(payload), "PCP0,%u", static_cast<unsigned>(index));
                label = "charger source priority";
                selected = CHARGER_SOURCE_PRIORITY_OPTIONS[index];
                break;
            case SELECT_BATTERY_TYPE:
                if (index >= BATTERY_TYPE_OPTIONS.size())
                    return;
                std::snprintf(payload, sizeof(payload), "PBT%u", static_cast<unsigned>(index));
                label = "battery type";
                selected = BATTERY_TYPE_OPTIONS[index];
                break;
            case SELECT_SOLAR_POWER_PRIORITY:
                if (index >= SOLAR_POWER_PRIORITY_OPTIONS.size())
                    return;
                std::snprintf(payload, sizeof(payload), "PSP%u", static_cast<unsigned>(index));
                label = "solar power priority";
                selected = SOLAR_POWER_PRIORITY_OPTIONS[index];
                break;
            case SELECT_OUTPUT_MODEL:
                if (index >= OUTPUT_MODEL_OPTIONS.size())
                    return;
                std::snprintf(payload, sizeof(payload), "POPM0,%u", static_cast<unsigned>(index));
                label = "output model";
                selected = OUTPUT_MODEL_OPTIONS[index];
                break;
            case SELECT_AC_OUTPUT_FREQUENCY:
                if (index >= AC_OUTPUT_FREQUENCY_OPTIONS.size())
                    return;
                std::snprintf(payload, sizeof(payload), "%s", index == 0 ? "F50" : "F60");
                label = "AC output frequency";
                selected = AC_OUTPUT_FREQUENCY_OPTIONS[index];
                break;
            case SELECT_MACHINE_TYPE:
                if (index >= MACHINE_TYPE_OPTIONS.size())
                    return;
                // PI18 uses the generic enable/disable command format: P + state + parameter.
                std::snprintf(payload, sizeof(payload), "P%cI", index == 0 ? 'D' : 'E');
                label = "machine type";
                selected = MACHINE_TYPE_OPTIONS[index];
                break;
            default:
                ESP_LOGW(TAG, "Unknown PI18 select kind %u", static_cast<unsigned>(this->kind_));
                return;
            }

            std::string response;
            if (!this->parent_->send_protocol_command('S', payload, &response, 1000) || !accepted_response(response))
            {
                ESP_LOGW(TAG, "Failed to set %s", label != nullptr ? label : "PI18 select");
                return;
            }

            if (selected != nullptr)
                this->publish_state(selected);
        }

        void PI18SettingNumber::control(float value)
        {
            if (this->parent_ == nullptr)
            {
                ESP_LOGW(TAG, "PI18 number has no parent");
                return;
            }

            char payload[32];
            float set_value = value;

            switch (this->kind_)
            {
            case NUMBER_BATTERY_CUTOFF_VOLTAGE:
                set_value = std::clamp(value, 40.0f, 48.0f);
                std::snprintf(payload, sizeof(payload), "PSDV%03u",
                              static_cast<unsigned>(std::lroundf(set_value * 10.0f)));
                break;
            case NUMBER_BATTERY_MAX_CHARGE_VOLTAGE:
            {
                set_value = std::clamp(value, 48.0f, 58.4f);
                const float float_voltage = this->parent_->has_battery_float_voltage()
                                                ? this->parent_->battery_float_voltage()
                                                : set_value;
                std::snprintf(payload, sizeof(payload), "MCHGV%03u,%03u",
                              static_cast<unsigned>(std::lroundf(set_value * 10.0f)),
                              static_cast<unsigned>(std::lroundf(float_voltage * 10.0f)));
                break;
            }
            case NUMBER_BATTERY_FLOAT_VOLTAGE:
            {
                set_value = std::clamp(value, 48.0f, 58.4f);
                const float max_charge_voltage = this->parent_->has_battery_max_charge_voltage()
                                                     ? this->parent_->battery_max_charge_voltage()
                                                     : set_value;
                std::snprintf(payload, sizeof(payload), "MCHGV%03u,%03u",
                              static_cast<unsigned>(std::lroundf(max_charge_voltage * 10.0f)),
                              static_cast<unsigned>(std::lroundf(set_value * 10.0f)));
                break;
            }
            case NUMBER_BATTERY_RECHARGE_VOLTAGE:
            {
                set_value = std::clamp(value, 44.0f, 51.0f);
                const float redischarge_voltage = this->parent_->has_battery_redischarge_voltage()
                                                      ? this->parent_->battery_redischarge_voltage()
                                                      : set_value;
                std::snprintf(payload, sizeof(payload), "BUCD%03u,%03u",
                              static_cast<unsigned>(std::lroundf(set_value * 10.0f)),
                              static_cast<unsigned>(std::lroundf(redischarge_voltage * 10.0f)));
                break;
            }
            case NUMBER_BATTERY_REDISCHARGE_VOLTAGE:
            {
                set_value = std::clamp(value, 44.0f, 51.0f);
                const float recharge_voltage = this->parent_->has_battery_recharge_voltage()
                                                   ? this->parent_->battery_recharge_voltage()
                                                   : set_value;
                std::snprintf(payload, sizeof(payload), "BUCD%03u,%03u",
                              static_cast<unsigned>(std::lroundf(recharge_voltage * 10.0f)),
                              static_cast<unsigned>(std::lroundf(set_value * 10.0f)));
                break;
            }
            case NUMBER_MAX_AC_CHARGING_CURRENT:
                set_value = std::clamp(value, 0.0f, 9.0f);
                std::snprintf(payload, sizeof(payload), "MUCHGC0,%03u",
                              static_cast<unsigned>(std::lroundf(set_value)));
                break;
            case NUMBER_MAX_CHARGING_CURRENT:
                set_value = std::clamp(value, 0.0f, 9.0f);
                std::snprintf(payload, sizeof(payload), "MCHGC0,%03u", static_cast<unsigned>(std::lroundf(set_value)));
                break;
            default:
                ESP_LOGW(TAG, "Unknown PI18 number kind %u", static_cast<unsigned>(this->kind_));
                return;
            }

            std::string response;
            if (!this->parent_->send_protocol_command('S', payload, &response, 1000) || !accepted_response(response))
            {
                ESP_LOGW(TAG, "Failed to set PI18 number");
                return;
            }

            this->publish_state(set_value);
            switch (this->kind_)
            {
            case NUMBER_BATTERY_MAX_CHARGE_VOLTAGE:
                this->parent_->set_battery_max_charge_voltage(set_value);
                break;
            case NUMBER_BATTERY_FLOAT_VOLTAGE:
                this->parent_->set_battery_float_voltage(set_value);
                break;
            case NUMBER_BATTERY_RECHARGE_VOLTAGE:
                this->parent_->set_battery_recharge_voltage(set_value);
                break;
            case NUMBER_BATTERY_REDISCHARGE_VOLTAGE:
                this->parent_->set_battery_redischarge_voltage(set_value);
                break;
            default:
                break;
            }
        }

        void PI18SettingSwitch::write_state(bool state)
        {
            if (this->parent_ == nullptr)
            {
                ESP_LOGW(TAG, "PI18 switch has no parent");
                return;
            }

            char payload[16];
            const char *label = nullptr;

            switch (this->kind_)
            {
            case SWITCH_LOAD_POWER:
                std::snprintf(payload, sizeof(payload), "LON%u", state ? 1U : 0U);
                label = "load power";
                break;
            case SWITCH_SILENCE_BUZZER:
                std::snprintf(payload, sizeof(payload), "P%cA", state ? 'E' : 'D');
                label = "silence buzzer";
                break;
            case SWITCH_OVERLOAD_BYPASS:
                std::snprintf(payload, sizeof(payload), "P%cB", state ? 'E' : 'D');
                label = "overload bypass";
                break;
            case SWITCH_LCD_ESCAPE:
                std::snprintf(payload, sizeof(payload), "P%cC", state ? 'E' : 'D');
                label = "LCD default page escape";
                break;
            case SWITCH_OVERLOAD_RESTART:
                std::snprintf(payload, sizeof(payload), "P%cD", state ? 'E' : 'D');
                label = "overload restart";
                break;
            case SWITCH_OVER_TEMP_RESTART:
                std::snprintf(payload, sizeof(payload), "P%cE", state ? 'E' : 'D');
                label = "over temperature restart";
                break;
            case SWITCH_BACKLIGHT:
                std::snprintf(payload, sizeof(payload), "P%cF", state ? 'E' : 'D');
                label = "backlight";
                break;
            case SWITCH_ALARM_PRIMARY_SOURCE_INTERRUPT:
                std::snprintf(payload, sizeof(payload), "P%cG", state ? 'E' : 'D');
                label = "primary source alarm";
                break;
            case SWITCH_FAULT_CODE_RECORD:
                std::snprintf(payload, sizeof(payload), "P%cH", state ? 'E' : 'D');
                label = "fault code record";
                break;
            default:
                ESP_LOGW(TAG, "Unknown PI18 switch kind %u", static_cast<unsigned>(this->kind_));
                return;
            }

            std::string response;
            if (!this->parent_->send_protocol_command('S', payload, &response, 1000) || !accepted_response(response))
            {
                ESP_LOGW(TAG, "Failed to set %s", label != nullptr ? label : "PI18 switch");
                return;
            }

            this->publish_state(state);
        }

        void PI18DebugButton::press_action()
        {
            if (this->parent_ == nullptr)
            {
                ESP_LOGW(TAG, "PI18 button has no parent");
                return;
            }

            switch (this->kind_)
            {
            case BUTTON_FLUSH_UART:
            {
                const size_t discarded = this->parent_->flush_uart_rx();
                char buffer[32];
                std::snprintf(buffer, sizeof(buffer), "Flushed %u bytes", static_cast<unsigned>(discarded));
                ESP_LOGD(TAG, "%s", buffer);
                this->parent_->publish_manual_response(buffer);
                break;
            }
            case BUTTON_READ_UART:
            {
                std::string frame;
                if (!this->parent_->read_uart_frame(frame, 1000))
                {
                    ESP_LOGD(TAG, "UART read button timed out");
                    this->parent_->publish_manual_response("No UART frame");
                    break;
                }

                if (frame.size() >= 3)
                    frame.resize(frame.size() - 3);
                ESP_LOGD(TAG, "UART frame: %s", frame.c_str());
                this->parent_->publish_manual_response(frame);
                break;
            }
            case BUTTON_SYNC_CONFIGURATION:
            {
                const bool ok = this->parent_->sync_configuration();
                ESP_LOGI(TAG, "PI18 sync configuration %s", ok ? "completed" : "failed");
                if (this->parent_->has_manual_response_text_sensor())
                    this->parent_->publish_manual_response(ok ? "Sync complete" : "Sync failed");
                break;
            }
            default:
                ESP_LOGW(TAG, "Unknown PI18 button kind %u", static_cast<unsigned>(this->kind_));
                return;
            }
        }

        void PI18PollingSwitch::setup()
        {
            if (this->parent_ == nullptr)
            {
                ESP_LOGW(TAG, "PI18 polling switch has no parent");
                return;
            }

            this->write_state(this->get_initial_state_with_restore_mode().value_or(true));
        }

        void PI18PollingSwitch::dump_config()
        {
            LOG_SWITCH("", "PI18 Polling Enabled", this);
        }

        void PI18PollingSwitch::write_state(bool state)
        {
            if (this->parent_ == nullptr)
            {
                ESP_LOGW(TAG, "PI18 polling switch has no parent");
                return;
            }

            this->parent_->set_polling_enabled(state);
            this->publish_state(state);
        }

        void PI18Component::publish_mode_(uint8_t code)
        {
            if (!this->mode_text_)
                return;

            const char *text = "?";
            // PI18 MOD map per protocol doc:
            // 00 Power on, 01 Standby, 02 Bypass, 03 Battery, 04 Fault, 05 Hybrid/Line.
            switch (code)
            {
            case 0:
                text = "PowerOn";
                break;
            case 1:
                text = "Standby";
                break;
            case 2:
                text = "Bypass";
                break;
            case 3:
                text = "Battery";
                break;
            case 4:
                text = "Fault";
                break;
            case 5:
                text = "Line";
                break;
            default:
                text = "Unknown";
                break;
            }

            this->mode_text_->publish_state(text);
        }

        uint16_t PI18Component::crc16_pi18_(const uint8_t *data, size_t len)
        {
            uint16_t crc = 0x0000;
            const uint16_t poly = 0x1021;

            for (size_t i = 0; i < len; i++)
            {
                crc ^= static_cast<uint16_t>(data[i]) << 8;
                for (int b = 0; b < 8; b++)
                {
                    if (crc & 0x8000)
                        crc = static_cast<uint16_t>((crc << 1) ^ poly);
                    else
                        crc <<= 1;
                }
            }

            return crc;
        }

        void PI18CommandText::dump_config()
        {
            LOG_TEXT("", "PI18 Manual Command", this);
        }

        void PI18CommandText::control(const std::string &value)
        {
            if (this->parent_ == nullptr)
            {
                ESP_LOGW(TAG, "Manual command text has no PI18 parent");
                return;
            }

            std::string command = normalize_manual_command(value);
            if (command.empty())
            {
                ESP_LOGW(TAG, "Manual command is empty");
                return;
            }

            if (this->parent_->has_manual_response_text_sensor())
            {
                std::string response;
                if (!this->parent_->send_manual_command(command, &response, 1000))
                {
                    ESP_LOGW(TAG, "Failed to send manual command: %s", command.c_str());
                    this->parent_->publish_manual_response("Send failed");
                    return;
                }

                if (response.size() >= 3)
                {
                    response.resize(response.size() - 3);
                    this->parent_->publish_manual_response(response);
                }
                else
                {
                    this->parent_->publish_manual_response("No response");
                }
            }
            else if (!this->parent_->send_manual_command(command))
            {
                ESP_LOGW(TAG, "Failed to send manual command: %s", command.c_str());
                return;
            }

            this->publish_state(command);
        }

    }  // namespace pi18
}  // namespace esphome
