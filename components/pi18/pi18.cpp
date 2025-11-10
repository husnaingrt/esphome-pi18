#include "pi18.h"
#include "esphome/core/helpers.h"

namespace esphome
{
    namespace pi18
    {

        static const char *const TAG = "pi18";

        void PI18Component::setup()
        {
            ESP_LOGI(TAG, "PI18 driver init (UART %u baud)", this->parent_->get_baud_rate());
        }

        void PI18Component::dump_config()
        {
            ESP_LOGCONFIG(TAG, "PI18 Inverter (PI18 protocol)");
        }

        void PI18Component::update()
        {
            // 1) MOD (mode)
            {
                std::string cmd = build_command_("MOD");
                this->write_array((const uint8_t *)cmd.data(), cmd.size());
                std::string line;
                if (read_line_(line, 500))
                {
                    // Expect: ^D<LLL><digits><CRC><CR>
                    auto dpos = line.find("^D");
                    if (dpos != std::string::npos && line.size() > dpos + 5)
                    {
                        size_t pos = dpos + 5; // after "^D" + length
                        // read contiguous digits as the mode code
                        size_t end = pos;
                        while (end < line.size() && isdigit((unsigned char)line[end]))
                            end++;
                        if (end > pos)
                        {
                            uint8_t mode = (uint8_t)atoi(line.substr(pos, end - pos).c_str());
                            publish_mode_(mode);
                        }
                    }
                }
            }

            // 2) GS (general status)
            {
                std::string cmd = build_command_("GS");
                this->write_array((const uint8_t *)cmd.data(), cmd.size());
                std::string line;
                if (read_line_(line, 800))
                {
                    // Expect: ^D<LLL><CSV payload><CRC><CR>  (LLL = 3 digits)
                    auto dpos = line.find("^D");
                    if (dpos != std::string::npos && line.size() > dpos + 5)
                    {
                        size_t csv_start = dpos + 5; // skip "^D" + 3 length digits
                        size_t csv_end = line.find('\r', csv_start);
                        std::string csv = (csv_end == std::string::npos)
                                              ? line.substr(csv_start)
                                              : line.substr(csv_start, csv_end - csv_start);

                        // strip anything after the CSV that isn't digits or commas (removes CRC bytes, if any)
                        size_t cut = csv.find_first_not_of("0123456789,");
                        if (cut != std::string::npos)
                            csv.resize(cut);

                        parse_gs_(csv);
                    }
                }
            }
        }

        std::string PI18Component::build_command_(const std::string &cmd)
        {
            // Make payload "^Pnnn<cmd><crc><cr>" where nnn includes crc+cr
            // Length = cmd.size() + 3
            uint16_t len = (uint16_t)(cmd.size() + 3);
            char header[7];
            snprintf(header, sizeof(header), "^P%03u", (unsigned)len);
            std::string out(header);
            out += cmd;
            uint16_t crc = crc16_pi18_((const uint8_t *)out.data(), out.size());
            out.push_back((char)((crc >> 8) & 0xFF));
            out.push_back((char)(crc & 0xFF));
            out.push_back('\r');
            return out;
        }

        // Read until CR or timeout
        bool PI18Component::read_line_(std::string &out, uint32_t timeout_ms)
        {
            uint32_t start = millis();
            out.clear();
            while (millis() - start < timeout_ms)
            {
                uint8_t ch;
                if (this->available() && this->read_byte(&ch))
                {
                    out.push_back((char)ch);
                    if (ch == '\r')
                        return true;
                }
                else
                {
                    delay(5);
                }
            }
            return !out.empty();
        }

        // Parse GS CSV fields and publish a subset of sensors
        bool PI18Component::parse_gs_(const std::string &payload)
        {
            // Split by commas
            std::vector<std::string> f;
            f.reserve(30);
            size_t start = 0, pos;
            while ((pos = payload.find(',', start)) != std::string::npos)
            {
                f.push_back(payload.substr(start, pos - start));
                start = pos + 1;
            }
            if (start < payload.size())
                f.push_back(payload.substr(start));

            if (f.size() < 28)
            {
                ESP_LOGW(TAG, "Unexpected GS field count: %u", (unsigned)f.size());
                return false;
            }

            ESP_LOGD(TAG, "GS[0..5]=%s,%s,%s,%s,%s,%s",
                     f.size() > 0 ? f[0].c_str() : "", f.size() > 1 ? f[1].c_str() : "",
                     f.size() > 2 ? f[2].c_str() : "", f.size() > 3 ? f[3].c_str() : "",
                     f.size() > 4 ? f[4].c_str() : "", f.size() > 5 ? f[5].c_str() : "");

            auto to_float = [](const std::string &s) -> float
            { return s.empty() ? NAN : atof(s.c_str()); };

            // Field mapping per PI18 spec:
            // 0 AAAA grid V(0.1)   1 BBB freq(0.1)  2 CCCC out V(0.1)  3 DDD out Hz(0.1)
            // 4 EEEE VA            5 FFFF W         6 GGG load %      7 HHH batt V(0.1)
            // 8 III batt V from SCC  9 JJJ batt V SCC2
            // 10 KKK batt discharge A   11 LLL batt charge A
            // 12 MMM batt %        13 NNN heatsink °C
            // 16 QQQQ PV1 power W  18 SSSS PV1 voltage(0.1)
            if (grid_voltage_)
                grid_voltage_->publish_state(to_float(f[0]) / 10.0f);
            if (grid_frequency_)
                grid_frequency_->publish_state(to_float(f[1]) / 10.0f);
            if (ac_output_voltage_)
                ac_output_voltage_->publish_state(to_float(f[2]) / 10.0f);
            if (ac_output_frequency_)
                ac_output_frequency_->publish_state(to_float(f[3]) / 10.0f);
            if (output_apparent_power_)
                output_apparent_power_->publish_state(to_float(f[4]));
            if (output_active_power_)
                output_active_power_->publish_state(to_float(f[5]));
            if (load_percent_)
                load_percent_->publish_state(to_float(f[6]));
            if (battery_voltage_)
                battery_voltage_->publish_state(to_float(f[7]) / 10.0f);
            if (battery_discharge_current_)
                battery_discharge_current_->publish_state(to_float(f[10]));
            if (battery_charge_current_)
                battery_charge_current_->publish_state(to_float(f[11]));
            if (battery_capacity_)
                battery_capacity_->publish_state(to_float(f[12]));
            if (heatsink_temperature_)
                heatsink_temperature_->publish_state(to_float(f[13]));
            if (pv1_power_)
                pv1_power_->publish_state(to_float(f[16]));
            if (pv1_voltage_)
                pv1_voltage_->publish_state(to_float(f[18]) / 10.0f);

            return true;
        }

        void PI18Component::publish_mode_(uint8_t code)
        {
            if (!mode_text_)
                return;
            const char *text = "?";
            // PI18 MOD map per protocol doc:
            // 00 Power on, 01 Standby, 02 Bypass, 03 Battery, 04 Fault, 05 Hybrid (Line), etc.
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
            mode_text_->publish_state(text);
        }

        // --- CRC ---
        // Many community tools implement PI18 with a CRC-16 variant. We use the widely referenced 0x1021 polynomial,
        // XMODEM-style (init=0x0000), which matches public PI18 implementations. If your unit differs, adjust here.
        uint16_t PI18Component::crc16_pi18_(const uint8_t *data, size_t len)
        {
            uint16_t crc = 0x0000;
            const uint16_t poly = 0x1021;
            for (size_t i = 0; i < len; i++)
            {
                crc ^= (uint16_t)data[i] << 8;
                for (int b = 0; b < 8; b++)
                {
                    if (crc & 0x8000)
                        crc = (crc << 1) ^ poly;
                    else
                        crc <<= 1;
                }
            }
            return crc;
        }

    } // namespace pi18
} // namespace esphome
