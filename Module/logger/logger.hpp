/**
 * @file logger.hpp
 * @author zhy (Howie41)
 * @brief 
 * @date 2026-05-23
 */
#include <cstdarg>
#include <cstdio>

#include "UartPort.hpp"

class Logger {
    public:
        static constexpr size_t BUFFER_LENGTH = 256;

        Logger(UartPort &uart) : uart_(uart) {}
        ~Logger() = default;

        void log(const char *format, ...) {
            char buffer[BUFFER_LENGTH];
            va_list args;
            va_start(args, format);
            int len = vsnprintf(buffer, sizeof(buffer), format, args);
            va_end(args);
            if (len > 0) {
                size_t write_len = static_cast<size_t>(len);
                if (write_len >= sizeof(buffer)) {
                    write_len = sizeof(buffer) - 1;
                }
                uart_.writeDma(reinterpret_cast<const uint8_t *>(buffer), write_len);
            }
        }

        void log_priority(uint8_t priority, const char *format, ...) {
            char buffer[BUFFER_LENGTH];
            va_list args;
            va_start(args, format);
            int len = vsnprintf(buffer, sizeof(buffer), format, args);
            va_end(args);
            if (len > 0 && priority >= current_priority_) {
                size_t write_len = static_cast<size_t>(len);
                if (write_len >= sizeof(buffer)) {
                    write_len = sizeof(buffer) - 1;
                }
                uart_.writeDma(reinterpret_cast<const uint8_t *>(buffer), write_len);
            }
        }

        void set_priority(uint8_t priority) {
            current_priority_ = priority;
        }
    private:
        UartPort &uart_;
        uint8_t current_priority_{255}; // 默认最高优先级
};