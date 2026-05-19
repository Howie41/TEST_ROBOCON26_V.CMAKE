/**
 * @file infrared_com.hpp
 * @author zhy (Howie41)
 * @brief 红外通信模块
 * @date 2026-05-16
 *
 * @note 模块使用了CMSIS-OS API，务必在RTOS内核后启动后再发送数据！
 * @note 模块波特率默认为9600bps
 */

#pragma once
#include "cmsis_os2.h"
#include "main.h"
#include "UartPort.hpp"
#include "stm32h7xx_hal_uart.h"
#include "topic_pool.h"
#include "topics.hpp"
#include "usart.h"

#include <cstddef>
#include <cstdint>
#include <atomic>

class InfraredModule {
    public:
        static constexpr uint16_t BUFFER_SIZE = 4;
        static constexpr uint32_t TX_MAX_TIMEOUT = 1000;
        static constexpr const char *INFRARED_MSG_TOPIC = "infrared_msg";

        enum class state {
            NOT_INITIALIZED,        // 未初始化
            READY_TO_RECEIVE_DATA,  // 正在等待接收数据
            AWAITING_FOR_ACK,       // 刚刚发送了指令，等待模块应答
            ACK_SUCCESS,            // 收到模块应答，且应答正确
            ACK_ERROR               // 收到模块应答，但应答错误
        };

        explicit InfraredModule(UartPort &uart_port) : uart_port_(uart_port) {}
        ~InfraredModule() = default;

        /**
         * @brief 初始化，开启DMA接收
         * @return 开启DMA接收的状态码
         */
        HAL_StatusTypeDef init() {
            changeStateTo(state::READY_TO_RECEIVE_DATA);
            return HAL_OK;
        }

        /**
         * @brief 由 UartPort 的接收回调转发进来
         * @param data 新收到的数据片段
         * @param len 数据长度
         */
        void UartPortRxCbHandler(const uint8_t *data, size_t len) {
            if (data == nullptr || len == 0) return;

            if (current_state_.load() == state::READY_TO_RECEIVE_DATA) {
                if (len < 3) return;

                // 红外模块正常帧：地址1 + 地址2 + data
                infrared_msg_.address1 = data[0];
                infrared_msg_.address2 = data[1];
                infrared_msg_.data = data[2];
                infrared_pub_.Publish(infrared_msg_);
                return;
            }

            if (current_state_.load() == state::AWAITING_FOR_ACK) {
                // ACK只有一个字节
                if (data[0] == ACK_CODE) {
                    changeStateTo(state::ACK_SUCCESS);
                } else {
                    changeStateTo(state::ACK_ERROR);
                }
            }
        }
        
        /**
         * @brief 发送一帧红外数据
         * @param address1 地址1
         * @param address2 地址2
         * @param data 数据
         * @note 地址1和地址2类似ID一样，区分不同设备，应该和接收方约定使用一样的地址
         * @note 似乎是不会像CAN那样根据ID过滤消息的，所以两个地址也可以作为数据载荷的一部分（？
         * @return 状态码，成功发送指令且收到模块应答反馈才会算成功
         */
        HAL_StatusTypeDef emitData(uint8_t address1, uint8_t address2, uint8_t data, uint32_t timeout = TX_MAX_TIMEOUT) {
            HAL_StatusTypeDef status;
            uint32_t start_time = osKernelGetTickCount();

            uint8_t buffer[5] = {0};
            buffer[0] = 0xFA;
            buffer[1] = 0xF1;
            buffer[2] = address1;
            buffer[3] = address2;
            buffer[4] = data;

            status = uart_port_.write(buffer, sizeof(buffer), timeout);
            if (status != HAL_OK) return status;

            // 等待遥控模块应答
            changeStateTo(state::AWAITING_FOR_ACK);

            while (osKernelGetTickCount() - start_time < timeout) {
                // 此时状态是 AWAITING_FOR_ACK，此时状态由 UartPortRxCbHandler 接管

                if (current_state_.load() == state::ACK_SUCCESS) {
                    changeStateTo(state::READY_TO_RECEIVE_DATA);
                    return HAL_OK;
                } else if (current_state_.load() == state::ACK_ERROR) {
                    changeStateTo(state::READY_TO_RECEIVE_DATA);
                    return HAL_ERROR;
                }
                osDelay(1);
            }
            // 超时没有应答 恢复正常的收信息状态
            changeStateTo(state::READY_TO_RECEIVE_DATA);
            return HAL_TIMEOUT;
        }

    private:
        static constexpr uint8_t ACK_CODE = 0xF1;

        UartPort &uart_port_;

        std::atomic<state> current_state_{state::NOT_INITIALIZED};

        TypedTopicPublisher<pub_infrared_msg> infrared_pub_{INFRARED_MSG_TOPIC};
        pub_infrared_msg infrared_msg_{};

        void changeStateTo(state new_state) {
            if (new_state == state::NOT_INITIALIZED) {
                return;
            }

            current_state_.store(new_state);
        }

};