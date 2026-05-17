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
#include "topic_pool.h"
#include "topics.hpp"

#include <cstdint>
#include <atomic>

class InfraredModule {
    public:
        static constexpr uint16_t BUFFER_SIZE = 4;
        static constexpr uint32_t TX_MAX_TIMEOUT = 1000;
        static constexpr const char *infrared_msg_topic = "infrared_msg_";

        enum class state {
            NOT_INITIALIZED,        // 未初始化
            READY_TO_RECEIVE_DATA,  // 正在等待接收数据
            AWAITING_FOR_ACK,       // 刚刚发送了指令，等待模块应答
            ACK_SUCCESS,            // 收到模块应答，且应答正确
            ACK_ERROR               // 收到模块应答，但应答错误
        };

        InfraredModule(UART_HandleTypeDef &huart, DMA_HandleTypeDef &hdma) : hal_huart_(huart), hal_hdma_(hdma) {}
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
         * @brief HAL_UARTEx_RxEventCallback 回调处理函数
         * @param event_huart 事件发生的串口句柄
         * @param event_size 事件的数据长度
         */
        void rxEventCallbackHandler(UART_HandleTypeDef *event_huart, uint16_t event_size) {
            if (event_huart->Instance == hal_huart_.Instance) {
                if (current_state_.load(std::memory_order_acquire) == state::READY_TO_RECEIVE_DATA) {
                    // 把接收到的数据发送出去
                    infrared_msg_.address1 = rx_buffer_[0];
                    infrared_msg_.address2 = rx_buffer_[1];
                    infrared_msg_.data = rx_buffer_[2];
                    infrared_pub_.Publish(infrared_msg_);

                } else if (current_state_.load(std::memory_order_acquire) == state::AWAITING_FOR_ACK) {
                    // 处理接收到的ACK数据
                    if (rx_buffer_[0] == ACK_CODE) {
                        changeStateTo(state::ACK_SUCCESS);
                    } else {
                        changeStateTo(state::ACK_ERROR);
                    }
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

            status = HAL_UART_Transmit(&hal_huart_, buffer, sizeof(buffer), timeout);
            if (status != HAL_OK) return status;

            // 等待遥控模块应答
            changeStateTo(state::AWAITING_FOR_ACK);

            while (osKernelGetTickCount() - start_time < timeout) {
                // 此时状态是 AWAITING_FOR_ACK，此时状态由 rxEventCallbackHandler 接管

                if (current_state_.load(std::memory_order_acquire) == state::ACK_SUCCESS) {
                    changeStateTo(state::READY_TO_RECEIVE_DATA);
                    return HAL_OK;
                } else if (current_state_.load(std::memory_order_acquire) == state::ACK_ERROR) {
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

        UART_HandleTypeDef &hal_huart_;
        DMA_HandleTypeDef &hal_hdma_;

        std::atomic<state> current_state_{state::NOT_INITIALIZED};

        uint8_t rx_buffer_[BUFFER_SIZE] = {0};

        TypedTopicPublisher<pub_infrared_msg> infrared_pub_{infrared_msg_topic};
        pub_infrared_msg infrared_msg_{};

        HAL_StatusTypeDef beginDMAReceive(uint16_t size) {
            // 清空接收缓冲区
            for (uint8_t i = 0; i < BUFFER_SIZE; ++i) {
                rx_buffer_[i] = 0;
            }
            HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&hal_huart_, rx_buffer_, size);
            __HAL_DMA_DISABLE_IT(&hal_hdma_, DMA_IT_HT); // 关闭DMA传输过半中断
            return status;
        }

        HAL_StatusTypeDef stopDMAReceive() {
            return HAL_UART_DMAStop(&hal_huart_);
        }

        void changeStateTo(state new_state) {
            HAL_StatusTypeDef status;
            switch (new_state) {
                case state::NOT_INITIALIZED:

                    status = stopDMAReceive();
                    if (status != HAL_OK) return;

                    break;

                case state::READY_TO_RECEIVE_DATA:
                    
                    status = stopDMAReceive();
                    if (status != HAL_OK) return;

                    status = beginDMAReceive(BUFFER_SIZE);
                    if (status != HAL_OK) return;

                    break;

                case state::AWAITING_FOR_ACK:

                    status = stopDMAReceive();
                    if (status != HAL_OK) return;

                    status = beginDMAReceive(1);
                    if (status != HAL_OK) return;

                    break;

                default:
                    break;

            }
            current_state_.store(new_state, std::memory_order_release);
        }

};