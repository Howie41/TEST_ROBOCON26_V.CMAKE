/**
 * @file Motor.hpp
 * @author Keten (2863861004@qq.com)，大帅将军
 * @brief 电机基类
 * @version 0.1
 * @date 2026-04-18
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :因为达妙电机自带mit控制板，只需要发送目标值，电机板会自己根据当前状态进行闭环控制
 *             所以现在当电机状态切换（模式切换）和目标值变化时，才会调用电机发送can帧，这和大疆电机不同，后者是一直在发送can帧的，
 *             这样可以一定程度上减少can总线的负载，相应的，你只发一针电机也只返回一针，所以无法得知电机当前状态，就连电机任务是否完成都不知道
 *             如果你需要得到电机状态，只需要去cmakelist文件把DM_MOTOR_SINGLE这个宏注释掉就行了
 *             还有一件事，我把达妙电机修改成可以在运动过程中切换模式了，我的发一针可以作为一个保险，例如posWithSpeedControl(1,1)切换到speedControl(1)，是不会发送的因为速度的目标值没有变，
 *             你需要切换到speedControl(0)再切换回speedControl(1)，这样就会发送can帧了，当然如果你不需要切换模式，那就无所谓了，你也可以理解为这是一坨屎一样的代码逻辑，可能你满脑子都在何意味，但是就这样吧
 * @note :
 * @versioninfo :
 */

#pragma once

#include "Canbus.hpp"
#include <cmath>
#include <stdint.h>

#define RAD_2_DEGREE 57.2957795f       // 180/pi  弧度转换成角度
#define DEGREE_2_RAD 0.01745329252f    // pi/180  角度转化成弧度
#define RPM_2_ANGLE_PER_SEC 6.0f       // 360/60,转每分钟 转化 度每秒
#define RPM_2_RAD_PER_SEC 0.104719755f // ×2pi/60sec,转每分钟 转化 弧度每秒

class C610Motor;

class MotorBase {
public:
    MotorBase() = default;

    // 对外接口
    /**
     * @brief 设置电机输出命令，并按最大允许值自动限幅。
     * @param cmd 目标命令值。
     */
    void setMotorCmd(float cmd) {
        if (cmd > max_cmd_)
            cmd = max_cmd_;
        if (cmd < -max_cmd_)
            cmd = -max_cmd_;
        cmd_ = cmd;
    }

    /**
     * @brief 设置电机减速比。
     * @param config 减速比配置值。
     */
    void setMotorReduction(const float config) { reduction_ratio_ = config; }

    /**
     * @brief 设置命令值的绝对值上限。
     * @param config 最大命令值。
     */
    void setMaxCmd(const float config) { max_cmd_ = config; }

    /**
     * @brief 获取当前输出端单圈位置。
     * @return 单圈位置。
     */
    float getCurrentSinglePos(void) const { return single_pos_; }

    /**
     * @brief 获取当前输出端累计位置。
     * @return 累计位置。
     */
    float getCurrentSumPos(void) const { return sum_pos_; }

    /**
     * @brief 获取当前输出端速度。
     * @return 速度值。
     */
    float getCurrentSpeed(void) const { return speed_; }

    /**
     * @brief 获取当前输出端力矩。
     * @return 力矩值。
     */
    float getCurrentTorque(void) const { return torque_; }

    /**
     * @brief 获取当前电机温度。
     * @return 温度值，单位摄氏度。
     */
    float getCurrentTemperature(void) const { return temperature_; }

    /**
     * @brief 获取转子侧单圈位置原始值。
     * @return 原始单圈位置。
     */
    float getRawCurrentSinglePos(void) const { return raw_single_pos_; }

    /**
     * @brief 获取转子侧累计位置原始值。
     * @return 原始累计位置。
     */
    float getRawCurrentSumPos(void) const { return raw_sum_pos_; }

    /**
     * @brief 获取转子侧速度原始值。
     * @return 原始速度。
     */
    float getRawCurrentSpeed(void) const { return raw_speed_; }

    /**
     * @brief 获取转子侧力矩原始值。
     * @return 原始力矩。
     */
    float getRawCurrentTorque(void) const { return raw_torque_; }

    // 电机最原始output指令(速度/位置/电流)
    float cmd_;
    float max_cmd_{99999};

    // 电机减速比
    float reduction_ratio_{1};

    // 电机状态量(最原始)
    float raw_single_pos_{0}; // 单圈位置
  float raw_sum_pos_{0};    // 多圈累加
  float raw_speed_{0};      // 速度
  float raw_torque_{0};     // 力矩

    // 配置减速比(转子->输出端)
  float single_pos_{0};  // 单圈位置
  float sum_pos_{0};     // 多圈累加
  float speed_{0};       // 速度
  float torque_{0};      // 力矩
    float temperature_{0}; // 温度
};

enum DJIMotorCanGroup {
    GROUP1 = 0, // 0x1FF
    GROUP2      // 0x200
};

// 打包大疆电机can帧 - 聚合 4 个电机的命令到单个 8 字节帧
// tx_id=0x200 对应 0x201-0x204; tx_id=0x1FF 对应 0x205-0x208
// 调用方负责提取电机ID和命令值，函数只负责数据聚合
/**
 * @brief 将大疆电机命令聚合到单个 CAN 帧数据中。
 * @param tx_id 目标发送 ID，决定当前帧对应的 4 路槽位。
 * @param motor_ids 电机 ID 列表。
 * @param commands 与 motor_ids 对应的命令值。
 * @param motor_count 电机数量。
 * @param data 输出数据区，长度至少 8 字节。
 * @param len 输出帧长度，固定为 8。
 */
void packDJIMotorCanMsg(uint32_t tx_id, const uint32_t motor_ids[],
                                                const int16_t commands[], uint8_t motor_count,
                                                uint8_t data[8], uint8_t &len);

class C610Motor : public CanDevice, public MotorBase {
public:
    /**
     * @brief 构造一个 C610 电机对象。
     * @param manager CAN 总线管理器。
     * @param id 设备接收 ID。
     * @param is_extid 接收 ID 是否为扩展帧。
     * @param tx_id 设备发送 ID。
     * @param tx_is_extid 发送 ID 是否为扩展帧。
     */
    C610Motor(CanBus *manager, uint32_t id, bool is_extid, uint32_t tx_id,
                        bool tx_is_extid)
            : CanDevice(manager, id, is_extid, tx_id, tx_is_extid) {}

    /**
     * @brief 初始化减速比和命令上限。
     * @param reduction_ratio 输出端减速比。
     * @param max_cmd 允许的最大命令值。
     */
    void init(float reduction_ratio = 36, float max_cmd = 10000.f) {
        setMotorReduction(reduction_ratio);
        setMaxCmd(max_cmd);
    }

    /**
     * @brief 解析 C610 电机上报的反馈数据。
     * @param data 原始 8 字节 CAN 数据。
     * @param len 数据长度。
     */
    void onRx(const uint8_t data[8], uint8_t len) override {
        if (len < 8)
            return;

        // byte 0-1: 编码器(单圈位置 0-8191)
        encoder_ = (uint16_t)(data[0] << 8 | data[1]);
        if (is_encoder_init) {
            int16_t delta_encoder = encoder_ - last_encoder_;
            // 消圈：大于4096则是反向一圈，小于-4096则是正向一圈
            if (delta_encoder < -4096)
                round_cnt_++;
            else if (delta_encoder > 4096)
                round_cnt_--;
            // 累计编码值
            int32_t total_encoder = round_cnt_ * 8192 + encoder_ - encoder_offset_;

            // 转子侧位置
            raw_single_pos_ = static_cast<float>(encoder_) / encoder_angle_ratio_;
            raw_sum_pos_ = static_cast<float>(total_encoder) / encoder_angle_ratio_;

            // 输出端位置（考虑减速比）
            single_pos_ = raw_single_pos_ / reduction_ratio_;
            sum_pos_ = raw_sum_pos_ / reduction_ratio_;
            single_pos_ = std::fmod(sum_pos_, 360.0f);
            if (single_pos_ < 0.0f)
                single_pos_ += 360.0f;
        } else {
            encoder_offset_ = encoder_;
            is_encoder_init = true;
        }
        last_encoder_ = encoder_;

        // byte 2-3: 转子速度 (int16, 单位: dps/min，取决于驱动器)
        int16_t raw_speed = (int16_t)((data[2] << 8) | data[3]);
        raw_speed_ = static_cast<float>(raw_speed) * RPM_2_RAD_PER_SEC;
        speed_ = raw_speed_ / reduction_ratio_;

        // byte 4-5: 实际电流 (int16, 单位: 0.1A, 范围 -2000~2000 = -200~200A)
        // 电流(A) = value * 0.1
        int16_t raw_current = (int16_t)((data[4] << 8) | data[5]);
        raw_torque_ = static_cast<float>(raw_current) * 0.1f;          // A
        torque_ = raw_torque_ * current_to_torque_ * reduction_ratio_; // 输出端力矩

        // byte 6: 电机温度 (uint8, 单位: °C)
        temperature_ = static_cast<float>(data[6]);
    }

    // buildTx 返回自己的 int16 命令，不发送整个帧
    // 聚合由应用层的 packDJIMotorCanMsg() 负责

    /**
     * @brief C610 不单独组帧，交由上层聚合发送。
     * @param data 发送缓冲区。
     * @param len 输出长度。
     * @return 始终返回 false，表示不生成独立帧。
     */
    bool buildTx(uint8_t data[8], uint8_t &len) override {
        // C610 不单独发帧，返回 false
        len = 0;
        return false;
    }

    /**
     * @brief 将内部命令换算为驱动器使用的原始指令值。
     * @return 原始命令值。
     */
    float cmdTrans() { return cmd_ * (10000.f / 10000.0f); }

private:
    // 编码器相关
    bool is_encoder_init{false};
    uint16_t encoder_offset_{0};
    uint16_t encoder_{0};
    uint16_t last_encoder_{0};
    int32_t round_cnt_{0};
    float encoder_angle_ratio_ = 8192.0f / 360.0f;

    // 电流转力矩
    float current_to_torque_{0.0f}; // M3508: 0.2 Nm/A
};

class C620Motor : public CanDevice, public MotorBase {
public:
    /**
     * @brief 构造一个 C620 电机对象。
     * @param manager CAN 总线管理器。
     * @param id 设备接收 ID。
     * @param is_extid 接收 ID 是否为扩展帧。
     * @param tx_id 设备发送 ID。
     * @param tx_is_extid 发送 ID 是否为扩展帧。
     */
    C620Motor(CanBus *manager, uint32_t id, bool is_extid, uint32_t tx_id,
                        bool tx_is_extid)
            : CanDevice(manager, id, is_extid, tx_id, tx_is_extid) {}

    /**
     * @brief 初始化减速比和命令上限。
     * @param reduction_ratio 输出端减速比。
     * @param max_cmd 允许的最大命令值。
     */
    void init(float reduction_ratio = 19, float max_cmd = 20000.0f) {
        setMotorReduction(reduction_ratio);
        setMaxCmd(max_cmd);
    }

    /**
     * @brief 解析 C620 电机上报的反馈数据。
     * @param data 原始 8 字节 CAN 数据。
     * @param len 数据长度。
     */
    void onRx(const uint8_t data[8], uint8_t len) override {
        if (len < 8)
            return;

        // byte 0-1: 编码器(单圈位置 0-8191)
        encoder_ = (uint16_t)(data[0] << 8 | data[1]);
        if (is_encoder_init) {
            int16_t delta_encoder = encoder_ - last_encoder_;
            // 消圈：大于4096则是反向一圈，小于-4096则是正向一圈
            if (delta_encoder < -4096)
                round_cnt_++;
            else if (delta_encoder > 4096)
                round_cnt_--;
            // 累计编码值
            int32_t total_encoder = round_cnt_ * 8192 + encoder_ - encoder_offset_;

            // 转子侧位置
            raw_single_pos_ = static_cast<float>(encoder_) / encoder_angle_ratio_;
            raw_sum_pos_ = static_cast<float>(total_encoder) / encoder_angle_ratio_;

            // 输出端位置（考虑减速比）
            single_pos_ = raw_single_pos_ / reduction_ratio_;
            sum_pos_ = raw_sum_pos_ / reduction_ratio_;
            single_pos_ = std::fmod(sum_pos_, 360.0f);
            if (single_pos_ < 0.0f)
                single_pos_ += 360.0f;
        } else {
            encoder_offset_ = encoder_;
            is_encoder_init = true;
        }
        last_encoder_ = encoder_;

        // byte 2-3: 转子速度 (int16, 单位: rpm，取决于驱动器)
        int16_t raw_speed = (int16_t)((data[2] << 8) | data[3]);
        raw_speed_ = static_cast<float>(raw_speed) * RPM_2_RAD_PER_SEC;
        speed_ = raw_speed_ / reduction_ratio_;

        // byte 4-5: 实际电流
        // 电流(A) = value * 0.1
        int16_t raw_current = (int16_t)((data[4] << 8) | data[5]);
        raw_torque_ = static_cast<float>(raw_current);                 // A
        torque_ = raw_torque_ * current_to_torque_ * reduction_ratio_; // 输出端力矩

        // byte 6: 电机温度 (uint8, 单位: °C)
        temperature_ = static_cast<float>(data[6]);
    }

    /**
     * @brief 将内部命令换算为驱动器使用的原始指令值。
     * @return 原始命令值。
     */
    float cmdTrans() { return cmd_ * 16384.f / 20000.0f; }

    /**
     * @brief C620 不单独组帧，交由上层聚合发送。
     * @param data 发送缓冲区。
     * @param len 输出长度。
     * @return 始终返回 false，表示不生成独立帧。
     */
    bool buildTx(uint8_t data[8], uint8_t &len) override {
        // C610 不单独发帧，返回 false
        len = 0;
        return false;
    }

private:
    // 编码器相关
    bool is_encoder_init{false};
    uint16_t encoder_offset_{0};
    uint16_t encoder_{0};
    uint16_t last_encoder_{0};
    int32_t round_cnt_{0};
    float encoder_angle_ratio_ = 8192.0f / 360.0f;

    // 电流转力矩
    float current_to_torque_{0.0f}; // M2006: 0.2 Nm/A
};

class GM6020Motor : public CanDevice, public MotorBase {
public:
    /**
     * @brief 构造一个 GM6020 电机对象。
     * @param manager CAN 总线管理器。
     * @param id 设备接收 ID。
     * @param is_extid 接收 ID 是否为扩展帧。
     * @param tx_id 设备发送 ID。
     * @param tx_is_extid 发送 ID 是否为扩展帧。
     */
    GM6020Motor(CanBus *manager, uint32_t id, bool is_extid, uint32_t tx_id,
                            bool tx_is_extid)
            : CanDevice(manager, id, is_extid, tx_id, tx_is_extid) {}

private:
};

class DM4310Motor : public CanDevice, public MotorBase {
public:
    /**
     * @brief DM4310 支持的控制模式。
     */
    enum ControlMode : uint8_t {
        Mit = 0x01,
        PosWithSpeed = 0x02, // 位置速度模式
        Speed = 0x03,
        Psi = 0x04,
    };

    /**
     * @brief DM4310 支持的模式控制命令。
     */
    enum MotorModeCmd : uint8_t {
        ModeNone = 0x00,
        Enable = 0x01,
        Disable = 0x02,
        ZeroPosition = 0x03,
        ClearError = 0x04,
        ChangeMode = 0x05,
        SaveConfig = 0x06,
    };

    /**
     * @brief 构造一个 DM4310 电机对象。
     * @param manager CAN 总线管理器。
     * @param id 设备接收 ID。
     * @param is_extid 接收 ID 是否为扩展帧。
     * @param tx_id 设备发送 ID。
     * @param tx_is_extid 发送 ID 是否为扩展帧。
     * @param mode 初始控制模式。
     */
    DM4310Motor(CanBus *manager, uint32_t id, bool is_extid, uint32_t tx_id,
                            bool tx_is_extid, ControlMode mode = PosWithSpeed)
            : CanDevice(manager, id, is_extid, tx_id, tx_is_extid) {
        tx_base_id_ = tx_id;
        ctrl_mode_ = mode;
    }

    void init(float reduction_ratio = 10.0f, float max_cmd = 18.0f,
                        ControlMode mode = PosWithSpeed) {
        setMotorReduction(reduction_ratio);
        setMaxCmd(max_cmd);
        ctrl_mode_ = mode;
        dmMotorEnable();
    }

    /**
     * @note 电机模式需要上位机配置，如果模式不对，是没有效果的
     * @brief 位置-速度复合控制。
     * @param pos 目标位置。
     * @param speed 目标速度。
     */
    void posWithSpeedControl(float pos_deg, float speed_deg) {
        target_pos_ = pos_deg * M_PI / 180.0f;
        target_speed_ = speed_deg * M_PI / 180.0f;

        CanBus::ClassicPack pack{};
        uint8_t len = 0;
        if (!buildTx(pack.data, len)) {
            return;
        }

        pack.id = tx_id_;
        pack.type = CanBus::Type::STANDARD;
        (void)manager_->addCanMsg(pack);
    }

    /**
     * @note 电机模式需要上位机配置，如果模式不对，是没有效果的
     * @brief 速度控制。
     * @param speed 目标速度。
     */
    void speedControl(float speed) {
        target_speed_ = speed;
            if (manager_ == nullptr) {
            return;
        }

        CanBus::ClassicPack pack{};
        uint8_t len = 0;
        if (!buildTx(pack.data, len)) {
            return;
        }

        pack.id = tx_id_;
        pack.type = CanBus::Type::STANDARD;
        (void)manager_->addCanMsg(pack);
    }

    /**
     * @note 电机模式需要上位机配置，如果模式不对，是没有效果的
     * @brief MIT 模式控制。
     * @param speed 目标速度。
     * @param pos 目标位置。
     * @param torque 目标力矩。
     * @param Kp 位置环比例系数。
     * @param Kd 速度环比例系数。
     */
    void mitControl(float speed, float pos, float torque, float Kp, float Kd) {
        target_speed_ = speed;
        target_pos_ = pos;
        setMotorCmd(torque);
        target_kp_ = Kp;
        target_kd_ = Kd;
            if (manager_ == nullptr) {
            return;
        }

        CanBus::ClassicPack pack{};
        uint8_t len = 0;
        if (!buildTx(pack.data, len)) {
            return;
        }

        pack.id = tx_id_;
        pack.type = CanBus::Type::STANDARD;
        (void)manager_->addCanMsg(pack);
    }

    /**
     * @note 电机模式需要上位机配置，如果模式不对，是没有效果的
     * @brief PSI 模式控制。
     * @param pos 目标位置。
     * @param speed 目标速度。
     * @param current 目标电流。
     */
    void psiControl(float pos, float speed, float current) {
        // const bool cs_changed = (target_pos_ != pos) || (target_speed_ != speed) || (target_current_ != current);
        target_pos_ = pos;
        target_speed_ = speed;
        target_current_ = current;
            if (manager_ == nullptr) {
            return;
        }

        CanBus::ClassicPack pack{};
        uint8_t len = 0;
        if (!buildTx(pack.data, len)) {
            return;
        }

        pack.id = tx_id_;
        pack.type = CanBus::Type::STANDARD;
        (void)manager_->addCanMsg(pack);
    }

    /**
     * @brief 发送电机使能命令。
     */
    void dmMotorEnable(void) {
        motor_mode_cmd_ = Enable;

        if (manager_ == nullptr) {
            return;
        }

        CanBus::ClassicPack pack{};
        uint8_t len = 0;
        if (!buildTx(pack.data, len)) {
            return;
        }

        pack.id = tx_id_;
        pack.type = CanBus::Type::STANDARD;
        (void)manager_->addCanMsg(pack);
    }

    /**
     * @brief 发送电机失能命令。
     */
    void dmMotorDisable(void) {
         motor_mode_cmd_ = Disable; 
        if (manager_ == nullptr) {
            return;
        }

        CanBus::ClassicPack pack{};
        uint8_t len = 0;
        if (!buildTx(pack.data, len)) {
            return;
        }

        pack.id = tx_id_;
        pack.type = CanBus::Type::STANDARD;
        (void)manager_->addCanMsg(pack);
    }



    /**
     * @brief 发送电机配置保存命令。
     */
    void dmMotorSave(void){
        // save config removed: no-op
        (void)0;
    }

    /**
     * @brief 发送电机回零位命令。
     */
    void dmMotorZeroPosition(void) { motor_mode_cmd_ = ZeroPosition; }

    /**
     * @brief 发送清除错误命令。
     */
    void dmMotorClearError(void) { motor_mode_cmd_ = ClearError; }

    /**
     * @brief 按状态机推进 DM4310 的模式切换流程。
     * @param mode 目标控制模式。
     * @param status 当前切换状态。
     * @return 1 表示切换完成或无需切换，0 表示还需继续推进。
     */
    

    /**
     * @brief 解析 DM4310 反馈帧。
     * @param data 原始 8 字节 CAN 数据。
     * @param len 数据长度。
     */
    void onRx(const uint8_t data[8], uint8_t len) override {
        if (len < 6)
            return;

        // 达妙 MIT 反馈：byte1-2 位置，byte3-4高4bit 速度，byte4低4bit-5 扭矩
        pos_raw_ = static_cast<uint16_t>((data[1] << 8) | data[2]);
        speed_raw_ = static_cast<uint16_t>((data[3] << 4) | (data[4] >> 4));
        torque_raw_ = static_cast<uint16_t>(((data[4] & 0x0F) << 8) | data[5]);
        // 电机线圈温度
        // 根据文档，MOS管温度也可以监测，这里只保留线圈温度
        temperature_raw_ = data[7];


        {
            float pos_mapped = uint_to_float(pos_raw_, MIT_P_MIN, MIT_P_MAX, 16);
            raw_single_pos_ = (pos_mapped - MIT_P_MIN) * (360.0f / (MIT_P_MAX - MIT_P_MIN));
            if (raw_single_pos_ < 0.0f)
                raw_single_pos_ = 0.0f;
            else if (raw_single_pos_ > 360.0f)
                raw_single_pos_ = 360.0f;
        }
        raw_speed_ = uint_to_float(speed_raw_, MIT_V_MIN, MIT_V_MAX, 12);
        raw_torque_ = uint_to_float(torque_raw_, MIT_T_MIN, MIT_T_MAX, 12);

        single_pos_ = raw_single_pos_;
        raw_sum_pos_ = raw_single_pos_;
        sum_pos_ = single_pos_;
        speed_ = raw_speed_ / reduction_ratio_;
        torque_ = raw_torque_ * reduction_ratio_;
        temperature_ = static_cast<float>(temperature_raw_);
    }

    /**
     * @brief 根据当前模式打包 DM4310 发送帧。
     * @param data 发送缓冲区。
     * @param len 输出长度。
     * @return 是否成功生成发送帧。
     */
    bool buildTx(uint8_t data[8], uint8_t &len) override {
        for (uint8_t i = 0; i < 8; ++i) {
            data[i] = 0;
        }

        // 模式命令帧
        if (motor_mode_cmd_ != ModeNone) {
            len = 8;
            tx_id_ = tx_base_id_ + modeOffsetFromCtrlMode(ctrl_mode_);
            for (uint8_t i = 0; i < 7; ++i) {
                data[i] = 0xFF;
            }

            if (motor_mode_cmd_ == Enable) {
                data[7] = 0xFC;
            } else if (motor_mode_cmd_ == Disable) {
                data[7] = 0xFD;
            } else if (motor_mode_cmd_ == ZeroPosition) {
                data[7] = 0xFE;
            } else {
                data[7] = 0xFB;
            }

            motor_mode_cmd_ = ModeNone;
            return true;
        }

        if (ctrl_mode_ == PosWithSpeed) {
            len = 8;
            tx_id_ = tx_base_id_ + modeOffsetFromCtrlMode(ctrl_mode_);
            const uint8_t *pbuf = reinterpret_cast<const uint8_t *>(&target_pos_);
            const uint8_t *vbuf = reinterpret_cast<const uint8_t *>(&target_speed_);
            data[0] = pbuf[0];
            data[1] = pbuf[1];
            data[2] = pbuf[2];
            data[3] = pbuf[3];
            data[4] = vbuf[0];
            data[5] = vbuf[1];
            data[6] = vbuf[2];
            data[7] = vbuf[3];
            return true;
        }

        if (ctrl_mode_ == Speed) {
            len = 4;
            tx_id_ = tx_base_id_ + modeOffsetFromCtrlMode(ctrl_mode_);
            const uint8_t *vbuf = reinterpret_cast<const uint8_t *>(&target_speed_);
            data[0] = vbuf[0];
            data[1] = vbuf[1];
            data[2] = vbuf[2];
            data[3] = vbuf[3];
            return true;
        }

        if (ctrl_mode_ == Psi) {
            len = 8;
            tx_id_ = tx_base_id_ + modeOffsetFromCtrlMode(ctrl_mode_);

            const uint8_t *pbuf = reinterpret_cast<const uint8_t *>(&target_pos_);
            const uint16_t u16_speed =
                    static_cast<uint16_t>(constrain(target_speed_, MIT_V_MIN, MIT_V_MAX) *
                                                                100.0f);
            const uint16_t u16_current = static_cast<uint16_t>(
                    constrain(target_current_, PSI_I_MIN, PSI_I_MAX) * 10000.0f);
            const uint8_t *vbuf = reinterpret_cast<const uint8_t *>(&u16_speed);
            const uint8_t *ibuf = reinterpret_cast<const uint8_t *>(&u16_current);

            data[0] = pbuf[0];
            data[1] = pbuf[1];
            data[2] = pbuf[2];
            data[3] = pbuf[3];
            data[4] = vbuf[0];
            data[5] = vbuf[1];
            data[6] = ibuf[0];
            data[7] = ibuf[1];
            return true;
        }

        // MIT 控制
        len = 8;
        tx_id_ = tx_base_id_ + modeOffsetFromCtrlMode(ctrl_mode_);

        const uint16_t pos_tmp = static_cast<uint16_t>(
                float_to_uint(constrain(target_pos_, MIT_P_MIN, MIT_P_MAX), MIT_P_MIN,
                                            MIT_P_MAX, 16));
        const uint16_t vel_tmp = static_cast<uint16_t>(
                float_to_uint(constrain(target_speed_, MIT_V_MIN, MIT_V_MAX), MIT_V_MIN,
                                            MIT_V_MAX, 12));
        const uint16_t kp_tmp = static_cast<uint16_t>(float_to_uint(
                constrain(target_kp_, Kp_MIN, Kp_MAX), Kp_MIN, Kp_MAX, 12));
        const uint16_t kd_tmp = static_cast<uint16_t>(float_to_uint(
                constrain(target_kd_, Kd_MIN, Kd_MAX), Kd_MIN, Kd_MAX, 12));
        const uint16_t tor_tmp = static_cast<uint16_t>(
            float_to_uint(constrain(cmd_, MIT_T_MIN, MIT_T_MAX), MIT_T_MIN,
                            MIT_T_MAX, 12));

        data[0] = static_cast<uint8_t>(pos_tmp >> 8);
        data[1] = static_cast<uint8_t>(pos_tmp);
        data[2] = static_cast<uint8_t>(vel_tmp >> 4);
        data[3] = static_cast<uint8_t>(((vel_tmp & 0x0F) << 4) | (kp_tmp >> 8));
        data[4] = static_cast<uint8_t>(kp_tmp);
        data[5] = static_cast<uint8_t>(kd_tmp >> 4);
        data[6] = static_cast<uint8_t>(((kd_tmp & 0x0F) << 4) | (tor_tmp >> 8));
        data[7] = static_cast<uint8_t>(tor_tmp);
        return true;
    }

private:
    static constexpr float MIT_P_MIN = -12.5f;
    static constexpr float MIT_P_MAX = 12.5f;
    static constexpr float MIT_V_MIN = -30.0f;
    static constexpr float MIT_V_MAX = 30.0f;
    static constexpr float MIT_T_MIN = -10.0f;
    static constexpr float MIT_T_MAX = 10.0f;
    static constexpr float Kp_MIN = 0.0f;
    static constexpr float Kp_MAX = 500.0f;
    static constexpr float Kd_MIN = 0.0f;
    static constexpr float Kd_MAX = 5.0f;
    static constexpr float PSI_I_MIN = 0.0f;
    static constexpr float PSI_I_MAX = 18.0f;

    /**
     * @brief 获取指定控制模式对应的发送 ID 偏移。
     * @param mode 控制模式。
     * @return 相对基址的 ID 偏移。
     */
    static uint16_t modeOffsetFromCtrlMode(ControlMode mode) {
        if (mode == Mit) {
            return 0x000;
        }
        if (mode == PosWithSpeed) {
            return 0x100;
        }
        if (mode == Speed) {
            return 0x200;
        }
        return 0x300;
    }

    /**
     * @brief 将输入值限制在给定范围内。
     * @param v 待限制的值。
     * @param lo 下限。
     * @param hi 上限。
     * @return 限幅后的值。
     */
    static float constrain(float v, float lo, float hi) {
        if (v < lo)
            return lo;
        if (v > hi)
            return hi;
        return v;
    }

    /**
     * @brief 将浮点数映射为定点整数。
     * @param x1 输入值。
     * @param x1_min 输入下限。
     * @param x1_max 输入上限。
     * @param bits 输出位宽。
     * @return 映射后的整数值。
     */
    static int float_to_uint(float x1, float x1_min, float x1_max, int bits) {
        const float span = x1_max - x1_min;
        const float offset = x1_min;
        return static_cast<int>((x1 - offset) *
                                                        ((static_cast<float>((1 << bits) - 1)) / span));
    }

    /**
     * @brief 将定点整数映射回浮点数。
     * @param x1_int 输入整数。
     * @param x1_min 输出下限。
     * @param x1_max 输出上限。
     * @param bits 输入位宽。
     * @return 映射后的浮点值。
     */
    static float uint_to_float(int x1_int, float x1_min, float x1_max, int bits) {
        const float span = x1_max - x1_min;
        const float offset = x1_min;
        return (static_cast<float>(x1_int) * span /
                        static_cast<float>((1 << bits) - 1)) +
                     offset;
    }

    uint16_t pos_raw_{0};
    uint16_t speed_raw_{0};
    uint16_t torque_raw_{0};
    uint8_t temperature_raw_{0};

    float target_pos_{0.0f};
    float target_speed_{0.0f};
    float target_kp_{0.0f};
    float target_kd_{0.0f};
    float target_current_{0.0f};

    ControlMode ctrl_mode_{PosWithSpeed};
    MotorModeCmd motor_mode_cmd_{ModeNone};
    uint32_t tx_base_id_{0};
};