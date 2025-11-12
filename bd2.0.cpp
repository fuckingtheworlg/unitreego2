#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <algorithm>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_LOWCMD  "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

// -------------------- CRC --------------------
static uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }
    return CRC32;
}

// ----------------- Controller ----------------
class OptimizedCrawlMotion
{
public:
    OptimizedCrawlMotion() = default;
    ~OptimizedCrawlMotion() = default;

    void Init();

private:
    // lifecycle
    void InitLowCmd();
    void LowStateMessageHandler(const void *messages);
    void LowCmdWrite();

    // control utils
    void GenerateOptimizedCrawlGait();
    void UpdateBalanceControl();
    void ApplyMotorControl();
    void CalculateIMUData();
    inline double ApplyLowPassFilter(double new_value, double old_value, double alpha)
    { return alpha * new_value + (1.0 - alpha) * old_value; }

private:
    // ---- Targets / buffers ----
    // 半高度（更稳定）匍匐姿态：弯曲角更小
    double optimized_crawl_pos[12] = {
        0.12,  1.05, -2.05,   // 右前：髋/大腿/小腿
       -0.12,  1.05, -2.05,   // 左前
        0.12,  1.05, -2.05,   // 右后
       -0.12,  1.05, -2.05    // 左后
    };

    // 卧倒姿态（起/停用）
    double lie_down_pos[12] = {
        0.05, 1.15, -2.25,
       -0.05, 1.15, -2.25,
        0.05, 1.15, -2.25,
       -0.05, 1.15, -2.25
    };

    double crawl_target_pos[12] = {0.0};     // 当前步态目标
    double balance_adjustment[12] = {0.0};   // 姿态补偿

    // ---- Timing ----
    const double dt = 0.002;     // 500 Hz
    double runing_time = 0.0;
    double phase = 0.0;

    // ---- Gait params (更保守) ----
    double gait_frequency = 0.30;   // 降一点频率
    double stride_length  = 0.04;   // 步幅减半
    double lift_height    = 0.02;   // 抬腿减半

    // ---- Balance / IMU ----
    double body_roll = 0.0,  body_pitch = 0.0;
    double filtered_roll = 0.0, filtered_pitch = 0.0;
    double prev_roll = 0.0,     prev_pitch = 0.0;

    // 姿态与电机 PD（更温和），并设置限幅防发散
    double kp_balance = 0.8;
    double kd_balance = 0.05;
    double kp_motor   = 55.0;
    double kd_motor   = 5.5;
    const double KP_MIN = 30.0, KP_MAX = 80.0;
    const double KD_MIN = 3.0,  KD_MAX = 8.0;

    // 支撑相 / 地面接触
    bool support_phase[4] = {true, true, true, true};
    double foot_contact_force[4] = {0};
    bool ground_contact[4] = {true, true, true, true};

    // 监控
    double motor_positions[12] = {0};
    double motor_velocities[12] = {0};

    // 任务参数
    double mission_duration = 60.0;
    bool mission_complete = false;
    int total_cycles = 0;
    double cycle_progress = 0.0;

    // DDS objects
    unitree_go::msg::dds_::LowCmd_   low_cmd{};
    unitree_go::msg::dds_::LowState_ low_state{};

    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_>   lowcmd_publisher;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    ThreadPtr lowCmdWriteThreadPtr;
};

// -------------- Impl ----------------
void OptimizedCrawlMotion::Init()
{
    InitLowCmd();

    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(
        std::bind(&OptimizedCrawlMotion::LowStateMessageHandler, this, std::placeholders::_1), 1);

    lowCmdWriteThreadPtr = CreateRecurrentThreadEx(
        "writebasiccmd", UT_CPU_ID_NONE, int(dt * 1e6),
        &OptimizedCrawlMotion::LowCmdWrite, this);

    std::cout << "=== Optimized Ultra Low Crawl Motion (STABLE) ===\n";
    std::cout << "Config:\n";
    std::cout << "- gait_frequency=" << gait_frequency
              << ", stride_length=" << stride_length
              << ", lift_height=" << lift_height << "\n";
    std::cout << "- Motor PD: KP=" << kp_motor << " KD=" << kd_motor
              << " (clamped to [" << KP_MIN << "," << KP_MAX
              << "] / [" << KD_MIN << "," << KD_MAX << "])\n";
}

void OptimizedCrawlMotion::InitLowCmd()
{
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for (int i = 0; i < 20; i++)
    {
        low_cmd.motor_cmd()[i].mode() = 0x01;
        low_cmd.motor_cmd()[i].q()    = PosStopF;
        low_cmd.motor_cmd()[i].kp()   = 0;
        low_cmd.motor_cmd()[i].dq()   = VelStopF;
        low_cmd.motor_cmd()[i].kd()   = 0;
        low_cmd.motor_cmd()[i].tau()  = 0;
    }
}

void OptimizedCrawlMotion::LowStateMessageHandler(const void *message)
{
    low_state = *(unitree_go::msg::dds_::LowState_ *)message;

    // 电机状态
    for (int i = 0; i < 12; i++) {
        motor_positions[i]  = low_state.motor_state()[i].q();
        motor_velocities[i] = low_state.motor_state()[i].dq();
    }

    // 足端力与接触
    for (int i = 0; i < 4; i++) {
        foot_contact_force[i] = low_state.foot_force()[i];
        ground_contact[i]     = (foot_contact_force[i] > 5.0);
    }

    // IMU 姿态
    CalculateIMUData();
}

void OptimizedCrawlMotion::CalculateIMUData()
{
    // 仿真直接提供 RPY，避免对陀螺积分造成的漂移
    if (low_state.imu_state().rpy().size() >= 3) {
        body_roll  = low_state.imu_state().rpy()[0];
        body_pitch = low_state.imu_state().rpy()[1];

        // 更强的低通，抑制高频噪声
        filtered_roll  = ApplyLowPassFilter(body_roll,  prev_roll,  0.05);
        filtered_pitch = ApplyLowPassFilter(body_pitch, prev_pitch, 0.05);
        prev_roll  = filtered_roll;
        prev_pitch = filtered_pitch;
    }
}

void OptimizedCrawlMotion::UpdateBalanceControl()
{
    std::fill(std::begin(balance_adjustment), std::end(balance_adjustment), 0.0);

    // Roll 补偿（左右抬/降）
    if (fabs(filtered_roll) > 0.01) {
        const double roll_adjust = kp_balance * filtered_roll;
        // 右侧
        balance_adjustment[1]  -= roll_adjust * 0.30;  // 右前大腿
        balance_adjustment[2]  += roll_adjust * 0.20;  // 右前小腿
        balance_adjustment[7]  -= roll_adjust * 0.30;  // 右后大腿
        balance_adjustment[8]  += roll_adjust * 0.20;  // 右后小腿
        // 左侧
        balance_adjustment[4]  += roll_adjust * 0.30;  // 左前大腿
        balance_adjustment[5]  -= roll_adjust * 0.20;  // 左前小腿
        balance_adjustment[10] += roll_adjust * 0.30;  // 左后大腿
        balance_adjustment[11] -= roll_adjust * 0.20;  // 左后小腿
    }

    // Pitch 补偿（前后抬/降）
    if (fabs(filtered_pitch) > 0.01) {
        const double pitch_adjust = kp_balance * filtered_pitch;
        // 前腿
        balance_adjustment[1]  -= pitch_adjust * 0.30;
        balance_adjustment[2]  += pitch_adjust * 0.20;
        balance_adjustment[4]  -= pitch_adjust * 0.30;
        balance_adjustment[5]  += pitch_adjust * 0.20;
        // 后腿
        balance_adjustment[7]  += pitch_adjust * 0.30;
        balance_adjustment[8]  -= pitch_adjust * 0.20;
        balance_adjustment[10] += pitch_adjust * 0.30;
        balance_adjustment[11] -= pitch_adjust * 0.20;
    }

    // 失联足端微降（帮助再接触）
    for (int leg = 0; leg < 4; leg++) {
        if (!ground_contact[leg]) {
            const int base = leg * 3;
            balance_adjustment[base + 1] -= 0.015; // 大腿
            balance_adjustment[base + 2] += 0.010; // 小腿
        }
    }

    // 姿态补偿限幅，防过补
    for (double &x : balance_adjustment) {
        x = std::max(-0.15, std::min(0.15, x));
    }
}

void OptimizedCrawlMotion::GenerateOptimizedCrawlGait()
{
    // 相位推进
    const double phase_offset = 2.0 * M_PI * gait_frequency * runing_time;

    // 统计周期
    static double last_phase = 0.0;
    const double current_phase_val = fmod(phase_offset, 2.0 * M_PI);
    if (last_phase > M_PI && current_phase_val <= M_PI) total_cycles++;
    last_phase = current_phase_val;
    cycle_progress = current_phase_val / (2.0 * M_PI);

    // 对角相位（略错相，利于稳定）
    const double phase_fr = phase_offset;                 // 右前
    const double phase_rl = phase_offset;                 // 左后
    const double phase_fl = phase_offset + M_PI * 0.75;   // 左前
    const double phase_rr = phase_offset + M_PI * 0.75;   // 右后

    // 平滑摆动轮廓
    auto swing_profile = [](double phase) -> double {
        double np = fmod(phase, 2*M_PI) / (2*M_PI);
        if (np < 0.5) {
            return 0.5 - 0.5 * cos(2 * M_PI * np);  // 支撑
        } else {
            double sp = (np - 0.5) / 0.5;           // 摆动
            return 0.5 + 0.5 * (1.0 - cos(M_PI * sp));
        }
    };

    // 支撑判定
    support_phase[0] = (swing_profile(phase_fr) < 0.5);
    support_phase[1] = (swing_profile(phase_fl) < 0.5);
    support_phase[2] = (swing_profile(phase_rr) < 0.5);
    support_phase[3] = (swing_profile(phase_rl) < 0.5);

    // 右前 0,1,2
    const double sw_fr = swing_profile(phase_fr);
    crawl_target_pos[0] = optimized_crawl_pos[0] + stride_length * (sw_fr - 0.5);
    crawl_target_pos[1] = optimized_crawl_pos[1] + lift_height * sw_fr;
    crawl_target_pos[2] = optimized_crawl_pos[2] - 0.4 * lift_height * sw_fr;

    // 左前 3,4,5
    const double sw_fl = swing_profile(phase_fl);
    crawl_target_pos[3] = optimized_crawl_pos[3] + stride_length * (sw_fl - 0.5);
    crawl_target_pos[4] = optimized_crawl_pos[4] + lift_height * sw_fl;
    crawl_target_pos[5] = optimized_crawl_pos[5] - 0.4 * lift_height * sw_fl;

    // 右后 6,7,8
    const double sw_rr = swing_profile(phase_rr);
    crawl_target_pos[6] = optimized_crawl_pos[6] + stride_length * (sw_rr - 0.5);
    crawl_target_pos[7] = optimized_crawl_pos[7] + lift_height * sw_rr;
    crawl_target_pos[8] = optimized_crawl_pos[8] - 0.4 * lift_height * sw_rr;

    // 左后 9,10,11
    const double sw_rl = swing_profile(phase_rl);
    crawl_target_pos[9]  = optimized_crawl_pos[9]  + stride_length * (sw_rl - 0.5);
    crawl_target_pos[10] = optimized_crawl_pos[10] + lift_height * sw_rl;
    crawl_target_pos[11] = optimized_crawl_pos[11] - 0.4 * lift_height * sw_rl;

    // 更新姿态补偿
    UpdateBalanceControl();

    // 叠加补偿 + 关节限位（给补偿预留余量）
    for (int i = 0; i < 12; i++) {
        crawl_target_pos[i] += balance_adjustment[i];

        if (i % 3 == 1) {        // 大腿
            crawl_target_pos[i] = std::max(0.75, std::min(1.30, crawl_target_pos[i]));
        } else if (i % 3 == 2) { // 小腿
            crawl_target_pos[i] = std::max(-2.35, std::min(-1.75, crawl_target_pos[i]));
        }
    }
}

void OptimizedCrawlMotion::ApplyMotorControl()
{
    // 统一钳位KP/KD，兜底安全
    const double kp_cmd = std::max(KP_MIN, std::min(KP_MAX, kp_motor));
    const double kd_cmd = std::max(KD_MIN, std::min(KD_MAX, kd_motor));

    for (int i = 0; i < 12; i++)
    {
        auto &mc = low_cmd.motor_cmd()[i];
        mc.mode() = 0x01;
        mc.q()    = crawl_target_pos[i];
        mc.dq()   = 0.0;
        mc.kp()   = kp_cmd;
        mc.kd()   = kd_cmd;
        mc.tau()  = 0.0;
    }

    // 其余保留电机安全设置
    for (int i = 12; i < 20; i++)
    {
        auto &mc = low_cmd.motor_cmd()[i];
        mc.mode() = 0x01;
        mc.q()    = PosStopF;
        mc.dq()   = 0.0;
        mc.kp()   = 0.0;
        mc.kd()   = 0.0;
        mc.tau()  = 0.0;
    }
}

void OptimizedCrawlMotion::LowCmdWrite()
{
    runing_time += dt;

    // 任务阶段 1：3s 过渡卧倒 -> 半蹲
    if (runing_time < 3.0)
    {
        phase = tanh(runing_time / 1.5);
        for (int i = 0; i < 12; i++)
            crawl_target_pos[i] = phase * optimized_crawl_pos[i] + (1 - phase) * lie_down_pos[i];

        static int transition_count = 0;
        if (transition_count % 500 == 0) {
            std::cout << "Transition to STABLE HALF-HEIGHT posture... " << runing_time << "s\n";
        }
        transition_count++;
    }
    // 阶段 2：持续匍匐
    else if (!mission_complete)
    {
        GenerateOptimizedCrawlGait();

        // 按支撑腿数温和自适应
        double kp_adjust = kp_motor;
        double kd_adjust = kd_motor;

        int support_legs = 0;
        for (int i = 0; i < 4; i++) if (support_phase[i]) support_legs++;

        if (support_legs < 3) {  // 支撑少→略提升
            kp_adjust += 5.0;
            kd_adjust += 0.5;
        } else if (support_legs == 4) { // 支撑多→略放软
            kp_adjust -= 2.0;
            kd_adjust -= 0.2;
        }

        // 限幅
        kp_motor = std::max(KP_MIN, std::min(KP_MAX, kp_adjust));
        kd_motor = std::max(KD_MIN, std::min(KD_MAX, kd_adjust));

        // 周期进度日志（每 ~10s）
        static int progress_count = 0;
        if (progress_count % 5000 == 0) {
            double elapsed = runing_time - 3.0;
            double percent = std::max(0.0, std::min(100.0, (elapsed / mission_duration) * 100.0));
            std::cout << "Stable Crawl Progress: " << elapsed << "s / " << mission_duration
                      << "s (" << percent << "%)\n";
            std::cout << "Cycles: " << total_cycles << ", Support Legs: " << support_legs << "\n";
            std::cout << "Motor PD (clamped): KP=" << kp_motor << ", KD=" << kd_motor << "\n";
        }
        progress_count++;

        if (runing_time >= (3.0 + mission_duration)) mission_complete = true;
    }
    // 阶段 3：回到卧倒
    else
    {
        double stop_phase = tanh((runing_time - (3.0 + mission_duration)) / 1.0);
        for (int i = 0; i < 12; i++)
            crawl_target_pos[i] = stop_phase * lie_down_pos[i] + (1 - stop_phase) * optimized_crawl_pos[i];

        static int complete_count = 0;
        if (complete_count % 2500 == 0)
            std::cout << "Mission Complete! Transitioning to lying posture...\n";
        complete_count++;
    }

    // 下发命令
    ApplyMotorControl();
    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher->Write(low_cmd);
}

// ----------------- main -----------------
int main(int argc, const char **argv)
{
    if (argc < 2) {
        // 仿真默认使用 lo（loopback）
        ChannelFactory::Instance()->Init(1, "lo");
        std::cout << "Using loopback interface for simulation (lo)\n";
    } else {
        ChannelFactory::Instance()->Init(0, argv[1]);
        std::cout << "Using network interface: " << argv[1] << std::endl;
    }

    std::cout << "=== Unitree GO2 Optimized Half-Height Crawl (STABLE) ===\n";
    std::cout << "Press ENTER to start mission..."; std::cout.flush();
    std::cin.get();

    OptimizedCrawlMotion ctrl;
    ctrl.Init();

    std::cout << "Stable crawl mission started! Press Ctrl+C to stop.\n";

    while (true) { sleep(10); }
    return 0;
}
