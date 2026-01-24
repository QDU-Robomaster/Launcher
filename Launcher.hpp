#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - task_stack_depth: 2048
  - cmd: '@&cmd'
  - pid_param_trig_angle:
      k: 1.0
      p: 20.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_param_trig_speed:
      k: 1.0
      p: 0.2
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_param_fric_0:
      k: 1.0
      p: 0.000002
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_param_fric_1:
      k: 1.0
      p: 0.000002
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - motor_fric_0: '@&motor_fric_0'
  - motor_fric_1: '@&motor_fric_1'
  - motor_trig: '@&motor_trig'
  - launcher_param:
      default_bullet_speed: 10.0
      fric_radius: 0.03
      trig_gear_ratio: 36.0
      num_trig_tooth: 10
      expect_trig_freq_: 15
      fric_rpm_: 3000
template_args:
required_hardware:
  - dr16
  - can
depends:
  - qdu-future/CMD
  - qdu-future/RMMotor
=== END MANIFEST === */
// clang-format on

#include <cmath>
#include <cstdint>
#include <deque>

#include "CMD.hpp"
#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "lockfree_queue.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "timer.hpp"
#define TrigStep static_cast<float>(M_2PI / PARAM.num_trig_tooth)
#define MinFricRpm 3500
#define jam_current 10
class Launcher : public LibXR::Application {
 public:
  enum class LauncherState {
    STOP,
    NORMAL,
    JAMMED,
    OVERHEAT,
  };
  enum class TRIGMODE : uint8_t { RELAX, SAFE, SINGLE, CONTINUE, JAM };

  enum class FRICMODE : uint8_t {
    RELAX,
    SAFE,
    READY,
  };

  typedef struct {
    float heat_limit;
    float heat_cooling;
  } RefereeData;
  typedef struct {
    float default_bullet_speed;
    float fric_radius;
    float trig_gear_ratio;
    uint8_t num_trig_tooth;
    float expect_trig_freq_;
    float fric_rpm_;
  } LauncherParam;
  typedef struct {
    float single_heat;  // 单发热量
    float launched_num;
    float current_heat;
    float heat_threshold;  // 热量限制阈值
    bool allow_fire;
  } HeatLimit;
  /**
   * @brief Launcher 构造函数
   *
   * @param hw 硬件容器
   * @param app 应用管理器
   * @param task_stack_depth 任务堆栈深度
   * @param pid_param_trig 拨弹盘PID参数
   * @param pid_param_fric 摩擦轮PID参数
   * @param motor_can1 CAN1上的电机实例（trig）
   * @param motor_can2 CAN2上的电机实例 (fric)
   * @param  min_launch_delay_
   * @param  default_bullet_speed_ 默认弹丸初速度
   * @param  fric_radius_ 摩擦轮半径
   * @param trig_gear_ratio_ 拨弹电机减速比
   * @param  num_trig_tooth_ 拨弹盘中一圈能存储几颗弹丸
   * @param bullet_speed 弹丸速度
   */

  Launcher(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
           uint32_t task_stack_depth, CMD *cmd,
           LibXR::PID<float>::Param pid_param_trig_angle,
           LibXR::PID<float>::Param pid_param_trig_speed,
           LibXR::PID<float>::Param pid_param_fric_0,
           LibXR::PID<float>::Param pid_param_fric_1, RMMotor *motor_fric_0,
           RMMotor *motor_fric_1, RMMotor *motor_trig,
           LauncherParam launcher_param)
      : PARAM(launcher_param),
        motor_fric_0_(motor_fric_0),
        motor_fric_1_(motor_fric_1),
        motor_trig_(motor_trig),
        pid_fric_0_(pid_param_fric_0),
        pid_fric_1_(pid_param_fric_1),
        pid_trig_sp_(pid_param_trig_speed),
        pid_trig_angle_(pid_param_trig_angle),
        cmd_(cmd) {
    UNUSED(hw);
    UNUSED(app);
    thread_.Create(this, ThreadFunction, "LauncherThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);

    if (PARAM.expect_trig_freq_ > 0.0f) {
      min_launch_delay_ = 1.0f / PARAM.expect_trig_freq_;
    } else {
      min_launch_delay_ = 0.0f;
    }
    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Launcher *launcher, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          launcher->SetMode(static_cast<uint32_t>(FRICMODE::RELAX));
          launcher->trig_mod_ = TRIGMODE::RELAX;
        },
        this);

    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);

    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Launcher *launcher, uint32_t event_id) {
          UNUSED(in_isr);
          launcher->SetMode(event_id);
        },
        this);
    launcher_event_.Register(static_cast<uint32_t>(FRICMODE::RELAX), callback);
    launcher_event_.Register(static_cast<uint32_t>(FRICMODE::SAFE), callback);
    launcher_event_.Register(static_cast<uint32_t>(FRICMODE::READY), callback);
    auto launcher_cmd_callback = LibXR::Callback<LibXR::RawData &>::Create(
        [](bool in_isr, Launcher *Launcher, LibXR::RawData &raw_data) {
          UNUSED(in_isr);
          CMD::LauncherCMD cmd_lau =
              *reinterpret_cast<CMD::LauncherCMD *>(raw_data.addr_);
          Launcher->launcher_cmd_.isfire = cmd_lau.isfire;
        },
        this);

    auto tp_cmd_launcher =
        LibXR::Topic(LibXR::Topic::Find("launcher_cmd", nullptr));

    tp_cmd_launcher.RegisterCallback(launcher_cmd_callback);

  }

  static void ThreadFunction(Launcher *launcher) {
    while (1) {
      launcher->mutex_.Lock();
      launcher->Update();
      launcher->Heat();
      launcher->Control();
      launcher->mutex_.Unlock();

      LibXR::Thread::Sleep(2);
    }
  }
  /**
   * @brief 更新函数
   *
   */
  void Update() {
    // 获取数据
    referee_data_.heat_limit = 260.0f;
    referee_data_.heat_cooling = 30.0f;
    heat_limit_.single_heat = 10.0f;
    heat_limit_.heat_threshold = 200.0f;

    static float last_motor_angle = 0.0f;
    static bool initialized = false;
    // 更新
    motor_fric_0_->Update();
    motor_fric_1_->Update();
    motor_trig_->Update();

    // 计算trig_angle
    float current_motor_angle = motor_trig_->GetAngle();

    if (!initialized) {
      last_motor_angle = current_motor_angle;
      initialized = true;
      return;
    }

    float delta_trig_angle = LibXR::CycleValue<float>(current_motor_angle) -
                             LibXR::CycleValue<float>(last_motor_angle);

    if (!motor_trig_->GetReverse()) {
      trig_angle_ += delta_trig_angle / PARAM.trig_gear_ratio;
    } else {
      trig_angle_ -= delta_trig_angle / PARAM.trig_gear_ratio;
    }
    last_motor_angle = current_motor_angle;
  }
  void FricControl() {
    auto now = LibXR::Timebase::GetMilliseconds();
    switch (fric_mod_) {
      case FRICMODE::RELAX: {
      motor_fric_0_->CurrentControl(0);
      motor_fric_1_->CurrentControl(0);
      } break;
      case FRICMODE::SAFE: {
        float out_rpm_0 = LowPass(0, motor_fric_0_->GetRPM());
        float out_rpm_1 = LowPass(0, motor_fric_1_->GetRPM());
        fric_out_left =
            pid_fric_0_.Calculate(out_rpm_0, motor_fric_0_->GetRPM(), dt_);
        fric_out_right =
            pid_fric_1_.Calculate(out_rpm_1, motor_fric_1_->GetRPM(), dt_);
        motor_fric_0_->CurrentControl(fric_out_left);
        motor_fric_1_->CurrentControl(fric_out_right);
      } break;
      case FRICMODE::READY: {
        float out_rpm_0 = LowPass(PARAM.fric_rpm_, motor_fric_0_->GetRPM());
        float out_rpm_1 = LowPass(PARAM.fric_rpm_, motor_fric_1_->GetRPM());
        fric_out_left =
            pid_fric_0_.Calculate(out_rpm_0, motor_fric_0_->GetRPM(), dt_);
        fric_out_right =
            pid_fric_1_.Calculate(out_rpm_1, motor_fric_1_->GetRPM(), dt_);
        motor_fric_0_->CurrentControl(fric_out_left);
        motor_fric_1_->CurrentControl(fric_out_right);
      }
      default:
        break;
    }
    // 后面需要加转速滤波
  }

  void SetTrig() {
    auto now = LibXR::Timebase::GetMilliseconds();
    switch (trig_mod_) {
      case TRIGMODE::RELAX:
        motor_trig_->CurrentControl(0);
        break;
      case TRIGMODE::SAFE:
        TrigAngleControl(trig_angle_);
        break;
      case TRIGMODE::SINGLE: {
        if (last_trig_mod == TRIGMODE::SAFE) {
          target_trig_angle_ += TrigStep;
        }
        TrigAngleControl(target_trig_angle_);
        if (target_trig_angle_ > last_trig_angle) {
          MarkShot();
        }
        last_trig_angle = target_trig_angle_;
      } break;

      case TRIGMODE::CONTINUE: {
        float since_last = (now - last_trig_time_).ToSecondf();
        if (trig_freq_ > 0.0f) {
          float trig_speed = 1.0f / trig_freq_;
          if (since_last >= trig_speed) {
            target_trig_angle_ = target_trig_angle_ + TrigStep;
            last_trig_time_ = now;
          }
        }
        TrigAngleControl(target_trig_angle_);

        if (target_trig_angle_ > last_trig_angle) {
          MarkShot();
        }
        last_trig_angle = target_trig_angle_;
      } break;
      case TRIGMODE::JAM: {
        // 正转卡弹时反转，反转卡弹时正转
        jam_keep_time_ = (now - last_jam_time).ToSecond();
        if (jam_keep_time_ > 0.02) {
          if (last_trig_mod != TRIGMODE::JAM) {
            is_reverse = 1;
          }
          if (is_reverse) {
            target_trig_angle_ = trig_angle_ - 2 * TrigStep;
            TrigAngleControl(target_trig_angle_);
          } else if (!is_reverse) {
            target_trig_angle_ = trig_angle_ + TrigStep;
            TrigAngleControl(target_trig_angle_);
          }
          is_reverse = !is_reverse;
          last_jam_time = now;
        }
      } break;

      default:
        break;
    }
    // 判断是否成功发射
    CheckShot();

    last_trig_mod = trig_mod_;
  }
  void Control() {
    auto now = LibXR::Timebase::GetMilliseconds();

    if (fabs(motor_trig_->GetOmega()) < 0.5f &&
        motor_trig_->GetCurrent() > jam_current) {
      launcherstate_ = LauncherState::JAMMED;

    } else if (!heat_limit_.allow_fire) {
      launcherstate_ = LauncherState::OVERHEAT;
    } else if (fric_mod_ != FRICMODE::READY) {
      launcherstate_ = LauncherState::STOP;
    } else if (launcher_cmd_.isfire) {
      launcherstate_ = LauncherState::NORMAL;
    }


    switch (launcherstate_) {
      case LauncherState::STOP:
      case LauncherState::OVERHEAT:
        trig_mod_ = TRIGMODE::SAFE;
        break;
      case LauncherState::NORMAL:
        // 根据波轮按压的时间判断发射状态
        if (!last_fire_notify_) {
          fire_press_time_ = now;
          press_continue_ = false;
          trig_mod_ = TRIGMODE::SINGLE;
        } else {
          if (!press_continue_ && (now - fire_press_time_ > 500)) {
            press_continue_ = true;
          }
          if (press_continue_) {
            trig_mod_ = TRIGMODE::CONTINUE;
          }
        }
        break;
      case LauncherState::JAMMED:
        trig_mod_ = TRIGMODE::JAM;
        break;

      default:
        break;
    }
    last_fire_notify_ = launcher_cmd_.isfire;
  }

  void Heat() {
    auto now = LibXR::Timebase::GetMilliseconds();

    static LibXR::MillisecondTimestamp last_heat_time = 0.0f;
    float delta_time = (now - last_heat_time).ToSecondf();

    if (delta_time >= 0.1) {
      // 每周期都计算此周期的剩余热量
      last_heat_time = now;
      heat_limit_.current_heat +=
          heat_limit_.single_heat * heat_limit_.launched_num;
      heat_limit_.launched_num = 0;

      if (heat_limit_.current_heat <
          (static_cast<float>(referee_data_.heat_cooling / 10.0))) {
        heat_limit_.current_heat = 0;
      } else {
        heat_limit_.current_heat -=
            static_cast<float>(referee_data_.heat_cooling / 10.0);
      }

      float residuary_heat =
          referee_data_.heat_limit - heat_limit_.current_heat;  // 剩余热量

      // 控制control里的launcherstate
      if (residuary_heat >= heat_limit_.single_heat) {
        heat_limit_.allow_fire = true;
      } else {
        heat_limit_.allow_fire = false;
      }

      // 不同剩余热量启用不同实际弹频
      if (heat_limit_.allow_fire) {
        if (residuary_heat <= heat_limit_.single_heat + 0.001) {
          trig_freq_ = referee_data_.heat_cooling / heat_limit_.single_heat;
        } else if (residuary_heat <=
                   heat_limit_.single_heat * heat_limit_.heat_threshold) {
          float ratio = (residuary_heat - heat_limit_.single_heat) /
                        (heat_limit_.single_heat * heat_limit_.heat_threshold -
                         heat_limit_.single_heat);

          ratio = std::max(0.0f, std::min(1.0f, ratio));
          // 计算实际发射频率
          float safe_freq =
              referee_data_.heat_cooling / heat_limit_.single_heat;
          trig_freq_ =
              ratio * PARAM.expect_trig_freq_ + (1.0f - ratio) * safe_freq;
        } else {
          trig_freq_ = PARAM.expect_trig_freq_;
        }
      }
    }
  }
  /**
   * @brief 监控函数 (在此应用中未使用)
   *
   */
  void OnMonitor() override {}

  LibXR::Event &GetEvent() { return launcher_event_; }

 private:
  LauncherState launcherstate_ = LauncherState::STOP;
  LauncherParam PARAM;
  RefereeData referee_data_;
  HeatLimit heat_limit_;
  TRIGMODE last_trig_mod = TRIGMODE::SAFE;
  TRIGMODE trig_mod_ = TRIGMODE::SAFE;
  FRICMODE fric_mod_ = FRICMODE::SAFE;

  LibXR::CycleValue<float> target_trig_angle_ = 0.0f;
  LibXR::CycleValue<float> trig_angle_ = 0.0f;
  LibXR::CycleValue<float> last_trig_angle = 0.0f;
  float min_launch_delay_ = 0.0f;
  float trig_freq_ = 30.0f;
  LibXR::MillisecondTimestamp last_trig_time_ = 0;

  RMMotor *motor_fric_0_;
  RMMotor *motor_fric_1_;
  RMMotor *motor_trig_;

  LibXR::PID<float> pid_fric_0_;
  LibXR::PID<float> pid_fric_1_;
  LibXR::PID<float> pid_trig_sp_;
  LibXR::PID<float> pid_trig_angle_;
  CMD::LauncherCMD launcher_cmd_;

  CMD *cmd_;
  float dt_ = 0;
  LibXR::MillisecondTimestamp last_jam_time = 0.0f;
  LibXR::MillisecondTimestamp jam_keep_time_ = 0.0f;
  bool press_continue_ = false;
  bool last_fire_notify_ = false;
  LibXR::MillisecondTimestamp fire_press_time_ = 0;

  LibXR::Thread thread_;
  LibXR::Semaphore semaphore_;
  LibXR::Mutex mutex_;
  LibXR::Event launcher_event_;
  struct ShotEvent {
    LibXR::MillisecondTimestamp ts;
  };
  std::deque<ShotEvent> shot_event_;
  float fric_out_left = 0.0f;
  float fric_out_right = 0.0f;
  int is_reverse = 0;
  /*---------------------工具函数--------------------------------------------------*/
  void TrigAngleControl(float target_angle) {
    float plate_omega_ref = pid_trig_angle_.Calculate(
        target_angle, trig_angle_,
        motor_trig_->GetOmega() / PARAM.trig_gear_ratio, dt_);

    float motor_omega_ref = plate_omega_ref;
    float out = pid_trig_sp_.Calculate(
        motor_omega_ref, motor_trig_->GetOmega() / PARAM.trig_gear_ratio, dt_);
    // omega需要限幅，防止卡弹时十分大
    motor_trig_->CurrentControl(out);
  }
  // 指数缓变
  float LowPass(float target, float cur) {
    constexpr float TAU = 0.15f;
    float alpha = dt_ / (TAU + dt_);
    return cur + alpha * (target - cur);
  }
  void MarkShot() {
    ShotEvent ev;
    ev.ts = LibXR::Timebase::GetMilliseconds();
    shot_event_.push_back(ev);
  }
  void CheckShot() {
    if (shot_event_.empty()) return;

    auto now_ts = LibXR::Timebase::GetMilliseconds();
    while (!shot_event_.empty()) {
      auto &ev = shot_event_.front();
      float elapsed = (now_ts - ev.ts).ToSecondf();
      if (elapsed >= (1 / trig_freq_)) {
        if (motor_trig_->GetOmega() >= 0.88 * trig_freq_ &&
            motor_fric_0_->GetRPM() < MinFricRpm &&
            motor_fric_1_->GetRPM() < MinFricRpm) {
          heat_limit_.launched_num++;
        }
      }
      shot_event_.pop_front();
    }
  }

  void SetMode(uint32_t mode) { fric_mod_ = static_cast<FRICMODE>(mode); }
};
