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
      fric_rpm: 5000
      trig_gear_ratio: 36.0
      num_trig_tooth: 10
      trig_freq_: 10.0
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
#include <initializer_list>

#include "CMD.hpp"
#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_cb.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "pid.hpp"
#include "timebase.hpp"
#include "timer.hpp"

constexpr float TRIGSTEP = static_cast<float>(M_2PI) / 10;
constexpr float MIN_FRIC_RPM = 200.0f;

class TestLauncher {
 public:
  enum class LauncherState : uint8_t {
    STOP,
    NORMAL,
    JAMMED,
    OVERHEAT,
  };

  enum class TRIGMODE : uint8_t { RELAX, SAFE, CONTINUE, SINGLE, JAM };

  enum class FRICMODE : uint8_t {
    RELAX,
    SAFE,
    READY,
  };

  struct LauncherParam {
    /*一级摩擦轮转速*/
    float fric_rpm_;
    /*拨弹盘电机减速比*/
    float trig_gear_ratio;
    /*拨齿数目*/
    uint8_t num_trig_tooth;
    /*弹频*/
    float trig_freq_;
  };
  /**
   * @brief Launcher 构造函数
   *
   * @param hw 硬件容器
   * @param app 应用管理器
   * @param task_stack_depth 任务堆栈深度
   * @param pid_param_trig 拨弹盘PID参数
   * @param pid_param_fric 摩擦轮PID参数
   * @param  fric_radius_ 摩擦轮半径
   * @param trig_gear_ratio_ 拨弹电机减速比
   * @param  num_trig_tooth_ 拨弹盘中一圈能存储几颗弹丸
   */
  TestLauncher(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
               uint32_t task_stack_depth, CMD *cmd,
               LibXR::PID<float>::Param pid_param_trig_angle,
               LibXR::PID<float>::Param pid_param_trig_speed,
               LibXR::PID<float>::Param pid_param_fric_0,
               LibXR::PID<float>::Param pid_param_fric_1,
               RMMotor *motor_fric_front_left, RMMotor *motor_fric_front_right,
               RMMotor *motor_trig, LauncherParam launch_param)
      : param_(launch_param),
        motor_fric_0_(motor_fric_front_left),
        motor_fric_1_(motor_fric_front_right),
        motor_trig_(motor_trig),
        pid_fric_0_(pid_param_fric_0),
        pid_fric_1_(pid_param_fric_1),
        pid_trig_sp_(pid_param_trig_speed),
        pid_trig_angle_(pid_param_trig_angle),
        cmd_(cmd) {
    UNUSED(hw);
    UNUSED(app);

    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, TestLauncher *launcher, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          launcher->SetMode(0);
        },
        this);

    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);

    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, TestLauncher *launcher, uint32_t event_id) {
          UNUSED(in_isr);
          launcher->SetMode(event_id);
        },
        this);
    launcher_event_.Register(static_cast<uint32_t>(FRICMODE::RELAX), callback);
    launcher_event_.Register(static_cast<uint32_t>(FRICMODE::SAFE), callback);
    launcher_event_.Register(static_cast<uint32_t>(FRICMODE::READY), callback);

    thread_.Create(this, ThreadFunction, "LauncherThread", task_stack_depth,
                   LibXR::Thread::Priority::HIGH);
  }

  static void ThreadFunction(TestLauncher *launcher) {
    auto now = LibXR::Timebase::GetMilliseconds();
    launcher->dt_ = (now - launcher->last_online_time_).ToSecondf();
    launcher->last_online_time_ = now;

    while (1) {
      launcher->mutex_.Lock();
      launcher->Update();
      launcher->FricControl();
      launcher->SetTrig();
      launcher->mutex_.Unlock();

      LibXR::Thread::Sleep(2);
    }
  }
  /**
   * @brief 更新函数
   *
   */
  void Update() {
    /*获取数据*/
    static float last_motor_angle = 0.0f;
    static bool initialized = false;

    motor_fric_0_->Update();
    motor_fric_1_->Update();
    motor_trig_->Update();

    float current_motor_angle = motor_trig_->GetAngle();

    if (!initialized) {
      last_motor_angle = current_motor_angle;
      initialized = true;
      return;
    }

    float delta_trig_angle = LibXR::CycleValue<float>(current_motor_angle) -
                             LibXR::CycleValue<float>(last_motor_angle);

    if (!motor_trig_->GetReverse()) {
      trig_angle_ += delta_trig_angle / param_.trig_gear_ratio;
    } else {
      trig_angle_ -= delta_trig_angle / param_.trig_gear_ratio;
    }
    last_motor_angle = current_motor_angle;
  }
  void FricControl() {
    switch (fric_mod_) {
      case FRICMODE::RELAX: {
        motor_fric_1_->CurrentControl(0);
        motor_fric_0_->CurrentControl(0);

      } break;
      case FRICMODE::SAFE: {
        out_rpm_0_ = LowPass(0, motor_fric_0_->GetRPM());
        out_rpm_1_ = LowPass(0, motor_fric_1_->GetRPM());
        /*防止震荡*/
        if (motor_fric_0_->GetRPM() < MIN_FRIC_RPM ||
            motor_fric_1_->GetRPM() < MIN_FRIC_RPM) {
          out_rpm_0_ = 0;
          out_rpm_1_ = 0;
        }
        fric_out_left_ =
            pid_fric_0_.Calculate(out_rpm_0_, motor_fric_0_->GetRPM(), dt_);
        fric_out_right_ =
            pid_fric_1_.Calculate(out_rpm_1_, motor_fric_1_->GetRPM(), dt_);
        motor_fric_0_->CurrentControl(fric_out_left_);
        motor_fric_1_->CurrentControl(fric_out_right_);
      } break;
      case FRICMODE::READY: {
        out_rpm_0_ = param_.fric_rpm_;
        out_rpm_1_ = param_.fric_rpm_;

        fric_out_left_ =
            pid_fric_0_.Calculate(out_rpm_0_, motor_fric_0_->GetRPM(), dt_);
        fric_out_right_ =
            pid_fric_1_.Calculate(out_rpm_1_, motor_fric_1_->GetRPM(), dt_);
        motor_fric_0_->CurrentControl(fric_out_left_);
        motor_fric_1_->CurrentControl(fric_out_right_);
      } break;
      default:
        break;
    }
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
        if (last_trig_mod_ == TRIGMODE::SAFE) {
          target_trig_angle_ += TRIGSTEP;
        }
        TrigAngleControl(target_trig_angle_);
        last_trig_angle_ = target_trig_angle_;
      } break;

      case TRIGMODE::CONTINUE: {
        float since_last = (now - last_trig_time_).ToSecondf();
        if (param_.trig_freq_ > 0.0f) {
          float trig_speed = 1.0f / param_.trig_freq_;
          if (since_last >= trig_speed) {
            target_trig_angle_ = trig_angle_ + TRIGSTEP;
            last_trig_time_ = now;
          }
        }
        TrigAngleControl(target_trig_angle_);

        last_trig_angle_ = target_trig_angle_;
      } break;

      default:
        break;
    }
    last_trig_mod_ = trig_mod_;
  }

  void SetMode(uint32_t mode) {
    switch (mode) {
      case 0:
        trig_mod_ = TRIGMODE::RELAX;
        fric_mod_ = FRICMODE::RELAX;
        break;
      case 1:
        trig_mod_ = TRIGMODE::RELAX;
        fric_mod_ = FRICMODE::READY;

        break;
      case 2:
        trig_mod_ = TRIGMODE::CONTINUE;
        fric_mod_ = FRICMODE::READY;

        break;
      default:
        break;
    }
  }
  LibXR::Event &GetEvent() { return launcher_event_; }

 private:
  LauncherParam param_;
  TRIGMODE last_trig_mod_ = TRIGMODE::RELAX;
  TRIGMODE trig_mod_ = TRIGMODE::RELAX;
  FRICMODE fric_mod_ = FRICMODE::RELAX;

  float target_trig_angle_ = 0.0f;
  float trig_angle_ = 0.0f;
  float last_trig_angle_ = 0.0f;
  LibXR::MillisecondTimestamp last_trig_time_ = 0;

  RMMotor *motor_fric_0_;
  RMMotor *motor_fric_1_;
  RMMotor *motor_trig_;

  LibXR::PID<float> pid_fric_0_;
  LibXR::PID<float> pid_fric_1_;
  LibXR::PID<float> pid_trig_sp_;
  LibXR::PID<float> pid_trig_angle_;

  CMD *cmd_;
  LibXR::MillisecondTimestamp last_online_time_ = 0.0f;
  float dt_ = 0;
  LibXR::MillisecondTimestamp last_jam_time_ = 0.0f;
  LibXR::MillisecondTimestamp jam_keep_time_ = 0.0f;
  LibXR::MillisecondTimestamp fire_press_time_ = 0;
  LibXR::MillisecondTimestamp last_heat_time_ = 0.0f;
  LibXR::MillisecondTimestamp last_check_time_ = 0.0f;
  LibXR::Thread thread_;
  LibXR::Semaphore semaphore_;
  LibXR::Mutex mutex_;
  LibXR::Event launcher_event_;
  float fric_out_left_ = 0.0f;
  float fric_out_right_ = 0.0f;
  float out_rpm_0_ = 0;
  float out_rpm_1_ = 0;
  /*---------------------工具函数--------------------------------------------------*/
  void TrigAngleControl(float target_angle) {
    float plate_omega_ref = pid_trig_angle_.Calculate(
        target_angle, trig_angle_,
        motor_trig_->GetOmega() / param_.trig_gear_ratio, dt_);
    float motor_omega_ref = plate_omega_ref;
    float out = pid_trig_sp_.Calculate(
        motor_omega_ref, motor_trig_->GetOmega() / param_.trig_gear_ratio, dt_);

    motor_trig_->CurrentControl(out);
  }
  /*指数缓变*/
  float LowPass(float target, float cur) {
    constexpr float TAU = 0.15f;
    float alpha = dt_ / (TAU + dt_);
    return cur + alpha * (target - cur);
  }
};
