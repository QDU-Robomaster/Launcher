#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args: []
template_args: []
required_hardware: []
depends: []
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
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "timer.hpp"

namespace launcher::param {
constexpr float TRIGSTEP = static_cast<float>(M_2PI) / 10;
}  // namespace launcher::param

class InfantryLauncher {
 public:
  enum class LauncherState : uint8_t {
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
  struct LauncherParam {
    /*一级摩擦轮转速*/
    float fric_rpm_;
    /*二级摩擦轮转速*/
    float fric2_setpoint_speed;
    /*拨弹盘电机减速比*/
    float trig_gear_ratio;
    /*拨齿数目*/
    uint8_t num_trig_tooth;
    /*弹频*/
    float trig_freq_;
  };
  typedef struct {
    float single_heat;
    float launched_num;
    float current_heat;
    float heat_threshold;
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
  InfantryLauncher(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
                   RMMotor *motor_fric_front_left,
                   RMMotor *motor_fric_front_right,
                   RMMotor *motor_fric_back_left,
                   RMMotor *motor_fric_back_right, RMMotor *motor_trig,
                   uint32_t task_stack_depth,
                   LibXR::PID<float>::Param pid_param_trig_angle,
                   LibXR::PID<float>::Param pid_param_trig_speed,
                   LibXR::PID<float>::Param pid_param_fric_0,
                   LibXR::PID<float>::Param pid_param_fric_1,
                   LibXR::PID<float>::Param pid_param_fric_2,
                   LibXR::PID<float>::Param pid_param_fric_3,
                   LauncherParam launch_param, CMD *cmd)
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
    UNUSED(pid_param_fric_2);
    UNUSED(pid_param_fric_3);
    UNUSED(motor_fric_back_left);
    UNUSED(motor_fric_back_right);
    UNUSED(cmd);
    thread_.Create(this, ThreadFunction, "LauncherThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);
    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, InfantryLauncher *launcher, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          launcher->SetMode(static_cast<uint32_t>(FRICMODE::RELAX));
          launcher->trig_mod_ = TRIGMODE::RELAX;
        },
        this);

    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);

    auto launcher_cmd_callback = LibXR::Callback<LibXR::RawData &>::Create(
        [](bool in_isr, InfantryLauncher *Launcher, LibXR::RawData &raw_data) {
          UNUSED(in_isr);
          CMD::LauncherCMD cmd_lau =
              *reinterpret_cast<CMD::LauncherCMD *>(raw_data.addr_);
          Launcher->launcher_cmd_.isfire = cmd_lau.isfire;
        },
        this);

    auto tp_cmd_launcher =
        LibXR::Topic(LibXR::Topic::Find("launcher_cmd", nullptr));

    tp_cmd_launcher.RegisterCallback(launcher_cmd_callback);

    this->SetMode(launcher_event_);
  }

  static void ThreadFunction(InfantryLauncher *launcher) {

    LibXR::Topic::ASyncSubscriber<CMD::LauncherCMD> launcher_cmd_tp(
        "launcher_cmd");
    launcher_cmd_tp.StartWaiting();
    while (1) {
      auto now = LibXR::Timebase::GetMilliseconds();
      launcher->dt_ = (now - launcher->last_online_time_).ToSecondf();
      launcher->last_online_time_ = now;

      launcher->Update();
      launcher->FricControl();
      launcher->Control();

      LibXR::Thread::Sleep(2);
    }
  }
  /**
   * @brief 更新函数
   *
   */
  void Update() {
    referee_data_.heat_limit = 260.0f;
    referee_data_.heat_cooling = 30.0f;
    heat_limit_.single_heat = 10.0f;
    heat_limit_.heat_threshold = 200.0f;

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
if (fric_mod_ != FRICMODE::READY) {
      launcherstate_ = LauncherState::STOP;
    } else if (launcher_cmd_.isfire) {
      launcherstate_ = LauncherState::NORMAL;
    } else {
      trig_mod_ = TRIGMODE::RELAX;
    }
  }
  void FricControl() {

    switch (fric_mod_) {
      case FRICMODE::RELAX: {
        motor_fric_0_->CurrentControl(0);
        motor_fric_1_->CurrentControl(0);
      } break;
      case FRICMODE::SAFE: {
        float out_rpm_0 = LowPass(0, motor_fric_0_->GetRPM());
        float out_rpm_1 = LowPass(0, motor_fric_1_->GetRPM());
        if (motor_fric_0_->GetRPM() < 200 || motor_fric_1_->GetRPM() < 200) {
          out_rpm_0 = 0;
          out_rpm_1 = 0;
        }
        fric_out_left_ =
            pid_fric_0_.Calculate(out_rpm_0, motor_fric_0_->GetRPM(), dt_);
        fric_out_right_ =
            pid_fric_1_.Calculate(out_rpm_1, motor_fric_1_->GetRPM(), dt_);
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
      }
      default:
        break;
    }
    last_fric_mod_ = fric_mod_;
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
          target_trig_angle_ += launcher::param::TRIGSTEP;
        }
        TrigAngleControl(target_trig_angle_);
        last_trig_angle_ = target_trig_angle_;
      } break;

      case TRIGMODE::CONTINUE: {
        float since_last = (now - last_trig_time_).ToSecondf();
        if (param_.trig_freq_ > 0.0f) {
          float trig_speed = 1.0f / param_.trig_freq_;
          if (since_last >= trig_speed) {
            target_trig_angle_ = target_trig_angle_ + launcher::param::TRIGSTEP;
            last_trig_time_ = now;
          }
        }
        TrigAngleControl(target_trig_angle_);

        last_trig_angle_ = target_trig_angle_;
      } break;
      case TRIGMODE::JAM: {
        // 正转卡弹时反转，反转卡弹时正转
        jam_keep_time_ =
            static_cast<uint32_t>((now - last_jam_time_).ToSecondf());
        if (jam_keep_time_ > 0.02) {
          if (last_trig_mod_ != TRIGMODE::JAM) {
            is_reverse_ = 1;
          }
          if (is_reverse_) {
            target_trig_angle_ = trig_angle_ - 2 * launcher::param::TRIGSTEP;
            TrigAngleControl(target_trig_angle_);
          } else if (!is_reverse_) {
            target_trig_angle_ = trig_angle_ + launcher::param::TRIGSTEP;
            TrigAngleControl(target_trig_angle_);
          }
          is_reverse_ = !is_reverse_;
          last_jam_time_ = now;
        }
      } break;

      default:
        break;
    }

    last_trig_mod_ = trig_mod_;
  }
  void Control() {
    auto now = LibXR::Timebase::GetMilliseconds();

    this->SetTrig();

    switch (launcherstate_) {
      case LauncherState::STOP:
      case LauncherState::OVERHEAT:
        trig_mod_ = TRIGMODE::RELAX;
        break;
      case LauncherState::NORMAL:

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


  void SetMode(uint32_t mode) { fric_mod_ = static_cast<FRICMODE>(mode); }

 private:
  LauncherState launcherstate_ = LauncherState::STOP;
  LauncherParam param_;
  RefereeData referee_data_;
  HeatLimit heat_limit_;
  TRIGMODE last_trig_mod_ = TRIGMODE::RELAX;
  TRIGMODE trig_mod_ = TRIGMODE::RELAX;
  FRICMODE fric_mod_ = FRICMODE::RELAX;
  FRICMODE last_fric_mod_ = FRICMODE::RELAX;

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
  CMD::LauncherCMD launcher_cmd_;

  CMD *cmd_;
  LibXR::MillisecondTimestamp last_online_time_ = 0.0f;
  float dt_ = 0;
  LibXR::MillisecondTimestamp last_jam_time_ = 0.0f;
  LibXR::MillisecondTimestamp jam_keep_time_ = 0.0f;
  bool press_continue_ = false;
  bool last_fire_notify_ = false;
  LibXR::MillisecondTimestamp fire_press_time_ = 0;
  LibXR::MillisecondTimestamp last_heat_time_ = 0.0f;
  LibXR::MillisecondTimestamp last_check_time_ = 0.0f;
  LibXR::Thread thread_;
  LibXR::Semaphore semaphore_;
  LibXR::Mutex mutex_;
  uint32_t launcher_event_;
  struct ShotJudge {
    bool active = false;
    float t = 0.0f;
  };

  ShotJudge shot_;

  float fric_out_left_ = 0.0f;
  float fric_out_right_ = 0.0f;
  bool is_reverse_ = 0;
  float out_rpm_0_ = 0;
  float out_rpm_1_ = 0;
  /*---------------------工具函数--------------------------------------------------*/
  void TrigAngleControl(float target_angle) {
    float plate_omega_ref = pid_trig_angle_.Calculate(
        target_angle, trig_angle_,
        motor_trig_->GetOmega() / param_.trig_gear_ratio, dt_);
    float motor_omega_ref = std::clamp(
        plate_omega_ref,
        static_cast<float>(-1.5 * M_2PI * param_.trig_freq_ / param_.num_trig_tooth),
        static_cast<float>(1.5 * M_2PI * param_.trig_freq_ / param_.num_trig_tooth));
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
