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
      default_bullet_speed: 0.0
      fric_radius: 0.0
      trig_gear_ratio: 0.0
      num_trig_tooth: 0
      expect_trig_freq_: 0.0
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
#include "Launcher.hpp"
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

namespace launcher::param {
constexpr float TRIGSTEP = static_cast<float>(M_2PI) / 10;
constexpr float HEAT_CHECK_TIME = 0.1;
constexpr float CHECK_JAM_TIME = 0.02;
constexpr float JAM_CURRENT = 8.0f;
constexpr float SHOT_WINDOW = 0.004f;
constexpr float MAX_FRIC_CUR = 2.0f;
constexpr float MIN_FRIC_RPM = 200.0f;
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
    /*默认弹速*/
    float default_bullet_speed;
    /*摩擦轮半径*/
    float fric_radius;
    /*拨弹盘电机减速比*/
    float trig_gear_ratio;
    /*拨齿数目*/
    uint8_t num_trig_tooth;
    /*弹频*/
    float expect_trig_freq_;
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
   * @param  fric_radius_ 摩擦轮半径
   * @param trig_gear_ratio_ 拨弹电机减速比
   * @param  num_trig_tooth_ 拨弹盘中一圈能存储几颗弹丸
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

    this->SetMode(launcher_event_);
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
    thread_.Create(this, ThreadFunction, "LauncherThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);
  }

  static void ThreadFunction(InfantryLauncher *launcher) {
    auto now = LibXR::Timebase::GetMilliseconds();
    launcher->dt_ = (now - launcher->last_online_time_).ToSecondf();
    launcher->last_online_time_ = now;

    while (1) {
      launcher->mutex_.Lock();
      launcher->Update();
      launcher->Heat();
      launcher->FricControl();
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
    /*获取数据*/
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
  }
  void FricControl() {
    switch (fric_mod_) {
      case FRICMODE::RELAX: {
        ready_ = false;
        motor_fric_0_->CurrentControl(0);
        motor_fric_1_->CurrentControl(0);
      } break;
      case FRICMODE::SAFE: {
        ready_ = false;
        out_rpm_0_ = LowPass(0, motor_fric_0_->GetRPM());
        out_rpm_1_ = LowPass(0, motor_fric_1_->GetRPM());
        /*防止震荡*/
        if (motor_fric_0_->GetRPM() < launcher::param::MIN_FRIC_RPM ||
            motor_fric_1_->GetRPM() < launcher::param::MIN_FRIC_RPM) {
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
      }
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
          target_trig_angle_ += launcher::param::TRIGSTEP;
        }
        TrigAngleControl(target_trig_angle_);
        if (target_trig_angle_ > last_trig_angle_) {
          BeginShotJudge();
        }
        last_trig_angle_ = target_trig_angle_;
      } break;

      case TRIGMODE::CONTINUE: {
        float since_last = (now - last_trig_time_).ToSecondf();
        if (trig_freq_ > 0.0f) {
          float trig_speed = 1.0f / trig_freq_;
          if (since_last >= trig_speed) {
            target_trig_angle_ = target_trig_angle_ + launcher::param::TRIGSTEP;
            last_trig_time_ = now;
          }
        }
        TrigAngleControl(target_trig_angle_);

        if (target_trig_angle_ > last_trig_angle_) {
          BeginShotJudge();
        }
        last_trig_angle_ = target_trig_angle_;
      } break;
      case TRIGMODE::JAM: {
        // 正转卡弹时反转，反转卡弹时正转
        jam_keep_time_ =
            static_cast<uint32_t>((now - last_jam_time_).ToSecondf());
        if (static_cast<float>(jam_keep_time_) >
            launcher::param::CHECK_JAM_TIME) {
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

    if (fabs(motor_trig_->GetCurrent()) > launcher::param::JAM_CURRENT) {
      launcherstate_ = LauncherState::JAMMED;
    } else if (!heat_limit_.allow_fire) {
      launcherstate_ = LauncherState::OVERHEAT;
    } else if (fric_mod_ != FRICMODE::READY) {
      launcherstate_ = LauncherState::STOP;
    } else if (launcher_cmd_.isfire) {
      launcherstate_ = LauncherState::NORMAL;
    } else {
      trig_mod_ = TRIGMODE::SAFE;
    }

    switch (launcherstate_) {
      case LauncherState::STOP:
      case LauncherState::OVERHEAT:
        trig_mod_ = TRIGMODE::SAFE;
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
    /*判断是否成功发射*/
    UpdateShotJudge();
    last_fire_notify_ = launcher_cmd_.isfire;
  }

  void Heat() {
    auto now = LibXR::Timebase::GetMilliseconds();

    float delta_time = (now - last_heat_time_).ToSecondf();
    if (delta_time >= launcher::param::HEAT_CHECK_TIME) {
      /*每周期都计算此周期的剩余热量*/
      last_heat_time_ = now;
      heat_limit_.current_heat +=
          heat_limit_.single_heat * heat_limit_.launched_num;
      heat_limit_.launched_num = 0;

      if (heat_limit_.current_heat <
          (referee_data_.heat_cooling /
           static_cast<float>(param_.num_trig_tooth))) {
        heat_limit_.current_heat = 0;
      } else {
        heat_limit_.current_heat -= referee_data_.heat_cooling /
                                    static_cast<float>(param_.num_trig_tooth);
      }

      float residuary_heat =
          referee_data_.heat_limit - heat_limit_.current_heat;

      /*控制control里的launcherstate*/
      if (residuary_heat >= heat_limit_.single_heat) {
        heat_limit_.allow_fire = true;
      } else {
        heat_limit_.allow_fire = false;
      }

      /*不同剩余热量启用不同实际弹频*/
      if (heat_limit_.allow_fire) {
        if (residuary_heat <= heat_limit_.single_heat + 0.001) {
          trig_freq_ = referee_data_.heat_cooling / heat_limit_.single_heat;
        } else if (residuary_heat <=
                   heat_limit_.single_heat * heat_limit_.heat_threshold) {
          float ratio = (residuary_heat - heat_limit_.single_heat) /
                        (heat_limit_.single_heat * heat_limit_.heat_threshold -
                         heat_limit_.single_heat);

          ratio = std::max(0.0f, std::min(1.0f, ratio));
          /*计算实际发射频率*/
          float safe_freq =
              referee_data_.heat_cooling / heat_limit_.single_heat;
          trig_freq_ =
              ratio * param_.expect_trig_freq_ + (1.0f - ratio) * safe_freq;
        } else {
          trig_freq_ = param_.expect_trig_freq_;
        }
      }
    }
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

  float target_trig_angle_ = 0.0f;
  float trig_angle_ = 0.0f;
  float last_trig_angle_ = 0.0f;
  float trig_freq_ = 0.0f;
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
  bool ready_ = false;
  float out_rpm_0_ = 0;
  float out_rpm_1_ = 0;
  /*---------------------工具函数--------------------------------------------------*/
  void TrigAngleControl(float target_angle) {
    float plate_omega_ref = pid_trig_angle_.Calculate(
        target_angle, trig_angle_,
        motor_trig_->GetOmega() / param_.trig_gear_ratio, dt_);
    float motor_omega_ref = std::clamp(
        plate_omega_ref,
        static_cast<float>(-1.5 * M_2PI * trig_freq_ / param_.num_trig_tooth),
        static_cast<float>(1.5 * M_2PI * trig_freq_ / param_.num_trig_tooth));
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
  void BeginShotJudge() {
    shot_.active = true;
    shot_.t = 0.0f;
  }

  void UpdateShotJudge() {
    auto now = LibXR::Timebase::GetMilliseconds();

    if (!shot_.active) {
      return;
    }
    shot_.t = (now - last_check_time_).ToSecondf();

    if (shot_.t < launcher::param::SHOT_WINDOW) {
      return;
    }
    bool success =
        (motor_fric_0_->GetCurrent() > launcher::param::MAX_FRIC_CUR) &&
        (motor_fric_1_->GetCurrent() > launcher::param::MAX_FRIC_CUR);
    if (success) {
      heat_limit_.launched_num++;
      shot_.active = false;
    }
    last_check_time_ = now;
  }
};
