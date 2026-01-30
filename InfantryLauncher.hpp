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
#include "Motor.hpp"
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
constexpr float JAM_CURRENT = 5.0f;
constexpr float SHOT_WINDOW = 0.004f;  // 20 ms
constexpr float DELTA_RPM = 50.0f;     // rpm
}  // namespace launcher::param

class InfantryLauncher {
 public:
  enum class LauncherState : uint8_t {
    STOP,
    NORMAL,
    JAMMED,
    OVERHEAT,
  };

  enum class LauncherEvent : uint8_t {
    SET_FRICMODE_RELAX,
    SET_FRICMODE_SAFE,
    SET_FRICMODE_READY,
  };

  enum class TRIGMODE : uint8_t { RELAX, SAFE, SINGLE, CONTINUE, JAM };

  typedef struct {
    float heat_limit;
    float heat_cooling;
  } RefereeData;
  struct LauncherParam {
    /*一级摩擦轮转速*/
    float fric1_setpoint_speed;
    /*二级摩擦轮转速*/
    float fric2_setpoint_speed;
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
                   LibXR::Thread::Priority::HIGH);
    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, InfantryLauncher *launcher, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          launcher->SetMode(
              static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_RELAX));
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
  }

  static void ThreadFunction(InfantryLauncher *launcher) {
    LibXR::Topic::ASyncSubscriber<CMD::LauncherCMD> launcher_cmd_tp(
        "launcher_cmd");
    launcher_cmd_tp.StartWaiting();
    auto now = LibXR::Timebase::GetMilliseconds();
    launcher->dt_ = (now - launcher->last_online_time_).ToSecondf();
    launcher->last_online_time_ = now;

    while (1) {
      launcher->Update();
      launcher->Heat();
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

    param_frirc_0_ = motor_fric_0_->GetFeedback();
    param_frirc_1_ = motor_fric_1_->GetFeedback();
    param_trig_ = motor_trig_->GetFeedback();

    cmd_trig_.mode = Motor::ControlMode::MODE_CURRENT;
    cmd_fric_0_.mode = Motor::ControlMode::MODE_CURRENT;
    cmd_fric_1_.mode = Motor::ControlMode::MODE_CURRENT;

    static float last_motor_angle = 0.0f;
    static bool initialized = false;

    motor_fric_0_->Update();
    motor_fric_1_->Update();
    motor_trig_->Update();

    float current_motor_angle = param_trig_.position;

    if (!initialized) {
      last_motor_angle = current_motor_angle;
      initialized = true;
      return;
    }

    float delta_trig_angle = LibXR::CycleValue<float>(current_motor_angle) -
                             LibXR::CycleValue<float>(last_motor_angle);

    trig_angle_ += delta_trig_angle / param_.trig_gear_ratio;
    last_motor_angle = current_motor_angle;

    if (fabs(param_trig_.omega) < 0.5f &&
        fabs(param_trig_.torque) > launcher::param::JAM_CURRENT) {
      launcherstate_ = LauncherState::JAMMED;
    } else if (!heat_limit_.allow_fire) {
      launcherstate_ = LauncherState::OVERHEAT;
    } else if (launcher_event_ != LauncherEvent::SET_FRICMODE_READY) {
      launcherstate_ = LauncherState::STOP;
    } else if (launcher_cmd_.isfire) {
      launcherstate_ = LauncherState::NORMAL;
    } else {
      trig_mod_ = TRIGMODE::RELAX;
    }
  }
  void FricControl() {
    switch (launcher_event_) {
      case (LauncherEvent::SET_FRICMODE_RELAX): {
        motor_fric_0_->Relax();
        motor_fric_1_->Relax();
      } break;
      case LauncherEvent::SET_FRICMODE_SAFE: {
        float out_rpm_0 = SoftTranslate(0, param_frirc_0_.velocity);
        float out_rpm_1 = SoftTranslate(0, param_frirc_1_.velocity);
        if (param_frirc_0_.velocity < 200 || param_frirc_1_.velocity < 200) {
          out_rpm_0 = 0;
          out_rpm_1 = 0;
        }
        fric_out_left_ =
            pid_fric_0_.Calculate(out_rpm_0, param_frirc_0_.velocity, dt_);
        fric_out_right_ =
            pid_fric_1_.Calculate(out_rpm_1, param_frirc_1_.velocity, dt_);
        motor_fric_0_->Control(cmd_fric_0_);
        motor_fric_1_->Control(cmd_fric_1_);
      } break;
      case LauncherEvent::SET_FRICMODE_READY: {
        cmd_fric_0_.velocity = param_.fric1_setpoint_speed;
        cmd_fric_1_.velocity = param_.fric1_setpoint_speed;

        fric_out_left_ =
            pid_fric_0_.Calculate(out_rpm_0_, param_frirc_0_.velocity, dt_);
        fric_out_right_ =
            pid_fric_1_.Calculate(out_rpm_1_, param_frirc_1_.velocity, dt_);
        motor_fric_0_->Control(cmd_fric_0_);
        motor_fric_1_->Control(cmd_fric_1_);
      }
      default:
        break;
    }
  }

  void SetTrig() {
    auto now = LibXR::Timebase::GetMilliseconds();
    switch (trig_mod_) {
      case TRIGMODE::RELAX:
        motor_trig_->Relax();
        break;
      case TRIGMODE::SAFE:
        cmd_trig_.position = trig_angle_;

        break;
      case TRIGMODE::SINGLE: {
        if (last_trig_mod_ == TRIGMODE::SAFE) {
          cmd_trig_.position += launcher::param::TRIGSTEP;
        }
        if (cmd_trig_.position > last_trig_angle_) {
          BeginShotJudge();
        }
        last_trig_angle_ = cmd_trig_.position;
      } break;

      case TRIGMODE::CONTINUE: {
        float since_last = (now - last_trig_time_).ToSecondf();
        if (trig_freq_ > 0.0f) {
          float trig_speed = 1.0f / trig_freq_;
          if (since_last >= trig_speed) {
            cmd_trig_.position = launcher::param::TRIGSTEP;
            last_trig_time_ = now;
          }
        }

        if (cmd_trig_.position > last_trig_angle_) {
          BeginShotJudge();
        }
        last_trig_angle_ = cmd_trig_.position;
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
            cmd_trig_.position = trig_angle_ - 2 * launcher::param::TRIGSTEP;
          } else if (!is_reverse_) {
            cmd_trig_.position = trig_angle_ + launcher::param::TRIGSTEP;
          }
          is_reverse_ = !is_reverse_;
          last_jam_time_ = now;
        }
      } break;

      default:
        break;
    }
    float plate_omega_ref = pid_trig_angle_.Calculate(
        cmd_trig_.position, trig_angle_,
        param_trig_.omega / param_.trig_gear_ratio, dt_);
    float motor_omega_ref = std::clamp(
        plate_omega_ref,
        static_cast<float>(-1.5 * M_2PI * trig_freq_ / param_.num_trig_tooth),
        static_cast<float>(1.5 * M_2PI * trig_freq_ / param_.num_trig_tooth));
    cmd_trig_.velocity = pid_trig_sp_.Calculate(
        motor_omega_ref, param_trig_.omega / param_.trig_gear_ratio, dt_);

    motor_trig_->Control(cmd_trig_);

    last_trig_mod_ = trig_mod_;
  }
  void Control() {
    auto now = LibXR::Timebase::GetMilliseconds();

    this->SetTrig();

    switch (launcherstate_) {
      case LauncherState::STOP:
      trig_mod_ = TRIGMODE::SAFE;
      break;
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
    /*判断是否成功发射*/
    UpdateShotJudge();
    last_fire_notify_ = launcher_cmd_.isfire;
  }

  void Heat() {
    auto now = LibXR::Timebase::GetMilliseconds();

    float delta_time = (now - last_heat_time_).ToSecondf();
    if (delta_time >= 0.1) {
      /*每周期都计算此周期的剩余热量*/
      last_heat_time_ = now;
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

  void SetMode(uint32_t mode) {
    mutex_.Lock();
    launcher_event_ = static_cast<LauncherEvent>(mode);
    pid_fric_0_.Reset();
    pid_fric_1_.Reset();
    pid_trig_angle_.Reset();
    pid_trig_sp_.Reset();
    mutex_.Unlock();
  }
  /*指数缓变*/
  float SoftTranslate(float target, float cur) {
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

    /*未激活，直接返回*/
    if (!shot_.active) {
      return;
    }
    shot_.t = (now - last_check_time_).ToSecondf();

    if (shot_.t < launcher::param::SHOT_WINDOW) {
      return;
    }
    /*暂时这样检测发射弹丸数，高速时准确，低弹频低速不准确*/
    bool success =
        (fabs(param_frirc_0_.velocity) <
         (param_.fric1_setpoint_speed - launcher::param::DELTA_RPM)) &&
        (fabs(param_frirc_1_.velocity) <
         (param_.fric1_setpoint_speed - launcher::param::DELTA_RPM));
    if (success) {
      heat_limit_.launched_num++;
      shot_.active = false;
    }
    last_check_time_ = now;
  }

 private:
  LauncherState launcherstate_ = LauncherState::STOP;
  LauncherParam param_;
  RefereeData referee_data_;
  HeatLimit heat_limit_;
  TRIGMODE last_trig_mod_ = TRIGMODE::RELAX;
  TRIGMODE trig_mod_ = TRIGMODE::RELAX;

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
  LauncherEvent launcher_event_ = LauncherEvent::SET_FRICMODE_RELAX;
  struct ShotJudge {
    bool active = false;
    float t = 0.0f;
  };

  ShotJudge shot_;
  Motor::Feedback param_frirc_0_;
  Motor::Feedback param_frirc_1_;
  Motor::Feedback param_trig_;
  Motor::MotorCmd cmd_fric_0_;
  Motor::MotorCmd cmd_fric_1_;
  Motor::MotorCmd cmd_trig_;

  float fric_out_left_ = 0.0f;
  float fric_out_right_ = 0.0f;
  bool is_reverse_ = 0;
  float out_rpm_0_ = 0;
  float out_rpm_1_ = 0;
};
