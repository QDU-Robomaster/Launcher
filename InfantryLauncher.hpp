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

#include <algorithm>
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
                   LibXR::Thread::Priority::MEDIUM);

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

    while (1) {
      auto now = LibXR::Timebase::GetMilliseconds();
      launcher->dt_ = (now - launcher->last_online_time_).ToSecondf();
      launcher->last_online_time_ = now;

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
    referee_data_.heat_cooling = 20.0f;
    heat_limit_.single_heat = 10.0f;
    heat_limit_.heat_threshold = 200.0f;

    motor_fric_0_->Update();
    motor_fric_1_->Update();
    motor_trig_->Update();

    param_frirc_0_ = motor_fric_0_->GetFeedback();
    param_frirc_1_ = motor_fric_1_->GetFeedback();
    param_trig_ = motor_trig_->GetFeedback();

    static float last_motor_angle = 0.0f;
    static bool initialized = false;

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


  }
  void FricControl() {
    switch (launcher_event_) {

      case LauncherEvent::SET_FRICMODE_RELAX: {
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

        cmd_fric_0_.velocity =
            pid_fric_0_.Calculate(out_rpm_0, param_frirc_0_.velocity, dt_);
        cmd_fric_1_.velocity =
            pid_fric_1_.Calculate(out_rpm_1, param_frirc_1_.velocity, dt_);
      } break;
      case LauncherEvent::SET_FRICMODE_READY: {
        out_rpm_0_ = param_.fric1_setpoint_speed;
        out_rpm_1_ = param_.fric1_setpoint_speed;

        cmd_fric_0_.velocity =
            pid_fric_0_.Calculate(out_rpm_0_, param_frirc_0_.velocity, dt_);
        cmd_fric_1_.velocity =
            pid_fric_1_.Calculate(out_rpm_1_, param_frirc_1_.velocity, dt_);
      } break;
      default:
        break;
    }
    motor_fric_0_->Control(cmd_fric_0_);
    motor_fric_1_->Control(cmd_fric_1_);
  }

  void SetTrig() {
    auto now = LibXR::Timebase::GetMilliseconds();

    switch (trig_mod_) {
      case TRIGMODE::RELAX:
        motor_trig_->Relax();
        break;
      case TRIGMODE::SAFE:
        target_trig_angle_ = trig_angle_;

        break;
      case TRIGMODE::SINGLE: {
        if (last_trig_mod_ == TRIGMODE::SAFE ||
            last_trig_mod_ == TRIGMODE::RELAX) {
          target_trig_angle_ += launcher::param::TRIGSTEP;
        }
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
            target_trig_angle_ += launcher::param::TRIGSTEP;
            last_trig_time_ = now;
          }
        }

        if (target_trig_angle_ > last_trig_angle_) {
          BeginShotJudge();
        }
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
          } else if (!is_reverse_) {
            target_trig_angle_ = trig_angle_ + launcher::param::TRIGSTEP;
          }
          is_reverse_ = !is_reverse_;
          last_jam_time_ = now;
        }
      } break;

      default:
        break;
    }

    float plate_omega_ref = pid_trig_angle_.Calculate(
        target_trig_angle_, trig_angle_,
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

    if (fabs(param_trig_.torque) > launcher::param::JAM_CURRENT) {
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

    switch (launcherstate_) {
      case LauncherState::STOP:
        trig_mod_ = TRIGMODE::RELAX;
        break;
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
    this->SetTrig();

    /*判断是否成功发射*/
    UpdateShotJudge();
    last_fire_notify_ = launcher_cmd_.isfire;
  }
  void Heat() {
    auto now = LibXR::Timebase::GetMilliseconds();
    float dt = (now - last_heat_time_).ToSecondf();
    if (dt <= 0.0f){ return;}
    last_heat_time_ = now;

    heat_limit_.current_heat = std::max(
        0.0f, heat_limit_.current_heat - referee_data_.heat_cooling * dt);

    const float Q0 = referee_data_.heat_limit;
    const float Q = heat_limit_.current_heat;
    const float H = heat_limit_.single_heat;
    const float C = referee_data_.heat_cooling;

     float freq_expect= param_.expect_trig_freq_;
     float freq_safe = C / H;


    if (Q < 0.6f * Q0) {
      trig_freq_ = freq_expect;
    } else if (Q < Q0 - H) {
      constexpr float PREDICT_T = 0.25f;

      float f_max = (Q0 - Q + C * PREDICT_T) / (H * PREDICT_T);

      trig_freq_ = std::clamp(f_max, freq_safe, freq_expect);
    } else {
      trig_freq_ = freq_safe;
    }

    trig_freq_ = std::clamp(trig_freq_, 0.0f, freq_expect);
  }

  void SetMode(uint32_t mode) {
    mutex_.Lock();
    launcher_event_ = static_cast<LauncherEvent>(mode);
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
    last_check_time_ = LibXR::Timebase::GetMilliseconds();
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
      number_++;
    }
    last_check_time_ = now;
  }

 private:
  LauncherState launcherstate_ = LauncherState::STOP;
  LauncherParam param_;
  RefereeData referee_data_;
  HeatLimit heat_limit_{
      .single_heat = 0.0f,
      .launched_num = 0.0f,
      .current_heat = 0.0f,
      .heat_threshold = 0.0f,
      .allow_fire = true,
  };
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
  Motor::MotorCmd cmd_fric_0_ =
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 19.2f,
                      .velocity = 0};
  Motor::MotorCmd cmd_fric_1_ =
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 19.2f,
                      .velocity = 0};
  Motor::MotorCmd cmd_trig_ =
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 36.0f,
                      .velocity = 0};

  bool is_reverse_ = 0;
  float out_rpm_0_ = 0;
  float out_rpm_1_ = 0;
  float target_trig_angle_ = 0.0f;
  float number_=0;
};
