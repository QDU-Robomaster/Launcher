#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "CMD.hpp"
#ifdef DEBUG
#include "DebugCore.hpp"
#include "ramfs.hpp"
#endif
#include "Motor.hpp"
#include "RMMotor.hpp"
#include "Referee.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "message.hpp"
#include "pid.hpp"
#include "timebase.hpp"

namespace launcher::param {
/// 拨弹盘每发转过的角度 (rad)
/// Angle the trigger disc advances per round (rad)
constexpr float TRIG_STEP = static_cast<float>(LibXR::TWO_PI) / 10.0f;
/// 判定卡弹的拨弹电机力矩
/// Trigger motor torque that is taken as a jam
constexpr float JAM_TORQUE = 0.015f;
/// 判定出弹时摩擦轮转速相对目标的下降量 (rpm)
/// Friction wheel speed drop below the target taken as a round leaving (rpm)
constexpr float FRIC_DROP_RPM = 218.0f;
/// 距上一个卡弹处理周期超过该时间才重新设定退弹目标 (s)
/// Time since the previous jam handling cycle after which a new back-off
/// target is set (s)
constexpr float JAM_TOGGLE_INTERVAL_SEC = 0.08f;
/// 按住超过该时间转为连发 (s)
/// Holding for longer than this switches to continuous fire (s)
constexpr float LONG_PRESS_THRESHOLD_SEC = 0.5f;
/// 热量控制的更新周期 (s)
/// Update period of the heat control (s)
constexpr float HEAT_TICK_SEC = 0.05f;
}  // namespace launcher::param

/**
 * @brief 步兵发射机构实现：摩擦轮与拨弹盘控制，按热量限制射频。
 *        Infantry launcher implementation: friction wheel and trigger disc
 *        control, with the fire rate limited by the heat.
 *
 * 作为 Launcher<InfantryLauncher> 的内部逻辑类，线程和事件注册由外壳提供。
 * As the internal logic class of Launcher<InfantryLauncher>, the thread and
 * the event registration are provided by the shell.
 */
class InfantryLauncher {
 public:
  /**
   * @brief 发射状态。
   *        Launcher states.
   */
  enum class LauncherState : uint8_t {
    RELAX,   ///< 放松 Relax
    STOP,    ///< 停止发射 Firing stopped
    NORMAL,  ///< 正常发射 Normal firing
    JAMMED,  ///< 卡弹 Jammed
  };

  /**
   * @brief 摩擦轮模式事件，数值同时是事件 ID。
   *        Friction wheel mode events; the values are also the event IDs.
   */
  enum class LauncherEvent : uint8_t {
    SET_FRICMODE_RELAX,  ///< 放松 Relax
    SET_FRICMODE_SAFE,   ///< 安全：目标转速为 0 Safe: target speed 0
    SET_FRICMODE_READY,  ///< 就绪 Ready
  };

  /**
   * @brief 拨弹模式。
   *        Trigger modes.
   */
  enum class TrigMode : uint8_t {
    RELAX,     ///< 放松 Relax
    SAFE,      ///< 安全：保持当前位置 Safe: hold the current position
    SINGLE,    ///< 单发 Single shot
    CONTINUE,  ///< 连发 Continuous fire
    JAM,       ///< 卡弹处理 Jam handling
  };

  /**
   * @brief 发射器参数。
   *        Launcher parameters.
   */
  struct LauncherParam {
    float fric1_setpoint_speed;  ///< 摩擦轮目标转速
                                 ///< Friction wheel target speed
    float fric2_setpoint_speed;  ///< 二级摩擦轮目标转速，此实现不使用
                                 ///< Second-stage wheel target speed, not used
                                 ///< by this implementation
    float trig_gear_ratio;       ///< 拨弹电机减速比
                                 ///< Trigger motor reduction ratio
    uint8_t num_trig_tooth;      ///< 拨弹盘齿数
                                 ///< Number of trigger disc teeth
    float expect_trig_freq_;     ///< 期望弹频 (Hz)
                                 ///< Expected fire rate (Hz)
  };

  /**
   * @brief 热量控制状态。
   *        Heat control state.
   */
  struct HeatLimit {
    float single_heat;     ///< 单发热量 Heat per round
    float launched_num;    ///< 本周期判定的发射数 Rounds counted this cycle
    float current_heat;    ///< 当前热量 Current heat
    float heat_threshold;  ///< 开始降低弹频的剩余热量，以单发热量为单位
                           ///< Remaining heat at which the fire rate starts to
                           ///< drop, in units of the heat per round
    bool allow_fire;       ///< 是否允许发射 Whether firing is allowed
  };

  /**
   * @brief 构造 InfantryLauncher。
   *        Construct InfantryLauncher.
   *
   * @param motor_fric_front_left 前左摩擦轮电机。
   *                              Front-left friction wheel motor.
   * @param motor_fric_front_right 前右摩擦轮电机。
   *                               Front-right friction wheel motor.
   * @param motor_fric_back_left 后左摩擦轮电机，此实现不使用。
   *                             Back-left friction wheel motor, not used by
   *                             this implementation.
   * @param motor_fric_back_right 后右摩擦轮电机，此实现不使用。
   *                              Back-right friction wheel motor, not used by
   *                              this implementation.
   * @param motor_trig 拨弹电机。
   *                   Trigger motor.
   * @param task_stack_depth 控制线程栈深，由外壳使用。
   *                         Control thread stack depth, used by the shell.
   * @param pid_param_trig_angle 拨弹角度环参数。
   *                             Trigger angle-loop parameters.
   * @param pid_param_trig_speed 拨弹速度环参数。
   *                             Trigger speed-loop parameters.
   * @param pid_param_fric_0 摩擦轮 0 速度环参数。
   *                         Friction wheel 0 speed-loop parameters.
   * @param pid_param_fric_1 摩擦轮 1 速度环参数。
   *                         Friction wheel 1 speed-loop parameters.
   * @param pid_param_fric_2 预留参数，此实现不使用。
   *                         Reserved, not used by this implementation.
   * @param pid_param_fric_3 预留参数，此实现不使用。
   *                         Reserved, not used by this implementation.
   * @param launch_param 发射机构参数。
   *                     Launcher parameters.
   * @param cmd CMD 实例指针。
   *            Pointer to the CMD instance.
   */
  InfantryLauncher(RMMotor* motor_fric_front_left,
                   RMMotor* motor_fric_front_right,
                   RMMotor* motor_fric_back_left,
                   RMMotor* motor_fric_back_right, RMMotor* motor_trig,
                   uint32_t task_stack_depth,
                   LibXR::PID<float>::Param pid_param_trig_angle,
                   LibXR::PID<float>::Param pid_param_trig_speed,
                   LibXR::PID<float>::Param pid_param_fric_0,
                   LibXR::PID<float>::Param pid_param_fric_1,
                   LibXR::PID<float>::Param pid_param_fric_2,
                   LibXR::PID<float>::Param pid_param_fric_3,
                   LauncherParam launch_param, CMD* cmd)
      : motor_fric_0_(motor_fric_front_left),
        motor_fric_1_(motor_fric_front_right),
        motor_trig_(motor_trig),
        pid_trig_angle_(pid_param_trig_angle),
        pid_trig_sp_(pid_param_trig_speed),
        pid_fric_0_(pid_param_fric_0),
        pid_fric_1_(pid_param_fric_1),
        param_(launch_param) {
    UNUSED(task_stack_depth);
    UNUSED(pid_param_fric_2);
    UNUSED(pid_param_fric_3);
    UNUSED(motor_fric_back_left);
    UNUSED(motor_fric_back_right);
    UNUSED(cmd);

    last_online_time_ = LibXR::Timebase::GetMicroseconds();
    last_heat_time_ = LibXR::Timebase::GetMilliseconds();
  }

  /**
   * @brief 更新电机反馈与拨弹盘角度，并刷新发射器状态。
   *        Update the motor feedback and the trigger disc angle, and refresh
   *        the launcher state.
   */
  void Update() {
    last_online_time_ = LibXR::Timebase::GetMicroseconds();

    motor_fric_0_->Update();
    motor_fric_1_->Update();
    motor_trig_->Update();

    param_fric_0_ = motor_fric_0_->GetFeedback();
    param_fric_1_ = motor_fric_1_->GetFeedback();
    param_trig_ = motor_trig_->GetFeedback();

    float current_motor_angle = param_trig_.position;
    float delta_trig_angle = LibXR::CycleValue<float>(current_motor_angle) -
                             LibXR::CycleValue<float>(last_motor_angle_);
    trig_angle_ += delta_trig_angle / param_.trig_gear_ratio;
    last_motor_angle_ = current_motor_angle;

    UpdateLauncherState();
  }

  /**
   * @brief 热量控制、拨弹状态机与话题发布，包含卡弹处理。
   *        Heat control, trigger state machine and Topic publishing, including
   *        jam handling.
   */
  void Solve() {
    UpdateHeatControl();
    RunStateMachine();
    UpdateShotLatency();
    PublishTopics();
  }

  /**
   * @brief 计算拨弹与摩擦轮的控制量并下发，包含电机状态检查与清错。
   *        Compute the control values of the trigger and the friction wheels
   *        and send them, including the motor state check and error clearing.
   */
  void Control() {
    float out_trig = 0.0f;
    float out_fric_0 = 0.0f;
    float out_fric_1 = 0.0f;
    Motor::Feedback trig_fb{};
    Motor::Feedback fric_0_fb{};
    Motor::Feedback fric_1_fb{};
    bool relax = false;

    SetFricTargetByEvent();

    if (launcher_event_ == LauncherEvent::SET_FRICMODE_RELAX) {
      relax = true;
    } else {
      if (trig_mode_ != TrigMode::RELAX) {
        TrigControl(out_trig, target_trig_angle_, dt_);
      }
      FricControl(out_fric_0, out_fric_1, target_rpm_, dt_);
      trig_fb = param_trig_;
      fric_0_fb = param_fric_0_;
      fric_1_fb = param_fric_1_;
    }

    if (relax) {
      motor_trig_->Relax();
      motor_fric_0_->Relax();
      motor_fric_1_->Relax();
      return;
    }

    auto cmd_trig = Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                                    .reduction_ratio = 36.0f,
                                    .velocity = out_trig};
    auto cmd_fric_0 = Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                                      .reduction_ratio = 1.0f,
                                      .velocity = out_fric_0};
    auto cmd_fric_1 = Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                                      .reduction_ratio = 1.0f,
                                      .velocity = out_fric_1};

    auto motor_control = [&](Motor* motor, const Motor::Feedback& fb,
                             const Motor::MotorCmd& cmd) {
      if (fb.state == 0) {
        motor->Enable();
      } else if (fb.state != 0 && fb.state != 1) {
        motor->ClearError();
      } else {
        motor->Control(cmd);
      }
    };

    motor_control(motor_trig_, trig_fb, cmd_trig);
    motor_control(motor_fric_0_, fric_0_fb, cmd_fric_0);
    motor_control(motor_fric_1_, fric_1_fb, cmd_fric_1);
  }

  /**
   * @brief 设置控制周期。
   *        Set the control period.
   *
   * @param dt 控制周期，单位 s。
   *           Control period in s.
   */
  void SetControlDt(float dt) { dt_ = dt; }

  /**
   * @brief 设置摩擦轮模式，并复位相关 PID。
   *        Set the friction wheel mode and reset the related PIDs.
   *
   * @param mode 事件 ID，对应 LauncherEvent。
   *             Event ID, corresponding to LauncherEvent.
   */
  void SetMode(uint32_t mode) {
    launcher_event_ = static_cast<LauncherEvent>(mode);
    pid_fric_0_.Reset();
    pid_fric_1_.Reset();
    pid_trig_angle_.Reset();
    pid_trig_sp_.Reset();
  }

  /**
   * @brief 失去控制时复位状态机与 PID，失能拨弹电机并放松摩擦轮。
   *        Reset the state machine and the PIDs when control is lost, disable
   *        the trigger motor and relax the friction wheels.
   */
  void LostCtrl() {
    launcher_event_ = LauncherEvent::SET_FRICMODE_RELAX;
    launcher_state_ = LauncherState::RELAX;
    trig_mode_ = TrigMode::RELAX;

    pid_fric_0_.Reset();
    pid_fric_1_.Reset();
    pid_trig_angle_.Reset();
    pid_trig_sp_.Reset();

    target_trig_angle_ = trig_angle_;
    shoot_active_ = false;
    shot_start_time_ = 0;
    press_continue_ = false;
    launcher_cmd_.isfire = false;

    motor_trig_->Disable();
    motor_fric_0_->Relax();
    motor_fric_1_->Relax();
  }

#ifdef DEBUG
  /**
   * @brief 调试命令入口，实现位于 InfantryLauncherDebug.inl。
   *        Debug command entry, implemented in InfantryLauncherDebug.inl.
   *
   * @param argc 参数数量。
   *             Argument count.
   * @param argv 参数数组。
   *             Argument array.
   * @return 命令返回值。
   *         Command return value.
   */
  int DebugCommand(int argc, char** argv);
#endif

  /// 外壳写入的发射命令
  /// Fire command written by the shell
  CMD::LauncherCMD launcher_cmd_{};  // NOLINT
  /// 外壳写入的裁判系统发射数据，提供热量上限与冷却值
  /// Referee launcher data written by the shell, providing the heat limit and
  /// the cooling value
  Referee::LauncherPack ref_data_{};  // NOLINT

 private:
  RMMotor* motor_fric_0_;
  RMMotor* motor_fric_1_;
  RMMotor* motor_trig_;

  Motor::Feedback param_fric_0_{};
  Motor::Feedback param_fric_1_{};
  Motor::Feedback param_trig_{};

  LibXR::PID<float> pid_trig_angle_;
  LibXR::PID<float> pid_trig_sp_;
  LibXR::PID<float> pid_fric_0_;
  LibXR::PID<float> pid_fric_1_;

  LauncherParam param_;

  float dt_ = 0.0f;
  float target_rpm_ = 0.0f;
  float trig_freq_ = 0.0f;

  float trig_angle_ = 0.0f;
  float target_trig_angle_ = 0.0f;
  float last_motor_angle_ = 0.0f;

  float number_ = 0.0f;
  float shoot_dt_ = 0.0f;

  bool last_fire_notify_ = false;
  bool press_continue_ = false;
  bool is_reverse_ = false;
  bool shoot_active_ = false;

  float jam_keep_time_s_ = 0.0f;

  LibXR::MillisecondTimestamp fire_press_time_ = 0;
  LibXR::MillisecondTimestamp last_trig_time_ = 0;
  LibXR::MillisecondTimestamp last_jam_time_ = 0;
  LibXR::MillisecondTimestamp last_heat_time_ = 0;
  LibXR::MicrosecondTimestamp last_online_time_ = 0;
  LibXR::MillisecondTimestamp shoot_time_ = 0;
  LibXR::MillisecondTimestamp receive_fire_time_ = 0;
  LibXR::MillisecondTimestamp shot_start_time_ = 0;

  LibXR::Topic shoot_waiting_ = LibXR::Topic::CreateTopic<float>("shoot_dt");
  LibXR::Topic shoot_number_ = LibXR::Topic::CreateTopic<float>("shoot_number");
  LibXR::Topic shoot_freq_ = LibXR::Topic::CreateTopic<float>("trig_freq");

  LauncherEvent launcher_event_ = LauncherEvent::SET_FRICMODE_RELAX;
  LauncherState launcher_state_ = LauncherState::RELAX;
  TrigMode trig_mode_ = TrigMode::RELAX;
  TrigMode last_trig_mode_ = TrigMode::RELAX;

  HeatLimit heat_limit_{
      .single_heat = 10.0f,
      .launched_num = 0.0f,
      .current_heat = 0.0f,
      .heat_threshold = 2.30f,
      .allow_fire = true,
  };

  /**
   * @brief 由卡弹判据、摩擦轮模式与热量许可计算发射状态。
   *        Compute the launcher state from the jam criterion, the friction
   *        wheel mode and the heat permission.
   */
  void UpdateLauncherState() {
    if (fabsf(param_trig_.torque) > launcher::param::JAM_TORQUE) {
      launcher_state_ = LauncherState::JAMMED;
      return;
    }

    if (launcher_event_ != LauncherEvent::SET_FRICMODE_READY) {
      launcher_state_ = LauncherState::RELAX;
      return;
    }

    if (!heat_limit_.allow_fire) {
      launcher_state_ = LauncherState::STOP;
      return;
    }

    launcher_state_ =
        launcher_cmd_.isfire ? LauncherState::NORMAL : LauncherState::STOP;
  }

  /**
   * @brief 依次更新拨弹模式、目标角度和出弹判定，并刷新开火边沿状态。
   *        Update the trigger mode, the target angle and the round decision in
   *        turn, and refresh the fire edge state.
   */
  void RunStateMachine() {
    auto now = LibXR::Timebase::GetMilliseconds();
    UpdateTriggerMode(now);
    UpdateTriggerSetpoint(now);
    UpdateShotJudge(now);
    last_fire_notify_ = launcher_cmd_.isfire;
  }

  /**
   * @brief 按发射状态切换拨弹模式，长按超过阈值进入连发。
   *        Switch the trigger mode from the launcher state; holding beyond the
   *        threshold enters continuous fire.
   *
   * @param now 当前时间戳。
   *            Current timestamp.
   */
  void UpdateTriggerMode(LibXR::MillisecondTimestamp now) {
    switch (launcher_state_) {
      case LauncherState::RELAX:
        trig_mode_ = TrigMode::RELAX;
        press_continue_ = false;
        break;

      case LauncherState::STOP:
        trig_mode_ = TrigMode::SAFE;
        press_continue_ = false;
        break;

      case LauncherState::NORMAL:
        if (!last_fire_notify_) {
          fire_press_time_ = now;
          press_continue_ = false;
          trig_mode_ = TrigMode::SINGLE;
        } else {
          if (!press_continue_ &&
              (now - fire_press_time_).ToSecondf() >
                  launcher::param::LONG_PRESS_THRESHOLD_SEC) {
            press_continue_ = true;
          }
          trig_mode_ = press_continue_ ? TrigMode::CONTINUE : TrigMode::SINGLE;
        }
        break;

      case LauncherState::JAMMED:
        trig_mode_ = TrigMode::JAM;
        break;
    }
  }

  /**
   * @brief 按拨弹模式生成目标角度；进入卡弹处理时拨弹盘反转 0.8 发的角度。
   *        Generate the target angle from the trigger mode; on entering jam
   *        handling the trigger disc turns back by 0.8 of the per-round angle.
   *
   * @param now 当前时间戳。
   *            Current timestamp.
   */
  void UpdateTriggerSetpoint(LibXR::MillisecondTimestamp now) {
    switch (trig_mode_) {
      case TrigMode::RELAX:
      case TrigMode::SAFE:
        target_trig_angle_ = trig_angle_;
        shoot_active_ = false;
        shot_start_time_ = 0;
        break;

      case TrigMode::SINGLE:
        if (last_trig_mode_ == TrigMode::SAFE ||
            last_trig_mode_ == TrigMode::RELAX ||
            last_trig_mode_ == TrigMode::JAM) {
          target_trig_angle_ = trig_angle_ + launcher::param::TRIG_STEP;
          shoot_active_ = true;
          shot_start_time_ = now;
        }
        break;

      case TrigMode::CONTINUE: {
        if (!shoot_active_) {
          float trig_freq = std::max(trig_freq_, 1e-3f);
          float interval_s = 1.0f / trig_freq;
          float since_last = (now - last_trig_time_).ToSecondf();
          if (since_last >= interval_s) {
            target_trig_angle_ = trig_angle_ + launcher::param::TRIG_STEP;
            last_trig_time_ = now;
            shoot_active_ = true;
            shot_start_time_ = now;
          }
        }
      } break;

      case TrigMode::JAM: {
        shoot_active_ = false;
        shot_start_time_ = 0;
        jam_keep_time_s_ = (now - last_jam_time_).ToSecondf();
        if (jam_keep_time_s_ >= launcher::param::JAM_TOGGLE_INTERVAL_SEC) {
          if (last_trig_mode_ != TrigMode::JAM) {
            is_reverse_ = true;
          }
          target_trig_angle_ =
              trig_angle_ + (is_reverse_ ? -0.80f * launcher::param::TRIG_STEP
                                         : launcher::param::TRIG_STEP);
          is_reverse_ = !is_reverse_;
        }
        last_jam_time_ = now;
      } break;
    }

    last_trig_mode_ = trig_mode_;
  }

  /**
   * @brief 由摩擦轮转速下降判定出弹，并更新热量计数与累计发射数。
   *        Detect a round leaving from the friction wheel speed drop, and
   *        update the heat count and the cumulative rounds.
   *
   * @param now 当前时间戳。
   *            Current timestamp.
   */
  void UpdateShotJudge(LibXR::MillisecondTimestamp now) {
    if (!shoot_active_) {
      return;
    }

    bool success =
        (fabsf(param_fric_0_.velocity) <
         (param_.fric1_setpoint_speed - launcher::param::FRIC_DROP_RPM)) &&
        (fabsf(param_fric_1_.velocity) <
         (param_.fric1_setpoint_speed - launcher::param::FRIC_DROP_RPM));

    if (success) {
      shoot_time_ = now;
      heat_limit_.launched_num += 1.0f;
      shoot_active_ = false;
      shot_start_time_ = 0;
      number_ += 1.0f;
      return;
    }

    if (shot_start_time_ != 0 && (now - shot_start_time_).ToSecondf() > 0.2f) {
      shoot_active_ = false;
      shot_start_time_ = 0;
    }
  }

  /**
   * @brief 记录开火命令边沿到出弹的时间差，作为 shoot_dt。
   *        Record the time from the fire command edge to the round leaving as
   *        shoot_dt.
   */
  void UpdateShotLatency() {
    auto now = LibXR::Timebase::GetMilliseconds();
    if (!last_fire_notify_ && launcher_cmd_.isfire) {
      receive_fire_time_ = now;
    }

    if (receive_fire_time_ <= shoot_time_) {
      shoot_dt_ = (shoot_time_ - receive_fire_time_).ToSecondf();
    }
  }

  /**
   * @brief 按摩擦轮模式设置目标转速：RELAX 与 SAFE 为 0，READY 为配置转速。
   *        Set the target speed from the friction wheel mode: 0 in RELAX and
   *        SAFE, the configured speed in READY.
   */
  void SetFricTargetByEvent() {
    switch (launcher_event_) {
      case LauncherEvent::SET_FRICMODE_RELAX:
      case LauncherEvent::SET_FRICMODE_SAFE:
        target_rpm_ = 0.0f;
        break;
      case LauncherEvent::SET_FRICMODE_READY:
        target_rpm_ = param_.fric1_setpoint_speed;
        break;
      default:
        break;
    }
  }

  /**
   * @brief 按裁判系统的热量上限与冷却值周期更新热量，计算是否允许发射，并按
   *        剩余热量调整弹频。
   *        Update the heat periodically from the referee heat limit and cooling
   *        value, decide whether firing is allowed and adjust the fire rate
   *        from the remaining heat.
   */
  void UpdateHeatControl() {
    auto now = LibXR::Timebase::GetMilliseconds();
    float delta_time = (now - last_heat_time_).ToSecondf();
    if (delta_time < launcher::param::HEAT_TICK_SEC) {
      return;
    }
    last_heat_time_ = now;

    heat_limit_.current_heat +=
        heat_limit_.single_heat * heat_limit_.launched_num;
    heat_limit_.launched_num = 0.0f;

    const float HEAT_COOLING = ref_data_.rs.shooter_cooling_value;
    const float COOLING_PER_TICK =
        HEAT_COOLING * launcher::param::HEAT_TICK_SEC;
    if (heat_limit_.current_heat < COOLING_PER_TICK) {
      heat_limit_.current_heat = 0.0f;
    } else {
      heat_limit_.current_heat -= COOLING_PER_TICK;
    }

    float residuary_heat =
        static_cast<float>(ref_data_.rs.shooter_heat_limit) -
        heat_limit_.current_heat;
    heat_limit_.allow_fire = residuary_heat > heat_limit_.single_heat;

    /* 不允许发射时弹频保持上一次的值 */
    if (!heat_limit_.allow_fire) {
      return;
    }

    if (residuary_heat <=
        heat_limit_.single_heat * heat_limit_.heat_threshold) {
      float safe_freq = HEAT_COOLING / heat_limit_.single_heat;
      float ratio = residuary_heat /
                    (heat_limit_.single_heat * heat_limit_.heat_threshold);
      trig_freq_ = ratio * (param_.expect_trig_freq_ - safe_freq) + safe_freq;
      return;
    }

    trig_freq_ = param_.expect_trig_freq_;
  }

  /**
   * @brief 发布 shoot_dt、shoot_number 与 trig_freq 三个 Topic。
   *        Publish the three Topics shoot_dt, shoot_number and trig_freq.
   */
  void PublishTopics() {
    shoot_waiting_.Publish(shoot_dt_);
    shoot_number_.Publish(number_);
    shoot_freq_.Publish(trig_freq_);
  }

  /**
   * @brief 角度环生成参考速度并限幅，速度环生成拨弹控制量。
   *        The angle loop generates a limited speed reference and the speed
   *        loop generates the trigger control value.
   *
   * @param out_trig 输出：拨弹控制量。
   *                 Output: trigger control value.
   * @param target_trig_angle 拨弹盘目标角度。
   *                          Trigger disc target angle.
   * @param dt 控制周期，单位 s。
   *           Control period in s.
   */
  void TrigControl(float& out_trig, float target_trig_angle, float dt) {
    float plate_omega_ref = pid_trig_angle_.Calculate(
        target_trig_angle, trig_angle_,
        param_trig_.omega / param_.trig_gear_ratio, dt);
    float omega_limit =
        static_cast<float>(1.5f * LibXR::TWO_PI * trig_freq_ /
                           param_.num_trig_tooth);
    float motor_omega_ref =
        std::clamp(plate_omega_ref, -omega_limit, omega_limit);
    out_trig = pid_trig_sp_.Calculate(
        motor_omega_ref, param_trig_.omega / param_.trig_gear_ratio, dt);
  }

  /**
   * @brief 速度环计算摩擦轮控制量，SAFE 模式下控制量缩小为 1/50。
   *        The speed loops compute the friction wheel control values, which are
   *        scaled down to 1/50 in SAFE mode.
   *
   * @param out_fric_0 输出：摩擦轮 0 控制量。
   *                   Output: friction wheel 0 control value.
   * @param out_fric_1 输出：摩擦轮 1 控制量。
   *                   Output: friction wheel 1 control value.
   * @param target_rpm 摩擦轮目标转速，单位 rpm。
   *                   Friction wheel target speed in rpm.
   * @param dt 控制周期，单位 s。
   *           Control period in s.
   */
  void FricControl(float& out_fric_0, float& out_fric_1, float target_rpm,
                   float dt) {
    out_fric_0 = pid_fric_0_.Calculate(target_rpm, param_fric_0_.velocity, dt);
    out_fric_1 = pid_fric_1_.Calculate(target_rpm, param_fric_1_.velocity, dt);

    if (launcher_event_ == LauncherEvent::SET_FRICMODE_SAFE) {
      out_fric_0 /= 50.0f;
      out_fric_1 /= 50.0f;
    }
  }
};

#ifdef DEBUG
#define INFANTRY_LAUNCHER_DEBUG_IMPL
#include "InfantryLauncherDebug.inl"
#undef INFANTRY_LAUNCHER_DEBUG_IMPL
#endif
