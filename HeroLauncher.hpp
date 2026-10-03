#pragma once

#include <algorithm>
#include <cstdint>

#include "CMD.hpp"
#include "Motor.hpp"
#include "RMMotor.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "timebase.hpp"

#ifdef DEBUG
#include "DebugCore.hpp"
#include "ramfs.hpp"
#endif

/**
 * @brief 英雄发射机构实现：摩擦轮与拨弹盘控制，按热量限制发射。
 *        Hero launcher implementation: friction wheel and trigger disc control,
 *        with firing limited by the heat.
 *
 * 作为 Launcher<HeroLauncher> 的内部逻辑类，线程和事件注册由外壳提供。
 * As the internal logic class of Launcher<HeroLauncher>, the thread and the
 * event registration are provided by the shell.
 */
class HeroLauncher {
 public:
  /// 首发标定完成后，设定角相对零点的后退量 (rad)
  /// Setpoint offset behind the zero point after the first-shot calibration
  /// (rad)
  static constexpr float TRIG_ZERO_ANGLE_OFFSET = 0.50f;
  /// 首发标定时每周期后退的角度 (rad)
  /// Angle moved back per cycle during the first-shot calibration (rad)
  static constexpr float TRIG_LOADING_ANGLE_STEP =
      static_cast<float>(LibXR::TWO_PI) / 1002.0f;
  /// M3508 的转矩常数，用于由力矩换算电流
  /// Torque constant of the M3508, used to convert torque to current
  static constexpr float M3508_TORQUE_CONSTANT = 0.3f;

  /**
   * @brief 拨弹模式。
   *        Trigger modes.
   */
  enum class TrigMode : uint8_t {
    RELAX = 0,  ///< 放松 Relax
    SAFE,       ///< 安全：保持当前设定角 Safe: hold the current setpoint
    SINGLE,     ///< 单发 Single shot
    CONTINUE,   ///< 持续按下 Fire held
  };

  /**
   * @brief 摩擦轮模式事件，数值同时是事件 ID。
   *        Friction wheel mode events; the values are also the event IDs.
   */
  enum class LauncherEvent : uint8_t {
    SET_FRICMODE_RELAX,  ///< 放松 Relax
    SET_FRICMODE_SAFE,   ///< 安全：目标转速为 0 Safe: target speed 0
    SET_FRICMODE_READY,  ///< 就绪：按目标转速运行 Ready: run at the target
                         ///< speed
  };

  /**
   * @brief 裁判系统数据。
   *        Referee system data.
   */
  struct RefereeData {
    float heat_limit;    ///< 热量上限 Heat limit
    float cooling_rate;  ///< 冷却速率 Cooling rate
    uint8_t level;       ///< 机器人等级 Robot level
  };

  /**
   * @brief 热量控制状态。
   *        Heat control state.
   */
  struct HeatControl {
    float heat;           ///< 当前热量 Current heat
    float last_heat;      ///< 上一次的热量 Previous heat
    float heat_limit;     ///< 热量上限 Heat limit
    float speed_limit;    ///< 弹丸初速上限 Projectile speed limit
    float cooling_rate;   ///< 冷却速率 Cooling rate
    float heat_increase;  ///< 每发增加的热量 Heat added per round

    uint8_t cooling_acc;  ///< 冷却增益 Cooling gain

    uint32_t available_shot;  ///< 热量范围内还可发射的数量
                              ///< Rounds still available within the heat
  };

  /**
   * @brief 发射器参数。
   *        Launcher parameters.
   */
  struct LauncherParam {
    float fric1_setpoint_speed;  ///< 一级摩擦轮目标转速
                                 ///< First-stage wheel target speed
    float fric2_setpoint_speed;  ///< 二级摩擦轮目标转速
                                 ///< Second-stage wheel target speed
    float trig_gear_ratio;       ///< 拨弹电机减速比
                                 ///< Trigger motor reduction ratio
    uint8_t num_trig_tooth;      ///< 拨弹盘齿数
                                 ///< Number of trigger disc teeth
    float trig_freq;             ///< 弹频
                                 ///< Fire rate
  };

  /**
   * @brief 构造 HeroLauncher。
   *        Construct HeroLauncher.
   *
   * @param motor_fric_front_left 前左摩擦轮电机。
   *                              Front-left friction wheel motor.
   * @param motor_fric_front_right 前右摩擦轮电机。
   *                               Front-right friction wheel motor.
   * @param motor_fric_back_left 后左摩擦轮电机，非空。
   *                             Back-left friction wheel motor, non-null.
   * @param motor_fric_back_right 后右摩擦轮电机，非空。
   *                              Back-right friction wheel motor, non-null.
   * @param motor_trig 拨弹电机。
   *                   Trigger motor.
   * @param task_stack_depth 线程栈深，由外壳使用。
   *                         Thread stack depth, used by the shell.
   * @param trig_angle_pid 拨弹角度环参数。
   *                       Trigger angle-loop parameters.
   * @param trig_speed_pid 拨弹速度环参数。
   *                       Trigger speed-loop parameters.
   * @param fric_speed_pid_0 摩擦轮 0 速度环参数。
   *                         Friction wheel 0 speed-loop parameters.
   * @param fric_speed_pid_1 摩擦轮 1 速度环参数。
   *                         Friction wheel 1 speed-loop parameters.
   * @param fric_speed_pid_2 摩擦轮 2 速度环参数。
   *                         Friction wheel 2 speed-loop parameters.
   * @param fric_speed_pid_3 摩擦轮 3 速度环参数。
   *                         Friction wheel 3 speed-loop parameters.
   * @param launcher_param 发射器参数。
   *                       Launcher parameters.
   * @param cmd CMD 实例指针。
   *            Pointer to the CMD instance.
   */
  HeroLauncher(RMMotor* motor_fric_front_left, RMMotor* motor_fric_front_right,
               RMMotor* motor_fric_back_left, RMMotor* motor_fric_back_right,
               RMMotor* motor_trig, uint32_t task_stack_depth,
               LibXR::PID<float>::Param trig_angle_pid,
               LibXR::PID<float>::Param trig_speed_pid,
               LibXR::PID<float>::Param fric_speed_pid_0,
               LibXR::PID<float>::Param fric_speed_pid_1,
               LibXR::PID<float>::Param fric_speed_pid_2,
               LibXR::PID<float>::Param fric_speed_pid_3,
               LauncherParam launcher_param, CMD* cmd)
      : param_(launcher_param),
        motor_fric_front_left_(motor_fric_front_left),
        motor_fric_front_right_(motor_fric_front_right),
        motor_fric_back_left_(motor_fric_back_left),
        motor_fric_back_right_(motor_fric_back_right),
        motor_trig_(motor_trig),
        trig_angle_pid_(trig_angle_pid),
        trig_speed_pid_(trig_speed_pid),
        fric_speed_pid_{fric_speed_pid_0, fric_speed_pid_1, fric_speed_pid_2,
                        fric_speed_pid_3} {
    UNUSED(task_stack_depth);
    UNUSED(cmd);

    /* 英雄发射机构的控制循环无条件访问四个摩擦轮电机 */
    ASSERT(motor_fric_back_left != nullptr);
    ASSERT(motor_fric_back_right != nullptr);

    last_wakeup_ = LibXR::Timebase::GetMicroseconds();
  }

  /**
   * @brief 更新电机反馈、拨弹盘角度和状态量。
   *        Update the motor feedback, the trigger disc angle and the state.
   */
  void Update() {
    this->last_wakeup_ = LibXR::Timebase::GetMicroseconds();

    const float LAST_TRIG_MOTOR_ANGLE =
        LibXR::CycleValue<float>(param_trig_.abs_angle);

    motor_fric_front_left_->Update();
    motor_fric_front_right_->Update();
    motor_fric_back_left_->Update();
    motor_fric_back_right_->Update();
    motor_trig_->Update();

    param_motor_fric_front_left_ = motor_fric_front_left_->GetFeedback();
    param_motor_fric_front_right_ = motor_fric_front_right_->GetFeedback();
    param_motor_fric_back_left_ = motor_fric_back_left_->GetFeedback();
    param_motor_fric_back_right_ = motor_fric_back_right_->GetFeedback();
    param_trig_ = motor_trig_->GetFeedback();
    const float DELTA_MOTOR_ANGLE =
        LibXR::CycleValue<float>(param_trig_.abs_angle) - LAST_TRIG_MOTOR_ANGLE;
    this->trig_angle_ += DELTA_MOTOR_ANGLE / param_.trig_gear_ratio;
  }

  /**
   * @brief 热量计算、拨弹状态机与摩擦轮目标更新。
   *        Heat calculation, trigger state machine and friction wheel target
   *        update.
   */
  void Solve() {
    HeatLimit();
    UpdateTrigMode();
    UpdateFricTarget();
  }

  /**
   * @brief 拨弹控制、出弹检测以及摩擦轮与拨弹的 PID 输出。
   *        Trigger control, round detection and the PID outputs of the friction
   *        wheels and the trigger.
   */
  void Control() {
    /*电流cur=tor/K*/
    current_back_left_ =
        param_motor_fric_back_left_.torque / M3508_TORQUE_CONSTANT;

    if (first_loading_) {
      FirstLoadingControl();
    } else {
      NormalFireControl();
    }
    real_launch_delay_ = (finish_fire_time_ - start_fire_time_).ToMillisecond();

    FricPidControl();
    TrigPidControl();
  }

  /**
   * @brief 设置摩擦轮模式。
   *        Set the friction wheel mode.
   *
   * @param mode 事件 ID，对应 LauncherEvent。
   *             Event ID, corresponding to LauncherEvent.
   */
  void SetMode(uint32_t mode) {
    launcher_event_ = static_cast<LauncherEvent>(mode);
  }

  /**
   * @brief 失去控制时复位全部发射状态，摩擦轮模式切换到 SAFE。
   *        Reset all launch states when control is lost and switch the friction
   *        wheel mode to SAFE.
   */
  void LostCtrl() {
    // 重置所有发射相关的状态变量到初始模式
    launcher_event_ = LauncherEvent::SET_FRICMODE_SAFE;
    trig_mode_ = TrigMode::RELAX;

    // 重置发射控制标志
    fire_flag_ = false;
    enable_fire_ = false;
    mark_launch_ = false;
    first_loading_ = true;
    press_continue_ = false;

    // 重置计数器
    fired_ = 0;
    delay_time_ = 0;

    // 重置时间戳
    fire_press_time_ = 0;
    start_fire_time_ = 0;
    finish_fire_time_ = 0;
    start_loading_time_ = 0;
    last_change_angle_time_ = 0;

    // 重置角度相关变量
    trig_zero_angle_ = 0.0f;
    trig_angle_ = 0.0f;
    trig_setpoint_angle_ = 0.0f;

    trig_output_ = 0.0f;

    // 重置速度目标值
    fric_target_speed_[0] = 0.0f;
    fric_target_speed_[1] = 0.0f;
    fric_target_speed_[2] = 0.0f;
    fric_target_speed_[3] = 0.0f;

    // 重置发射命令
    launcher_cmd_.isfire = false;
    last_fire_notify_ = false;

    // 重置延迟计算
    real_launch_delay_ = 0.0f;
  }

  /**
   * @brief 设置控制周期。
   *        Set the control period.
   *
   * @param dt 控制周期，单位 s。
   *           Control period in s.
   */
  void SetControlDt(float dt) { dt_ = dt; }

#ifdef DEBUG
  /**
   * @brief 调试命令入口，实现位于 HeroLauncherDebug.inl。
   *        Debug command entry, implemented in HeroLauncherDebug.inl.
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
  CMD::LauncherCMD launcher_cmd_;  // NOLINT

 private:
  LauncherParam param_;
  RefereeData referee_data_;
  TrigMode trig_mode_ = TrigMode::SAFE;

  HeatControl heat_ctrl_;

  bool first_loading_ = true;

  float dt_ = 0.0f;

  LibXR::MillisecondTimestamp now_ = 0;

  LibXR::MicrosecondTimestamp last_wakeup_;

  LibXR::MillisecondTimestamp last_change_angle_time_ = 0;

  LibXR::MillisecondTimestamp start_loading_time_ = 0;

  RMMotor* motor_fric_front_left_;
  RMMotor* motor_fric_front_right_;
  RMMotor* motor_fric_back_left_;
  RMMotor* motor_fric_back_right_;
  RMMotor* motor_trig_;

  float trig_setpoint_angle_ = 0.0f;
  float trig_setpoint_speed_ = 0.0f;

  float trig_zero_angle_ = 0.0f;
  float trig_angle_ = 0.0f;
  float trig_output_ = 0.0f;

  float fric_target_speed_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  LibXR::PID<float> trig_angle_pid_;
  LibXR::PID<float> trig_speed_pid_;

  LibXR::PID<float> fric_speed_pid_[4] = {
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param())};

  float current_back_left_ = 0.0f;

  bool fire_flag_ = false;    // 发射命令标志位
  uint8_t fired_ = 0;         // 已发射弹丸
  bool enable_fire_ = false;  // 拨弹盘旋转命令发出标志位
  bool mark_launch_ = false;  // 拨弹发射完成标志位

  LibXR::MillisecondTimestamp start_fire_time_ = 0;
  LibXR::MillisecondTimestamp finish_fire_time_ = 0;
  uint32_t real_launch_delay_ = 0.0f;

  bool last_fire_notify_ = false;
  bool press_continue_ = false;
  LibXR::MillisecondTimestamp fire_press_time_ = 0;

  uint8_t delay_time_ = 0;

  LauncherEvent launcher_event_ = LauncherEvent::SET_FRICMODE_RELAX;

  Motor::Feedback param_motor_fric_front_left_;
  Motor::Feedback param_motor_fric_front_right_;
  Motor::Feedback param_motor_fric_back_left_;
  Motor::Feedback param_motor_fric_back_right_;
  Motor::Feedback param_trig_;

  Motor::MotorCmd cmd_fric_front_left_ =
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 1.0f,
                      .velocity = 0};
  Motor::MotorCmd cmd_fric_front_right_ =
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 1.0f,
                      .velocity = 0};
  Motor::MotorCmd cmd_fric_back_left_ =
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 1.0f,
                      .velocity = 0};
  Motor::MotorCmd cmd_fric_back_right_ =
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 1.0f,
                      .velocity = 0};
  Motor::MotorCmd cmd_trig_ =
      Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                      .reduction_ratio = 19.2032f,
                      .velocity = 0};

  /**
   * @brief 按发射命令与摩擦轮模式更新拨弹模式。
   *        Update the trigger mode from the fire command and the friction wheel
   *        mode.
   */
  void UpdateTrigMode() {
    LibXR::MillisecondTimestamp now_time = LibXR::Timebase::GetMilliseconds();

    if (launcher_event_ != LauncherEvent::SET_FRICMODE_RELAX) {
      if (launcher_cmd_.isfire && !last_fire_notify_) {
        fire_press_time_ = now_time;
        press_continue_ = false;
        trig_mode_ = TrigMode::SINGLE;
      } else if (launcher_cmd_.isfire && last_fire_notify_) {
        if (!press_continue_ && (now_time - fire_press_time_ > 200)) {
          press_continue_ = true;
        }
        if (press_continue_) {
          trig_mode_ = TrigMode::CONTINUE;
        }
      } else {
        trig_mode_ = TrigMode::SAFE;
        press_continue_ = false;
      }
    } else {
      trig_mode_ = TrigMode::RELAX;
    }

    last_fire_notify_ = launcher_cmd_.isfire;
  }

  /**
   * @brief 按摩擦轮模式设置摩擦轮目标转速与输出限幅。
   *        Set the friction wheel target speeds and output limits from the
   *        friction wheel mode.
   */
  void UpdateFricTarget() {
    switch (launcher_event_) {
      case LauncherEvent::SET_FRICMODE_RELAX:
      case LauncherEvent::SET_FRICMODE_SAFE:
        fric_target_speed_[0] = 0;
        fric_target_speed_[1] = 0;
        fric_target_speed_[2] = 0;
        fric_target_speed_[3] = 0;
        for (LibXR::PID<float>& i : fric_speed_pid_) {
          i.SetOutLimit(0.1f);
        }
        break;
      case LauncherEvent::SET_FRICMODE_READY:
        fric_target_speed_[0] = param_.fric2_setpoint_speed;
        fric_target_speed_[1] = param_.fric2_setpoint_speed;
        fric_target_speed_[2] = param_.fric1_setpoint_speed;
        fric_target_speed_[3] = param_.fric1_setpoint_speed;
        if (motor_fric_back_left_->GetFeedback().velocity >
            param_.fric1_setpoint_speed) {
          fric_speed_pid_[0].SetOutLimit(1.0f);
          fric_speed_pid_[1].SetOutLimit(1.0f);
          fric_speed_pid_[2].SetOutLimit(0.8f);
          fric_speed_pid_[3].SetOutLimit(0.8f);
        }
        break;
      default:
        break;
    }
  }

  /**
   * @brief 首发标定：拨弹盘后退直到检测到出弹，并记录零点。
   *        First-shot calibration: move the trigger disc back until a round is
   *        detected and record the zero point.
   */
  void FirstLoadingControl() {
    if (trig_mode_ == TrigMode::SINGLE) {
      fire_flag_ = true;
    }
    if (fire_flag_) {
      if (start_loading_time_ == 0) {
        start_loading_time_ = LibXR::Timebase::GetMilliseconds();
      }

      trig_setpoint_angle_ -= TRIG_LOADING_ANGLE_STEP;
      last_change_angle_time_ = LibXR::Timebase::GetMilliseconds();

      delay_time_++;
    }

    if (delay_time_ > 50) {  // 延迟50个控制周期
      if (std::abs(param_motor_fric_back_left_.torque) / M3508_TORQUE_CONSTANT >
          0.5) {                         // 发弹检测
        trig_zero_angle_ = trig_angle_;  // 获取电机当前位置
        trig_setpoint_angle_ = trig_angle_ - TRIG_ZERO_ANGLE_OFFSET;  // 偏移量

        fire_flag_ = false;
        first_loading_ = false;
        fired_++;

        mark_launch_ = true;
      }
    }
  }

  /**
   * @brief 常规发弹：热量允许时推进一格并检测出弹。
   *        Normal firing: advance one tooth when heat allows and detect the
   *        round.
   */
  void NormalFireControl() {
    if (trig_mode_ == TrigMode::SINGLE) {
      mark_launch_ = false;
      if (!enable_fire_) {
        if (heat_ctrl_.available_shot) {
          trig_setpoint_angle_ -= static_cast<float>(LibXR::TWO_PI) /
                                  static_cast<float>(param_.num_trig_tooth);

          enable_fire_ = true;
          mark_launch_ = false;
          start_fire_time_ = LibXR::Timebase::GetMilliseconds();

          trig_mode_ = TrigMode::SAFE;
        }
      }
    }
    now_ = LibXR::Timebase::GetMilliseconds();

    // 发射超时检测：超过 100 ms 未检测到出弹则重置状态
    if (start_fire_time_ > 0 && (now_ - start_fire_time_ > 100) &&
        !mark_launch_) {
      fire_flag_ = false;
      enable_fire_ = false;
      start_fire_time_ = now_;
    }

    if (!mark_launch_) {  // 发弹状态检测
      if (std::abs(param_motor_fric_back_left_.torque) / M3508_TORQUE_CONSTANT >
          0.5) {
        fire_flag_ = false;

        fired_++;

        mark_launch_ = true;
        enable_fire_ = false;
        finish_fire_time_ = LibXR::Timebase::GetMilliseconds();
      }
    }
  }

  /**
   * @brief 计算并下发摩擦轮速度环输出。
   *        Compute and send the friction wheel speed-loop outputs.
   */
  void FricPidControl() {
    cmd_fric_front_left_.velocity = fric_speed_pid_[0].Calculate(
        fric_target_speed_[0], param_motor_fric_front_left_.velocity, dt_);
    cmd_fric_front_right_.velocity = fric_speed_pid_[1].Calculate(
        fric_target_speed_[1], param_motor_fric_front_right_.velocity, dt_);
    cmd_fric_back_left_.velocity = fric_speed_pid_[2].Calculate(
        fric_target_speed_[2], param_motor_fric_back_left_.velocity, dt_);
    cmd_fric_back_right_.velocity = fric_speed_pid_[3].Calculate(
        fric_target_speed_[3], param_motor_fric_back_right_.velocity, dt_);

    motor_fric_front_left_->Control(cmd_fric_front_left_);
    motor_fric_front_right_->Control(cmd_fric_front_right_);
    motor_fric_back_left_->Control(cmd_fric_back_left_);
    motor_fric_back_right_->Control(cmd_fric_back_right_);
  }

  /**
   * @brief 计算并下发拨弹角度环与速度环输出。
   *        Compute and send the trigger angle-loop and speed-loop outputs.
   */
  void TrigPidControl() {
    trig_setpoint_speed_ =
        trig_angle_pid_.Calculate(trig_setpoint_angle_, trig_angle_, dt_);

    trig_output_ = trig_speed_pid_.Calculate(trig_setpoint_speed_,
                                             param_trig_.velocity, dt_);
    switch (trig_mode_) {
      case TrigMode::RELAX:
        cmd_trig_.velocity = 0;
        break;
      case TrigMode::SAFE:
      case TrigMode::SINGLE:
      case TrigMode::CONTINUE:
        cmd_trig_.velocity = trig_output_;
        break;
      default:
        break;
    }
    motor_trig_->Control(cmd_trig_);
  }

  /**
   * @brief 计算热量并更新可发射数量。
   *        Compute the heat and update the number of available shots.
   */
  void HeatLimit() {
    heat_ctrl_.heat_limit = referee_data_.heat_limit;
    heat_ctrl_.heat_limit = 129.0f;  // 调试用固定值
    heat_ctrl_.heat_increase = 100.0f;
    heat_ctrl_.cooling_rate = referee_data_.cooling_rate;
    heat_ctrl_.cooling_rate = 13.0f;  // 调试用固定值
    if (fired_ >= 1) {
      heat_ctrl_.heat += heat_ctrl_.heat_increase;
      fired_ = 0;
    }
    heat_ctrl_.heat -=
        heat_ctrl_.cooling_rate / (1 / dt_);  // 每个控制周期的冷却恢复
    heat_ctrl_.heat = std::max(heat_ctrl_.heat, 0.0f);
    float available_float =
        (this->heat_ctrl_.heat_limit - this->heat_ctrl_.heat) /
        this->heat_ctrl_.heat_increase;
    heat_ctrl_.available_shot = static_cast<uint32_t>(available_float);
  }
};

#ifdef DEBUG
#define HERO_LAUNCHER_DEBUG_IMPL
#include "HeroLauncherDebug.inl"
#undef HERO_LAUNCHER_DEBUG_IMPL
#endif
