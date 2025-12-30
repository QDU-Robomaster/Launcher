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
      trig_gear_ratio: 19.0
      num_trig_tooth: 10
      expect_trig_freq_: 15
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
#include "mutex.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "timer.hpp"

class Launcher : public LibXR::Application {
 public:
  enum class TRIGMODE : uint8_t {
    SAFE = 0,
    SINGLE,
    CONTINUE,
    STOP,
    AIM,
  };

  enum class FRICMODE : uint8_t {
    SAFE = 0,
    READY,
  };

  enum class STATE : uint8_t { RESET = 0, FIRE, STOP, JAM };

  typedef struct {
    float heat_limit;
    float heat_cooling;
  } RefereeData;
  struct LauncherParam {
    /*默认弹速*/
    float default_bullet_speed;
    /*摩擦轮半径*/
    float fric_radius;
    /*拨弹盘电机减速比*/
    float trig_gear_ratio;
    /*拨齿数目*/
    uint8_t num_trig_tooth;
    /*期望弹频*/
    float expect_trig_freq_;
  };
  typedef struct {
    float single_heat;  // 单发热量
    float launched_num;
    float current_heat;
    float heat_threshold;  // 热量限制阈值
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

    // if (PARAM.expect_trig_freq_ > 0.0f) {
    //   min_launch_delay_ = 1.0f / PARAM.expect_trig_freq_;
    // } else {
    //   min_launch_delay_ = 0.0f;
    // }
    // auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
    //     [](bool in_isr, Launcher *launcher, uint32_t event_id) {
    //       UNUSED(in_isr);
    //       UNUSED(event_id);
    //       launcher->SetMode(static_cast<uint32_t>(FRICMODE::SAFE));
    //     },
    //     this);

    // cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);

    // auto callback = LibXR::Callback<uint32_t>::Create(
    //     [](bool in_isr, Launcher *launcher, uint32_t event_id) {
    //       UNUSED(in_isr);
    //       launcher->SetMode(event_id);
    //     },
    //     this);
    // launcher_event_.Register(static_cast<uint32_t>(FRICMODE::SAFE),
    // callback);
    // launcher_event_.Register(static_cast<uint32_t>(FRICMODE::READY),
    // callback); cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL,
    // lost_ctrl_callback);
  }

  static void ThreadFunction(Launcher *launcher) {
    // LibXR::Topic::ASyncSubscriber<CMD::LauncherCMD> launcher_cmd_tp(
    //     "launcher_cmd");
    // launcher_cmd_tp.StartWaiting();
    while (1) {
      launcher->mutex_.Lock();
      // if (!launcher->heat_limit_active_ && launcher_cmd_tp.Available()) {
      //   launcher->launcher_cmd_ = launcher_cmd_tp.GetData();
      //   launcher_cmd_tp.StartWaiting();
      // }

      launcher->Update();
      // launcher->TrigModeSelection();
      // launcher->Heat();
      launcher->IS_JAM();
      // launcher->CaclTarget();
      // launcher->CaclTarget();
      launcher->mutex_.Unlock();

      LibXR::Thread ::Sleep(2);
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

    // 如果存在挂起的发射请求队列，收集摩擦轮电流峰值以补偿时序延迟
    // if (!pending_shots_.empty()) {
    //   float cur0 = motor_fric_0_->GetCurrent();
    //   float cur1 = motor_fric_1_->GetCurrent();
    //   for (auto &ev : pending_shots_) {
    //     ev.sample_max_fric_0 = std::max(ev.sample_max_fric_0, cur0);
    //     ev.sample_max_fric_1 = std::max(ev.sample_max_fric_1, cur1);
    //   }
    // }

    float current_motor_angle = motor_trig_->GetAngle();

    if (!initialized) {
      last_motor_angle = current_motor_angle;
      initialized = true;
      return;
    }

    float last_angle = last_motor_angle;
    float current_angle = current_motor_angle;

    float delta_trig_angle = LibXR::CycleValue<float>(current_angle) -
                             LibXR::CycleValue<float>(last_angle);

    if (!motor_trig_->GetReverse()) {
      trig_angle_ += delta_trig_angle / PARAM.trig_gear_ratio;
    } else {
      trig_angle_ -= delta_trig_angle / PARAM.trig_gear_ratio;
    }
    last_motor_angle = current_motor_angle;
  }

  // void TrigModeSelection() {
  //   if (heat_limit_active_) {
  //     return;
  //   }
  //   auto now = LibXR::Timebase::GetMilliseconds();

  //   if (launcher_cmd_.isfire) {
  //     if (!last_fire_notify_) {
  //       fire_press_time_ = now;
  //       press_continue_ = false;
  //       trig_mod_ = TRIGMODE::SINGLE;
  //     } else {
  //       if (!press_continue_ && (now - fire_press_time_ > 500)) {
  //         press_continue_ = true;
  //       }
  //       if (press_continue_) {
  //         trig_mod_ = TRIGMODE::CONTINUE;
  //       }
  //     }
  //   } else {
  //     trig_mod_ = TRIGMODE::SAFE;
  //     press_continue_ = false;
  //   }

  //   last_fire_notify_ = launcher_cmd_.isfire;

  // }
  void IS_JAM() {
    // this->Update();
    auto now = LibXR::Timebase::GetMilliseconds();
    dt_ = (now - last_online_time_).ToSecondf();
    last_online_time_ = now;

    static LibXR::MillisecondTimestamp last_fire_notify = 0;
    // if (now - last_fire_notify >= 50) {
    static bool is_blocked = false;
    static bool set_point_ed = false;
    if (abs(motor_trig_->GetCurrent()) >= 6) {
      is_blocked = true;
    }

    if (is_blocked == false) {
      /* 没阻塞 */
      target_trig_angle_ += 2000;
    } else {
      if (set_point_ed == false) {
        target_trig_angle_ =
            trig_angle_ - 2 * static_cast<float>(M_2PI / PARAM.num_trig_tooth);
      }
      set_point_ed = true;
      if (abs(trig_angle_ - target_trig_angle_) <= 0.1) {
        is_blocked = false;
        set_point_ed = false;
      }
    }
    last_fire_notify = now;
    TrigAngleControl(target_trig_angle_);
    FricRPMControl(0);
    // }
  }
  void CaclTarget() {
    IS_JAM();
    auto now = LibXR::Timebase::GetMilliseconds();
    dt_ = (now - last_online_time_).ToSecondf();
    last_online_time_ = now;

    switch (fric_mod_) {
      case FRICMODE::SAFE:
        fric_rpm_ = 0.0f;
        FricRPMControl(fric_rpm_);
        break;
      case FRICMODE::READY:
        fric_rpm_ = 4000.0f;
        FricRPMControl(fric_rpm_);
        break;
      default:
        break;
    }

    // static bool is_reverse = true;
    // if(motor_trig_->GetCurrent()>12.0){
    //   if(is_reverse){
    //     target_trig_angle_-=2*M_2PI/PARAM.num_trig_tooth;
    //   }
    //   else{
    //     target_trig_angle_+=M_2PI/PARAM.num_trig_tooth;
    // }

    // is_reverse = !is_reverse;
    // }

    if (last_trig_mod == TRIGMODE::SAFE && trig_mod_ == TRIGMODE::SINGLE) {
      target_trig_angle_ =
          static_cast<float>(M_2PI / PARAM.num_trig_tooth + trig_angle_);
    }

    static float last_trig_angle = 0.0;
    switch (trig_mod_) {
      case TRIGMODE::STOP:
        motor_trig_->CurrentControl(0);
        break;
      case TRIGMODE::SAFE: {
        TrigAngleControl(trig_angle_);

      } break;

      case TRIGMODE::SINGLE: {
        if (last_trig_mod == TRIGMODE::SAFE) {
          target_trig_angle_ = target_trig_angle_ +
                               static_cast<float>(M_2PI / PARAM.num_trig_tooth);
        }
        TrigAngleControl(target_trig_angle_);
        //   float angle_err = static_cast<float>(target_trig_angle_ -
        //   trig_angle_);
        // if (std::fabs(angle_err) == 0.0f) {
        //   trig_mod_ = TRIGMODE::SAFE;
        // }
        // if (target_trig_angle_ > last_trig_angle) {
        //   MarkPendingShot();
        // }

        // last_trig_angle = target_trig_angle_;

      } break;

      case TRIGMODE::CONTINUE: {
        float since_last = (now - last_trig_time_).ToSecondf();
        if (trig_freq_ > 0.0f) {
          float trig_speed = 1.0f / trig_freq_;
          if (since_last >= trig_speed) {
            target_trig_angle_ =
                target_trig_angle_ +
                static_cast<float>(M_2PI / PARAM.num_trig_tooth);
            last_trig_time_ = now;
          }
        }

        TrigAngleControl(target_trig_angle_);
        // 获取上一次是否发射成功弹丸：仅标记挂起，采样在 Update() 中进行
        //  if (target_trig_angle_ > last_trig_angle) {
        //    MarkPendingShot();
        //  }
        //  last_trig_angle = target_trig_angle_;
      } break;
        //   case TRIGMODE::JAM:{
        //     target_trig_angle_=trig_angle_-2*static_cast<float>(M_2PI /
        //     PARAM.num_trig_tooth); TrigAngleControl(target_trig_angle_);
        // } break;
      default:
        break;
    }

    // 检查挂起的发射并根据采样峰值确认是否成功发射
    // CheckPendingShot();

    // last_trig_mod = trig_mod_;
  }

  //   void Heat(){
  //     auto now = LibXR::Timebase::GetMilliseconds();
  //     static LibXR::MillisecondTimestamp last_heat_time = 0.0f;
  //     float heat_speed = (now - last_heat_time).ToSecondf();

  //     if(heat_speed>=0.1)
  //    {
  //     last_heat_time = now;
  //      heat_limit_.current_heat +=
  //      heat_limit_.single_heat*heat_limit_.launched_num;
  //     heat_limit_.launched_num=0;

  //     if (heat_limit_.current_heat <
  //         (static_cast<float>(referee_data_.heat_cooling/10.0)))
  //         {
  //           heat_limit_.current_heat=0;
  //         }
  //      else{
  //        heat_limit_.current_heat -=
  //           static_cast<float>(referee_data_.heat_cooling/10.0 );
  //         }

  //       float residuary_heat
  //       =referee_data_.heat_limit-heat_limit_.current_heat;//剩余热量

  //     /*不同剩余热量启用不同实际弹频*/
  //     if (residuary_heat<heat_limit_.single_heat ) {
  //       heat_limit_active_ = true;
  //       trig_mod_ = TRIGMODE::STOP;
  //       launcher_cmd_.isfire = false;

  //   }else if(residuary_heat==heat_limit_.single_heat){
  //     trig_freq_ = referee_data_.heat_cooling/heat_limit_.single_heat;
  //   } else if (residuary_heat <=
  //              heat_limit_.single_heat * heat_limit_.heat_threshold) {
  //     float ratio = (residuary_heat - heat_limit_.single_heat) /
  //                   (heat_limit_.single_heat * heat_limit_.heat_threshold -
  //                    heat_limit_.single_heat);

  //     ratio = std::max(0.0f, std::min(1.0f, ratio));
  //     // 计算实际发射频率
  //     float safe_freq = referee_data_.heat_cooling / heat_limit_.single_heat;
  //     trig_freq_ = ratio * PARAM.expect_trig_freq_ + (1.0f - ratio) *
  //     safe_freq;
  //   } else {
  //     trig_freq_ = PARAM.expect_trig_freq_;
  //   }

  // /*是否继续接受遥控器模式 */
  //    if (residuary_heat>=0 &&
  //              heat_limit_active_) {
  //            heat_limit_active_ = false;
  //   }
  // }

  //   }
  /**
   * @brief 监控函数 (在此应用中未使用)
   *
   */
  void OnMonitor() override {}

  LibXR::Event &GetEvent() { return launcher_event_; }

 private:
  LauncherParam PARAM;
  RefereeData referee_data_;
  HeatLimit heat_limit_;
  TRIGMODE last_trig_mod = TRIGMODE::SAFE;
  TRIGMODE trig_mod_ = TRIGMODE::CONTINUE;
  FRICMODE fric_mod_ = FRICMODE::READY;

  float target_trig_angle_ = 0.0f;
  float trig_angle_ = 0.0f;
  float fric_rpm_ = 0.0f;
  float min_launch_delay_ = 0.0f;
  float trig_freq_ = 50.0f;
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
  LibXR::MillisecondTimestamp last_online_time_ = 0;

  bool last_fire_notify_ = false;
  bool press_continue_ = false;
  bool heat_limit_active_ = false;  // 是否启用STOP（trig）
  LibXR::MillisecondTimestamp fire_press_time_ = 0;

  LibXR::Thread thread_;
  LibXR::Semaphore semaphore_;
  LibXR::Mutex mutex_;
  LibXR::Event launcher_event_;
  float out = 0;

  // 发射检测：使用队列保存每次拨弹的 pending 事件，避免连续发射时挂起冲突
  struct PendingShot {
    LibXR::MillisecondTimestamp ts;
    float sample_max_fric_0;
    float sample_max_fric_1;
  };
  std::deque<PendingShot> pending_shots_;
  float shot_current_threshold_ = 2.5f;   // A，电流阈值（可调整）
  float shot_detection_window_s_ = 0.5f;  // s，检测时窗（可调整）
  float min_sample_time_s_ = 0.02f;       // s，至少采样时间以避免读到旧值
  float out_left = 0.0f;
  float out_right = 0.0f;

  /*---------------------工具函数--------------------------------------------------*/
  void FricRPMControl(float output) {
    out_left = pid_fric_0_.Calculate(output, motor_fric_0_->GetRPM(), dt_);
    out_right = pid_fric_1_.Calculate(output, motor_fric_1_->GetRPM(), dt_);
    motor_fric_0_->CurrentControl(out_left);
    motor_fric_1_->CurrentControl(out_right);
  }

  void TrigAngleControl(float target_angle) {
    float plate_omega_ref = pid_trig_angle_.Calculate(
        target_angle, trig_angle_,
        motor_trig_->GetOmega() / PARAM.trig_gear_ratio, dt_);

    float motor_omega_ref = plate_omega_ref;
    out = pid_trig_sp_.Calculate(
        motor_omega_ref, motor_trig_->GetOmega() / PARAM.trig_gear_ratio, dt_);

    motor_trig_->CurrentControl(out);
  }
  // void MarkPendingShot() {
  //   PendingShot ev;
  //   ev.ts = LibXR::Timebase::GetMilliseconds();
  //   ev.sample_max_fric_0 = 0.0f;
  //   ev.sample_max_fric_1 = 0.0f;
  //   pending_shots_.push_back(ev);
  // }

  // void CheckPendingShot() {
  //   if (pending_shots_.empty()) return;
  //   auto now = LibXR::Timebase::GetMilliseconds();

  //   // 逐个检查队首事件并在满足条件后出队
  //   while (!pending_shots_.empty()) {
  //     auto &ev = pending_shots_.front();
  //     float elapsed = (now - ev.ts).ToSecondf();

  //     if (elapsed >= min_sample_time_s_) {
  //       float trig_omega = motor_trig_->GetOmega() / PARAM.trig_gear_ratio;
  //       // 成功发射判断：拨盘正在正转且两个摩擦轮在采样窗口内出现阻塞电流峰值
  //       if (trig_omega > 0.1f &&
  //           ev.sample_max_fric_0 >= shot_current_threshold_ &&
  //           ev.sample_max_fric_1 >= shot_current_threshold_) {
  //         heat_limit_.launched_num++;
  //         pending_shots_.pop_front();
  //         continue;
  //       }
  //     }

  //     //
  //     超时处理（视为空发的条件）：当拨盘仍在正转且在检测窗口内未检测到电流峰值，判定为空发并出队。
  //     if (elapsed >= shot_detection_window_s_) {
  //       float trig_omega = motor_trig_->GetOmega() / PARAM.trig_gear_ratio;
  //       if (trig_omega > 0.1f &&
  //           (ev.sample_max_fric_0 < shot_current_threshold_ ||
  //           ev.sample_max_fric_1 < shot_current_threshold_)) {
  //         // 拨盘正转但没有电流峰值 -> 空发，出队
  //         pending_shots_.pop_front();
  //         continue;
  //       } else {
  //         // 拨盘未转或其它异常情况，同样出队以避免永久挂起
  //         pending_shots_.pop_front();
  //         continue;
  //       }
  //     }

  //     // 队首还需要更多采样，停止处理
  //     break;
  //   }
  // }
  // void SetMode(uint32_t mode) { fric_mod_ = static_cast<FRICMODE>(mode);
  //  }
};
