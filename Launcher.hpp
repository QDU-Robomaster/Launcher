#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - cmd: '@cmd'
  - task_stack_depth: 2048
  - pid_param_trig:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_param_fric:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - motor_can1: '@motor_can1'
  - motor_can2: '@motor_can2'
  - num_trig_tooth: 0.0
  - trig_gear_ratio: 0.0
  - fric_radius: 0.0
  - default_bullet_speed: 0.0
  - min_launch_delay: 0.0
template_args:
  - MotorType: RMMotorContainer
required_hardware:
  - cmd
  - motor_can1
  - motor_can2
depends:
  - qdu-future/CMD
  - qdu-future/Motor
=== END MANIFEST === */
// clang-format on

#include <cmath>
#include <cstdint>
#include "DR16.hpp"
#include "CMD.hpp"
#include "Eigen/Core"
#include "Motor.hpp"
#include "app_framework.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "message.hpp"
#include "pid.hpp"
#define LAUNCHER_TRIG_SPEED_MAX (16000.0f)

enum LauncherEvent : uint8_t {
  CHANGE_FRIC_MODE_RELAX,
  CHANGE_FRIC_MODE_SAFE,
  CHANGE_FRIC_MODE_LOADED,

  LAUNCHER_START_FIRE, /* 开火，拨弹盘开始发弹 */

  CHANGE_TRIG_MODE_SINGLE,
  CHANGE_TRIG_MODE_BURST,
  CHANGE_TRIG_MODE_CONTINUOUS,
  CHANGE_TRIG_MODE_STOP,

  CHANGE_TRIG_MODE,

  OPEN_COVER,
  CLOSE_COVER,
};

enum FricMode : uint8_t {
  RELAX, /* 放松模式，电机不输出 */
  SAFE,  /* 保险模式，电机闭环控制保持静止 */
  LOADED,
};

enum TrigMode : uint8_t {
  SINGLE,    //单发开火模式
  BURST,    //n发模式
  CONTINUOUS,    //持续发射模式
  STOP,    //停止模式
};
struct Mode {
  TrigMode trig_mode_ = SINGLE;
  FricMode fric_mode_ =RELAX;
  // JamMode jam_mode_;
};

template <typename MotorType>
class Launcher : public LibXR::Application {
 public:
  /**
   * @brief Launcher 构造函数
   *
   * @param hw 硬件容器
   * @param app 应用管理器
   * @param cmd 命令模块实例
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

  Launcher(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,DR16 &dr16,
           CMD &cmd, uint32_t task_stack_depth,
           LibXR::PID<float>::Param pid_param_trig,
           LibXR::PID<float>::Param pid_param_fric,
           Motor<MotorType> &motor_can1, Motor<MotorType> &motor_can2,
           float num_trig_tooth, float trig_gear_ratio, float fric_radius,
           float default_bullet_speed, uint32_t min_launch_delay)
      : min_launch_delay_(min_launch_delay),
        default_bullet_speed_(default_bullet_speed),
        fric_radius_(fric_radius),
        trig_gear_ratio_(trig_gear_ratio),
        num_trig_tooth_(num_trig_tooth),
        motor_can2_(motor_can2),
        motor_can1_(motor_can1),
        pid_fric_(pid_param_fric),
        pid_trig_(pid_param_trig),
        cmd_(cmd),dr16_(dr16) {
    UNUSED(hw);
    UNUSED(app);
    thread_.Create(this, ThreadFunction, "LauncherThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);

    // Hardware initialization example:
    // auto dev = hw.template Find<LibXR::GPIO>("led");
  }

  static void ThreadFunction(Launcher *launcher) {
    auto now = LibXR::Timebase::GetMilliseconds();
    launcher->dt_ = (now - launcher->last_online_time_).ToSecondf();
    launcher->last_online_time_ = now;
    auto &dr16_event_source = launcher->dr16_.GetEvent();
    auto &launcher_event_herandler = launcher->launcher_.GetEvent();
    launcher_event_herandler.Bind(
        dr16_event_source,
        static_cast<uint32_t>(DR16::SwitchPos::DR16_SW_R_POS_TOP),
        static_cast<uint32_t>(LauncherEvent::CHANGE_FRIC_MODE_RELAX));
    launcher_event_herandler.Bind(
        dr16_event_source,
        static_cast<uint32_t>(DR16::SwitchPos::DR16_SW_R_POS_MID),
        static_cast<uint32_t>(LauncherEvent::CHANGE_FRIC_MODE_SAFE));
    launcher_event_herandler.Bind(
        dr16_event_source,
        static_cast<uint32_t>(DR16::SwitchPos::DR16_SW_R_POS_BOT),
        static_cast<uint32_t>(LauncherEvent::CHANGE_FRIC_MODE_LOADED));

    auto event_callback = [](LauncherEvent event, Launcher *launcher) {
      launcher->semaphore_.Wait(UINT32_MAX);
      switch (event) {
        case CHANGE_FRIC_MODE_RELAX:
          launcher->mode_.fric_mode_ = RELAX;
          break;
        case CHANGE_FRIC_MODE_SAFE:
          launcher->mode_.fric_mode_ = SAFE;
          break;
        case CHANGE_FRIC_MODE_LOADED:
          launcher->mode_.fric_mode_ = LOADED;
          break;
        case LAUNCHER_START_FIRE:
          if (launcher->mode_.fric_mode_ == LOADED) {
            launcher->fire_ = true;
          }
          break;

        case CHANGE_TRIG_MODE_SINGLE:
          launcher->mode_.trig_mode_ = SINGLE;
          break;
        case CHANGE_TRIG_MODE_BURST:
          launcher->mode_.trig_mode_ = BURST;
          break;
        case CHANGE_TRIG_MODE_CONTINUOUS:
          launcher->mode_.trig_mode_ = CONTINUOUS;
        case CHANGE_TRIG_MODE_STOP:
          launcher->mode_.trig_mode_ = STOP;

        case CHANGE_TRIG_MODE:
        int current_mode=static_cast<int>(launcher->mode_.trig_mode_);
        current_mode=(current_mode+1)%4;
        launcher->mode_.trig_mode_=static_cast<TrigMode>(current_mode);
          break;
          // case:  弹丸舱的开闭：OPEN_COVER, CLOSE_COVER:

      }
      launcher->semaphore_.Post();
    };

        while (1) {
      launcher->semaphore_.Wait(UINT32_MAX);
      launcher->Update();
      launcher->semaphore_.Post();
      launcher->Control();
      launcher->SelfResolution();
      // event_active 接受dr16 mode转变
    }
  }
  /**
   * @brief 获取底盘的事件处理器
   * @return LibXR::Event& 事件处理器的引用
   */
  LibXR::Event &GetEvent() { return launcherevent_; }

  void Update() {
    auto now = LibXR::Timebase::GetMilliseconds();
    this->dt_ = (now - this->last_time_).ToSecondf();
    this->last_time_ = now;


    const float LAST_TRIG_MOTOR_ANGLE = now_angle_trig_;
    motor_can1_.Update(0);
    motor_can2_.Update(0);
    motor_can2_.Update(0);
    const float DELTA_MOTOR_ANGLE = now_angle_trig_ - LAST_TRIG_MOTOR_ANGLE;
    trig_angle_ += DELTA_MOTOR_ANGLE / trig_gear_ratio_;//实际已经转的角度
  }

  void SelfResolution() {
    prev_omega_fric_ = now_omega_fric_;
    prev_omega_trig1_ = now_omega_trig1_;
    prev_omega_trig2_ = now_omega_trig2_;

    now_angle_trig_ = motor_can1_.GetAngle(0);
    now_angle_fric1_ = motor_can2_.GetAngle(0);
    now_angle_fric2_ = motor_can2_.GetAngle(0);
    now_omega_fric_ = motor_can1_.GetSpeed(5);
    now_omega_trig1_ = motor_can2_.GetSpeed(2);
    now_angle_fric2_ = motor_can2_.GetSpeed(3);
  }

  void Control() {
//根据模式改变每次的发弹量
    switch (mode_.trig_mode_) {
      case SINGLE:
        max_burst_=1;
        break;
      case BURST:
       max_burst_=5;
        break;
      case CONTINUOUS:
        max_burst_=1000000;//要改
        break;
      case STOP:
      max_burst_=0;
        break;
      default:
      max_burst_=1;
        break;
    }
    //执行发射量
    switch(mode_.trig_mode_){
      case SINGLE:
      case BURST:
      case STOP:
      case CONTINUOUS:

        break;
      default:

        break;

    }
   switch (mode_.fric_mode_) {
      case RELAX:
      case SAFE:
        fric_rpm_ = 0.0f;
        break;
      case LOADED:
        fric_rpm_ = BulletSpeedToFricRpm(default_bullet_speed_, fric_radius_, 1);
        break;
      default:
        fric_rpm_ = 0.0f;
        break;

   }

    const float OUTPUT_TRIG = std::clamp(
        pid_trig_.Calculate(target_angle_trig_,
                            now_angle_trig_ / LAUNCHER_TRIG_SPEED_MAX, dt_),
        0.0f, LAUNCHER_TRIG_SPEED_MAX);

    float output_fric1 =
        BulletSpeedToFricRpm(default_bullet_speed_, fric_radius_, 1);
    float output_fric2 = -output_fric1;

    motor_can1_.SetCurrent(5, OUTPUT_TRIG);
    motor_can2_.SetCurrent(2, output_fric1);
    motor_can2_.SetCurrent(3, output_fric2);


    if ((last_trig_angle_-trig_angle_)/M_2PI*num_trig_tooth_>=0.9) {
          last_trig_angle_ = trig_angle_;
          trig_angle_ -= M_2PI/num_trig_tooth_;
      }

  }

  float BulletSpeedToFricRpm(float bullet_speed, float fric_radius,
                             bool is17mm) {
    UNUSED(fric_radius);

    if (bullet_speed == 0.0f) {
      return 0.0f;
    } else if (bullet_speed > 0.0f) {
      if (is17mm) {
        if (bullet_speed == 15.0f) {
          return 4670.0f;
        }
        if (bullet_speed == 18.0f) {
          return 5200.0f;
        }
        if (bullet_speed == 25.0f) {
          return 7400.0f;
        }
      } else {
        if (bullet_speed == 10.0f) {
          return 4450.0f;
        }
        if (bullet_speed == 16.0f) {
          return 5700.0f;
        }
      }
    }
    return 0.0f;
  }

  /**
   * @brief 监控函数 (在此应用中未使用)
   *
   */
  void OnMonitor() override {}
  /**
   * @brief 设置发射模式
   * @param mode 要设置的新模式
   */
  void SetMode(Mode mode) {
    mutex_.Lock();
    launcher_event_ = mode;
    mutex_.Unlock();
  }

 private:
  bool stall_=false;
  bool fire_ = false;
  // 最小发射间隔
  uint32_t min_launch_delay_ = 0.0f;
  // 默认弹丸初速度
  float default_bullet_speed_ = 0.0f;
  // 摩擦轮半径
  float fric_radius_ = 0.0f;
  // 拨弹电机减速比 3508:19, 2006:36
  float trig_gear_ratio_ = 0.0f;
  // 拨弹盘中一圈能存储几颗弹丸
  float num_trig_tooth_ = 0.0f;
  //发弹量
  uint32_t max_burst_=0;

  float trig_angle_ = 0.0f;
  float last_trig_angle_ = 1.0f;
  float launcher_delay_ = UINT32_MAX;
  float last_launch_time_=0.0f;
  float now_angle_trig_ = 0.0f;
  float now_angle_fric1_ = 0.0f;
  float now_angle_fric2_ = 0.0f;

  float prev_omega_fric_ = 0.0f;
  float prev_omega_trig1_ = 0.0f;
  float prev_omega_trig2_ = 0.0f;

  float now_omega_fric_ = 0.0f;
  float now_omega_trig1_ = 0.0f;
  float now_omega_trig2_ = 0.0f;

  float target_angle_trig_ = M_2PI / num_trig_tooth_;

  float fric_rpm_ = 0.0f;

  Motor<MotorType> &motor_can2_;

  Motor<MotorType> &motor_can1_;

  LibXR::PID<float> pid_fric_;
  LibXR::PID<float> pid_trig_;

  float dt_ = 0;
  LibXR::MillisecondTimestamp last_online_time_ = 0;
  LibXR::MillisecondTimestamp last_time_ = 0;

  CMD &cmd_;
  Mode mode_;
  LibXR::Thread thread_;
  LibXR::Mutex mutex_;

  std::vector<CMD::EventMapItem> event_map_;
  LibXR::Semaphore semaphore_;
  DR16 &dr16_;
  Launcher<MotorType> &launcher_;
  LibXR::Event launcherevent_;
  Mode launcher_event_;
};
