#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - motor_fric_front_left: '@&motor_fric_front_left'
  - motor_fric_front_right: '@&motor_fric_front_right'
  - motor_fric_back_left: '@&motor_fric_back_left'
  - motor_fric_back_right: '@&motor_fric_back_right'
  - motor_trig: '@&motor_trig'
  - cmd: '@&cmd'
  - task_stack_depth: 2048
  - launcher_param:
      fric1_setpoint_speed: 4500.0
      fric2_setpoint_speed: 3100.0
      default_bullet_speed: 0.0
      fric_radius: 6.0
      trig_gear_ratio: 19.0
      num_trig_tooth: 6
      trig_freq_: 0.0
  - pid_trig_angle:
      k: 1.0
      p: 4000.0
      i: 3000.0
      d: 0.0
      i_limit: 0.0
      out_limit: 2000.0
      cycle: false
  - pid_trig_speed:
      k: 1.0
      p: 0.0012
      i: 0.001
      d: 0.0
      i_limit: 1.0
      out_limit: 1.0
      cycle: false
  - pid_fric_speed_0:
      k: 1.0
      p: 0.002
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
  - pid_fric_speed_1:
      k: 1.0
      p: 0.002
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
  - pid_fric_speed_2:
      k: 1.0
      p: 0.002
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
  - pid_fric_speed_3:
      k: 1.0
      p: 0.002
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
template_args:
  - LauncherType: HeroLauncher
  - FRIC_NUM: 4
  - TRIG_NUM: 1
required_hardware:
  - dr16
  - can
depends:
  - qdu-future/CMD
  - qdu-future/RMMotor
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cstdint>

#include "CMD.hpp"
#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "libxr_type.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "pid.hpp"
#include "pwm.hpp"
#include "semaphore.hpp"
#include "thread.hpp"
#include "timebase.hpp"
#include "uart.hpp"

enum class TRIGMODE : uint8_t {
  RELAX = 0,
  SAFE,
  SINGLE,
  CONTINUE,
  AIM,
};

enum class FRICMODE : uint8_t {
  SAFE = 0,
  READY,
};

typedef struct {
  float heat_limit;
  float heat_cooling;
  float cooling_rate;
  uint8_t level;
} RefereeData;

struct HeatControl {
  float heat;          /* 现在热量水平 */
  float last_heat;     /* 之前的热量水平 */
  float heat_limit;    /* 热量上限 */
  float speed_limit;   /* 弹丸初速是上限 */
  float cooling_rate;  /* 冷却速率 */
  float heat_increase; /* 每发热量增加值 */

  uint8_t cooling_acc;  // 冷却增益

  uint32_t available_shot; /* 热量范围内还可以发射的数量 */
};

struct LauncherParam {
  /*一级摩擦轮转速*/
  float fric1_setpoint_speed;
  /*二级摩擦轮转速*/
  float fric2_setpoint_speed;
  /*默认弹速*/
  float default_bullet_speed;
  /*摩擦轮半径*/
  float fric_radius;
  /*拨弹盘电机减速比*/
  float trig_gear_ratio;
  /*拨齿数目*/
  float num_trig_tooth;
  /*弹频*/
  float trig_freq_;
};

template <int FRIC_NUM, int TRIG_NUM>
class HeroLauncher {
 public:
  HeroLauncher(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
               std::array<RMMotor *, FRIC_NUM> motor_fric,
               std::array<RMMotor *, TRIG_NUM> motor_trig,
               uint32_t task_stack_depth,
               LibXR::PID<float>::Param trig_angle_pid,
               LibXR::PID<float>::Param trig_speed_pid,
               LibXR::PID<float>::Param fric_speed_pid_0,
               LibXR::PID<float>::Param fric_speed_pid_1,
               LibXR::PID<float>::Param fric_speed_pid_2,
               LibXR::PID<float>::Param fric_speed_pid_3,
               LauncherParam launcher_param, CMD *cmd)
      : PARAM(launcher_param),
        motor_fric_(motor_fric),
        motor_trig_(motor_trig),
        trig_angle_pid_(trig_angle_pid),
        trig_speed_pid_(trig_speed_pid),
        fric_speed_pid_{fric_speed_pid_0, fric_speed_pid_1, fric_speed_pid_2,
                        fric_speed_pid_3},
        cmd_(cmd) {
    UNUSED(hw);
    UNUSED(app);
    // Hardware initialization example:
    // auto dev = hw.template Find<LibXR::GPIO>("led");

  }

 private:
  const LauncherParam PARAM;
  RefereeData referee_data_;
  TRIGMODE trig_mod_ = TRIGMODE::SAFE;

  HeatControl heat_ctrl_;

  bool first_loading_ = false;

  float dt_ = 0.0f;

  LibXR::MillisecondTimestamp now_ = 0;

  LibXR::MicrosecondTimestamp last_wakeup_;

  LibXR::MillisecondTimestamp last_change_angle_time_ = 0;

  LibXR::MillisecondTimestamp start_loading_time_ = 0;

  RMMotor *motor_fric_front_left_;
  RMMotor *motor_fric_front_right_;
  RMMotor *motor_fric_back_left_;
  RMMotor *motor_fric_back_right_;

  std::array<RMMotor *, FRIC_NUM> motor_fric_;
  std::array<RMMotor *, TRIG_NUM> motor_trig_;

  float trig_setpoint_angle_ = 0.0f;
  float trig_setpoint_speed_ = 0.0f;

  float trig_zero_angle_ = 0.0f;
  float trig_angle_ = 0.0f;
  float trig_output_ = 0.0f;

  float fric_target_speed_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  float fric_output_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  LibXR::PID<float> trig_angle_pid_;
  LibXR::PID<float> trig_speed_pid_;

  LibXR::PID<float> fric_speed_pid_[4] = {
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param())};

  CMD::LauncherCMD launcher_cmd_;

  float current_back_left_ = 0.0f;

  bool fire_flag_ = false;  // 发射命令标志位

  uint8_t fired_ = 0;  // 已发射弹丸

  bool enable_fire_ = false;  // 拨弹盘旋转命令发出标志位
  bool mark_launch_ = false;  // 拨弹发射完成标志位

  LibXR::MillisecondTimestamp start_fire_time_ = 0;
  LibXR::MillisecondTimestamp finish_fire_time_ = 0;
  float real_launch_delay_ = 0.0f;

  bool last_fire_notify_ = false;
  bool press_continue_ = false;
  LibXR::MillisecondTimestamp fire_press_time_ = 0;

  uint8_t delay_time_ = 0;

  CMD *cmd_;
  LibXR::Thread thread_;
  LibXR::Semaphore semaphore_;
  LibXR::Mutex mutex_;
  uint32_t launcher_event_;
};

