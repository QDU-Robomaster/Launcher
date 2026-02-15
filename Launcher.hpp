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
  - task_stack_depth: 4096
  - pid_trig_angle:
      k: 1.0
      p: 4000.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 4000.0
      cycle: false
  - pid_trig_speed:
      k: 1.0
      p: 0.0012
      i: 0.0005
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
  - launcher_param:
      fric1_setpoint_speed: 4950.0
      fric2_setpoint_speed: 3820.0
      trig_gear_ratio: 19.2032
      num_trig_tooth: 6
      trig_freq_: 0.0
  - cmd: '@&cmd'
template_args:
  - LauncherType: HeroLauncher
required_hardware:
  - dr16
  - can
depends:
  - qdu-future/CMD
  - qdu-future/RMMotor
=== END MANIFEST === */
// clang-format on

#include <array>
#include <cstdint>

#include "CMD.hpp"
#include "HeroLauncher.hpp"
#include "InfantryLauncher.hpp"
#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "libxr_def.hpp"
#include "pid.hpp"

template <class LauncherType>
class Launcher : public LibXR::Application {
 public:
  using LauncherEvent = typename LauncherType::LauncherEvent;
  struct LauncherParam {
    float fric1_setpoint_speed;
    float fric2_setpoint_speed;
    float trig_gear_ratio;
    uint8_t num_trig_tooth;
    /*弹频*/
    float trig_freq_;
  };

  Launcher(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
           RMMotor* motor_fric_front_left, RMMotor* motor_fric_front_right,
           RMMotor* motor_fric_back_left, RMMotor* motor_fric_back_right,
           RMMotor* motor_trig, uint32_t task_stack_depth,
           LibXR::PID<float>::Param pid_trig_angle,
           LibXR::PID<float>::Param pid_trig_speed,
           LibXR::PID<float>::Param pid_fric_speed_0,
           LibXR::PID<float>::Param pid_fric_speed_1,
           LibXR::PID<float>::Param pid_fric_speed_2,
           LibXR::PID<float>::Param pid_fric_speed_3,
           LauncherParam launcher_param, CMD* cmd)
      : launcher_(hw, app, motor_fric_front_left, motor_fric_front_right,
                  motor_fric_back_left, motor_fric_back_right, motor_trig,
                  task_stack_depth, pid_trig_angle, pid_trig_speed,
                  pid_fric_speed_0, pid_fric_speed_1, pid_fric_speed_2,
                  pid_fric_speed_3,
                  typename LauncherType::LauncherParam{
                      launcher_param.fric1_setpoint_speed,
                      launcher_param.fric2_setpoint_speed,
                      launcher_param.trig_gear_ratio,
                      launcher_param.num_trig_tooth, launcher_param.trig_freq_},
                  cmd) {
    UNUSED(hw);
    UNUSED(app);

    auto callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Launcher* launcher, uint32_t event_id) {
          UNUSED(in_isr);
          launcher->EventHandler(event_id);
        },
        this);
    launcher_event_.Register(
        static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_RELAX), callback);
    launcher_event_.Register(
        static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_SAFE), callback);
    launcher_event_.Register(
        static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_READY), callback);
  }

  /**
   * @brief 获取发射器的事件处理器
   * @details 通过此事件处理器可以向发射器发送事件消息，控制发射器的行为模式
   * @return LibXR::Event& 事件处理器的引用
   */
  LibXR::Event& GetEvent() { return launcher_event_; }

  /**
   * @brief 事件处理器，根据传入的事件ID执行相应操作
   * @param event_id 触发的事件ID
   */
  void EventHandler(uint32_t event_id) { launcher_.SetMode(event_id); }

  void OnMonitor() override {}

 private:
  LauncherType launcher_;
  LibXR::Event launcher_event_;
};
