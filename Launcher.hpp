#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 发射机构模板外壳，负责控制线程、CMD 失控与恢复事件、摩擦轮模式事件，发射逻辑由模板参数 LauncherType 选择本仓库内的 HeroLauncher 或 InfantryLauncher 实现
depends:
- id: QDU-Robomaster/CMD
  ref: same-or-dev
- id: QDU-Robomaster/RMMotor
  ref: same-or-dev
- id: QDU-Robomaster/Motor
  ref: same-or-dev
- id: QDU-Robomaster/DebugCore
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include <cstdint>

#include "CMD.hpp"
#include "HeroLauncher.hpp"
#include "InfantryLauncher.hpp"
#include "RMMotor.hpp"
#include "event.hpp"
#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "pid.hpp"
#include "ramfs.hpp"
#include "thread.hpp"
#include "timebase.hpp"

#ifdef DEBUG
#include "DebugCore.hpp"
#endif

template <class LauncherType>
class Launcher {
 public:
  using LauncherEvent = typename LauncherType::LauncherEvent;

  struct LauncherParam {
    float fric1_setpoint_speed;
    float fric2_setpoint_speed;
    float trig_gear_ratio;
    uint8_t num_trig_tooth;
    float trig_freq_;
  };

  struct Param {
    uint32_t task_stack_depth;  ///< 控制线程栈深度
    LibXR::PID<float>::Param pid_trig_angle;    ///< 拨弹角度环参数
    LibXR::PID<float>::Param pid_trig_speed;    ///< 拨弹速度环参数
    LibXR::PID<float>::Param pid_fric_speed_0;  ///< 摩擦轮0速度环参数
    LibXR::PID<float>::Param pid_fric_speed_1;  ///< 摩擦轮1速度环参数
    LibXR::PID<float>::Param pid_fric_speed_2;  ///< 摩擦轮2速度环参数
    LibXR::PID<float>::Param pid_fric_speed_3;  ///< 摩擦轮3速度环参数
    LauncherParam launcher_param;               ///< 发射机构参数
    LibXR::Thread::Priority thread_priority;    ///< 控制线程优先级
    const char* launcher_cmd_topic_name;        ///< 订阅的发射控制命令 Topic 名称
  };

  /**
   * @brief 构造发射机构外壳
   * @param motor_fric_front_left 前左摩擦轮电机
   * @param motor_fric_front_right 前右摩擦轮电机
   * @param motor_fric_back_left 后左摩擦轮电机；HeroLauncher 必须提供，
   *        InfantryLauncher 不使用，可传 nullptr
   * @param motor_fric_back_right 后右摩擦轮电机；HeroLauncher 必须提供，
   *        InfantryLauncher 不使用，可传 nullptr
   * @param motor_trig 拨弹电机
   * @param cmd CMD 模块
   * @param ramfs 定义 DEBUG 时注册 "launcher" 调试命令文件的 RamFS；
   *        未定义 DEBUG 时不使用
   * @param param 线程、PID 与发射机构参数
   */
  Launcher(
      RMMotor& motor_fric_front_left, RMMotor& motor_fric_front_right,
      RMMotor* motor_fric_back_left, RMMotor* motor_fric_back_right,
      RMMotor& motor_trig, CMD& cmd, LibXR::RamFS& ramfs,
      const Param& param = {.task_stack_depth = 4096, .pid_trig_angle = {.k = 1.0f, .p = 4000.0f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 4000.0f, .cycle = false}, .pid_trig_speed = {.k = 1.0f, .p = 0.0012f, .i = 0.0005f, .d = 0.0f, .i_limit = 1.0f, .out_limit = 1.0f, .cycle = false}, .pid_fric_speed_0 = {.k = 1.0f, .p = 0.002f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 1.0f, .cycle = false}, .pid_fric_speed_1 = {.k = 1.0f, .p = 0.002f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 1.0f, .cycle = false}, .pid_fric_speed_2 = {.k = 1.0f, .p = 0.002f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 1.0f, .cycle = false}, .pid_fric_speed_3 = {.k = 1.0f, .p = 0.002f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 1.0f, .cycle = false}, .launcher_param = {.fric1_setpoint_speed = 4950.0f, .fric2_setpoint_speed = 3820.0f, .trig_gear_ratio = 19.2032f, .num_trig_tooth = 6, .trig_freq_ = 0.0f}, .thread_priority = LibXR::Thread::Priority::HIGH, .launcher_cmd_topic_name = "launcher_cmd"})
      : launcher_(&motor_fric_front_left, &motor_fric_front_right,
                  motor_fric_back_left, motor_fric_back_right, &motor_trig,
                  param.task_stack_depth, param.pid_trig_angle,
                  param.pid_trig_speed, param.pid_fric_speed_0,
                  param.pid_fric_speed_1, param.pid_fric_speed_2,
                  param.pid_fric_speed_3,
                  typename LauncherType::LauncherParam{
                      param.launcher_param.fric1_setpoint_speed,
                      param.launcher_param.fric2_setpoint_speed,
                      param.launcher_param.trig_gear_ratio,
                      param.launcher_param.num_trig_tooth,
                      param.launcher_param.trig_freq_},
                  &cmd)
#ifdef DEBUG
        ,
        cmd_file_(LibXR::RamFS::CreateFile(
            "launcher",
            debug_core::command_thunk<LauncherType,
                                      &LauncherType::DebugCommand>,
            &launcher_))
#endif
  {
#ifdef DEBUG
    ramfs.Add(cmd_file_);
#else
    UNUSED(ramfs);
#endif

    launcher_cmd_topic_name_ = param.launcher_cmd_topic_name;
    thread_.Create(this, ThreadFunc, "LauncherThread", param.task_stack_depth,
                   param.thread_priority);

    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Launcher* self, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          self->mutex_.Lock();
          self->launcher_.LostCtrl();
          self->mutex_.Unlock();
        },
        this);

    auto start_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Launcher* self, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          self->mutex_.Lock();
          self->launcher_.SetMode(
              static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_RELAX));
          self->mutex_.Unlock();
        },
        this);

    cmd.GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
    cmd.GetEvent().Register(CMD::CMD_EVENT_START_CTRL, start_ctrl_callback);

    auto event_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Launcher* self, uint32_t event_id) {
          UNUSED(in_isr);
          self->mutex_.Lock();
          self->launcher_.SetMode(event_id);
          self->mutex_.Unlock();
        },
        this);
    launcher_event_.Register(
        static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_RELAX),
        event_callback);
    launcher_event_.Register(
        static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_SAFE),
        event_callback);
    launcher_event_.Register(
        static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_READY),
        event_callback);
  }

  LibXR::Event& GetEvent() { return launcher_event_; }

 private:
  LauncherType launcher_;
  LibXR::Event launcher_event_;
  const char* launcher_cmd_topic_name_ = nullptr;
  LibXR::Thread thread_;
  LibXR::Mutex mutex_;

#ifdef DEBUG
  LibXR::RamFS::File cmd_file_;
#endif

  static void ThreadFunc(Launcher* self) {
    LibXR::Topic::ASyncSubscriber<CMD::LauncherCMD> cmd_sub(self->launcher_cmd_topic_name_);
    cmd_sub.StartWaiting();
    self->last_wakeup_time_ = LibXR::Timebase::GetMilliseconds();
    self->last_online_time_ = LibXR::Timebase::GetMicroseconds();

    while (true) {
      LibXR::Thread::SleepUntil(self->last_wakeup_time_, 2);

      auto now = LibXR::Timebase::GetMicroseconds();
      self->launcher_.SetControlDt((now - self->last_online_time_).ToSecondf());
      self->last_online_time_ = now;

      if (cmd_sub.Available()) {
        self->launcher_.launcher_cmd_ = cmd_sub.GetData();
        cmd_sub.StartWaiting();
      }

      self->mutex_.Lock();
      self->launcher_.Update();
      self->launcher_.Solve();
      self->mutex_.Unlock();
      self->launcher_.Control();
    }
  }

  LibXR::MillisecondTimestamp last_wakeup_time_ = 0;
  LibXR::MicrosecondTimestamp last_online_time_ = 0;
};
