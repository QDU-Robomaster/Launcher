# Launcher

发射机构总控模块：模板外壳提供控制线程与事件，发射逻辑由 HeroLauncher 或 InfantryLauncher 实现 / Launcher master Module whose template shell provides the control thread and the events, with the launch logic implemented by HeroLauncher or InfantryLauncher

## 1. 模块作用 / Purpose

`Launcher<LauncherType>` 是一个模板外壳：外壳负责控制线程、CMD 失控与恢复事件和摩擦轮模式事件，具体的摩擦轮、拨弹和热量逻辑由模板参数选择本仓库内的实现：`HeroLauncher`（英雄，`HeroLauncher.hpp`）或 `InfantryLauncher`（步兵，`InfantryLauncher.hpp`）。

构造时，Launcher 创建控制线程 `LauncherThread`（栈深 `task_stack_depth`，优先级 `thread_priority`）。线程每次循环先休眠 2 ms，再用微秒时间戳计算控制周期 `dt`，读取两个 Topic 的最新数据，然后依次执行 `Update()`（电机反馈）、`Solve()`（状态机与热量）和 `Control()`（PID 输出）。发射命令 Topic（`CMD::LauncherCMD`，`isfire` 为开火命令）的名称由 `param.launcher_cmd_topic_name` 指定，默认 `launcher_cmd`，与 CMD 的 `launcher_cmd_topic_name` 一致。裁判系统发射数据 Topic（`Referee::LauncherPack`）的名称由 `param.launcher_ref_topic_name` 指定，默认 `launcher_ref`，与 Referee 的 `referee_launcher_tp_name` 一致；两种发射机构的热量上限取其中的 `rs.shooter_heat_limit`，冷却值取 `rs.shooter_cooling_value`。收到裁判系统数据之前热量上限为 0，此时 `InfantryLauncher` 不发射，`HeroLauncher` 只完成首发标定。

CMD 事件：`CMD_EVENT_LOST_CTRL` 时调用 `LostCtrl()`，复位发射状态并切换到 `SET_FRICMODE_RELAX`；`CMD_EVENT_START_CTRL` 时切换到 `SET_FRICMODE_RELAX`。

摩擦轮模式：`GetEvent()` 返回的 `LibXR::Event` 上注册了 `LauncherType::LauncherEvent` 的 `SET_FRICMODE_RELAX`、`SET_FRICMODE_SAFE`、`SET_FRICMODE_READY`，激活对应的事件 ID 即切换模式，通常由 `EventBinder` 把遥控器事件绑定到这些 ID。`RELAX` 与 `SAFE` 下摩擦轮目标转速为 0，`READY` 下使用配置转速。

定义 `DEBUG` 时，Launcher 向 `ramfs` 添加调试命令文件 `launcher`，见第 3 节。

`Launcher<LauncherType>` is a template shell: the shell provides the control thread, the CMD lost-control and recovery events and the friction wheel mode events, while the concrete friction wheel, trigger and heat logic is selected by the template parameter from the implementations in this repository: `HeroLauncher` (hero robot, `HeroLauncher.hpp`) or `InfantryLauncher` (infantry robot, `InfantryLauncher.hpp`).

Upon construction, Launcher creates the control thread `LauncherThread` (stack depth `task_stack_depth`, priority `thread_priority`). Each loop iteration first sleeps for 2 ms, then computes the control period `dt` from microsecond timestamps, reads the latest data of two Topics, and runs `Update()` (motor feedback), `Solve()` (state machine and heat) and `Control()` (PID output). The name of the launcher command Topic (`CMD::LauncherCMD`, where `isfire` is the fire command) is given by `param.launcher_cmd_topic_name`, default `launcher_cmd`, matching the `launcher_cmd_topic_name` of CMD. The name of the referee launcher data Topic (`Referee::LauncherPack`) is given by `param.launcher_ref_topic_name`, default `launcher_ref`, matching the `referee_launcher_tp_name` of Referee; both launcher implementations take the heat limit from its `rs.shooter_heat_limit` and the cooling value from `rs.shooter_cooling_value`. Until referee data arrives the heat limit is 0; `InfantryLauncher` then does not fire, and `HeroLauncher` only completes the first-shot calibration.

CMD events: `CMD_EVENT_LOST_CTRL` calls `LostCtrl()`, which resets the launch state and switches to `SET_FRICMODE_RELAX`; `CMD_EVENT_START_CTRL` switches to `SET_FRICMODE_RELAX`.

Friction wheel modes: `GetEvent()` returns a `LibXR::Event` on which `SET_FRICMODE_RELAX`, `SET_FRICMODE_SAFE` and `SET_FRICMODE_READY` of `LauncherType::LauncherEvent` are registered; activating the corresponding event ID switches the mode, and `EventBinder` usually binds remote controller events to these IDs. In `RELAX` and `SAFE` the friction wheel target speed is 0, and in `READY` the configured speed is used.

When `DEBUG` is defined, Launcher adds the debug command file `launcher` to `ramfs`, see section 3.

## 2. 发射机构实现 / Launcher Implementations

`HeroLauncher`：

- 四个摩擦轮：前左与前右为二级摩擦轮（目标 `fric2_setpoint_speed`），后左与后右为一级摩擦轮（目标 `fric1_setpoint_speed`），各有一个速度环（`pid_fric_speed_0` 至 `pid_fric_speed_3`）。`RELAX` 与 `SAFE` 下目标转速为 0，并复位发射状态（重新首发标定、拨弹角度清零）。
- 软启动：后左摩擦轮转速超过 `fric1_setpoint_speed` 之前，四个速度环的输出限幅为 0.05，之后为 1.0；进入 `RELAX` 或 `SAFE`、失控或复位时重新软启动。
- 拨弹为角度环（`pid_trig_angle`）串联速度环（`pid_trig_speed`），拨弹盘角度由电机角度除以 `trig_gear_ratio` 得到。首次发射时拨弹盘每周期转过 2π / 1000 缓慢上弹，软启动完成后检测到出弹即记录零点，设定角取零点沿拨弹方向再转 0.9 rad；之后上一发已判定出弹时，每次单发拨弹盘转过 `2π / num_trig_tooth`。开火上升沿为单发，按住超过 200 ms 进入连发模式。
- 出弹判定：后左摩擦轮力矩绝对值超过 0.04；1000 ms 内未检测到出弹时结束本次发射，允许下一发。
- 热量按每发 100 累计并按冷却值衰减，可发射数为 ⌊(热量上限 − 热量) / 100⌋，为 0 时拨弹盘不再推进。
- 拨弹电机的 `Update()` 返回值不为 `ErrorCode::OK`（例如拨弹电机未上电、没有反馈）时，不下发摩擦轮与拨弹的控制量，并复位发射状态。
- 后左与后右摩擦轮电机在构造时 `ASSERT` 非空。

`InfantryLauncher`：

- 使用前左与前右两个摩擦轮（目标均为 `fric1_setpoint_speed`，PID 为 `pid_fric_speed_0` 与 `pid_fric_speed_1`）和拨弹电机。
- 状态机为 `RELAX`、`STOP`、`NORMAL`、`JAMMED`。拨弹电机力矩绝对值超过 0.015 判为卡弹；进入卡弹处理时拨弹盘反转 0.8 发的角度（0.8 · 2π / 10），卡弹持续期间保持这一目标，离开卡弹状态超过 80 ms 后再次卡弹时重复。开火上升沿为单发，拨弹盘每发转过 2π / 10；按住超过 0.5 s 进入连发，弹频为 `launcher_param.trig_freq_`（期望弹频，单位 Hz），剩余热量不超过 2.3 发时线性降低到冷却弹频（冷却值 / 10）。拨弹盘角速度限制为 `1.5 · 2π · 弹频 / num_trig_tooth`。
- 出弹判定：两个摩擦轮转速都低于 `fric1_setpoint_speed - 218`，200 ms 内未检测到时复位本次发射。
- 热量每 50 ms 更新一次，每发热量按 10 计，每次扣除冷却值 × 0.05；剩余热量不足一发时停止发射，弹频保持原值。
- `SAFE` 下摩擦轮控制量缩小为速度环输出的 1/50。
- 电机反馈状态为 0 时自动使能，状态异常时清错；`RELAX` 下所有电机 `Relax()`，失控时拨弹电机失能。

`HeroLauncher`:

- Four friction wheels: front-left and front-right are the second-stage wheels (target `fric2_setpoint_speed`), and back-left and back-right are the first-stage wheels (target `fric1_setpoint_speed`), each with one speed loop (`pid_fric_speed_0` to `pid_fric_speed_3`). In `RELAX` and `SAFE` the target speed is 0 and the launch state is reset (the first-shot calibration starts over and the trigger angles are cleared).
- Soft start: until the back-left friction wheel speed exceeds `fric1_setpoint_speed`, the output limit of the four speed loops is 0.05, and 1.0 afterwards; entering `RELAX` or `SAFE`, losing control or a reset starts the soft start again.
- The trigger is an angle loop (`pid_trig_angle`) in series with a speed loop (`pid_trig_speed`), and the trigger disc angle comes from the motor angle divided by `trig_gear_ratio`. At the first shot the disc turns slowly by 2π / 1000 per cycle to load; once the soft start has finished, the zero point is recorded when a round is detected leaving, and the setpoint is placed 0.9 rad further in the feed direction. Afterwards, when the previous round has been detected, each single shot advances the disc by `2π / num_trig_tooth`. A rising edge of fire is a single shot, and holding for more than 200 ms enters the continuous mode.
- Round detection: the back-left friction wheel torque magnitude exceeds 0.04; when no round is detected within 1000 ms, the shot is ended and the next one is allowed.
- Heat accumulates 100 per round and decays at the cooling value; the number of available shots is ⌊(heat limit − heat) / 100⌋, and the trigger disc stops advancing when it is 0.
- When the `Update()` of the trigger motor does not return `ErrorCode::OK` (for example when the trigger motor is unpowered and sends no feedback), no friction wheel or trigger output is sent and the launch state is reset.
- The back-left and back-right friction wheel motors are checked with `ASSERT` for non-null at construction.

`InfantryLauncher`:

- It uses the front-left and front-right friction wheels (both with target `fric1_setpoint_speed`, PIDs `pid_fric_speed_0` and `pid_fric_speed_1`) and the trigger motor.
- The state machine is `RELAX`, `STOP`, `NORMAL` and `JAMMED`. A trigger motor torque magnitude above 0.015 is taken as a jam; on entering jam handling the disc turns back by 0.8 of the per-round angle (0.8 · 2π / 10) and keeps this target while the jam lasts, and a new jam more than 80 ms after leaving the jam state repeats this. A rising edge of fire is a single shot, and the disc advances 2π / 10 per round; holding for more than 0.5 s enters continuous fire at the fire rate `launcher_param.trig_freq_` (expected fire rate in Hz), which decreases linearly to the cooling fire rate (cooling value / 10) when the remaining heat is 2.3 rounds or less. The trigger disc angular velocity is limited to `1.5 · 2π · fire rate / num_trig_tooth`.
- Round detection: both friction wheel speeds are below `fric1_setpoint_speed - 218`; when none is detected within 200 ms, the shot is reset.
- The heat is updated every 50 ms with 10 heat per round, and each update subtracts the cooling value × 0.05; firing stops when the remaining heat is less than one round, and the fire rate keeps its value.
- In `SAFE` the friction wheel outputs are scaled down to 1/50 of the speed-loop outputs.
- A motor whose feedback state is 0 is enabled automatically, and errors are cleared when the state is abnormal; in `RELAX` all motors are `Relax()`ed, and on lost control the trigger motor is disabled.

## 3. 调试命令 / Debug Command

定义 `DEBUG` 时，在 RamFS 终端中使用 `launcher` 命令查看实时状态：

```sh
launcher once [state|motor|heat|shot|full]
launcher monitor <time_ms> [interval_ms] [state|motor|heat|shot|full]
```

视图：`state`（模式、开火命令、`dt` 等）、`motor`（电机反馈与目标）、`heat`（热量）、`shot`（发射计数、出弹延时等）、`full`（全部，默认）。命令解析与字段打印由 `QDU-Robomaster/DebugCore` 提供。

When `DEBUG` is defined, the `launcher` command in the RamFS terminal shows the live state with the commands in the code block above.

Views: `state` (mode, fire command, `dt` and so on), `motor` (motor feedback and targets), `heat` (heat), `shot` (shot counts, round latency and so on) and `full` (everything, the default). Command parsing and field printing are provided by `QDU-Robomaster/DebugCore`.

## 4. 构造接口 / Constructor

```cpp
template <class LauncherType>
class Launcher;

Launcher(RMMotor& motor_fric_front_left, RMMotor& motor_fric_front_right,
         RMMotor* motor_fric_back_left, RMMotor* motor_fric_back_right,
         RMMotor& motor_trig, CMD& cmd, LibXR::RamFS& ramfs,
         const Param& param = {...});  // 节选 / excerpt
```

模板参数：

- `LauncherType`：`HeroLauncher` 或 `InfantryLauncher`。

依赖：

- `motor_fric_front_left`、`motor_fric_front_right`：`RMMotor`，前左与前右摩擦轮电机。
- `motor_fric_back_left`、`motor_fric_back_right`：`RMMotor*`，后左与后右摩擦轮电机，`HeroLauncher` 使用。
- `motor_trig`：`RMMotor`，拨弹电机。
- `cmd`：`CMD` 实例，提供失控与恢复事件。
- `ramfs`：`LibXR::RamFS`，定义 `DEBUG` 时在其中注册 `launcher` 命令文件。

配置参数（`param`，默认值为步兵参数，只由 `HeroLauncher` 使用的 `pid_fric_speed_2`、`pid_fric_speed_3` 与 `fric2_setpoint_speed` 默认为英雄参数；PID 为 `LibXR::PID<float>::Param`，字段为 `k, p, i, d, i_limit, out_limit, cycle`）：

- `task_stack_depth`：控制线程栈深，默认 1536。
- `pid_trig_angle`：拨弹角度环，默认 `{.k = 1.0f, .p = 40.0f, .i = 0.1f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 0.0f, .cycle = false}`。
- `pid_trig_speed`：拨弹速度环，默认 `{.k = 1.0f, .p = 0.15f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 0.0f, .cycle = false}`。
- `pid_fric_speed_0` 至 `pid_fric_speed_3`：摩擦轮速度环，依次为前左、前右、后左、后右；`pid_fric_speed_0` 与 `pid_fric_speed_1` 默认为 `{.k = 0.8f, .p = 0.0003f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 0.6f, .cycle = false}`，`pid_fric_speed_2` 与 `pid_fric_speed_3` 默认为 `{.k = 1.0f, .p = 0.002f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 1.0f, .cycle = false}`。
- `launcher_param.fric1_setpoint_speed`：一级摩擦轮目标转速，`InfantryLauncher` 的两个摩擦轮都使用它，默认 6500。
- `launcher_param.fric2_setpoint_speed`：二级摩擦轮目标转速，`HeroLauncher` 使用，默认 3820。
- `launcher_param.trig_gear_ratio`：拨弹电机减速比，默认 36。
- `launcher_param.num_trig_tooth`：拨弹盘齿数，默认 10。
- `launcher_param.trig_freq_`：期望弹频，单位 Hz，`InfantryLauncher` 使用，默认 16。
- `thread_priority`：控制线程优先级，默认 `LibXR::Thread::Priority::HIGH`。
- `launcher_cmd_topic_name`：订阅的发射控制命令 Topic 名称，默认 `"launcher_cmd"`。
- `launcher_ref_topic_name`：订阅的裁判系统发射数据 Topic 名称，默认 `"launcher_ref"`。

Template parameter:

- `LauncherType`: `HeroLauncher` or `InfantryLauncher`.

Dependencies:

- `motor_fric_front_left`, `motor_fric_front_right`: `RMMotor` objects for the front-left and front-right friction wheel motors.
- `motor_fric_back_left`, `motor_fric_back_right`: `RMMotor*`, the back-left and back-right friction wheel motors, used by `HeroLauncher`.
- `motor_trig`: the `RMMotor` of the trigger motor.
- `cmd`: the `CMD` instance, providing the lost-control and recovery events.
- `ramfs`: the `LibXR::RamFS` in which the `launcher` command file is registered when `DEBUG` is defined.

Configuration parameters (`param`, with the infantry parameters as defaults, except that `pid_fric_speed_2`, `pid_fric_speed_3` and `fric2_setpoint_speed`, used only by `HeroLauncher`, default to the hero parameters; the PIDs are `LibXR::PID<float>::Param` with fields `k, p, i, d, i_limit, out_limit, cycle`):

- `task_stack_depth`: control thread stack depth, default 1536.
- `pid_trig_angle`: trigger angle loop, default `{.k = 1.0f, .p = 40.0f, .i = 0.1f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 0.0f, .cycle = false}`.
- `pid_trig_speed`: trigger speed loop, default `{.k = 1.0f, .p = 0.15f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 0.0f, .cycle = false}`.
- `pid_fric_speed_0` to `pid_fric_speed_3`: friction wheel speed loops for front-left, front-right, back-left and back-right; `pid_fric_speed_0` and `pid_fric_speed_1` default to `{.k = 0.8f, .p = 0.0003f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 0.6f, .cycle = false}`, and `pid_fric_speed_2` and `pid_fric_speed_3` to `{.k = 1.0f, .p = 0.002f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 1.0f, .cycle = false}`.
- `launcher_param.fric1_setpoint_speed`: first-stage friction wheel target speed, used by both friction wheels of `InfantryLauncher`, default 6500.
- `launcher_param.fric2_setpoint_speed`: second-stage friction wheel target speed, used by `HeroLauncher`, default 3820.
- `launcher_param.trig_gear_ratio`: trigger motor reduction ratio, default 36.
- `launcher_param.num_trig_tooth`: number of trigger disc teeth, default 10.
- `launcher_param.trig_freq_`: expected fire rate in Hz, used by `InfantryLauncher`, default 16.
- `thread_priority`: control thread priority, default `LibXR::Thread::Priority::HIGH`.
- `launcher_cmd_topic_name`: name of the subscribed launcher command Topic, default `"launcher_cmd"`.
- `launcher_ref_topic_name`: name of the subscribed referee launcher data Topic, default `"launcher_ref"`.

## 5. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `param.launcher_cmd_topic_name`（默认 `launcher_cmd`） | 订阅 | `CMD::LauncherCMD` | 发射命令 `isfire` |
| `param.launcher_ref_topic_name`（默认 `launcher_ref`） | 订阅 | `Referee::LauncherPack` | 热量上限 `rs.shooter_heat_limit` 与冷却值 `rs.shooter_cooling_value` |
| `shoot_dt` | 发布（`InfantryLauncher`） | `float` | 最近一次开火命令到出弹的延时，单位 s |
| `shoot_number` | 发布（`InfantryLauncher`） | `float` | 累计发射数 |
| `trig_freq` | 发布（`InfantryLauncher`） | `float` | 当前弹频，单位 Hz |

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `param.launcher_cmd_topic_name` (default `launcher_cmd`) | Subscribe | `CMD::LauncherCMD` | Fire command `isfire` |
| `param.launcher_ref_topic_name` (default `launcher_ref`) | Subscribe | `Referee::LauncherPack` | Heat limit `rs.shooter_heat_limit` and cooling value `rs.shooter_cooling_value` |
| `shoot_dt` | Publish (`InfantryLauncher`) | `float` | Delay from the latest fire command to the round leaving, in s |
| `shoot_number` | Publish (`InfantryLauncher`) | `float` | Cumulative number of rounds |
| `trig_freq` | Publish (`InfantryLauncher`) | `float` | Current fire rate in Hz |

## 6. 配置示例 / Configuration Example

`xrobot instance add QDU-Robomaster/Launcher --template-arg HeroLauncher` 写入的实例，`template_args` 取 `HeroLauncher` 或 `InfantryLauncher`，依赖填写为其他模块实例的 id：电机取自 `QDU-Robomaster/RMMotor` 实例，`cmd` 取自 `QDU-Robomaster/CMD` 实例，`ramfs` 取自 BSP 中注册的 RamFS 名称（`XR_REGISTER`），它们须在本实例之前列出；指针依赖写成 `'&id'`，`InfantryLauncher` 不使用的后左与后右摩擦轮可写 `nullptr`。工具按默认值写出步兵参数，下例为 `HeroLauncher`，`param` 已改为英雄发射机构的取值：

An instance written by `xrobot instance add QDU-Robomaster/Launcher --template-arg HeroLauncher`; `template_args` takes `HeroLauncher` or `InfantryLauncher`, and the dependencies are set to the ids of other Module instances: the motors come from `QDU-Robomaster/RMMotor` instances, `cmd` from a `QDU-Robomaster/CMD` instance, and `ramfs` from a RamFS name registered by the BSP (`XR_REGISTER`); they are listed before this instance, pointer dependencies are written as `'&id'`, and the back-left and back-right friction wheels, which `InfantryLauncher` does not use, may be `nullptr`. The tool writes the infantry parameters from the defaults; the example uses `HeroLauncher` with `param` changed to the values of the hero launcher:

```yaml
modules:
  - module: QDU-Robomaster/Launcher
    id: launcher_0
    template_args:
      - HeroLauncher
    args:
      - motor_fric_front_left: motor_fric_front_left
      - motor_fric_front_right: motor_fric_front_right
      - motor_fric_back_left: '&motor_fric_back_left'
      - motor_fric_back_right: '&motor_fric_back_right'
      - motor_trig: motor_trig
      - cmd: cmd
      - ramfs: ramfs
      - param:
          task_stack_depth: 4096
          pid_trig_angle:
            k: 1.0f
            p: 4000.0f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 4000.0f
            cycle: false
          pid_trig_speed:
            k: 1.0f
            p: 0.0012f
            i: 0.0005f
            d: 0.0f
            i_limit: 1.0f
            out_limit: 1.0f
            cycle: false
          pid_fric_speed_0:
            k: 1.0f
            p: 0.002f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 1.0f
            cycle: false
          pid_fric_speed_1:
            k: 1.0f
            p: 0.002f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 1.0f
            cycle: false
          pid_fric_speed_2:
            k: 1.0f
            p: 0.002f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 1.0f
            cycle: false
          pid_fric_speed_3:
            k: 1.0f
            p: 0.002f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 1.0f
            cycle: false
          launcher_param:
            fric1_setpoint_speed: 4950.0f
            fric2_setpoint_speed: 3820.0f
            trig_gear_ratio: 19.2032f
            num_trig_tooth: 6
            trig_freq_: 0.0f
          thread_priority: LibXR::Thread::Priority::HIGH
          launcher_cmd_topic_name: "launcher_cmd"
          launcher_ref_topic_name: "launcher_ref"
```

## 7. 依赖与硬件 / Dependencies and Hardware

依赖：

- `QDU-Robomaster/CMD`：`launcher_cmd` 的类型与 CMD 失控、恢复事件。
- `QDU-Robomaster/RMMotor`：摩擦轮与拨弹电机（`RMMotor`）。
- `QDU-Robomaster/Motor`：`Motor::Feedback` 与 `Motor::MotorCmd`。
- `QDU-Robomaster/DebugCore`：定义 `DEBUG` 时的 `launcher` 调试命令。
- `QDU-Robomaster/Referee`：`launcher_ref` 的类型 `Referee::LauncherPack`。
- LibXR。

硬件：摩擦轮电机与拨弹电机，均通过 `RMMotor` 实例接入；`HeroLauncher` 使用四个摩擦轮，`InfantryLauncher` 使用前两个摩擦轮。热量上限与冷却值来自发布 `launcher_ref` 的 Referee 实例。

Dependencies:

- `QDU-Robomaster/CMD`: the type of `launcher_cmd` and the CMD lost-control and recovery events.
- `QDU-Robomaster/RMMotor`: the friction wheel and trigger motors (`RMMotor`).
- `QDU-Robomaster/Motor`: `Motor::Feedback` and `Motor::MotorCmd`.
- `QDU-Robomaster/DebugCore`: the `launcher` debug command when `DEBUG` is defined.
- `QDU-Robomaster/Referee`: `Referee::LauncherPack`, the type of `launcher_ref`.
- LibXR.

Hardware: the friction wheel motors and the trigger motor, all attached through `RMMotor` instances; `HeroLauncher` uses four friction wheels and `InfantryLauncher` uses the first two. The heat limit and the cooling value come from the Referee instance that publishes `launcher_ref`.
