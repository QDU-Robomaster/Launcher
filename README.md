# Launcher

发射机构总控模块。`Launcher<LauncherType>` 是一个模板外壳：外壳负责控制线程、CMD 失控 / 恢复
事件和摩擦轮模式事件，具体的摩擦轮、拨弹和热量逻辑由模板参数选择本仓库内的实现：
`HeroLauncher`（英雄，`HeroLauncher.hpp`）或 `InfantryLauncher`（步兵，`InfantryLauncher.hpp`）。

> **不能与独立模块同时选用。** 本仓库的 `HeroLauncher` / `InfantryLauncher` 与独立模块
> `QDU-Robomaster/HeroLauncher`、`QDU-Robomaster/InfantryLauncher` 定义了同名的全局类
> （`InfantryLauncher` 还有同名的 `launcher::param` 常量），在同一工程中同时选用
> `QDU-Robomaster/Launcher` 和其中任何一个模块会导致编译失败。`xrobot setup` 不会检测这种冲突，
> 需要自己保证工程里只选其一。

## 工作方式

- 构造时创建控制线程 `LauncherThread`，栈深 `task_stack_depth`，优先级 `thread_priority`，
  周期 2 ms。每周期读取发射命令 Topic 的最新数据（`CMD::LauncherCMD`，`isfire` 为开火命令），
  用微秒时间戳计算控制周期 `dt`，然后依次执行 `Update()`（电机反馈）、`Solve()`（状态机与热量）
  和 `Control()`（PID 输出）。订阅的 Topic 名由 `param.launcher_cmd_topic_name` 指定（默认
  `launcher_cmd`），须与 `CMD` 的 `launcher_cmd_topic_name` 一致。
- CMD 事件：`CMD_EVENT_LOST_CTRL` 时调用 `LostCtrl()` 复位到安全状态；`CMD_EVENT_START_CTRL` 时
  切到 `SET_FRICMODE_RELAX`。
- 摩擦轮模式：`GetEvent()` 返回的 `LibXR::Event` 上注册了 `LauncherType::LauncherEvent` 的
  `SET_FRICMODE_RELAX`、`SET_FRICMODE_SAFE`、`SET_FRICMODE_READY`，激活对应事件 ID 即切换模式
  （通常由 `EventBinder` 把遥控器事件绑定过来）。RELAX / SAFE 下摩擦轮目标转速为 0，READY 下
  使用配置转速。
- 定义 `DEBUG` 时，向 `ramfs` 添加调试命令文件 `launcher`（见下文）；未定义时 `ramfs` 不使用。

### `HeroLauncher`

- 使用四个摩擦轮：前左 / 前右为二级摩擦轮（目标 `fric2_setpoint_speed`），后左 / 后右为一级
  摩擦轮（目标 `fric1_setpoint_speed`），各用一个速度环（`pid_fric_speed_0..3`）；RELAX / SAFE
  下输出限幅 0.1，READY 且后左摩擦轮转速超过一级目标后放开到 1.0（前）/ 0.8（后）。
- 拨弹为角度环（`pid_trig_angle`）串速度环（`pid_trig_speed`），拨盘角度由电机角度除以
  `trig_gear_ratio` 得到。首次发射时拨盘缓慢转动上弹，检测到出弹后记录零点；之后每次单发拨盘
  转过 `2π / num_trig_tooth`。按下开火为单发，按住超过 200 ms 进入连发模式。
- 出弹判定：后左摩擦轮电流（力矩 / 0.3）超过 0.5 A；100 ms 内未检测到出弹则复位本次发射。
- 热量限制使用代码内写死的值（上限 129、每发 100、冷却 13 /s），未接入裁判系统。
- `motor_fric_back_left` / `motor_fric_back_right` 必须非空，构造时 `ASSERT`。
  `launcher_param.trig_freq_` 不使用。

### `InfantryLauncher`

- 只使用前左 / 前右两个摩擦轮（目标都是 `fric1_setpoint_speed`，PID 为 `pid_fric_speed_0/1`）
  和拨弹电机；`motor_fric_back_left` / `motor_fric_back_right`、`pid_fric_speed_2/3` 和
  `fric2_setpoint_speed` 不使用，后两个电机可填 `nullptr`。
- 状态机：RELAX / STOP / NORMAL / JAMMED。拨弹电机力矩绝对值超过 0.1 判为卡弹，每 20 ms
  交替反转退弹。按下开火为单发（拨盘每发转 2π / 10），按住超过 0.5 s 进入连发，弹频为
  `launcher_param.trig_freq_`（期望弹频，Hz），热量接近上限时线性降低到冷却弹频。拨盘角速度
  限制为 `1.5 · 2π · 弹频 / num_trig_tooth`，因此 **`trig_freq_` 为 0 时拨盘不会转动**，
  步兵必须把它设为正值。
- 出弹判定：两个摩擦轮转速都低于 `fric1_setpoint_speed - 50`；200 ms 内未检测到则复位。
- 热量限制使用代码内写死的值（上限 260、每发 10、冷却参数 20），每 50 ms 更新一次，未接入裁判系统。
- 电机反馈状态为 0 时自动使能，异常时清错；RELAX 下所有电机 `Relax()`，失控时拨弹电机失能。
- 发布的 Topic（均为 `float`）：

| Topic | 含义 |
| --- | --- |
| `shoot_dt` | 最近一次开火命令到出弹的延时，s |
| `shoot_number` | 累计发射数 |
| `trig_freq` | 当前弹频，Hz |

## 依赖

- `QDU-Robomaster/CMD`：`launcher_cmd` 的类型与 CMD 失控 / 恢复事件。
- `QDU-Robomaster/RMMotor`：摩擦轮与拨弹电机（`RMMotor`）。
- `QDU-Robomaster/Motor`：`Motor::Feedback` / `Motor::MotorCmd`。
- `QDU-Robomaster/DebugCore`：定义 `DEBUG` 时的 `launcher` 调试命令。

无外部软件包，仅使用 LibXR。

## 构造接口

```cpp
template <class LauncherType>
class Launcher;

Launcher(RMMotor& motor_fric_front_left, RMMotor& motor_fric_front_right,
         RMMotor* motor_fric_back_left, RMMotor* motor_fric_back_right,
         RMMotor& motor_trig, CMD& cmd, LibXR::RamFS& ramfs,
         const Param& param = {...});
```

模板参数：

- `LauncherType`：`HeroLauncher` 或 `InfantryLauncher`，见上文。

依赖：

- `motor_fric_front_left` / `motor_fric_front_right`：`RMMotor`，前左 / 前右摩擦轮电机。
- `motor_fric_back_left` / `motor_fric_back_right`：`RMMotor*`，后左 / 后右摩擦轮电机；
  `HeroLauncher` 必须提供，`InfantryLauncher` 可填 `nullptr`。
- `motor_trig`：`RMMotor`，拨弹电机。
- `cmd`：`CMD` 实例，提供失控 / 恢复事件。
- `ramfs`：`LibXR::RamFS`，定义 `DEBUG` 时在其中注册 `launcher` 命令文件，否则不使用。

配置（`param`，默认值为英雄参数）：

- `task_stack_depth`：控制线程栈深，默认 4096。
- `pid_trig_angle`：拨弹角度环，默认 `{k 1, p 4000, i 0, d 0, i_limit 0, out_limit 4000, cycle false}`。
- `pid_trig_speed`：拨弹速度环，默认 `{k 1, p 0.0012, i 0.0005, d 0, i_limit 1, out_limit 1, cycle false}`。
- `pid_fric_speed_0..3`：摩擦轮速度环（0 前左、1 前右、2 后左、3 后右），默认均为
  `{k 1, p 0.002, i 0, d 0, i_limit 0, out_limit 1, cycle false}`。
- `launcher_param`：
  - `fric1_setpoint_speed`：一级摩擦轮目标转速，默认 4950；
  - `fric2_setpoint_speed`：二级摩擦轮目标转速（仅英雄），默认 3820；
  - `trig_gear_ratio`：拨弹电机减速比，默认 19.2032；
  - `num_trig_tooth`：拨盘齿数，默认 6；
  - `trig_freq_`：期望弹频，Hz（仅步兵），默认 0。
- `thread_priority`：控制线程优先级，默认 `LibXR::Thread::Priority::HIGH`。
- `launcher_cmd_topic_name`：订阅的发射控制命令 Topic，默认 `"launcher_cmd"`。

PID 参数字段依次为 `k`、`p`、`i`、`d`、`i_limit`、`out_limit`、`cycle`（`LibXR::PID<float>::Param`）。

## 使用

```sh
xrobot module add QDU-Robomaster/Launcher
xrobot setup
xrobot instance add QDU-Robomaster/Launcher
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出；
填好依赖，并在 `template_args` 中选择 `HeroLauncher` 或 `InfantryLauncher`。下面是英雄配置，
电机与 CMD 实例列在 Launcher 之前（指针依赖写成 `'&id'`，须加引号）：

```yaml
modules:
  - {module: QDU-Robomaster/RMMotor, id: motor_fric_front_left, args: [{can_bus: can1}, {param: {model: RMMotor::Model::MOTOR_M3508, reverse: 'false', feedback_id: '0x201'}}]}
  - {module: QDU-Robomaster/RMMotor, id: motor_fric_front_right, args: [{can_bus: can1}, {param: {model: RMMotor::Model::MOTOR_M3508, reverse: 'false', feedback_id: '0x202'}}]}
  - {module: QDU-Robomaster/RMMotor, id: motor_fric_back_left, args: [{can_bus: can1}, {param: {model: RMMotor::Model::MOTOR_M3508, reverse: 'false', feedback_id: '0x203'}}]}
  - {module: QDU-Robomaster/RMMotor, id: motor_fric_back_right, args: [{can_bus: can1}, {param: {model: RMMotor::Model::MOTOR_M3508, reverse: 'false', feedback_id: '0x204'}}]}
  - {module: QDU-Robomaster/RMMotor, id: motor_trig, args: [{can_bus: can2}, {param: {model: RMMotor::Model::MOTOR_M3508, reverse: 'false', feedback_id: '0x201'}}]}
  - {module: QDU-Robomaster/CMD, id: cmd, args: [{mode: 'CMD::Mode::CMD_OP_CTRL'}, {chassis_cmd_topic_name: '"chassis_cmd"'}, {gimbal_cmd_topic_name: '"gimbal_cmd"'}, {launcher_cmd_topic_name: '"launcher_cmd"'}]}
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
          task_stack_depth: '4096'
          pid_trig_angle:
            k: 1.0f
            p: 4000.0f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 4000.0f
            cycle: 'false'
          pid_trig_speed:
            k: 1.0f
            p: 0.0012f
            i: 0.0005f
            d: 0.0f
            i_limit: 1.0f
            out_limit: 1.0f
            cycle: 'false'
          pid_fric_speed_0:
            k: 1.0f
            p: 0.002f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 1.0f
            cycle: 'false'
          pid_fric_speed_1:
            k: 1.0f
            p: 0.002f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 1.0f
            cycle: 'false'
          pid_fric_speed_2:
            k: 1.0f
            p: 0.002f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 1.0f
            cycle: 'false'
          pid_fric_speed_3:
            k: 1.0f
            p: 0.002f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 1.0f
            cycle: 'false'
          launcher_param:
            fric1_setpoint_speed: 4950.0f
            fric2_setpoint_speed: 3820.0f
            trig_gear_ratio: 19.2032f
            num_trig_tooth: '6'
            trig_freq_: 0.0f
          thread_priority: LibXR::Thread::Priority::HIGH
          launcher_cmd_topic_name: '"launcher_cmd"'
```

`motor_*` 为 `QDU-Robomaster/RMMotor` 实例，`cmd` 为 `QDU-Robomaster/CMD` 实例，都必须在
`launcher_0` 之前列出；电机型号与 `feedback_id` 按实车填写。步兵改为 `InfantryLauncher`，
`motor_fric_back_left` / `motor_fric_back_right` 填 `nullptr`，按步兵机构调整 PID 与
`launcher_param`，并把 `trig_freq_` 设为期望弹频。

BSP 侧：

```cpp
XR_REGISTER(can1, LibXR::CAN);
XR_REGISTER(can2, LibXR::CAN);
XR_REGISTER(ramfs, LibXR::RamFS);
```

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/QDU-Robomaster/Launcher`
（在 BSP 中）打印当前的构造函数。

## 调试命令

定义 `DEBUG` 时，在 RamFS 终端中使用 `launcher` 命令查看实时状态：

```sh
launcher once [state|motor|heat|shot|full]
launcher monitor <time_ms> [interval_ms] [state|motor|heat|shot|full]
```

视图：`state`（模式、开火命令、`dt` 等）、`motor`（电机反馈与目标）、`heat`（热量）、
`shot`（发射计数、出弹延时等）、`full`（全部，默认）。
