# 用户手册

面向使用板子/写上位机的人。参数清单见 [parameters.md](parameters.md)，
协议内部细节见 [DEV.md](DEV.md)。

## 1. 连接

- 板子 USB 接电脑（USB CDC 虚拟串口）。Windows 记下 COM 号（如 `COM7`），
  Linux 是 `/dev/ttyACM0`。
- `pip install pyserial`
- 工具：`tools/mc_ep.py`

```bash
python tools/mc_ep.py --port COM7 tree
```

设备会先回送一份 JSON 描述符（名字/ID/类型/权限），工具据此把名字映射到 ID，
所以**固件升级后脚本不用改**。`--baud` 对 CDC 无实际作用。

> 同一个串口只能有一个使用者：`mc_ep.py` 运行时不要再开串口终端。原来的
> ASCII 菜单仍然可用（115200 8N1：`m` 电机、`c` 标定、`s` 设置、`e` 编码器、
> `z` 零点、`d` DFU）。

## 2. 读写参数

```bash
python tools/mc_ep.py --port COM7 tree                 # 参数树
python tools/mc_ep.py --port COM7 descriptor           # 原始 JSON 描述符
python tools/mc_ep.py --port COM7 get  <name>          # 读
python tools/mc_ep.py --port COM7 set  <name> <value>  # 写（打印实际值）
python tools/mc_ep.py --port COM7 call <action>        # 执行 action.*
python tools/mc_ep.py --port COM7 dump                 # 读所有可读参数
python tools/mc_ep.py --port COM7 monitor <name>... --rate 20
```

| 前缀 | 内容 | 权限 |
|---|---|---|
| `config.*` | 电流/位置/速度/电机/编码器/CAN/零点 | `rw` |
| `command.*` | 运行命令：`t_ff`/`p_des`/`v_des`/`kp`/`kd`（不持久化） | `rw` |
| `telemetry.*` | 实时测量 | `r` |
| `timing.*` | ISR 耗时统计 | `r` |
| `system.*` | 版本、配置序号、schema、保存状态 | `r` |
| `action.*` | 保存、置零、切模式、标定 | `w` |

规则：

- **写会被限幅**：超出 `min`/`max` 夹到边界，不报错。`set` 打印实际写入值。
- `NaN`/`Inf` → `EP_ERR_BAD_VALUE`（3）。
- 写只读端点 → `EP_ERR_READ_ONLY`（4）。
- **读动作端点 → `EP_ERR_WRITE_ONLY`（5）**。这是安全设计：一次读不能让电机动作。
- 状态前置条件不满足（如门极使能时改 `config.ppairs`）→ `EP_ERR_STATE`（6）。
- `config.p_min`/`v_min` 可单独写（通常由 `p_max`/`v_max` 联动），用于备份恢复。

状态码：0 OK、1 未知 ID、2 长度错、3 数值非法、4 只读、5 只写、6 状态不允许、
7 描述符版本不匹配（工具会自动重新下载描述符）。

### 在 Python 脚本里用

```python
import sys
sys.path.insert(0, "tools")
from mc_ep import Device

dev = Device("COM7", 115200, 1.0)
dev.fetch_descriptor()
print(dev.read("config.gr"))
dev.write("config.gr", 7.75)
dev.call("action.save")
print(dev.read("system.save_status"))
dev.close()
```

## 3. 保存

改参数只改 RAM，**断电会丢**：

```bash
python tools/mc_ep.py --port COM7 call action.save
python tools/mc_ep.py --port COM7 get system.save_status
# 0 还没执行 / 1 成功 / 2 因门极使能跳过 / 3 写失败
python tools/mc_ep.py --port COM7 get system.prefs_sequence   # 成功时 +1
```

`call action.save` 返回 OK 只表示**已受理**，真正的擦写在主循环里做；用
`save_status` 或 `prefs_sequence` 确认。

保存过程中掉电：上电后仍是上一次的完整配置（双页 + 序号）。

## 4. 动作与模式

```bash
python tools/mc_ep.py --port COM7 call action.idle_mode    # 空闲/停机：关门极、清锁存故障
python tools/mc_ep.py --port COM7 call action.motor_mode    # 进入电机模式
python tools/mc_ep.py --port COM7 call action.calibrate     # 标定（相序/编码器/R/L/磁链）
python tools/mc_ep.py --port COM7 call action.zero          # 当前位置设为零点并保存
python tools/mc_ep.py --port COM7 call action.clear_fault   # 等价于 action.idle_mode
python tools/mc_ep.py --port COM7 call action.reboot        # 重启（应答发出后 ~0.5 s 复位）
python tools/mc_ep.py --port COM7 call action.reset_config  # 恢复出厂默认值并保存
```

`action.idle_mode` 会停机并清故障，是正常的"急停"用法。

### 纯力矩加载测试

`command.*` 是运行命令，只在 MOTOR_MODE 里被控制环使用，**不持久化**：

| 名称 | 含义 |
|---|---|
| `command.t_ff` | 前馈力矩（输出轴 N·m），固件限幅到 ±`I_MAX*KT*GR` |
| `command.p_des` / `command.v_des` | 位置/速度设定值（输出轴） |
| `command.kp` / `command.kd` | 位置/速度增益 |

纯力矩：`kp = kd = 0`，只改 `command.t_ff`。

**命令租约**：`command.*` 的每次写入都会把 `controller.timeout` 清零，和 CAN 帧一样。
`config.can_timeout`（毫秒，默认 100）内没有新的写入，控制 ISR 就会把所有命令清零
——这是**保护**，不是麻烦：主机掉线/脚本崩溃时电机自动卸力。所以不要为了"设一次
就一直保持"去关它；要么按超时窗口持续刷新（≥3 次/窗口），要么把
`config.can_timeout` 调大到你想接受的失联时间（例如 500 ms）。

进入 MOTOR_MODE 时固件会先 `zero_commands()`，所以**门极一打开命令就是 0**：
在空闲态写 `command.*` 不会在使能瞬间生效，必须在电机模式下重新写。

⚠ 纯力矩没有速度环：**空载会一直加速**。必须接负载/测功机，并随时准备
`action.idle_mode` 或断电。

```bash
python tools/torque_test.py --port COM7 --torques 0,2,4,6 --dwell 5
```

脚本会：按 `can_timeout` 自动选刷新率 → 进 MOTOR_MODE → 逐档施加 `t_ff` 并在每轮
重写（刷新租约）→ 打印 `i_q`/`torque`/`v_bus` → 结束或 Ctrl+C 时卸力并回菜单。
故障位非 0 立即停机；链路断开时脚本会明确提示"力矩可能仍在输出"。
单档手动操作就是：

```bash
python tools/mc_ep.py --port COM7 call action.idle_mode
python tools/mc_ep.py --port COM7 call action.motor_mode
python tools/mc_ep.py --port COM7 get telemetry.gate_drive     # 应为 true
python tools/mc_ep.py --port COM7 set command.kp 0
python tools/mc_ep.py --port COM7 set command.kd 0
python tools/mc_ep.py --port COM7 set command.t_ff 3
python tools/mc_ep.py --port COM7 get telemetry.torque
python tools/mc_ep.py --port COM7 set command.t_ff 0
python tools/mc_ep.py --port COM7 call action.idle_mode
```

（手动分批敲命令时，两条之间可能超过 `can_timeout`，固件会先把命令清零——这是
正常的，重新写一次即可；要连续保持就用脚本。）

**`action.reset_config`（恢复出厂默认值）**：会连标定一起清掉——`ppairs`、零点、
编码器 LUT 全部回默认，必须**重新标定**；`CAN_ID / CAN_PROT / CAN_BAUD` 也会回默认
（`can_prot = 0` 即旧 11-bit 协议），所以复位后要**重新配 CAN**，否则主站可能再也
连不上。为避免"改了 CAN 设置却不重启"造成的失联和混合状态，**保存成功后固件会自动
重启**（约 0.5 s，门极先关闭）。和 `action.save` 一样，门极使能时会被直接拒绝
（`mc_ep.py` 报 `status 6` = `EP_ERR_STATE`，配置一个字都不动）；受理之后的结果看
`system.save_status`（1 成功 / 3 写失败，写失败时**不会**重启，默认值只在 RAM 里）。
运行中的编码器副本仍用
旧标定，直到重启才换成默认值——这是故意的（拿全零 LUT 去换相不安全）。

**`action.reboot`**：先回菜单（关门极）再在 0.5 s 后复位，配置按 flash 里已保存的值
重新加载。没保存过的改动会丢——先 `action.save` 再 `action.reboot`。

### 速度环与位置环（CSV / CSP）

固件自带速度环和位置环（CYBERBEAST 的 `0x02/0x82` CSV、`0x01/0x81` CSP 就是它们），
用 USB 也能直接用 `command.mode` 试：

| `command.mode` | 控制律 | 用的命令量 |
|---|---|---|
| 0（默认） | 阻抗/MIT | `t_ff` + `kp`·`p_des` + `kd`·`v_des` |
| 1 | 速度环（CSV） | `v_des` |
| 2 | 位置环（CSP） | `p_des` |

先配增益（都是**输出轴**量，控制输出是电机端安培）：

| 参数 | 单位 | 出厂值 | 说明 |
|---|---|---|---|
| `config.pos_gain` | (rad/s)/rad | 15 | 位置误差 × 它 = 速度指令 |
| `config.vel_gain` | A/(rad/s) | 2.469 | 速度误差 × 它 = 电流指令 |
| `config.vel_integrator_gain` | A/(rad/s)/s | 44.44 | 消掉重力/摩擦下的稳态误差 |
| `config.pos_ff_gain` | — | 1.0 | CSP 帧间速度前馈；1 = 全前馈，0 = 关 |
| `config.accel_limit` | rad/s² | 50 | 速度设定值斜率限制（输出轴）；0 = 关。**CSP 想快就先放开它**（30~100），比加 `pos_gain` 安全 |
| `config.pos_error_fault` | rad | 0 | 跟随误差故障阈值，0 = 关 |
| `config.pll_bandwidth` | rad/s | 2000 | 编码器速度估计带宽（约 318 Hz）；低速抖就降到 1000，想要更高带宽就升到 3000 |
| `config.inertia` | kg·m² | 0.0006 | 输出端惯量，由 `action.measure_inertia` 测得（只写 RAM）。**默认值只是本台架的实测值**，换电机/换关节必须重测 |
| `config.omega_c` | rad/s | 450 | `action.tune_gains` 的目标穿越频率（越大越激进，300–600 起试） |
| `config.tune_margin` | × | 2.2 | `action.tune_gains` 对速度环的放大系数：摩擦/齿隙大的关节用 2~3；1 = 纯公式（只适合纯惯性负载） |

```bash
python tools/mc_ep.py --port COM7 set config.pos_gain 20
python tools/mc_ep.py --port COM7 call action.motor_mode
python tools/mc_ep.py --port COM7 set command.mode 1
python tools/mc_ep.py --port COM7 set command.v_des 1.0
python tools/mc_ep.py --port COM7 monitor telemetry.vel_err telemetry.vel_integral
python tools/mc_ep.py --port COM7 set command.mode 0      # 回阻抗/MIT
python tools/mc_ep.py --port COM7 call action.idle_mode
```

#### 不用脚本，手工发 CSV / CSP

`tools/csv_csp_test.py` 只是把下面几条封装起来，手动测试完全可行，但有 4 个坑：

1. **命令租约**：每次写 `command.*` 都把租约重置为 `config.can_timeout`（默认
   **100 ms**）。超时后 `zero_commands()` 清掉设定值**并把 `command.mode` 复位成 0
   （MIT）** —— 于是关节卸力、你再写 `v_des` 也没反应（因为已经不是 CSV 模式了）。
   手动测试先放大租约，测完改回去：
   ```bash
   python tools/mc_ep.py --port COM6 set config.can_timeout 60000    # 60 s
   ```
2. **CSP 要先把 `p_des` 写成当前位置，再切模式**。先 `set command.mode 2` 的话，
   `p_des` 还是 0（或上一次的值），位置环会立刻按 `pos_gain·err` 冲过去。
3. **`p_des` 受 `config.p_max`（默认 ±12.5 rad）钳位**，超范围不是拒绝而是**钳到边界**，
   目标可能落在关节另一侧。`telemetry.position` 是绝对多圈位置（跑过之后可能远大于
   12.5）。要么临时放大 `config.p_max`，要么 `action.zero` **+ `action.reboot`**
   —— 零点只在重启后才生效（`M_ZERO` 在启动时重新对齐多圈计数）。
4. **`action.motor_mode` 只能在菜单态调用**，否则返回状态 6；`action.idle_mode`
   回菜单并关栅。`monitor` 会独占串口，Ctrl+C 退出后才能发下一条命令。

**CSV**（速度环）：

```bash
python tools/mc_ep.py --port COM6 set config.can_timeout 60000
python tools/mc_ep.py --port COM6 call action.motor_mode          # 开门极（需菜单态）
python tools/mc_ep.py --port COM6 set command.mode 1              # 0=MIT 1=CSV 2=CSP
python tools/mc_ep.py --port COM6 set command.v_des 1.0           # 输出轴 rad/s
python tools/mc_ep.py --port COM6 monitor telemetry.velocity_pll telemetry.vel_err telemetry.i_q --rate 5
python tools/mc_ep.py --port COM6 set command.v_des -1.0          # 反向
python tools/mc_ep.py --port COM6 set command.v_des 0
python tools/mc_ep.py --port COM6 call action.idle_mode
python tools/mc_ep.py --port COM6 set config.can_timeout 100
```

判读：`velocity_pll` 应跟住 `v_des`；低速时 ±0.2~0.4 rad/s 的抖动是干摩擦 + 齿隙，
正常。`vel_err` 长期很大 ⇒ `vel_gain` 不够；振荡发散 ⇒ 太大。

**CSP**（位置环）：

```bash
python tools/mc_ep.py --port COM6 set config.can_timeout 60000
python tools/mc_ep.py --port COM6 call action.motor_mode
python tools/mc_ep.py --port COM6 get telemetry.position          # 记下 P0（rad）
python tools/mc_ep.py --port COM6 set command.p_des <P0+0.3>      # ← 先给目标
python tools/mc_ep.py --port COM6 set command.mode 2              # ← 再切模式
python tools/mc_ep.py --port COM6 monitor telemetry.position telemetry.pos_err --rate 5
python tools/mc_ep.py --port COM6 call action.idle_mode
python tools/mc_ep.py --port COM6 set config.can_timeout 100
```

判读：`pos_err` 收敛到 0.0x mrad，`position` 最后精确等于 `p_des`
（台架实测 0.3 rad 阶跃最终 `pos_err ≈ 3e-5 rad`）。

- **增益要按关节整定**：出厂值（`vel_gain 2.469`、`vel_integrator_gain 44.44`、
  `pos_gain 15`、`accel_limit 50`）是本台架（GR 7.75、`KT 0.031`、`J 0.0006`）整定的
  结果，**换电机/换关节必须重做**；裸关节在 `vel_gain ≥ 4` 会失稳。方法见
  [DEV.md](DEV.md) 的“台架整定步骤”，工具是 `tools/csv_csp_test.py`。
- **两条路可选**：① 一条命令 `action.tune_gains`（先做 `action.measure_inertia`），
  配方 = 惯量 + `config.omega_c`(450) + `config.tune_margin`；② 手工扫
  `csv_csp_test.py sweep`。台架（GR 7.75、`KT 0.031`、`J 0.0006`）实测工作点：

  | 模式 | 参数 | 值 | 效果 |
  |---|---|---|---|
  | MIT | `command.kp` / `kd` | **10 / 0.155**（`kd = 2√(kp·J)`） | 0.2 rad 阶跃 43 ms 上升、0.1 mrad 稳态、0.96 A 峰值 |
  | CSV | `vel_gain` / `vel_integrator_gain` | **2.469 / 44.44**（= `tune_margin 2.2` 的配方值；手工最优 2.5/30 实测等效） | 1~65 rad/s 误差 ≤1%（40 rad/s 峰值 2.6 A） |
  | CSP | `pos_gain` / `accel_limit` | **15 / 50** | 0.2 rad：0.2%、0.26 s；1 rad：4.2%、0.34 s |

  `accel_limit` 从 10 提到 50 是 CSP 变快的关键：位置环是纯 P，它给出的速度设定值
  被斜率限制拖慢时会出现"追上了速度还没降下来"的过冲（1 rad 阶跃：`accel_limit 10`
  过冲 49%，50 时 4%）。**想要更快的 CSP 请先放开 `accel_limit`，不要只加 `pos_gain`。**
- ⚠ **`command.p_des` 会被 `config.p_max` 钳位，不是拒绝**：反复跑速度测试后
  绝对位置会漂移，一旦漂到 `p_max`（默认 ±12.5 rad）之外，CSP 的每个目标都被钳在
  边界上，看起来像“位置环完全不动”。处理：放大 `config.p_max`，或 `action.zero`
  **+`action.reboot`**（零点只在重启后生效）。
- 每条命令都刷新“命令租约”（同 `command.*`）：`config.can_timeout` 内没有新命令，
  固件就把命令清零 → **卸力**，不是保持位置。周期同步的主站不会碰到这个问题。
- 进电机模式、超时、故障都会把 `command.mode` 复位成 0（阻抗）。
- 速度限幅：CSV 用 `config.v_max`，CSP 用帧里的 `vel_limit`（RPM，输出端）。
  帧里给 0 视为“未指定”，按 `config.v_max` 处理。
- **位置跟随误差故障**默认关闭。打开后（`config.pos_error_fault > 0`）只要位置环
  没被速度限幅饱和、误差又连续超过阈值 100 ms，就锁存故障、关栅停机，需要
  `action.idle_mode` 才能清除。阈值必须**小于 `config.v_max / config.pos_gain`**，
  否则永远触发不了。
- 环率：速度环 = 40 kHz / `config.vel_loop_prescaler`（默认 5 → 8 kHz），
  位置环 = 速度环 / `config.pos_loop_prescaler`（默认 2 → 4 kHz）。设成 1/1 就是
  两个环都跑 40 kHz（电流环永远是 40 kHz，不受影响）。
- **惯量标定**：`action.measure_inertia` **一条指令完成**——自动进电机模式、打两个
  电流台阶测出输出端惯量、再自动回菜单（约 1.5 s，关节净行程 ~0.1 rad）。结果写入
  `config.inertia`，**不自动保存**；进度和结果看 `system.inertia_status`（2 = 成功）。
  有了它就能用公式算推荐增益，`telemetry.pos_error_fault_suggest` 也会给出跟随误差
  故障的推荐阈值。
  ```bash
  python tools/mc_ep.py --port COM6 call action.measure_inertia
  python tools/mc_ep.py --port COM6 get system.inertia_status
  python tools/mc_ep.py --port COM6 get config.inertia
  python tools/mc_ep.py --port COM6 call action.save      # 确认可用后再保存
  ```
  然后 `vel_gain = inertia·ω_c/(KT·GR)`、`pos_gain ≈ ω_c/5`（`ω_c` 300–600 rad/s 起步，
  见 [DEV.md](DEV.md) 的“惯量标定与增益推荐值”）。
- **自动算增益**：`action.tune_gains` 用上面的惯量一步算出三个外环增益并**直接写进
  RAM**（不自动保存）：
  ```bash
  python tools/mc_ep.py --port COM6 set config.omega_c 450       # 目标穿越频率
  python tools/mc_ep.py --port COM6 set config.tune_margin 2.6   # 摩擦大的关节放大速度环；1 = 纯公式
  python tools/mc_ep.py --port COM6 call action.tune_gains
  python tools/mc_ep.py --port COM6 get config.vel_gain          # 已经写进去了
  python tools/csv_csp_test.py --port COM6 csv --vel 1.0         # 验证
  python tools/csv_csp_test.py --port COM6 csp --step 0.2
  python tools/mc_ep.py --port COM6 call action.save             # 满意后再保存
  ```
  写入关系：`vel_gain = tune_margin·J·ω_c/(KT·GR)`、
  `vel_integrator_gain = vel_gain·clamp(ω_c/25, 5, 30)`、`pos_gain = ω_c/20`。
  两个分母（25 / 20）是按台架实测定的：积分开太快会 stick-slip，`pos_gain` 大了
  齿隙会让位置过冲。要求菜单态、`config.inertia > 0`（先做惯量标定）；门极打开时
  会返回状态 6。
- **`action.measure_inertia` 的“没反应”不一定是失败**：整个过程只有 ~0.5–1.5 s、
  电流 0.15–0.25 A、输出轴净行程 ~0.1 rad（5.7°），末了自动回菜单。用
  `get system.inertia_status` 判断（1 运行中 / 2 成功 / 3 关节本来在动 /
  4 加电流也不动 / 5 台阶太短 / 6 激励不足）；`call` 只表示"请求已受理"。

## 5. 标定

第一次使用、换电机/编码器、整片擦除后要做一次。**OTA 升级不需要重新标定。**

前置：电机轴能自由转动；动力电已上；`config.i_cal` 合适（默认 5 A，小电机
可降到 1–2 A，R/L/磁链测量时会自动限制在 `config.i_max` 以内）；门极关闭（菜单）。

```bash
python tools/mc_ep.py --port COM7 call action.idle_mode
python tools/mc_ep.py --port COM7 set config.i_cal 2.0
python tools/mc_ep.py --port COM7 call action.calibrate
```

标定分四个阶段，全部自动完成（14 极对时约 30 s，串口会打印每一步的结果）：

1. **相序/极对数**：对齐后正转一个电周期，**保持末相位等转子停稳**（否则转子滞后
   会让行程偏短，14 极对经常被算成 15），再按 `2π / 机械行程` 求极对数；
2. **编码器**：正反转各一圈采样，得到电角度零位 `E_ZERO` 和 128 点 LUT；
3. **R / L / Ld / Lq**：先用两段直流（`i_cal/2` 和 `i_cal`）外推相电阻，
   再用 ±2 V 电压翻转测电感；然后在 alpha、beta 两个方向分别锁住转子，
   测出 `Ld` 与 `Lq`（感差 `Ld-Lq`）；
4. **磁链 / Kt**：开环升速到 60 rad/s（电角度）后由稳态 q 轴电压方程算出
   磁链 `λ`，并令 `Kt = 1.5 × 极对数 × λ`。

完成后自动回菜单、应用结果并保存（LUT 写入隐藏记录 `internal.encoder_lut`）。核对：

```bash
python tools/mc_ep.py --port COM7 get system.save_status      # 1
python tools/mc_ep.py --port COM7 get system.prefs_sequence   # 比标定前 +1
python tools/mc_ep.py --port COM7 get config.r_phase          # 欧姆
python tools/mc_ep.py --port COM7 get config.l_d              # 亨利
python tools/mc_ep.py --port COM7 get config.l_q
python tools/mc_ep.py --port COM7 get config.kt               # N-m/A
python tools/mc_ep.py --port COM7 get telemetry.ld_lq_diff    # Ld-Lq
```

串口会在 R 阶段两步之间打印 `R step 1: <V> @ <A>`。相电阻大时第二步电压会被
`CAL_LOCK_V_MAX` 钳位，固件用**实测电流**计算 R，不会因此失败；只有两步电流差
太小（开路、母线电压太低）才失败。失败（电机卡住、动力电不足、`i_cal` 不合适、
电流跟不上）会自动回菜单，**不写 flash**；可先降低 `config.i_cal` 再试，随时
`call action.idle_mode` 中止。

标定结果里 `E_ZERO`/`PPAIRS`/零点/`r_phase`/`l_d`/`l_q`/`kt` 都在 `config.*`，
128 点 LUT 在隐藏记录里，**都持久化、都随 OTA 保留**；LUT 不可通过 endpoint 备份。
`telemetry.flux_linkage` 是 `kt` 与极对数换算出来的只读值（λ = Kt/(1.5·极对数)），
因此重启后仍能读到标定得到的磁链。

## 6. 备份与恢复

只备份"持久化 = 是"的参数（见 [parameters.md](parameters.md)）。LUT 是隐藏记录，
不在备份里。

```python
# backup
dev = Device("COM7", 115200, 1.0); dev.fetch_descriptor()
data = {e["name"]: dev.read(e["name"]) for e in dev.descriptor
        if e.get("persist") and e.get("access") in ("r", "rw")}
json.dump(data, open("backup.json", "w"), indent=2)

# restore（门极关闭；按 ID 升序写，p_max 先于 p_min）
by = {e["name"]: e for e in dev.descriptor}
for name in sorted(data, key=lambda n: by[n]["id"]):
    if by[name].get("access") in ("w", "rw") and by[name].get("persist"):
        dev.write(name, data[name])
dev.call("action.save")
```

恢复脚本会跳过当前固件里不存在的参数。换主板/整片擦除后必须重新标定。

## 7. OTA 升级

**只要 OTA 只擦 app 区（`0x08000000..0x0807BFFF`），升级后配置、零点、标定 LUT
全部保留。** 整片擦除会全丢——这是固件救不了的。

升级前（建议）：

```bash
python tools/mc_ep.py --port COM7 get system.prefs_sequence
python tools/mc_ep.py --port COM7 get config.gr
python tools/mc_ep.py --port COM7 get config.m_zero
```

升级后核对：

```bash
python tools/mc_ep.py --port COM7 get system.version        # 新版本
python tools/mc_ep.py --port COM7 get system.prefs_loaded   # true
python tools/mc_ep.py --port COM7 get system.config_schema  # 与发布说明一致
python tools/mc_ep.py --port COM7 get config.gr             # 与基线一致
```

LUT 的验证：把电机停到升级前同一机械角度，比较 `telemetry.encoder_raw`。

| 现象 | 原因 | 处理 |
|---|---|---|
| `prefs_loaded = false`，参数全默认 | 配置页被擦了 | 查 OTA 擦除范围；从备份恢复；重新标定 |
| 个别参数变默认 | 该 ID 的类型/长度变了（不应发生） | 查发布说明；备份恢复 |
| `system.version` 没变 | 没烧进去 | 重试 |
| 工具报状态 7 | 描述符 CRC 变了 | 用新版 `mc_ep.py`，自动重下描述符 |

**第一次刷入 TLV 配置格式的固件**（本仓库之前的版本用的是裸数组）会忽略旧块，
参数回默认值，需要重新设置 + 标定一次；之后每次升级都会保留。

回退到旧固件：旧固件不认识新格式，参数回默认值；在旧固件里保存会覆盖配置页，
再升回来也回不到新格式的值。回退前先备份。

## 8. 故障排查

| 现象 | 可能原因 | 处理 |
|---|---|---|
| `无法打开串口` | 端口错；被别的程序占用 | 关掉串口终端/另一个工具；确认 COM 号 |
| 一直超时 | 板子没运行；USB 没枚举成 CDC | 重新上电；看设备管理器 |
| `没有返回描述符版本信息` | 端口错或收到的是菜单输出 | 用 `descriptor` 看原始返回；只留一个使用者 |
| 状态 1 未知 ID | 名字错；上位机描述符过期 | `tree`；升级工具 |
| 状态 4/5/6 | 写只读、读动作、状态不允许 | 看参数手册；`call` 代替 `get`；先回菜单 |
| 状态 7 | 描述符 CRC 变了 | 新版工具会自动重下 |
| `save_status = 2` | 门极使能 | 回菜单再存 |
| `save_status = 3` | flash 写失败 | 看串口打印；`make check` 查容量；重试 |
| `set` 的值和给的不一样 | 被限幅 | 正常，`set` 打印的是实际值 |
| 串口终端乱码 | 两个程序抢一个串口 | 关掉多余连接，重新上电 |
| CSP 命令发了但关节完全不动 | 绝对位置已漂到 `config.p_max` 外，`command.p_des` 被**钳位**在边界（不报错） | 放大 `config.p_max`，或 `action.zero` + `action.reboot` 重新对零 |
| CSP 大阶跃过冲很大 | `config.accel_limit` 太小（速度设定值被斜率限制拖慢） | 先放开 `accel_limit`（30~100），再考虑动 `pos_gain` |
| 高速跑一段时间后突然卸力、`telemetry.fault=2` | `FAULT_OC`：相电流超过 1.5×`config.i_max`（猛加速或高压纹波） | `action.idle_mode` 清故障；降 `accel_limit` 或确认 `i_max` 与电机匹配 |
| 烧了新固件但行为没变（新参数不存在、遥测自相矛盾） | **其实没烧进去**：高 SWD 时钟下读回不稳，J-Link/Keil 的 verify 会失败但编程只写了一部分，设备继续跑旧镜像 | 用 `python tools/flash_jlink.py`（250 kHz + 分块写 + 回读校验）重烧；烧完核对 `system.prefs_loaded`、`config.kt` |
| 整片擦除后电流环/惯量标定结果离谱 | 标定数据被擦掉，跑的是默认值 | 重新 `action.calibrate`，再 `action.save`；然后重测惯量 |

## 9. FAQ

**升级后要重新标定吗？** 不用，除非换了硬件或整片擦除。

**参数存在哪？** `0x0807C000`/`0x0807E000` 两页，双份 + CRC32 + 序号；
写了要 `action.save` 才落盘。

**为什么 `get action.save` 报错？** 动作端点只能执行不能读——否则一次误读就能
让电机停机。

**为什么 `p_min` 可以单独写？** 正常由 `p_max` 联动；单独写是为了备份恢复，
限幅仍保证它 ≤ 0。

**LUT 能导出吗？** 不能，隐藏记录。它随配置页保留；换板/整片擦除要重新标定。

**能加多少参数？** 配置记录 256 条、总载荷 4 KB、描述符 64 KB。当前用了 30 条
配置 + 1 条 LUT（512 B）。超限会在 `make endpoints` 时直接报错。

**保存会磨损 flash 吗？** 每次擦一页（8 KB），只在显式 `action.save` 时发生。

**支持 CAN 吗？** 两套协议，用 `config.can_prot` 选择：`0` = 原有 11-bit 协议
（默认，500 kbit/s），`1` = CYBERBEAST。`config.can_baud` 决定速率与物理层：
≤ 1 Mbit/s 为 Classic CAN，> 1 Mbit/s 为 CAN FD（例：`can_baud = 5000000` 就是
CYBERBEAST 默认的 1M 仲裁 / 5M 数据）。两者都是**下次上电生效**，
`system.can_prot_active` / `system.can_fd` / `system.can_baud_active` 可核对
当前实际运行值。详见 [DEV.md](DEV.md#can协议选择与-cyberbeast)。
