# YoloDrop SITL 仿真测试

## 一、概述

本目录包含用于 SITL (Software-In-The-Loop) 仿真测试 YoloDrop 模块的工具。

- `yolo_sim.py` — YOLO 二进制协议模拟器，模拟 AI 模块 (yolo_to_fmu) 向飞控发送检测数据

### 网络拓扑

```
┌──────────────────────────────────────┐     ┌──────────────────────────────┐
│  Linux 主机 (运行 SITL)               │     │  Windows 电脑 192.168.3.139  │
│                                      │     │                              │
│  sim_vehicle.py                      │     │  Mission Planner             │
│    ├─ MAVProxy ──UDP转发──────────────────→ │    UDP 连接 :14550           │
│    └─ SERIAL5  :5770 (YOLO 模拟器)   │     │                              │
│                                      │     │  观察调试消息 / 设参 / 操控   │
│  yolo_sim.py --port 5770             │     │                              │
│    (本机 127.0.0.1 连接)              │     │                              │
└──────────────────────────────────────┘     └──────────────────────────────┘
```

## 二、启动 SITL

`sim_vehicle.py` 启动时会自动编译 SITL 固件，无需提前手动编译。

```bash
cd ~/apm/LJG/HXKY-APM-4.5.4
python3 ./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map \
    -A '--serial5=tcp:5770'
```

参数说明：
- `-A '--serial5=tcp:5770'` — 将 SERIAL5 映射为 TCP 端口 5770，供 YOLO 模拟器连接
- 如使用其他串口编号，修改 `--serial5` 为对应编号，并同步修改后续参数中的 `SERIAL5_PROTOCOL`
- 加 `-N` 参数可跳过编译，直接运行上次编译的固件（代码未修改时节省时间）

## 三、转发 MAVLink 到局域网 Mission Planner

SITL 启动后，在 MAVProxy 控制台中输入以下命令，将数据通过 UDP 转发给局域网中的 Windows 电脑：

```
output add 192.168.3.139:14550
```

> 参考：[怒飞垂云 - 建立APM飞控软件仿真环境](http://www.nufeichuiyun.com/?p=256)

## 四、Mission Planner 连接

1. 在 192.168.3.139 的 Windows 电脑上打开 Mission Planner
2. 右上角连接方式选择 **UDP**
3. 点击 **CONNECT**
4. 弹出端口窗口，填 **14550**，点击 **OK**

连接成功后即可在 Mission Planner 中：
- 查看飞行状态和地图
- 在 Messages 面板观察 `YDROP:` 调试消息
- 上传航点任务
- 设置参数
- 手动切换飞行模式

> **提示**：如果连接失败，检查：
> 1. Linux 主机防火墙是否放行了 UDP 端口：`sudo ufw allow 14550/udp`
> 2. MAVProxy 中确认输出已添加：输入 `output` 命令查看当前所有输出
> 3. 两台电脑能否互相 ping 通

## 五、配置参数

可在 Mission Planner 的 Config → Full Parameter List 中设置，也可在 MAVProxy 控制台中输入：

```
param set SERIAL5_PROTOCOL 57

param set YDROP_EN 2
param set YDROP_CONF 0.50
param set YDROP_SPD 300
param set YDROP_AREA 0.10
param set YDROP_PWMO 1900
param set YDROP_PWMC 1100

param set SERVO9_FUNCTION 28
param set SERVO9_MIN 1100
param set SERVO9_MAX 1900
param set SERVO9_TRIM 1100
```

设完后重启使串口配置生效：

```
reboot
```

> Mission Planner 中操作：Config → Full Parameter List → 修改参数 → Write Params → Actions → Reboot
>
> **注意**：reboot 后 MAVProxy 的 UDP 转发会丢失，需要重新执行 `output add 192.168.3.139:14550`，Mission Planner 重新连接。

## 六、起飞

### 方式 A：通过 MAVProxy 控制台

```
arm throttle
mode guided
takeoff 20
```

等待到达高度后：

```
mode auto
```

### 方式 B：通过 Mission Planner

1. Flight Plan 页面上传搜索航点
2. Actions → Arm
3. 模式切换到 GUIDED → 右键地图 → Takeoff (20m)
4. 到达高度后切换模式到 AUTO

快速测试时也可先 LOITER 悬停后再切 AUTO。

## 七、运行 YOLO 模拟器

在 SITL 主机（Linux）上另开一个终端：

```bash
cd ~/apm/LJG/HXKY-APM-4.5.4/Tools/sitl_tests
python3 yolo_sim.py --port 5770 --mode drop
```

### 模拟模式

| 模式 | 命令 | 说明 |
|------|------|------|
| `idle` | `--mode idle` | 持续发送空帧，验证串口通信和帧率，不触发抛投 |
| `static` | `--mode static` | 固定偏移目标，验证 GUIDED 速度方向是否正确 |
| `centering` | `--mode centering` | 目标从偏移逐渐移向中心，验证接近逻辑 |
| `drop` | `--mode drop` | **完整流程**：搜索→接近→下降→悬停→抛投→完成 |
| `lost` | `--mode lost` | 有目标→丢失→恢复，验证超时处理 |

### 完整参数

```
python3 yolo_sim.py --host 127.0.0.1 --port 5770 --hz 10 --mode drop
```

- `--host` — SITL 地址，默认 `127.0.0.1`（模拟器与 SITL 在同一台 Linux 主机上运行）
- `--port` — TCP 端口，默认 `5770`
- `--hz` — 发送频率，默认 `10` Hz
- `--mode` — 模拟模式，默认 `drop`

## 八、观察调试输出

在 Mission Planner 的 Messages 面板（或 MAVProxy 控制台）中关注 `YDROP:` 前缀消息：

```
YDROP: st=IDLE cfm=0 done=0 fps=10.0          ← 串口解析正常
YDROP: id=0 cf=0.85 ox=0.35 oy=-0.25 a=0.002  ← 检测到目标
YoloDrop: target confirmed, GUIDED              ← 确认，切 GUIDED
YDROP: st=APPR vn=75 ve=105 vd=0               ← 水平接近
YoloDrop: centered, descending                  ← 居中，开始下降
YDROP: st=DESC vn=3 ve=-2 vd=38                ← 下降中
YoloDrop: area OK, hovering                     ← 高度到位，悬停确认
YoloDrop: releasing                             ← 舵机打开
YoloDrop: drop complete, LOITER                 ← 完成，切 LOITER
```

## 九、测试检查清单

| 序号 | 检查项 | 预期结果 | 使用模式 |
|------|--------|---------|---------|
| 1 | Mission Planner 连接 | UDP 连接 14550 成功 | — |
| 2 | 串口帧率 | `fps` 与模拟器频率一致 (默认 10Hz) | `idle` |
| 3 | 空帧不触发 | `cfm` 保持 0，状态始终 IDLE | `idle` |
| 4 | 速度方向 | 偏移正 → 对应方向速度正 | `static` |
| 5 | 全流程 | IDLE→APPR→DESC→HOVR→RELS→DONE | `drop` |
| 6 | 模式切换 | AUTO→GUIDED→LOITER | `drop` |
| 7 | 目标丢失 | 出现 `target lost, LOITER` 并回到 IDLE | `lost` |
| 8 | 手动接管 | GUIDED 中通过 MP 切 LOITER，模块回到 IDLE | `drop` + 手动 |
| 9 | 单次抛投 | DONE 后再次出现目标不再触发 | `drop` 运行两次 |

## 十、推荐测试流程

```
1. 启动 SITL (自动编译)
2. MAVProxy 中: output add 192.168.3.139:14550
3. Mission Planner UDP 连接 14550
4. 配置参数 + reboot
5. reboot 后重新: output add 192.168.3.139:14550, MP 重新连接
6. 运行 yolo_sim.py --mode idle           → 确认 fps>0, 通信正常
7. Ctrl+C 停止模拟器
8. GUIDED 起飞 20m → 切 AUTO
9. 运行 yolo_sim.py --mode drop           → 观察完整流程
10. 观察 MP 地图上飞行轨迹 + Messages 面板消息
11. 运行 yolo_sim.py --mode lost           → 测试目标丢失处理
12. 在 MP 中手动切模式                      → 测试手动接管
```

## 十一、故障排查

| 问题 | 排查方向 |
|------|---------|
| Mission Planner 连接失败 | ① 确认 MAVProxy 已执行 `output add 192.168.3.139:14550` ② 防火墙 `sudo ufw allow 14550/udp` ③ 两机互 ping |
| reboot 后 MP 断连 | 正常现象，需重新执行 `output add` 并在 MP 重新连接 |
| 模拟器连接失败 | 确认 SITL 已启动，`--serial5=tcp:5770` 参数正确 |
| `fps=0` | 确认 `SERIAL5_PROTOCOL=57`，reboot 后生效 |
| `YoloDrop: YOLO serial not found` | 确认 `YDROP_EN≥1` 且 `SERIAL5_PROTOCOL=57`，reboot |
| 检测到目标但不触发 | 确认当前为 AUTO 模式，且 `YDROP_EN≥1` |
| 速度方向反 | 检查坐标映射（见设计文档第五节） |
| Mission Planner 断连 | 网络问题，重新 CONNECT 即可，SITL 不受影响 |
