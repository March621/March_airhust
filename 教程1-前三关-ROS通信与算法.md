# 教程一：前三关从零到避障（ROS 通信 + 算法 + 调试）

> 这份教程按「你在终端里实际会遇到什么」组织，不是按课本顺序。
> 每一节都包含：概念 → 你会在哪里用到 → 常见错误 → 怎么验证。
> 面向完全没有 ROS 和 C++ 基础的人，代码片段可以直接对照仓库里的实际文件。

---

## 第 0 章 先建立四个习惯

### 0.1 四段式记录法

每改一处代码，都按这个格式写下来（这也是答辩时最省事的讲稿）：

```
现象：飞机不动，/drone/state 里 setpoint_ok 一直是 false
原因：500ms 心跳要求发布间隔 < 500ms，1Hz 是 1000ms，超时两倍
改法：ros::Rate rate(5.0)，间隔 200ms
验证：rostopic hz /drone/setpoint 显示 5Hz，setpoint_ok 变 true
```

**只有“改法”没有“现象+原因+验证”，答辩一问就穿。**

### 0.2 单变量原则

一次只改一处，跑一次，看现象变了没有。一次改五处，等于没改——你不知道是哪处生效的。

### 0.3 先确认“现场”，再改代码

代码没错、环境没起，也会表现为“代码不对”。所以任何排查的第一个动作是**确认输入有没有数据**：

```bash
rosnode list          # 节点在不在
rostopic list         # 话题在不在
rostopic hz <话题>    # 数据在不在流
rostopic echo <话题>  # 数据内容对不对
```

### 0.4 优先看“输出”，而不是猜代码

```
rostopic echo -n1 /drone/setpoint      # 我到底发出去了什么
rosnode info /flight_node              # 我的节点在听谁、跟谁说话
rosparam get /R_outside                # 参数真的是这个值吗
```

**很多 bug（变量名抄错、参数没加载、编码错误）在代码里看不出来，一 echo 就现形。**

---

## 第 1 章 ROS 通信基础

### 1.1 ROS 是什么

ROS 不是操作系统，是一套“让很多程序互相传数据”的中间件。它解决三件事：

1. **找得到彼此**：每个程序叫一个“节点”，通过名字互相发现；
2. **传得动数据**：数据走“话题”，格式由“消息类型”定义；
3. **管得住配置**：参数放在“参数服务器”，启动时读取。

### 1.2 五个核心概念

| 概念 | 类比 | 代码里 |
| --- | --- | --- |
| 节点 node | 一个进程 | `ros::init(argc, argv, "flight_node")` |
| 话题 topic | 广播频道 | `/drone/setpoint`、`/mavros/state` |
| 消息 message | 信封的格式 | `geometry_msgs/PoseStamped`、`std_msgs/Float32` |
| 发布/订阅 | 广播/收听 | `advertise<>()` / `subscribe<>()` |
| 服务 service | 打电话问一次、立刻得到答复 | `mavros/cmd/arming`（解锁） |
| 参数 param | 全局配置表 | `nh.param<float>("R_outside", ...)` |

**话题 vs 服务**：话题是“持续广播，不关心谁在听”；服务是“我发出请求，你回一个响应”。位置、状态用话题；解锁、切模式用服务。

### 1.3 消息类型长什么样

任何消息都可以拆成“字段表”：

```cpp
// drone_backend/DroneState
bool armed          // 是否解锁
string mode         // 当前模式，如 AUTO.TAKEOFF / HOLD / LAND
float32 altitude    // 当前高度 [m]
bool setpoint_ok    // 设定点流是否有效（心跳是否满足）
```

C++ 里访问字段：`msg->armed`（指针用 `->`），Python 里 `msg.armed`。

### 1.4 catkin 工作空间的四个目录

```
airhust_tutorial26_ros_basics/     ← 工作空间根（有 .catkin_workspace）
├── src/          源码空间：包都放这里（CMakeLists.txt + package.xml + 代码）
├── build/        编译空间：catkin_make 的中间产物（可删，会自动重建）
├── devel/        开发空间：编出来的可执行文件、生成 devel/setup.zsh
└── install/      安装空间：catkin_make install 的成品（可分发、路径写死）
```

**关键认知**：

- `catkin_make` 只写 `build/` 和 `devel/`，**不会动 `install/`**；
- 每个工作空间有自己的一套 `devel/`，**必须站在那个工作空间根目录编译、source、运行**；
- 第一关的 `install/` 是别人在**另一台机器**上编好拷进来的（里面还写着旧路径 `/home/consefact/...`），所以要用作者的 `setup.sh` 自动改写路径。

### 1.5 setup 脚本：环境是怎么“加载”的

`source` 不是执行程序，而是**在当前 shell 里跑脚本**，脚本里的 `export` 会留在这个终端。

| 脚本 | 提供什么 |
| --- | --- |
| `/opt/ros/noetic/setup.zsh` | 系统 ROS：`roscore`/`rosrun` 在 PATH 里，`ROS_PACKAGE_PATH`、`CMAKE_PREFIX_PATH`、`PYTHONPATH` |
| `<ws>/install/setup.zsh` | 预编译的后端包（`drone_backend`） |
| `<ws>/devel/setup.zsh` | 你自己编出来的包（`drone_bringup`、`template`） |

**叠加顺序**：系统 ROS 先，工作空间后（后面的优先级更高）。顺序反了会丢失基础环境。

第一关工作空间还给了统一入口 `setup.sh`，它做三件事：

```bash
# 1. 用 install/.build_prefix 记录旧路径，把 install/ 里的写死路径原地改掉
# 2. source install/setup.zsh        → 得到 drone_backend
# 3. 有 devel/setup.zsh 就 source，没有就把 src 塞进 ROS_PACKAGE_PATH
source ~/March_airhust/1关/airhust_tutorial26_ros_basics/setup.sh
```

### 1.6 package.xml 和 CMakeLists.txt 分工

| 文件 | 作用 | 谁读它 |
| --- | --- | --- |
| `package.xml` | **声明**依赖、包名、版本 | catkin 工具、`rospack`、`rosdep` |
| `CMakeLists.txt` | **真正使用**依赖，生成可执行文件 | CMake / 编译器 |

**两者都要写**：声明了不代表编译时会找，找了不代表声明过。典型症状：

- `package.xml` 缺依赖 → `catkin_make` 警告或运行时找不到；
- `CMakeLists.txt` 的 `find_package` 缺组件 → 编译报“找不到 xxx 的头文件”；
- 只有 `add_executable` 没写进文件 → `roslaunch` 报 `Cannot locate node of type [...]`。

### 1.7 一条命令的“必经链路”

以 `roslaunch drone_bringup mission.launch` 为例：

```
roslaunch 读 launch 文件
   → 按 pkg="drone_bringup" 找到包（靠 ROS_PACKAGE_PATH）
   → 按 type="flight_node" 找到可执行文件（在 devel/lib/drone_bringup/）
   → 起进程、输出日志、把 stdout 打到屏幕
```

**任何一环断了，报错都不同**，这就是“看报错定位”的原理：

| 报错 | 断在哪 |
| --- | --- |
| `[rospack] Error: package 'x' not found` | 包路径（没 source） |
| `Cannot locate node of type [x] in package [y]` | 可执行文件（没编译，或没写 add_executable） |
| `Unable to communicate with master!` | roscore 没起 |
| `RLException: ... is neither a launch file in package ...` | launch 路径/包名错 |

---

## 第 2 章 第一关：环境修复与基础通信

### 2.1 这一关的真实结构

```
airhust_tutorial26_ros_basics/
├── src/drone_bringup/          你要修的包
│   ├── CMakeLists.txt          ← 阶段一改这里
│   ├── package.xml             ← 阶段一改这里
│   ├── launch/mission.launch   只启动 flight_node
│   └── src/{flight_node, example_listener}.cpp
├── install/                    预编译后端（不要改），提供 drone_backend
├── run_sim.sh                  起 roscore + sensor_node + drone_sim_node
└── setup.sh                    统一环境入口
```

**后端不是真 PX4/MAVROS**，是两个模拟节点：`drone_sim_node` 发 `/drone/state`、收 `/drone/setpoint`；`sensor_node` 发障碍距离。

### 2.2 接口表（第一关全部对外接口）

| 方向 | 话题 | 类型 | 含义 |
| --- | --- | --- | --- |
| 后端 → 你 | `/drone/state` | `drone_backend/DroneState` | armed / mode / altitude / setpoint_ok |
| 你 → 后端 | `/drone/setpoint` | `geometry_msgs/PoseStamped` | 目标位置（含 yaw 用四元数，本关不用） |
| 传感器 → 你 | `/drone/sensor/obstacle_distance` | `std_msgs/Float32` | 距障碍物距离 [m] |

### 2.3 阶段一：三处编译错误

| 现象 | 原因 | 改法 |
| --- | --- | --- |
| `Could not find a package configuration file provided by "drone_backend"` | 后端在 `install/` 里，环境没加载 | 先 `source setup.sh`；再从 `CMakeLists` 的 `find_package` 里补组件 |
| `fatal error: geometry_msgs/PoseStamped.h: No such file` | 依赖没在 CMake 里声明 | `find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs std_msgs)` + `catkin_package(CATKIN_DEPENDS ...)` |
| `Cannot locate node of type [flight_node]` | 从来没编过这个可执行文件 | `add_executable(flight_node src/flight_node.cpp)` + `target_link_libraries(flight_node ${catkin_LIBRARIES})` |

**注意第三条最容易被误诊**：文件明明在 `src/` 里，但 CMake 里没写 `add_executable`，所以 `devel/lib/` 里没有它，launch 自然找不到。

### 2.4 阶段二：六个 bug（都在 `flight_node.cpp`）

| # | 骨架里的写法 | 现象 | 正确方向 | 原理 |
| --- | --- | --- | --- | --- |
| 1 | `ros::Rate rate(1.0)` | 飞机不动，`setpoint_ok=false` | ≥ 2Hz，实践取 5~10Hz | 500ms 心跳，1Hz=1000ms 超时两倍 |
| 2 | `nh.subscribe("/drone", ...)` | 距离永远是初值 2.0 | `/drone/sensor/obstacle_distance` | 话题名字符串必须逐字对齐文档 |
| 3 | `setpoint.pose.position.z = 0.0` | 目标就是地面 | `1.5` | 目标高度 = 起飞高度 |
| 4 | `if (distance < 0.0)` | 永不停 | `< 0.5` | README 的数字 |
| 5 | `setpoint.header.stamp = ros::Time()` | 心跳被判失效 | `ros::Time::now()` | 0 时刻=“未指定”，心跳要新鲜时间戳 |
| 6 | `publish(fakepoint)` | 话题有数据、频率正常、就是不动 | `publish(setpoint)` | 发错变量：空点位置全 0 |

**第 6 条是这一关的灵魂**：它不能靠读代码发现（两个变量长得很像），只能靠 `rostopic echo -n1 /drone/setpoint` 看出“发出去的是全 0”。

### 2.5 验证判据（2A / 2B）

```bash
# 终端1：后端
./run_sim.sh
# 终端2：节点
source ./setup.sh && roslaunch drone_bringup mission.launch
# 终端3：观察
rostopic hz /drone/setpoint          # 应 ≈ rate 的值，绝不低于 2Hz
rostopic echo -n1 /drone/setpoint    # z=1.5，stamp 是当前时间
rostopic echo /drone/state           # setpoint_ok=true，altitude 爬到 1.5
```

**跑完后端会自己判定成功并退出**——这是唯一有效的通过判据，不要用眼睛估。

### 2.6 这一关踩过的三个环境坑

1. **`rospack find drone_backend` 找不到** → 没 source `setup.sh`；`install/` 里路径还写着别人的机器时，直接 source `install/setup.zsh` 也会指向不存在的目录；
2. **改了代码没反应** → 只 `catkin_make` 不够，**正在跑的节点还是旧程序**，必须重启 `roslaunch`；
3. **`.zshrc` 里的 `source ~/airhust_ws/devel/setup.zsh`** → 这是镜像提供的仿真环境（PX4/MAVROS/`simulation` 包），**不要删**；但两个工作空间同时加载时，同名包会互相抢，要注意 `rospack find <包>` 到底指向谁。

---

## 第 3 章 第二关：控制流与绕圈

### 3.1 程序的骨架（template.cpp）

```
main:
  ① 通信准备：订阅 state / odom，发布 setpoint_raw，创建 arming / set_mode 服务客户端
  ② 执行准备：读参数（err_max / if_debug / 自定义参数）、打印、设频率 rate(20)
  ③ 开始条件：等待输入 1 → 等待飞控连接 → 请求 OFFBOARD → 请求解锁
  ④ 起飞：持续发目标 (0,0,ALTITUDE) → 到达并稳定后 break
  ⑤ 任务状态机：switch (mission_num)，每个 case 是一个阶段
```

### 3.2 回调：谁在什么时候填数据

```cpp
ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
```

- `subscribe` 的第三个参数是**函数地址**，登记“以后有消息就交给它”；
- 消息到达时只是**进入队列**，不会立刻赋值；
- 主循环跑到 `ros::spinOnce()` 时，ROS 才把队列里的消息取出来、调用回调；
- 回调里 `current_state = *msg;` 完成赋值。

**三个直接推论**：

1. 你读到的永远是“上一次 spinOnce 处理过的位置”，天然滞后；
2. 程序刚启动时数据是全 0 —— 所以 `local_pos_cb` 里要等 `z != 0` 才记录起飞点；
3. **删掉 `spinOnce()`，回调一次都不会执行**，`connected` 永远 false，程序卡在等待循环里，而且不报错。

### 3.3 状态机：`mission_num` 与返回值

```cpp
case 2:
  if (mission_pos_cruise(4.0, 0.0, ALTITUDE, 1.57, err_max)) {
      mission_num = 3;
  }
  break;
```

**返回 false 的那一轮做了五件事**：

1. `if` 不成立 → `mission_num` 不变；
2. `break` 跳出 switch；
3. `publish(setpoint_raw)` 把这一轮填好的目标发出去；
4. `spinOnce()` 更新状态与位置；
5. `rate.sleep()` 睡到下一个周期，然后**再进同一个 case**，用最新位置重新判断。

所以“发目标 → 收反馈 → 检查进度 → 再发”是一个 20Hz 的循环，**不是一次调用就完成**。

### 3.4 函数参数怎么读：左右配对

```cpp
// 调用（template.cpp）
mission_pos_cruise(4.0, 0.0, ALTITUDE, 1.57, err_max)

// 定义（template.h）
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max)
```

| 位置 | 实参 | 形参 | 含义 |
| --- | --- | --- | --- |
| 1 | 4.0 | x | 相对起飞点的东向偏移 [m] |
| 2 | 0.0 | y | 北向偏移 [m] |
| 3 | ALTITUDE | z | 目标高度 [m] |
| 4 | 1.57 | target_yaw | 目标朝向 [**弧度**]，1.57=90° |
| 5 | err_max | error_max | 位置容差 [m] |

**最终目标 = 起飞点 + 偏移**：

```cpp
setpoint_raw.position.x = x + init_position_x_take_off;
```

因为起飞点通常不在世界原点，加偏移才能“从起飞点算起”。

### 3.5 到达判据与 yaw 的坑

```cpp
fabs(x误差) < error_max && fabs(y误差) < error_max && fabs(z误差) < error_max
                         && fabs(yaw - target_yaw) < 0.1
```

- yaw 阈值 0.1 弧度 ≈ **5.7°**；
- **四项全满足**才算到达（`&&`），先到的那项不会让任务提前结束；
- **±180° 有角度回绕问题**：实际 yaw 可能表示成 -3.13，目标 3.14，差 6.27，永远不成立。所以目标 yaw 尽量用 0 / ±90°。

### 3.6 计时套路：三个函数同一个模式

`lib_time_record_func()`、`hover()`、`precision_land()` 结构一样：

```
第一次进入 → 记录时间（和/或位置）→ 置标志
之后每轮   → 比较 "现在 - 记录时间"
           → 超过阈值 → 返回 true，并复位标志
           → 否则     → 返回 false
```

为什么必须用全局变量：局部变量每轮都会重新创建，记不住“已经等了多久”。

### 3.7 YAML 参数链路

```
template.yaml 文件
    ↓ rosparam load（或 launch 里的 <rosparam command="load">）
ROS 参数服务器：/circle_radius = 1.0
    ↓ nh.param<float>("circle_radius", circle_radius, 1.0f)
程序变量 circle_radius
    ↓ 后面的逻辑使用
```

四个必须知道的点：

1. **`nh.param` 只在节点启动时读一次**，之后参数服务器怎么变都不影响已读入的变量；
2. **名字拼错不报错**，静默使用默认值 → 所以必须 `print_param()` 打印出来核对；
3. 改 YAML **不用重新编译**，但要**重启节点**；
4. 如果 launch 里有 `<rosparam>`，加载发生在节点启动之前，顺序天然正确。

### 3.8 绕圈：思路 B（时间驱动）

公式（讲义给的）：

$$\theta(t)=\theta_0+s\,\omega(t-t_0),\qquad \omega=\frac{2\pi}{T}$$

实现要点：

```cpp
// 第一次进入记录起始时刻
if (!circle_started) { circle_t0 = ros::Time::now(); circle_started = true; }
double t = (ros::Time::now() - circle_t0).toSec();

double theta = 2 * M_PI * t / period;     // 角度随时间增长
double x = radius * cos(theta);           // 相对起飞点的偏移
double y = radius * sin(theta);

mission_pos_cruise(x, y, ALTITUDE, 0, err_max);   // 忽略返回值！
if (t >= period) { circle_started = false; return true; }
```

**两个关键设计决定**：

1. **忽略 `mission_pos_cruise` 的返回值**：目标每 50ms 都在移动，“是否到达这个点”永远为 false，用它推进会卡死；
2. **一圈结束用时间判断**（`t >= period`），而不是“到达某个点”。

### 3.9 参数与物理量的换算（必须会算）

目标点沿圆周移动的线速度：

$$v=\frac{2\pi r}{T}$$

`r = 1.0 m`、`T = 20 s` → `v = 0.314 m/s`，每个控制周期（50ms）移动 **1.57 cm**。

**判据：每周期目标移动距离 ≪ `err_max`（0.15 m），才追得上。** 如果 `T` 太小（比如 3 s，每周期 10 cm），飞机会一直落后，轨迹被拉成大椭圆甚至卡住。

### 3.10 短练习二：速度控制

要点清单：

| 要点 | 说明 |
| --- | --- |
| 掩码 | `1+2+4` 表示忽略位置，`8+16+32` 不加入表示**使用速度** |
| 误差符号 | 必须 `目标 − 实际`，写反了朝反方向飞 |
| vz | 直接置 **0**（高度保持，不是“飞向某个高度”） |
| 限速 | 分量限幅 vs 合速度限幅；合速度限幅**保持方向**，分量限幅会扭曲方向 |
| 到点停止 | 到达时先把三轴速度置 0，**再** `return true` |
| 只算一次会怎样 | 速度固定 → 匀速穿过目标 → 误差不再被纠正 → 永远飞下去 |

### 3.11 绕圈任务的“进度证据”

按时间结束的问题是：**跟踪落后时，时间到了可能只绕了 3/4 圈**。可以做三件事取证：

1. 把每轮的实际位置打到日志里（或写成 CSV），跑完画轨迹；
2. 累计“实际转过的角度” `Δφ = atan2(y−cy, x−cx)` 的增量，看总和是否接近 2π；
3. rviz 里加 Path 插件订阅里程计，直观看到闭合的圆。

---

## 第 4 章 第三关：人工势场法避障

### 4.1 算法核心：两种力

| 力 | 方向 | 公式 |
| --- | --- | --- |
| 引力 | 由飞机指向目标 | $(k_a(x_g-x),\; k_a(y_g-y))$ |
| 斥力 | 由障碍指向飞机 | 分段：见下 |

斥力大小（$R_{in}$ 内圈、$R_{out}$ 外圈）：

$$
F_{rep}(\rho)=
\begin{cases}
0, & \rho>R_{out}\\
p_R(R_{out}-\rho), & R_{in}<\rho\le R_{out}\\
p_R(R_{out}-R_{in})+p_r(R_{in}-\rho), & \rho\le R_{in}
\end{cases}
$$

**内圈公式必须包含外圈那一项**，否则在 $\rho=R_{in}$ 处跳变。验算（$R_{out}=1.6,R_{in}=0.9,p_R=0.45,p_r=1.5$）：

| 交界 | 外圈 | 内圈 | 结论 |
| --- | --- | --- | --- |
| ρ=1.6 | 0 | — | 与“无作用区”接上 |
| ρ=0.9 | 0.45×0.7=**0.315** | 0.45×0.7+1.5×0=**0.315** | 连续 |
| ρ→0 | — | 0.315+1.5×0.9=**1.665** | 最大斥力 |

### 4.2 矢量 = 大小 × 单位方向

```cpp
distance_cx = distance_c * cos(angle_c);        // 指向障碍的分量
vel_collision[0] -= F_c * distance_cx / distance_c;
```

`distance_cx / distance_c` 就是 `cos(angle_c)`，**距离被约掉了，剩下的是方向**。所以整行的含义是“大小为 `F_c`、方向背离障碍物的向量”。**不要写成 `F_c × distance_c`**——那样远处力反而更大，完全反了。

### 4.3 两个坐标系必须统一

| 量 | 坐标系 | 单位 |
| --- | --- | --- |
| `angle_c` | 机体系（0°=机头） | **度** |
| `yaw` | 机体相对世界 | **弧度** |
| `vel_track`（引力） | 世界系 | m/s |
| `vel_collision`（斥力） | 机体系 | m/s |
| `vel_sp_ENU`（输出） | 世界系 | m/s |

**机体系 → 世界系要绕 z 轴转 `+yaw`**：

```cpp
output[0] = input[0]*cos(yaw) - input[1]*sin(yaw);
output[1] = input[0]*sin(yaw) + input[1]*cos(yaw);
```

依据（文件里自己写着）：`cloud_cb` 里 `bearing = bearing_world - yaw_degree`，移项得 `bearing_world = angle_c + yaw`，所以**转回去要加 yaw**。

**自检向量**：`yaw = π/2`、输入 `(1,0)` → 输出必须是 `(0,1)`；得到 `(0,-1)` 就是转反了（表现为越躲越撞）。

### 4.4 两级限幅

| 限幅 | 防什么 |
| --- | --- |
| `vel_collision_max` | 斥力过大导致暴冲 |
| `vel_sp_max` | 总速度超限 |

**必须用等比缩放**（`v * max / |v|`），不能截断（`if (v>max) v=max`）——截断会改变方向。

**注意副作用**：`vel_collision_max = vel_sp_max = 0.3` 时，靠近到一定程度两个力都饱和，**抵消后合力恰好为 0**，这就是“飞机停在原地”的直接原因。

### 4.5 感知层：点云 → 距离

```
PointCloud2（世界系 ENU）
  ↓ pcl::fromROSMsg 转成可读点
  ↓ 只保留 |z - 飞机高度| ≤ z_band 的点（水平切片）
  ↓ 丢弃 r < min_obstacle_range 或 > max_obstacle_range 的点
  ↓ 算方位角 bearing = atan2(dy,dx) - yaw，换算到 0~360
  ↓ 每 1° 一个 bin，只留最近距离
  ↓ cal_min_distance()：取所有 bin 里最小的 → distance_c / angle_c
```

**判据**：飞机停在地面、柱子中心在 (3.05, ±0.52)、柱半径 0.5 时，到柱面距离

$$\sqrt{3.05^2+0.52^2}-0.5\approx 2.59\ \text{m}$$

所以起飞前 `Distance` 应该显示 ≈2.6 m。显示 `100`（`NO_OBSTACLE`）说明柱子没被算进来。

**`z_band` 太窄**（如 0.10）会导致“远处点落不进高度带 → 检测滞后 → 撞上才 Enabled”，而且**同参数两次运行结果会不同**（扫描线是否命中靠运气）。

### 4.6 局部最优（停滞）

成因：引力与斥力**共线反向、大小相等** → 合力 0。而两个限幅都是 0.30 时，**饱和区域很大**，所以平衡点不是“一个点”，而是一片区域。

三种应对：

| 方案 | 做法 | 特点 |
| --- | --- | --- |
| 不对称限幅 | 令 `vel_collision_max > vel_track_max` | 最简单，只是“慢慢挪” |
| **切向场** | 把斥力方向转 ±90°，沿障碍边缘滑 | 通用、确定性好（本项目采用） |
| 停滞检测 + 固定侧移 | 指令速度≈0 持续 1s → 强行往 +y 推 | 兜底，但要选边 |

本项目实际用的是「停滞检测 + 切向场」的组合：

```
指令速度 < 0.03 且 斥力 > 0.05  → 计时
计时 > 1 秒                      → 进入逃逸（持续 1 秒）
逃逸期间：切向 = rotation_yaw(90°, vel_collision_ENU)
          归一化到 vel_sp_max 输出
```

**为什么“切向”比“固定 +y”好**：

1. 换任何障碍布局都能用（方向由当前障碍决定，不是写死的）；
2. 只在最近障碍变化时才改方向，比“随机选边”稳定；
3. 复用已有的 `rotation_yaw()`，不引入新机制。

**为什么要归一化**：`vel_collision_ENU` 的大小受限幅约束（≤0.3），而且随距离变化；不归一化就直接用它，可能推出一个很小的速度，推不出平衡点。

停滞检测的定量依据：**满速 0.3 m/s、周期 0.05 s 时每轮只移动 0.015 m**，所以“用位置变化 < 0.02 判卡住”会把正常飞行也判成卡住——**阈值必须和系统的量级对齐**，或者干脆改用“指令速度”。

**切向场的副作用**（答辩加分项）：

| 副作用 | 原因 | 缓解 |
| --- | --- | --- |
| 绕远路 | 固定转 +90°（逆时针），可能绕向目标的反方向 | 改成“两个方向都算，取靠近目标的一侧” |
| 贴着障碍打转 | 切向 + 斥力形成圆形轨道 | 缩短逃逸时长（0.5s） |
| 绕到另一个障碍前再卡 | 只考虑最近障碍 | 逃逸可反复触发，能连续绕几段 |

### 4.7 目标不可达（GNRON）

障碍靠近目标时，斥力永远把飞机推离目标 → 差一点到不了。缓解：斥力乘以“到目标的距离”因子，使接近目标时斥力衰减。

### 4.8 `type_mask` 位掩码（本关必会）

| 位 | 字段 | 置 1 = 忽略 |
| --- | --- | --- |
| 1 / 2 / 4 | position x/y/z | |
| 8 / 16 / 32 | velocity x/y/z | |
| 64 / 128 / 256 | accel/force | |
| 512 | FORCE 标志（不是姿态位） | |
| 1024 / 2048 | yaw / yaw_rate | |

避障 `case 2` 的 `1+2+32+64+128+256+512+2048` 翻译：**xy 用速度控制，z 用位置控制（保持高度），yaw 固定**。所以写 `velocity.z` 没有用。

---

## 第 5 章 调试方法论

### 5.1 从现象到代码的四步

```
① 现象（屏幕上的字、飞机怎么动）
② 抓关键词：报错里的包名/文件名/节点名/话题名
③ grep -rn "关键词" src/     → 找到所有提到它的地方
④ 拿文档表格核对（话题名、参数名、字段名逐字比）
```

### 5.2 命令速查

| 目的 | 命令 |
| --- | --- |
| 节点在不在 | `rosnode list` / `rosnode info /节点名` |
| 话题在不在 | `rostopic list` / `rostopic info /话题` |
| 有没有数据、多快 | `rostopic hz /话题` |
| 数据内容 | `rostopic echo -n1 /话题` |
| 参数 | `rosparam list` / `get` / `set` / `load` / `dump` |
| 包在哪 | `rospack find 包名` |
| 服务 | `rosservice list` / `rosservice call /gazebo/reset_world` |
| 手动发一条消息 | `rostopic pub -r 10 /话题 类型 '{字段}'` |
| 节点图 | `rqt_graph` |
| 曲线 | `rqt_plot /话题/字段` |

### 5.3 复位与重启的层级

| 情况 | 动作 | 耗时 |
| --- | --- | --- |
| 改代码/改 yaml 后重跑 | Ctrl-C 节点 → `rosservice call /gazebo/reset_world` → 重启节点 | ~5 s |
| 飞机姿态乱了 | `rosrun mavros mavsafety disarm` + reset_world | ~5 s |
| 复位后起飞异常 | 重启 `sim.launch`（先 `pkill -f gzserver/gzclient/px4`） | ~20 s |
| 改了 world 文件 | 必须重启 `sim.launch` | — |

**改 `.launch` 和 `.yaml` 都不需要 `catkin_make`；只有 `.cpp/.h` 才要。**

### 5.4 报错对照表（全程汇总）

| 报错 | 原因 | 处理 |
| --- | --- | --- |
| `[rospack] Error: package 'x' not found` | 环境没加载 / 包路径不对 | source 对应工作空间的 setup |
| `Cannot locate node of type [x]` | 没编译 / 没写 add_executable | 检查 CMakeLists + `catkin_make` |
| `Unable to communicate with master!` | roscore 没起 | 先起仿真或 `roscore` |
| `fatal error: xxx.h: No such file` | 依赖没声明 | `find_package` 补组件 |
| 静默使用默认值 | 参数名拼错/没 load | `rosparam get` + `print_param()` |
| `UnicodeDecodeError: 'utf-8' codec can't decode byte 0xd6` | yaml 是 GBK 编码 | `iconv -f GBK -t UTF-8` 转码 |
| 改了代码没反应 | 没重启节点 / 没编译 | 重启 `roslaunch` |
| `rejected ... fetch first` | 远端有新提交 | `git pull --rebase` |
| 飞机不动、`setpoint_ok=false` | 发布频率不够/时间戳为 0 | 提高 rate、用 `ros::Time::now()` |
| 飞机朝反方向飞 | 误差符号反了 | 目标 − 实际 |

---

## 第 6 章 术语表与答辩要点

### 6.1 术语

| 术语 | 一句话解释 |
| --- | --- |
| 工作空间 | 有 `src` 和 `.catkin_workspace` 的目录，一套编译体系的根 |
| 包 package | 工作空间里最小可编译单元（有 `package.xml`） |
| 节点 node | 一个 ROS 进程 |
| 话题 topic | 持续广播的数据通道 |
| 消息 message | 话题上传输的数据格式 |
| 回调 callback | 数据到达后由 `spinOnce()` 触发的函数 |
| 参数服务器 | 全局配置表，`rosparam`/`nh.param` 读写 |
| underlay / overlay | 基础环境 / 叠加在其上的工作空间，后者优先 |
| 心跳 | 周期性发送同样的指令，让飞控确认程序还活着 |
| 位掩码 | 用整数的每一位表示“某个字段是否忽略” |
| 局部最优 | 合力为零导致的停滞 |
| GNRON | 目标附近有障碍导致“差一点到不了” |

### 6.2 答辩高频问题（每个都要能一句话答）

1. 为什么要 `source` 两个 setup？顺序能反吗？
2. `package.xml` 和 `CMakeLists.txt` 各管什么？
3. 1Hz 为什么不行？（500ms 心跳）
4. `stamp` 写 0 会怎样？
5. `publish(fakepoint)` 为什么难发现？
6. `spinOnce()` 做了什么？删掉会怎样？
7. `mission_num` 是什么？返回 false 那一轮发生了什么？
8. 为什么绕圈要忽略 `mission_pos_cruise` 的返回值？
9. `period` 为什么不能太大/太小？（每周期目标移动 vs `err_max`）
10. 斥力为什么分两段，交界处为什么必须连续？（0.315 验算）
11. 为什么必须做坐标变换？转反了什么现象？
12. 限幅为什么用等比缩放？
13. 两个 0.30 为什么会造成停滞？
14. `type_mask` 那串数字什么意思？为什么 `velocity.z` 无效？
15. `z_band` 是什么？太窄会怎样？
16. 局部最优和 GNRON 的区别与解法。
