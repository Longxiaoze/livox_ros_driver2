# Livox ROS Driver2 MID360 崩溃问题分析与修复记录

## 1. 问题概述

测试环境：

- ROS 2 Humble
- `livox_ros_driver2` 版本：1.0.0
- MID360 地址：`192.168.124.124`
- 工作空间：`~/fast_livo2_handhold2_livox_ws`
- 启动命令：

```bash
source ~/fast_livo2_handhold2_livox_ws/install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

本次排查发现两个相互独立的问题：

1. 驱动启动约 3 秒后抛出 `basic_string::_M_construct null not valid`，退出码为 `-6`。
2. 修复启动崩溃后，驱动在个别 `Ctrl+C` 退出过程中发生 `SIGBUS` 或退出码 `-11`。

第一个问题来自驱动中的时间戳共享定制代码；第二个问题来自 Livox-SDK2
内置的旧版 `spdlog` 与 ROS 2 Humble 使用的系统 `spdlog` 发生 ABI 和符号冲突。

---

## 2. 启动阶段崩溃

### 2.1 故障现象

LiDAR 已经成功连接并进入正常工作模式：

```text
successfully change work mode, handle: 2088544448
successfully enable Livox Lidar imu, ip: 192.168.124.124
livox/imu publish use imu format
terminate called after throwing an instance of 'std::logic_error'
what(): basic_string::_M_construct null not valid
process has died ... exit code -6
```

这说明网络、MID360 地址和基础配置都已生效。`exit code -6` 对应
`SIGABRT`，是未捕获的 C++ 异常触发 `std::terminate()` 后产生的，不是雷达掉线。

### 2.2 直接原因

原始代码位于
`livox_ros_driver2/src/lddc.cpp` 的
`Lddc::PollingLidarPointCloudData()`：

```cpp
const char *user_name = getlogin();
std::string path_for_time_stamp =
    "/home/" + std::string(user_name) + "/timeshare";
```

`getlogin()` 并不保证返回有效字符串。ROS launch、SSH、容器、systemd
或没有 controlling terminal 的会话中，它可以返回 `nullptr`。

在当前机器上已直接验证：

```text
getlogin returned NULL
HOME=/home/unitree
```

因此原代码实际执行了：

```cpp
std::string(nullptr);
```

libstdc++ 检测到空指针后抛出：

```text
std::logic_error:
basic_string::_M_construct null not valid
```

该异常发生在点云轮询线程中，没有被捕获，最终导致整个驱动进程退出。

### 2.3 为什么最后一条日志是 IMU 日志

`DriverNode` 创建点云和 IMU 两个线程后，两个线程都会先等待 3 秒：

```cpp
std::this_thread::sleep_for(std::chrono::seconds(3));
```

3 秒后：

- IMU 线程创建 `/livox/imu` publisher，并打印
  `livox/imu publish use imu format`；
- 点云线程几乎同时进入时间戳文件初始化，并在 `std::string(nullptr)` 处崩溃。

因此最后一条日志看起来与 IMU 有关，但真正抛异常的是并行运行的点云线程。

### 2.4 原时间戳代码中的其他风险

即使只替换 `getlogin()`，原代码仍存在以下问题：

1. `open()` 失败后仍继续调用 `lseek()`、`write()` 和 `mmap()`。
2. `mmap()` 返回 `MAP_FAILED` 时没有检查。
3. `pointt` 原来没有初始化。
4. 点云发布函数无条件执行 `pointt->low = timestamp`。
5. 析构函数无条件执行 `munmap(pointt, ...)`。
6. 原代码先跳到 `sizeof(time_stamp)`，再写入 1 字节，会把文件扩展成
   17 字节，而实际映射长度为 16 字节。
7. 时间戳共享属于辅助功能，其初始化失败不应该导致 LiDAR 数据停止发布。

所以本次没有采用单行替换，而是把文件打开、长度设置、内存映射和清理流程一起修复。

---

## 3. 启动崩溃的修复方案

修改文件：

- `livox_ros_driver2/src/lddc.cpp`
- `livox_ros_driver2/src/lddc.h`

### 3.1 使用 `HOME` 获取用户目录

当前 launch 环境中 `HOME=/home/unitree`，它不依赖 controlling terminal：

```cpp
const char *home_dir = std::getenv("HOME");
if (home_dir == nullptr || home_dir[0] == '\0')
{
  RCLCPP_ERROR(...,
               "HOME is unavailable; timestamp sharing is disabled");
}
else
{
  std::string path_for_time_stamp =
      std::string(home_dir) + "/timeshare";
  // 后续打开及映射文件
}
```

当 `HOME` 不可用时，只禁用时间戳共享，点云和 IMU 发布继续运行。

### 3.2 安全创建和映射时间戳文件

修复后的流程为：

1. 每个驱动进程只尝试初始化一次。
2. 使用 `open()` 创建 `/home/unitree/timeshare`。
3. 使用 `ftruncate(fd, sizeof(time_stamp))` 把文件精确设置为 16 字节。
4. 检查 `open()`、`ftruncate()` 和 `mmap()` 的返回值。
5. 成功映射后初始化 `high` 和 `low`。
6. 映射完成后关闭文件描述符；内存映射继续有效。
7. 任一步骤失败都只记录错误，不影响 ROS topic 发布。

关键逻辑：

```cpp
int fd = open(path_for_time_stamp.c_str(),
              O_CREAT | O_RDWR | O_TRUNC, 0666);

if (fd != -1 && ftruncate(fd, sizeof(time_stamp)) == 0)
{
  void *mapped = mmap(nullptr, sizeof(time_stamp),
                      PROT_READ | PROT_WRITE,
                      MAP_SHARED, fd, 0);
  if (mapped != MAP_FAILED)
  {
    pointt = static_cast<time_stamp *>(mapped);
    pointt->high = 0;
    pointt->low = 0;
  }
}
```

### 3.3 初始化、访问和释放指针

`pointt` 现在在声明处初始化：

```cpp
time_stamp *pointt = nullptr;
```

更新时间戳前检查指针：

```cpp
if (pointt != nullptr)
{
  pointt->low = timestamp;
}
```

析构时同样检查：

```cpp
if (pointt != nullptr)
{
  munmap(pointt, sizeof(time_stamp));
  pointt = nullptr;
}
```

这样即使时间戳文件不可创建，驱动也不会解引用或释放无效地址。

---

## 4. 为什么第一次修改后仍然不生效

检查发现当时有两个问题：

1. `livox_ros_driver2/src/lddc.cpp` 中仍保留原来的 `getlogin()` 代码，修改没有保存到实际构建的源码。
2. 15:24 的 `colcon build` 构建失败，`install` 中仍是 15:16 的旧程序。

当时的构建错误为：

```text
failed to create symbolic link ... because existing path cannot be removed:
Is a directory
```

之后直接执行普通 `colcon build` 又遇到：

```text
LIVOX_INTERFACES_INCLUDE_DIRECTORIES-NOTFOUND
```

该项目的 `CMakeLists.txt` 使用 `HUMBLE_ROS` 分支选择 ROS 2 Humble 的
typesupport 链接方式，因此手动构建时必须传入项目需要的 CMake 参数。

正确构建命令：

```bash
cd ~/fast_livo2_handhold2_livox_ws
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select livox_ros_driver2 \
  --symlink-install \
  --cmake-args \
    -DROS_EDITION=ROS2 \
    -DHUMBLE_ROS=humble

source install/setup.bash
```

本次实际构建结果：

```text
Finished <<< livox_ros_driver2
Summary: 1 package finished
```

可用下面的字符串确认当前安装库包含新逻辑：

```bash
strings \
  ~/fast_livo2_handhold2_livox_ws/install/livox_ros_driver2/lib/liblivox_ros_driver2.so \
  | grep "Timestamp sharing enabled"
```

---

## 5. 退出阶段偶发 SIGBUS/-11

### 5.1 现象

启动崩溃修复后，点云和 IMU 均能正常发布，但一次 `Ctrl+C` 测试中出现：

```text
Livox Lidar SDK Deinit completely!
lddc destory!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
process has died ... exit code -11
```

该问题只发生在退出清理阶段，与启动时的 `std::string(nullptr)` 无关。

### 5.2 GDB 定位结果

使用 GDB 运行驱动并自动发送 `SIGINT`，捕获到：

```text
Thread 1 received signal SIGBUS, Bus error.
std::_Sp_counted_ptr_inplace<spdlog::logger,...>::_M_dispose()
  from /lib/x86_64-linux-gnu/libspdlog.so.1
```

调用链包括：

```text
libspdlog.so.1: spdlog logger dispose/drop_all
Livox-SDK2:     UninitLogger()
Livox-SDK2:     LivoxLidarSdkUninit()
livox driver:   LdsLidar::DeInitLdsLidar()
livox driver:   Lddc::~Lddc()
```

因此退出崩溃发生在 Livox SDK 日志对象释放期间，不是
`timeshare` 内存映射释放造成的。

### 5.3 根本原因：两个不兼容的 spdlog 实现进入同一进程

本机 Livox-SDK2 源码自带：

```text
spdlog 1.3.1
fmt v5
```

ROS 2 Humble/Ubuntu 22.04 使用：

```text
libspdlog 1.9.2
libfmt 8.1.1
```

Livox SDK 将自带的 header-only `spdlog` 编译进：

```text
/usr/local/lib/liblivox_lidar_sdk_shared.so
```

但该 shared library 又导出了大量 `spdlog::*` 弱符号。实际检查到 SDK
导出约 7236 个 `spdlog` 弱符号。与此同时，ROS 2 的
`librcl_logging_spdlog.so` 会把系统 `libspdlog.so.1` 加载到同一进程。

Linux ELF 动态链接器允许默认可见符号被其他 shared library 抢占。
结果可能是：

- Livox SDK 使用 1.3.1 的类定义创建对象；
- 对象的部分成员函数或析构逻辑却解析到系统 1.9.2；
- 两个版本的类布局、模板实现和 fmt ABI 不一致；
- 日志对象释放时访问错误内存，产生 `SIGBUS` 或 `SIGSEGV`。

这也解释了该故障为什么具有偶发性：不同退出顺序可能让异常发生在
`drop_all()` 中，也可能推迟到进程静态对象析构阶段。

仅在 `UninitLogger()` 中添加 `logger.reset()` 不能解决问题。实测它只是
把异常提前到了 `spdlog::drop_all()`，因为根因是符号和 ABI 冲突，而不是
单纯漏掉一次 `shared_ptr::reset()`。

---

## 6. 退出崩溃的修复方案

Livox SDK 内部必须始终调用它自己编译进去的 `spdlog/fmt` 实现，不能让
ROS 2 的系统 `spdlog` 抢占这些调用。

修改文件：

```text
/home/unitree/packages/Livox-SDK2/sdk_core/CMakeLists.txt
```

在创建 shared library 后加入：

```cmake
# Keep Livox-SDK2's bundled spdlog/fmt implementation private to the SDK at
# runtime. ROS 2 Humble loads a different system spdlog ABI in the same process;
# allowing those symbols to preempt the SDK's bundled implementation can crash
# during logger destruction.
if(UNIX AND NOT APPLE)
  set_target_properties(${SDK_LIBRARY_SHARED} PROPERTIES
          LINK_FLAGS "-Wl,-Bsymbolic")
endif()
```

`-Wl,-Bsymbolic` 会让该 shared library 对自身已定义符号的内部引用绑定到
自身实现，避免 Livox SDK 的 `spdlog 1.3.1` 调用被 ROS 2 的
`spdlog 1.9.2` 替换。

重新构建 SDK：

```bash
cmake \
  -S ~/packages/Livox-SDK2 \
  -B ~/packages/Livox-SDK2/build

cmake --build \
  ~/packages/Livox-SDK2/build \
  --target livox_lidar_sdk_shared \
  --parallel "$(nproc)"
```

将修复后的 shared library 放入当前 overlay：

```bash
install -m 0644 \
  ~/packages/Livox-SDK2/build/sdk_core/liblivox_lidar_sdk_shared.so \
  ~/fast_livo2_handhold2_livox_ws/install/livox_ros_driver2/lib/
```

加载当前工作空间后，`LD_LIBRARY_PATH` 会优先包含
`install/livox_ros_driver2/lib`，所以不需要覆盖系统中的原始 SDK。

验证实际加载路径：

```bash
source ~/fast_livo2_handhold2_livox_ws/install/setup.bash

ldd \
  ~/fast_livo2_handhold2_livox_ws/install/livox_ros_driver2/lib/liblivox_ros_driver2.so \
  | grep livox_lidar_sdk
```

期望结果：

```text
liblivox_lidar_sdk_shared.so =>
/home/unitree/fast_livo2_handhold2_livox_ws/install/livox_ros_driver2/lib/liblivox_lidar_sdk_shared.so
```

如果以后删除整个 `install` 目录并全量重建，需要重新复制该 SDK；
也可以在确认无其他项目依赖旧 SDK 后，将修复版安装到 `/usr/local`：

```bash
sudo cmake --install \
  ~/packages/Livox-SDK2/build \
  --prefix /usr/local
sudo ldconfig
```

---

## 7. 验证结果

### 7.1 驱动运行

不启动 RViz 的 custom message 测试：

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

实测：

```text
/livox/lidar  ≈ 10.0 Hz
/livox/imu    ≈ 200.0 Hz
```

RViz/PointCloud2 测试：

```bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

实测：

```text
Topic type:         sensor_msgs/msg/PointCloud2
Publisher count:    1
Subscription count: 1
Point cloud rate:   ≈ 10.0 Hz
```

### 7.2 时间戳共享文件

```text
/home/unitree/timeshare
size = 16 bytes
```

连续读取时 `low` 字段持续变化，例如：

```text
0  1785396566900419058
0  1785396567900270860
```

说明文件映射及 LiDAR 时间戳更新正常。

### 7.3 退出测试

使用未修复 SDK 时，GDB 能稳定捕获 `spdlog` 析构阶段的 `SIGBUS`。

加入 `-Wl,-Bsymbolic` 并加载修复版 SDK 后，相同的启动、数据接收和
`SIGINT` 流程结果为：

```text
Livox Lidar SDK Deinit completely!
lddc destory!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
lds destory!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
process has finished cleanly
```

GDB 结果：

```text
Inferior exited normally
```

随后又使用正式的 `rviz_MID360_launch.py` 做了一次非 GDB 启动—退出测试，
驱动和 RViz 均正常退出。

---

## 8. 日常启动及快速检查

正常启动：

```bash
source ~/fast_livo2_handhold2_livox_ws/install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

检查 topic：

```bash
ros2 topic info /livox/lidar
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

如果再次看到：

```text
basic_string::_M_construct null not valid
```

说明运行的仍是旧驱动，检查：

```bash
ros2 pkg prefix livox_ros_driver2
```

它必须指向：

```text
/home/unitree/fast_livo2_handhold2_livox_ws/install/livox_ros_driver2
```

如果退出时再次出现 `SIGBUS`、`SIGSEGV` 或 `-11`，检查 `ldd` 输出。
若 `liblivox_lidar_sdk_shared.so` 仍来自 `/usr/local/lib`，说明当前终端
没有 source 正确的 overlay，或修复后的 SDK 文件不在当前 overlay 的
`lib` 目录中。

---

## 9. 本次变更清单

当前工作空间：

- `livox_ros_driver2/src/lddc.cpp`
  - 删除对 `getlogin()` 返回值的非法使用；
  - 安全创建、调整和映射 `timeshare`；
  - 所有失败路径不再影响点云发布；
  - 增加映射指针访问保护。
- `livox_ros_driver2/src/lddc.h`
  - 将 `pointt` 初始化为 `nullptr`。
- `fixbug.md`
  - 本问题分析及修复记录。

Livox-SDK2：

- `/home/unitree/packages/Livox-SDK2/sdk_core/CMakeLists.txt`
  - 为 shared SDK 增加 `-Wl,-Bsymbolic`，隔离 SDK 自带的旧版
    `spdlog/fmt`。

部署产物：

- `~/fast_livo2_handhold2_livox_ws/install/livox_ros_driver2/lib/liblivox_lidar_sdk_shared.so`

原有的 `livox_ros_driver2/config/MID360_config.json` 用户修改已保留，
本次修复没有覆盖该配置。
