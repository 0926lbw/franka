# libsurvive [![Build and Test](https://github.com/cntools/libsurvive/workflows/Build%20and%20Test/badge.svg)](https://github.com/cntools/libsurvive/actions/workflows/cmake.yml)[![Build Nuget](https://github.com/cntools/libsurvive/workflows/Build%20Nuget/badge.svg)](https://github.com/cntools/libsurvive/actions/workflows/build_nuget.yml)[![Build Wheels](https://github.com/cntools/libsurvive/workflows/Build%20Wheels/badge.svg)](https://github.com/cntools/libsurvive/actions/workflows/build_wheels.yml)

![Logo](https://cloud.githubusercontent.com/assets/2748168/24084003/9095c98a-0cb8-11e7-88a3-575f9f4c7bb4.png)

Libsurvive 是一组工具和库，用于在基于 [Lighthouse 和 Vive](https://en.wikipedia.org/wiki/HTC_Vive) 的系统上实现 6 自由度跟踪。它完全开源，可以在任意设备上运行。它目前同时支持 SteamVR 1.0 和 SteamVR 2.0 世代的设备，并且应该能够支持市面上可购买到的任何被跟踪对象。

由于项目重点是跟踪，它不会独立驱动 HMD。如果你需要能完成这件事的开源栈，请参考 [monado](https://monado.freedesktop.org/)。

大部分开发讨论都在 Discord 上进行。[加入我们的 Discord 聊天和讨论](https://discordapp.com/invite/7QbCAGS)。也可以通过 [Matrix 桥接房间](https://app.element.io/#/room/#libsurvive-main:matrix.org) 加入讨论。

下面是 libsurvive 在 Godot 中驱动手柄和 HMD 的示例应用：
[![Watch video](https://img.youtube.com/vi/yC75XknKTo0/0.jpg)](https://www.youtube.com/watch?v=yC75XknKTo0)

目录
====

   * [快速开始](#快速开始)
      * [Debian](#debian)
      * [Windows](#windows)
   * [当前状态](#当前状态)
   * [路线图](#路线图)
   * [入门](#入门)
      * [校准](#校准)
      * [可视化](#可视化)
      * [libsurvive 工具](#libsurvive-工具)
      * [在自己的应用中使用 libsurvive](#在自己的应用中使用-libsurvive)
         * [低层 API](#低层-api)
         * [高层 API](#高层-api)
         * [Python 绑定](#python-绑定)
         * [C# 绑定](#c-绑定)
      * [数据录制](#数据录制)
         * [普通录制](#普通录制)
         * [原始 USB 录制](#原始-usb-录制)
      * [常用命令行参数](#常用命令行参数)
   * [驱动](#驱动)
      * [自定义驱动](#自定义驱动)
   * [FAQ](#faq)
      * [附录和备注](#附录和备注)

# 快速开始

## Debian

```
git clone https://github.com/cntools/libsurvive.git
cd libsurvive
sudo cp ./useful_files/81-vive.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo apt update && sudo apt install build-essential zlib1g-dev libx11-dev libusb-1.0-0-dev freeglut3-dev liblapacke-dev libopenblas-dev libatlas-base-dev cmake
make
```

插入头显、追踪器、控制器等设备，然后运行：

```
./bin/survive-cli
```

它应该会校准并显示你的设置。

如果要可视化，你可以下载 [websocketd](https://github.com/joewalnes/websocketd/releases/) 的二进制文件，或者启用实验性的 apt 源并使用 `sudo apt install websocketd` 安装。之后可以运行：

```
./bin/survive-websocketd & xdg-open ./tools/viz/index.html
```

[![Watch video](https://img.youtube.com/vi/l4doRSXM0tU/0.jpg)](https://www.youtube.com/watch?v=l4doRSXM0tU)

## Windows

如果你的 `cmake` 已经安装并在 PATH 中，可以直接右键 `make.ps1` 脚本，选择 `Run with PowerShell` 运行。

更手动的方式是用类似 [CMake GUI](https://cmake.org/runningcmake/) 的工具打开 CMakeLists 文件，并使用某个 Visual Studio 生成器从源码构建。这也能让你设置各种构建选项。构建过程使用 NuGet 获取[必要的开发依赖](https://www.nuget.org/packages/lapacke/)。生成项目后，在 Visual Studio 中打开 solution 并运行 build all。

[Websocketd](http://websocketd.com/) 应该能以相同方式配合可视化工具工作，前提是你把它放到了系统 PATH 中。如果你通过 `make.ps1` 构建，构建二进制目录中，例如 `./build-win/Release`，应该会有一个 `survive-websocketd.ps1`，可以作为 PowerShell 文件运行。

在 Windows 上开始使用 libsurvive，最简单的方式可能是查看[发布版二进制文件](https://github.com/cntools/libsurvive/releases)。

# 当前状态

目前跟踪和设备枚举工作得相当不错；不过测试基数还不是很大，工具的打磨程度也不如 SteamVR 自带的同类工具。项目仍在持续量化跟踪精度，并改善用户体验。

# 路线图

下面是短期计划中的一些宽泛事项：

- 动态校正校准。它会检测可能移动过的 Lighthouse，并在后台重新校准。最终目标是让用户完全不需要意识到校准要求。
- Android 二进制文件或移植。
- 给出 libsurvive 作为跟踪系统的准确度和精密度硬指标。如果有人愿意贡献时间在 CNC 上测试，请加入我们的 [Discord](https://discordapp.com/invite/7QbCAGS)。
- 更好地处理数据饥饿。如果 USB 或无线连接卡顿太久，跟踪有时会短暂异常。
- 在 Windows 上使用类似 usbmon 的东西。

# 入门

如果你按照快速开始指南操作，你会注意到在 Linux 上第一件必须做的事是安装 udev 规则：

```
sudo cp ./useful_files/81-vive.rules to /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
```

这样可以在不使用 root 的情况下访问这些设备。Windows 上不需要这样的步骤。

之后，当你运行 `survive-cli` 或 `survive-websocketd` 时，它应该会识别所有已插入的 Vive 设备，并开始校准和跟踪这些设备。

**重要：为了获得最佳效果，请关闭 SteamVR。根据系统不同，libsurvive 要么会导致 SteamVR 失去设备连接，要么会与 SteamVR 竞争带宽。**

## 校准

校准用于确定 Lighthouse 相对于被跟踪对象的安装位置。第一次运行 libsurvive 时，它可能需要最多十秒钟与 Lighthouse 通信，并弄清楚它们的位置。

只要对象短暂静止，校准就会持续整合对象数据。因此你可能会注意到，在它获得较好锁定时，Lighthouse 会轻微移动。之后再次运行时，移动应该会小得多。

完成一次后，结果会保存到 `XDG_CONFIG_HOME/libsurvive` 下的 `config.json`。如果你删除这个文件，它会重新校准；不过使用 `--force-calibrate` 参数会更快。有些驱动会改变这个文件名，尤其是录制文件会改用 `<event_file>.json`。

如果你的空间很大，无法把单个设备放在中央以“看到”所有 Lighthouse，你可以先校准几个 Lighthouse，然后把被跟踪对象移动到尚未校准的 Lighthouse 视野中，同时保持它仍能被至少一个已校准 Lighthouse 看到。把它放下并保持静止，剩余 Lighthouse 就应该会完成校准。

**重要：当 Lighthouse 被移动时自动重置校准的功能已有计划，但目前还不可用。如果某个已校准 Lighthouse 被移动，你必须删除 config.json 文件重新校准，或者给任意 libsurvive 工具传入 `--force-calibrate`。**

## 可视化

主要的可视化工具是一个 THREE.js 页面，数据通过 [websocketd](http://websocketd.com/) 输入。要使用这个工具，请运行 `survive-websocketd [options]`，然后从克隆仓库的根目录在浏览器中打开 `./tools/viz/index.html`。

![Visuzliation Screenshot](https://raw.githubusercontent.com/cnlohr/libsurvive/master/useful_files/viz_screenshot.png)

## libsurvive 工具

- `survive-cli` - 这个库的主要命令行接口；本质上只是对库的非常薄的一层包装。
- `survive-websocketd` - 一个脚本，通过 `websocketd` 运行 `survive-cli`，并设置所有合适的参数。
- `sensors-readout` - 在 ncurses 界面中显示原始传感器信息。

## 在自己的应用中使用 libsurvive

### 低层 API

本节主要关注如何从库中*消费*数据；如果你想了解如何*提供*数据，请参阅[驱动章节](https://github.com/cntools/libsurvive/#drivers)。

扩展和使用 libsurvive 的主要方式，是使用库暴露的各种回调从系统获取信息。如果你需要访问所有输入数据，例如 IMU 数据、单独的光数据以及最终姿态数据，推荐用这种方式使用 libsurvive。不过需要小心，不要拖慢系统。通常这些回调会在采集数据的线程中调用；因此如果你在处理数据时引入不必要的延迟，就会丢数据，并导致跟踪性能变差。

完整 hook 列表在[这里](https://github.com/cntools/libsurvive/blob/master/include/libsurvive/survive_hooks.h)。函数类型在[这里](https://github.com/cntools/libsurvive/blob/master/include/libsurvive/survive_types.h#L168)。

你可以用下面的方式安装自定义 hook：

`<hook-name>_process_func survive_install_<hook-name>_fn(SurviveContext *ctx, <hook-name>_process_func fbp);`

它会返回该 hook 之前设置的函数，你可以选择在自己的回调中调用它。相比让回调直接返回 `true` 或 `false`，这种方式稍微麻烦一些；但它允许你在自己的代码之前或之后调用旧函数，或者完全不调用，灵活性更高。

这些 hook 在 libsurvive 内部也会使用；如果你提供了某个 hook，却既不调用之前定义的函数，也不调用默认函数，一些数据就不会传到 poser。

`SurviveContext` 和 `SurviveObject` 都有一个 `user_ptr` 变量，它会被零初始化，并且内部不会使用。它的用途是让库的使用者按自己的目的设置它。这样你可以安装 hook，而不必依赖全局变量。

这些接口相对稳定，但不保证永远不变。

可以查看仓库顶层的其他 libsurvive 工具，了解低层 API 的示例用法：

- survive-cli.c
- sensors-readout.c
- simple_pose_test.c

### 高层 API

如果应用只需要尽可能快地处理位置和速度数据，推荐使用高层 `Simple` API。它有几个主要优点：

- 用户代码运行在自己的线程中，因此不会让 libsurvive 饿死数据处理。
- 更好地隔离低层 API 变化，因此更容易升级 libsurvive 版本。

主循环逻辑在 `C` 中通常类似这样：

```C
while (survive_simple_wait_for_update(actx) && keepRunning) {
    for (const SurviveSimpleObject *it = survive_simple_get_next_updated(actx); it != 0;
         it = survive_simple_get_next_updated(actx)) {
        SurvivePose pose;
        uint32_t timecode = survive_simple_object_get_latest_pose(it, &pose);
        printf("%s %s (%u): %f %f %f %f %f %f %f\n", survive_simple_object_name(it),
               survive_simple_serial_number(it), timecode, pose.Pos[0], pose.Pos[1], pose.Pos[2], pose.Rot[0],
               pose.Rot[1], pose.Rot[2], pose.Rot[3]);
    }
}
```

应用接口的示例代码可以在 [api_example.c](https://github.com/cntools/libsurvive/blob/master/api_example.c) 中找到。

更简单 API 的完整头文件在[这里](https://github.com/cntools/libsurvive/blob/master/include/libsurvive/survive_api.h)。

### Python 绑定

Windows 和 Linux 上可以通过 https://pypi.org/project/pysurvive/ 使用面向 python3 的 Python 绑定。安装方式：

```
pip install pysurvive
```

要构建 Python 绑定，请在仓库根目录运行 `python setup.py install`。这应该会安装 `pysurvive` 包。

下面是一个实时输出姿态流的示例：

```
import pysurvive
import sys

actx = pysurvive.SimpleContext(sys.argv)

for obj in actx.Objects():
    print(obj.Name())

while actx.Running():
    updated = actx.NextUpdated()
    if updated:
        print(updated.Name(), updated.Pose())
```

`./bindings/python` 中还有更多示例。

### C# 绑定

C# 绑定同时包装了低层访问 API 和更易用的高层 API。推荐使用高层 API，因为低层 API 严重依赖回调，编组处理容易出错，而且错误并不总是容易解决。

标准二进制文件可以从 https://www.nuget.org/packages/libsurvive.net/ 获取。你可以通过 Visual Studio 的 NuGet 管理器把它们安装到指定 C# 项目中。

可以用 Visual Studio 构建[这个 solution](https://github.com/cntools/libsurvive/tree/master/bindings/cs)，也可以在终端中运行类似下面的命令：

```
dotnet build -c Release
```

来生成二进制文件 `libsurvive.net.dll`。这在 Linux 和 Windows 上都可以工作；不过文件名仍然以 `dll` 结尾。运行这个二进制文件时，`libsurvive.so` 需要位于同一目录中，并带有所需插件，或者位于系统路径中。

高层 API 通过 `libsurvive.SurviveAPI` 对象暴露。它的使用很简单，可以像 [Demo 项目](https://github.com/cntools/libsurvive/tree/master/bindings/cs/Demo/Program.cs) 中那样轮询对象位置或按钮事件更新：

```cs
using libsurvive;
using System;

namespace Demo
{
    
    class Program
    {
		static void Main() {
			string[] args = System.Environment.GetCommandLineArgs();
			var api = new SurviveAPI(args);

			while (api.WaitForUpdate()) {
				SurviveAPIOObject obj;
				while ((obj = api.GetNextUpdated()) != null) {
					Console.WriteLine(obj.Name + ": " + obj.LatestPose);
				}
			}

			api.Close();
		}
	}
}

```

它也旨在易于集成到基于帧更新的代码库中，例如 [Unity 示例](https://github.com/cntools/libsurvive/blob/master/bindings/cs/UnityViewer/Assets/SurviveObject.cs)：

```cs
// Update is called once per frame
void Update() {
    var updated = survive?.GetNextUpdated();

    if (updated == null)
        return;

    var updatedObject = getObject(updated.Name);

    Vector3 newPosition = Vector3.zero;
    Quaternion newRotation = Quaternion.identity;
    SurvivePose pose = updated.LatestPose;
    newPosition.x = (float) pose.Pos[0];
    newPosition.y = (float) pose.Pos[1];
    newPosition.z = (float) pose.Pos[2];
    newRotation.w = (float) pose.Rot[0];
    newRotation.x = (float) pose.Rot[1];
    newRotation.y = (float) pose.Rot[2];
    newRotation.z = (float) pose.Rot[3];
    updatedObject.transform.localPosition = newPosition;
    updatedObject.transform.localRotation = newRotation;
}
```

[![Watch video](https://img.youtube.com/vi/FiRLrWWOhLg/0.jpg)](https://www.youtube.com/watch?v=FiRLrWWOhLg&feature=youtu.be)

## 数据录制

有很多因素会导致跟踪或校准结果变差。考虑到设备、配置和使用场景非常多样，在排查 bug 时，如果能有 bug 的数据录制，会非常有帮助。这些录制也可以加入我们的 CI 系统，用来针对该用例做自动测试。

目前有两种录制数据的机制：`--record` 和 `--usbmon-record`。

`--record` 在所有安装中都可用，使用时需要的设置也更少。因此对于纯跟踪问题，通常应该使用这种方式。不过，如果问题出在解析和理解被跟踪设备的低层数据包上，`--usbmon-record` 选项可能更有帮助。

### 普通录制

运行任意 libsurvive 工具时，传入 `--record <filename>.rec.gz`。这会在该文件中创建一个数据日志，记录系统运行期间看到的一切。它会记录大量数据，所以如果运行很久，文件可能会变得很大。

要回放这个文件，请运行：

`./survive-cli --playback <filename>.rec.gz`

### 原始 USB 录制

偶尔在处理新硬件，或者某些会在 USB 层造成问题的 bug 时，需要获得看到或发送过的原始 USB 数据捕获。USBMON 驱动可以做到这一点。

目前这个驱动只在 Linux 上可用，并且必须安装 libpcap：`sudo apt install libpcap-dev`。你还需要安装 `usbmon` 内核模块；不过许多 Linux 发行版已经内置了它。

要启动 usbmon 并让所有用户都能使用它，请运行：

```
sudo modprobe usbmon
sudo setfacl -m u:$USER:r /dev/usbmon* # 在敏感环境中，你可以改用 sudo 运行 survive-cli。
```

要捕获 USB 数据，请运行：

```
./survive-cli --usbmon-record <filename>.pcap.gz --htcvive <additional options>` 
```

可以用下面的命令回放：

```
./survive-cli --usbmon-playback <filename>.pcap.gz [--playback-factor x] <additional options>` 
```

如果你要把这个文件发送给别人分析，请注意它需要配套的 `*.usbdevs` 文件才有用。如果你遵循 `*.pcap.gz` 命名约定，可以运行类似下面的命令：

```
zip logs.zip *.pcap* config.json
```

然后把 `logs.zip` 发到 issue 或 Discord。

这个驱动只会捕获白名单中的 VR 设备；不过如果你不想把原始 USB 数据发布到互联网上，可以在 Discord 上询问应该私信发给谁。

## 常用命令行参数

Libsurvive 的可配置性很强，根据驱动和构建时选项不同，它包含很多命令行选项。

如果你需要经常使用命令行选项，建议安装 libsurvive 的 bash 补全：

`sudo cp survive_autocomplete.sh /etc/bash_completion.d/`。

调试时最有用的命令行选项是 `--v`，它会设置报告级别。这个详细程度大致遵循下面的规则：

- `--v 10` - 在应用启动或结束时显示统计和信息。
- `--v 100` - 在跟踪过程中的许多常见位置显示信息。
- `--v 150` - 几乎每次姿态输出都显示信息。
- `--v 250` - 系统中几乎所有光数据事件都显示信息。
- `--v 1000` - 显示一切。

使用较高详细程度，大于 100，会让可视化工具变卡。

`--force-calibrate`：重新运行校准，但复用 OOTX，因此运行速度会快得多。

`--playback-factor`：回放录制文件时，用它加速回放，0 表示尽可能快地运行全部内容；也可以减速，2 表示耗时为原来的两倍。

`--lighthouse-gen`：强制系统使用某一代 Lighthouse。目前系统有时会把 Lighthouse 1，也就是纯方形基站，误识别成 Lighthouse 2，也就是圆角正面基站，反之亦然。随着我们发现这些情况，会持续修复；但这个选项可以让行为异常的系统暂时仍可使用。

# 驱动

这些是向 libsurvive 提供信息的不同驱动。它们都封装在 `src` 目录中，并带有 `driver_` 前缀。每个驱动都可以通过 `--<driver-name>` 参数指定。你可以用 `--no-<driver-name>` 禁用默认驱动，例如 `htcvive`。

- `htcvive` - 主要驱动，通过 USB 连接从 Vive 硬件提供数据。
- `simulator` - 模拟一个漂浮的设备，同时向 libsurvive 提供逼真的光数据和 IMU 数据。适合测试不同功能。
- `playback` - 回放驱动用于启用录制和回放功能。它会把文件回放到各个数据点中。
- `usbmon` - USBmon 可以与 SteamVR 并发运行，让两个系统同时使用被跟踪对象数据。
- `openvr` - 这个驱动暴露外部姿态和速度，可以与 `usbmon` 一起运行，用来比较两个系统。

## 自定义驱动

集成驱动的目标是保持相对直接，上面提到的驱动都是很好的参考。

通常做法是编译成一个共享对象或 DLL，名称为 `driver_<name>.so`，并放在 plugins 文件夹中。libsurvive 内部会枚举这些插件，并运行 libsurvive 注册函数，函数形式如下：

```C
int DriverRegExample(SurviveContext *ctx) {
    if(...error...) {
       return SURVIVE_DRIVER_ERROR;
    }
    return SURVIVE_DRIVER_NORMAL;
}
REGISTER_LINKTIME(DriverRegExample)
```

驱动不一定需要注册其他东西；但为了集成到 libsurvive 中，驱动必须暴露 poll/close 函数，或者 thread/close 函数。

最简单的方式是 poll 驱动：

`void survive_add_driver(SurviveContext *ctx, void *user_ptr, DeviceDriverCb poll, DeviceDriverCb close)`

poll 函数会在系统运行时被连续调用；close 函数会在关闭时调用。

更灵活的是 threaded 驱动：

`bool *survive_add_threaded_driver(SurviveContext *ctx, void *driver_data, const char *name, void *(routine)(void *), DeviceDriverCb close);`

它会以给定函数和名称启动一个线程。

在线程驱动场景中，访问 `SurviveContext` 或 `SurviveObject` 的任何成员时，都必须用下面的函数在访问前后加锁和解锁：

```
void survive_get_ctx_lock(SurviveContext *ctx);
void survive_release_ctx_lock(SurviveContext *ctx);
```

不正确的加锁或解锁可能导致竞态条件或死锁。

无论在线程函数还是 poll 函数中，驱动都需要自行用它暴露的数据调用合适的 hook 函数。通常驱动还会调用 `survive_create_device`，并只关注这个设备。多个驱动可以同时运行；但它们都假设使用相同的 Lighthouse 配置。

`driver_simulator.c` 是这方面的好例子，它用自定义 `SurviveObject` 类型调用各种光数据和 IMU 回调。`driver_openvr.cc` 演示了如何把外部位置数据纳入库中。

# FAQ

使用 libsurvive 的其他项目：

 * [Monado OpenXR runtime](https://monado.freedesktop.org/) 将 libsurvive 用作它的 HMD 和控制器驱动之一。
   * 在 Monado 上运行的 OpenXR 应用包括 [Godot 3.x 的 OpenXR 插件](https://github.com/GodotVR/godot_openxr)。
 * 有一个非常非官方，并且不适合上游合并的 [OpenHMD/libsurvive fork](https://github.com/ChristophHaag/OpenHMD/commits/libsurvive2)，它添加了 libsurvive 驱动。
   * 这个 OpenHMD/libsurvive fork 可以接入 [SteamVR-OpenHMD](https://github.com/ChristophHaag/SteamVR-OpenHMD)，也可以通过 [Godot 3.x 的 OpenHMD 插件](https://github.com/BastiaanOlij/godot_openhmd) 原生使用。

## 附录和备注

感谢 Faul 先生为我们制作 logo！
特别感谢 @nairol 在他的 https://github.com/nairol/LighthouseRedox 项目中对现有 HTC Vive 系统进行了极其细致的逆向工程。
